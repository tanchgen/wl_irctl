/*
 * process.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */


#include "stm32l0xx.h"

#include "main.h"
#include "my_time.h"
#include "bat.h"
#include "rfm69.h"
#include "button.h"
#include "process.h"

volatile uint8_t csmaCount = 0;
volatile uint8_t connectFlag = FALSE;
volatile uint8_t connectCount = 0;
tUxTime sendTryStopTime;
static uint8_t msgNum;      // Порядковый номер отправляемого пакета
extern uint8_t wutCount;

static void sensDataSend( void );
static uint32_t rngGet( void );

void mesure( void ){
  // Запускаем измерение напряжения батареи
  batStart();
  batEnd();
}

void wutIrqHandler( void ){

  // По какому поводу был включен WUT? - состояние машины
  switch( state ){
    case STAT_BTN_DBNC: {
        uint32_t ut;
        // Проверка кнопки после антидребезга
        if( (((BTN_PORT->IDR & BTN_PIN) >> BTN_PIN_NUM) == btn.stat )){
          // Выключаем прием пакетов
//          rfmSetMode_s( REG_OPMODE_SLEEP );
          // Снимаем прерывание, если оно возникло.
          EXTI->IMR &= ~(DIO3_PIN);
          EXTI->PR &= DIO3_PIN;

          ut = getRtcTime();
          // Проверка на первое нажатие кнопки ( включение контроллера )
          if( btn.tOnSec == 0 ) {
            if( btn.stat == BTN_OFF  ){
              btn.tOnSec = ut;
            }
          }
          else {
            buttonProcess( ut );
          }
        }

        // Выключаем маску внешнего прерывания от КНОПКИ
        EXTI->IMR |= BTN_PIN;
      }
      break;
    case STAT_IR_RX_STOP:
      // Выключаем маску внешнего прерывания от ИК-приемника
      EXTI->IMR |= IR_RX_PIN;
      break;
    case STAT_DRIV_SEND:
      csmaRun();
      break;
    case STAT_RF_CSMA_START:
      // Канал свободен - отправляем сообщение
      EXTI->PR &= DIO3_PIN;
      EXTI->IMR &= ~(DIO3_PIN);

      // Отмечаем останов RFM_RX
    #if DEBUG_TIME
    	dbgTime.rfmRxEnd = mTick;
    #endif // DEBUG_TIME

    	rfmSetMode_s( REG_OPMODE_SLEEP );
      // Отправить сообщение
      correctAlrm( ALRM_A );
      state = STAT_TX_START;
      sensDataSend();
      break;
    case STAT_TX_START:
    	rfmSetMode_s( REG_OPMODE_SLEEP );
      txEnd();
      break;

    case STAT_RF_CSMA_PAUSE:
      // Пробуем еще раз проверить частоту канала
      csmaRun();
      break;
    case STAT_LISTEN_START:
      // Никакого сообщения не пришло - засыпаем
      rfmSetMode_s( REG_OPMODE_SLEEP );
      state = STAT_READY;
      break;
    default:
      break;
  }
}

// Начинаем слушат эфир на предмет свободности канала
void csmaRun( void ){
  state = STAT_RF_CSMA_START;
  // Включаем прерывание от DIO3 (RSSI)
  EXTI->IMR |= (DIO3_PIN);
  EXTI->PR &= DIO3_PIN;

  // Отмечаем запуск RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxStart = mTick;
#endif // DEBUG_TIME

	csmaCount++;
  rfmSetMode_s( REG_OPMODE_RX );
  // Будем слушать эфир в течение времени передачи одного пакета * 2
  wutSet( TX_DURAT );
}

// Устанавливааем паузу случайной длительности (30-150 мс) в прослушивании канала на предмет тишины
void csmaPause( void ){
  uint32_t pause;
#ifdef STM32L052
  SYSCFG->CFGR3 |= SYSCFG_CFGR3_ENREF_RC48MHz;
  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  RCC->CCIPR |= RCC_CCIPR_HSI48MSEL;
  while( (RCC->CRRCR & RCC_CRRCR_HSI48RDY) == RESET )
  {}
  // Включаем генератор случайных чисел
  RCC->AHBENR |= RCC_AHBENR_RNGEN;
  RNG->CR |= RNG_CR_RNGEN;
  // Ждем готовности числа (~ 46 тактов)
  while( ((RNG->SR & RNG_SR_DRDY) == 0 ) &&
          ((RNG->SR & RNG_SR_CEIS) == 0 ) &&
          ((RNG->SR & RNG_SR_SEIS) == 0) ){
  }
  // Число RND готово или ошибка (тогда RND = 0)
  pause = RNG->DR;
  // Выключаем
  RNG->CR &= ~RNG_CR_RNGEN;
  RCC->AHBENR &= ~RCC_AHBENR_RNGEN;
  RCC->CCIPR &= ~RCC_CCIPR_HSI48MSEL;
  RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
  SYSCFG->CFGR3 &= ~SYSCFG_CFGR3_ENREF_RC48MHz;
#else
  pause = rngGet();
#endif
  // Длительность паузы
  pause = ((pause / (0xFFFFFFFFL/9)  ) + 1) * TX_DURAT * csmaCount;
  state = STAT_RF_CSMA_PAUSE;
  wutSet( pause );
}

static void sensDataSend( void ){
  // ---- Формируем пакет данных -----
	pkt.payDriveType = DRIV_TYPE_IRCTL;
  pkt.paySrcNode = rfm.nodeAddr;
  pkt.payMsgNum = msgNum++;
  pkt.payBat = driveData.bat;
  pkt.payState = driveData.devState;
  pkt.payCmdNum = driveData.cmdNum;

  // Передаем заполненую при измерении запись
  pkt.nodeAddr = BCRT_ADDR;
  // Длина payload = 1(nodeAddr) + 1(msgNum) + 1(bat) + 2(temp)
  pkt.payLen = sizeof(tDriveMsg);

  rfmTransmit( &pkt );
  // Таймаут до окончания передачи
  wutSet( TX_DURAT*10 );

}

void txEnd( void ){
  // Обнулили индекс тестового массива WUT
  wutCount = 0;
  flags.sensCplt = FALSE;
  flags.batCplt = FALSE;
  state = STAT_READY;
}

#ifndef STM32L052
static uint32_t rngGet( void ){
  uint32_t rand0;
  uint8_t b;
  uint32_t k;

  rand0 = (getRtcTime() << 16) + rtc.ss;
  k = 1220703125;              // Множитель (простое число)
  b = 7;                          // Приращение (простое число)
  rand0 = ( k * rand0 + b );
  return rand0;
}
#endif

