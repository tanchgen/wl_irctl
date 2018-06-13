/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"

#include "main.h"
#include "process.h"
#include "rfm69.h"
#include "button.h"
#include "ir.h"
#include "proto.h"
#include "stm32l0xx_it.h"

uint32_t cnt5mks = 0;
uint8_t rssiVol;    //
//uint8_t txCpltCount = 0;

//#if ! STOP_EN
//uint8_t rtcLogCount;
//struct {
//  uint32_t ssr;
//  eState state;
//} rtcLog[64];
//#endif

/* External variables --------------------------------------------------------*/

extern volatile uint8_t csmaCount;

//void extiPdTest( void ){
//  if(EXTI->PR != 0){
//    uint32_t tmp = EXTI->PR;
//    EXTI->PR = tmp;
//    if( tmp != 0x00020000 ){
//      return;
//    }
//    RTC->ISR &= ~RTC_ISR_ALRAF;
//    NVIC->ICPR[0] = NVIC->ISPR[0];
//  }
//}

inline void txToutSet( void );

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

void NMI_Handler(void){
}

void HardFault_Handler(void){
  while (1)
  {}
}

void SVC_Handler(void){
}

void PendSV_Handler(void){
}

void SysTick_Handler(void) {
//  mTick++;
}

/**
* RTC global interrupt through EXTI lines 17, 19 and 20.
*/
void RTC_IRQHandler(void){
// Восстанавливаем настройки портов
  restoreContext();
  // Отмечаем запуск MCU
#if DEBUG_TIME
	dbgTime.mcuStart = mTick;
#endif // DEBUG_TIME

  if( RTC->ISR & RTC_ISR_WUTF ){
    // Wake-Up timer interrupt
  	wutStop();
  	wutIrqHandler();
  }
  // =============== Сработал будильник приемника =======================
  if( RTC->ISR & RTC_ISR_ALRBF ){
    //Стираем флаг прерывания и будильника вкл. приема, и предачи (прием важнее!)
    RTC->ISR &= ~(RTC_ISR_ALRAF | RTC_ISR_ALRBF);
    // Стираем флаг прерывания EXTI
    EXTI->PR &= EXTI_PR_PR17;
    // Таймер на прослушивание канала для приема команд = 10мс
    wutSet(10000);
    // Включаем прием
    rfmSetMode_s( REG_OPMODE_RX );
    state = STAT_LISTEN_START;
  }
// =============== Сработал будильник передачи =======================
  else if( RTC->ISR & RTC_ISR_ALRAF ){
//    while( (RTC->ISR & RTC_ISR_RSF) == 0 )
//    {}
//    uint32_t dr = RTC->DR;
//    (void)dr;
//    uint32_t tr0 = RTC->TR;
//    uint32_t tr = RTC->TR;
//    if (tr0 != tr ){
//      tr = RTC->TR;
//    }

    uxTime = getRtcTime();

//    wutTest[wutCount++].wutVol = rtc.sec;
//    if( wutCount == 20){
//      wutCount = 0;
//    }
    if( ((rtc.sec % secToutTx) == 0) && ((rtc.min % minToutTx) == 0) ){
      if(state == STAT_READY){
        // Периодическое измерение - измеряем все
        mesure();
        // Настало время передачи: Передаем состояние
        csmaRun();
      }
    }
    //Clear ALRAF
    RTC->ISR &= ~RTC_ISR_ALRAF;
    // Стираем флаг прерывания EXTI
    EXTI->PR &= EXTI_PR_PR17;
  }

  // Отмечаем Останов MCU
#if DEBUG_TIME
	dbgTime.mcuEnd = mTick;
#endif // DEBUG_TIME

  // Стираем PWR_CR_WUF
  PWR->CR |= PWR_CR_CWUF;
  while( (PWR->CSR & PWR_CSR_WUF) != 0)
  {}
	// Сохраняем настройки портов
	saveContext();
	// Проверяем на наличие прерывания EXTI
//	extiPdTest();
}

/**
* Главное прерывание от RFM - DIO0
*/
void EXTI0_1_IRQHandler(void)
{
  tPkt rxPkt;

  // Восстанавливаем настройки портов
  restoreContext();

  // Стираем флаг прерывания EXTI
  EXTI->PR &= DIO0_PIN;
  if( rfm.mode == MODE_RX ){
    // Приняли команду.

    driveData.rssi = rfmRegRead( REG_RSSI_VAL );
    rfmReceive( &rxPkt );
    rfmRecvStop();
    if( rxPkt.payDriveType == DRIV_TYPE_IRCTL){
      driveData.cmdNum = rxPkt.payLoad.cmdMsg.cmdNum;
      if( connectFlag == FALSE ){
        enum eAcErr acErr;

        // Маскируем секунды в будильнике A (TX)
        setAlrmSecMask( RESET );
        // Включаем будильник B (RX)
        alrmBOn();
        secToutTx = 1;
        minToutTx = 6;
        connectFlag = TRUE;
        // Принята команда - Кодируем и отправляем команду на ИК
        acData = *((tAcData *)&(rxPkt.payState));
        if( (acErr = protoPktCod()) == AC_ERR_OK ){
          acErr = irPktSend();
        }
        acData.err = acErr;
        // Обновляем состояние устройства
        mesure();
        wutSet( 50000 );
        state = STAT_DRIV_SEND;
      }
      rfmListenStop();
      state = STAT_READY;
    }

  }
  else if( rfm.mode == MODE_TX ) {
    // Отправили какой-то пакет
    txEnd();
    if( connectFlag == FALSE ){
      // Если соединение еще не установленно
      // Через 50мс будем включать прослушивание канала
      wutSet(50000);
      state = STAT_TX_STOP;
    }
    else {
      wutStop();
    }
    if( connectFlag == FALSE ){
      txToutSet();
    }
  }
  // Отмечаем останов RFM_TX
#if DEBUG_TIME
	dbgTime.rfmTxEnd = mTick;
#endif // DEBUG_TIME

  // Выключаем RFM69
  rfmSetMode_s( REG_OPMODE_SLEEP );

	// Сохраняем настройки портов
	saveContext();
	// Проверяем на наличие прерывания EXTI
//	extiPdTest();
}

// Прерывание по PA3 - DIO3 RSSI
// Канал кем-то занят
void EXTI2_3_IRQHandler( void ){

  // Восстанавливаем настройки портов
  restoreContext();
  wutStop();

  // Выключаем прерывание от DIO3 (RSSI)
  EXTI->IMR &= ~(DIO3_PIN);
  EXTI->PR &= DIO3_PIN;

  rssiVol = rfmRegRead( REG_RSSI_VAL );
  rfmSetMode_s( REG_OPMODE_SLEEP );

  // Отмечаем останов RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxEnd = mTick;
#endif // DEBUG_TIME

  // Канал занят - Выжидаем паузу 30мс + x * 20мс
  if( csmaCount >= CSMA_COUNT_MAX ){
    // Количество попыток и время на попытки отправить данные вышло - все бросаем до следующего раза
  	csmaCount = 0;
    txEnd();
  }
  else {
  	// Можно еще попытатся - выждем паузу
    csmaPause();
  }

	// Сохраняем настройки портов
	saveContext();
	// Проверяем на наличие прерывания EXTI
//  extiPdTest();
  return;
}


// Прерывание по PA15 - Кнопка
// Прерывание по PB6 - IR-reseiver
void EXTI4_15_IRQHandler( void ){
  // Восстанавливаем настройки портов
  restoreContext();
  wutStop();

  if( EXTI->PR & BTN_PIN){
    // Выключаем прерывание от КНОПКИ
    EXTI->IMR &= ~(BTN_PIN);
    EXTI->PR &= BTN_PIN;

    // Что бы ни было ранее - отключаем обработку входа ИК-приемника
    EXTI->IMR &= ~(IR_RX_PIN);
    EXTI->PR &= IR_RX_PIN;

    // Сохраняем состояние кнопки на момент прерывания для антидребеза
    btn.stat = (BTN_PORT->IDR & BTN_PIN) >> BTN_PIN_NUM;
    state = STAT_BTN_DBNC;

    // Начало отсчета дребезга - 30ms
    wutSet(30000);
  }
  if ( EXTI->PR & IR_RX_PIN ){
    irRxProcess();
    EXTI->PR &= IR_RX_PIN;
  }

  // Сохраняем настройки портов
  saveContext();
  // Проверяем на наличие прерывания EXTI
//  extiPdTest();
  return;
}

// Обработчик прерывания от кнопки
void TIM6_IRQHandler( void ){
    TIM6->SR &= ~TIM_SR_UIF;
    // Переключаем вывод 1->0, 0->1
    BUZ_PORT->ODR ^= BUZ_PIN;
    cnt5mks++;
}

// Обработчик прерывания таймера несущей (38кГц) ИК-передатчика
void TIM21_IRQHandler( void ){
}

// Обработчик прерывания таймера модуляции (длительность импульсов и пауз) ИК-передатчика
void TIM22_IRQHandler( void ){

  if( TIM22->SR & TIM_SR_UIF){
    GPIOA->BRR |= GPIO_Pin_11;
//      IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x2 << (IR_TX_PIN_NUM * 2));

    uint16_t tmp;
    // Заполняем поле пульса
    TIM22->CCR1 = irPkt[txFieldCount++];
    // Заполняем поле паузы (Пауза = ARR - "пульс")
    tmp = TIM22->CCR1;
    if( txFieldCount < field0Num){
      tmp += irPkt[txFieldCount++];
    }
    TIM22->ARR = tmp;

    TIM22->SR &= ~TIM_SR_UIF;
  }
  else if( TIM22->SR & TIM_SR_CC1IF){
    GPIOA->BSRR |= GPIO_Pin_11;
    // Пульс закончился
    if( txFieldCount > field0Num ){
      // Передача закончена - все выключаем
      TIM22->CR1 &= ~TIM_CR1_CEN;
      IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));
    }
    TIM22->SR &= ~TIM_SR_CC1IF;
  }
}

void TIM2_IRQHandler( void ){
  TIM2->SR &= ~TIM_SR_UIF;
  TIM2->CR1 &= ~TIM_CR1_CEN;

  // Примой счет - прием (обучение)
  // Длительность паузы более 100мс -> пакет принят.
//    buzzerShortPulse();
  // Обнуляем счетчик импульсов и пауз
  if( irRxIndex > 0 ){
    if( protoName == PROTO_NONAME ){
      protoName = learnProcess();
      if( protoName != PROTO_NONAME ){
        // Протокол известен - конец обучению
        mDelay(125);
        buzzerLongPulse();
        onOffFlag = ON;
        field0Num = irRxIndex;
        eeIrProtoBak.fld0Num = field0Num;
        eeIrProtoBak.protoName = protoName;
      }
    }
    irRxGetFlag = SET;
    irRxIndex = 0;
  }
  // Включаем маску внешнего прерывания от ИК-приемника
  EXTI->IMR &= ~IR_RX_PIN;
  // Выждем паузу 250мс до ожидания следующего ИК-пакета
  state = STAT_IR_RX_STOP;
  wutSet(250000);
}

inline void txToutSet( void ){
  if( connectCount == 39 ){
    secToutTx = 1;
    minToutTx = 6;
  }
  else {
    connectCount++;
    if( connectCount == 30){
      secToutTx = 1;
      minToutTx = 2;
    }
    else if( connectCount == 20){
      // Переводим будильник на минутный интервал
      setAlrmSecMask( RESET );
      secToutTx = 1;
      minToutTx = 1;
    }
    else if( connectCount == 10){
      secToutTx = 30;
      minToutTx = 1;
    }

  }
}
