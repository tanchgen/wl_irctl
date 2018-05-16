/*
 * ir.c
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"
#include "main.h"
#include "gpio.h"
#include "button.h"
#include "ir.h"

// Массив НАЧАЛЬНОГО пакета
uint16_t ir0Pkt[255];

// Массив НЕ НАЧАЛЬНОГО пакета (приятый или отправляемый пакет)
uint16_t irPkt[255];

// Указатель на пакет: НАЧАЛЬНЫЙ, НЕ начальный, отправляемый
uint16_t *pIrPkt;

// Структура изменяемого поля ON/OFF
tRxFieldLst irOnOffField;

// Массив структур изменяемых полей TEMP
tRxFieldLst irTempField[15][6];

// Массив указателей на структуры изменяемых полей
tRxFieldLst * pIrField[5] = { &irOnOffField, irTempField[0], NULL, NULL, NULL };

// Индекс следующего поля пакета
uint8_t irRxIndex;
eRxStat rxStat;
uint8_t fieldCount;

uint8_t rxEdgeCnt;

const uint8_t paramValCountMax[5] = { ONOFF_VAL_COUNT_MAX, TEMP_VAL_COUNT_MAX, MODE_VAL_COUNT_MAX,
                                      FAN_VAL_COUNT_MAX, SWING_VAL_COUNT_MAX };
uint8_t paramValCount;


int8_t rxPktCmp( eRxStat rxSt, uint8_t paramCnt );

// Инициализация таймера несущей ИК-передатчика TIM21
void irCarierTimInit( void ){
  // Инициируем вывод ИК
  RCC->IOPENR |= (RCC_IOPENR_GPIOAEN << IR_TX_PORT_NUM);

  IR_TX_PORT->OTYPER &= ~(IR_TX_PIN);
  IR_TX_PORT->OSPEEDR = (IR_TX_PORT->OSPEEDR & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));
  IR_TX_PORT->PUPDR = (IR_TX_PORT->PUPDR & ~(0x3 << (IR_TX_PIN_NUM * 2)));
  IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));

  // Включакм тактирование таймера
   RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
   // Получаем частоту счета 38000Гц * 2: (55-1)
   TIM21->PSC = 54;
   // Перезагрузка каждый 2-й счет: (2-1)
   TIM21->ARR = 1;
   // Прерывание по переполнению
   TIM21->DIER |= TIM_DIER_UIE;
   // Конфигурация NVIC для прерывания по таймеру TIM6
   NVIC_EnableIRQ( TIM21_IRQn );
   NVIC_SetPriority( TIM21_IRQn, 2 );

}

// Инициализация таймера модуляции ИК-сигнала TIM22
// Таймер подсчитывает длительности имульсов и пауз взодящего/исходящего сигнала в ед. 10мкс
// При приеме (обучении) счетаем в прямом направлении, при передаче - обратный отсчет.
void irModulTimInit( void ){

  // Включакм тактирование таймера
   RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
   // TODO: Настроить отключение таймера в режиме STOP и востановление его по потребности

   // Получаем период счета 10 мкс ( 4194кГц / (41 + 1) ) = ~10.01мкс :
   TIM22->PSC = 41;
   // Перезагрузка по истечение 100мс
   TIM22->ARR = 10000;
   // Прерывание по переполнению
   TIM22->DIER |= TIM_DIER_UIE;
   // Конфигурация NVIC для прерывания по таймеру TIM6
   NVIC_EnableIRQ( TIM22_IRQn );
   NVIC_SetPriority( TIM22_IRQn, 1 );
}

void irRxInit( void ){
  // Инициализация пина

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  //---- Инициализация входа для: Вход, 2МГц, подтяжка ВВЕРХ ---
  IR_RX_PORT->OTYPER &= ~(IR_RX_PIN);
  IR_RX_PORT->OSPEEDR = (GPIOA->OSPEEDR & ~(0x3 << (IR_RX_PIN_NUM * 2))) | (0x1 << (IR_RX_PIN_NUM * 2));
  IR_RX_PORT->PUPDR = (GPIOA->PUPDR & ~(0x3 << (IR_RX_PIN_NUM * 2)));
  IR_RX_PORT->MODER &=  ~(0x3<< (IR_RX_PIN_NUM * 2));

  // Инициализация прерывания от КНОПКИ
  // Select Dio0-Port for Dio0-Pin extended interrupt by writing 0000 in EXTI0
  SYSCFG->EXTICR[IR_RX_PIN_NUM / 4] &= (uint16_t)~(0xF << ((IR_RX_PIN_NUM % 4) * 4));
  SYSCFG->EXTICR[IR_RX_PIN_NUM / 4] |= (uint16_t)( IR_RX_PORT_NUM << ((IR_RX_PIN_NUM % 4) * 4) );

  // Конфигурируем прерывание по падающему фронту (Первый фронт от ИК-приемника)
  EXTI->FTSR |= IR_RX_PIN;
  // ----------- Configure NVIC for Extended Interrupt --------
  NVIC_SetPriority( IR_RX_EXTI_IRQn, 1 );

  // Инициализация таймера Модулирующего сигнала
  irModulTimInit();

  rxStat = RX_STAT_0;
  pIrPkt = ir0Pkt;

  // =========== Для тестирования =====================
  // PA12 на вывод данных
  //---- Инициализация выхода для: Выход, 2МГц, без подтяжки ---
  GPIOA->OTYPER &= ~(GPIO_Pin_12);
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(0x3 << (12 * 2))) | (0x1 << (12 * 2));
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(0x3 << (12 * 2)));
  GPIOA->MODER = (GPIOA->MODER & ~(0x3<< (12 * 2))) | (0x1 << (12 * 2));
  GPIOA->BSRR |= GPIO_Pin_12;

}

void irRxProcess( void ){

  rxEdgeCnt++;

  if(IR_RX_PORT->IDR & IR_RX_PIN) {
    GPIOA->BSRR |= GPIO_Pin_12;
  }
  if( rxEdgeCnt > 2){
//  else {
    GPIOA->BRR |= GPIO_Pin_12;
  }

  if( (irRxIndex == 0) && ((IR_RX_PORT->IDR & IR_RX_PIN) == 0) ){
    // Начало пакета - запускаем счет таймера модуляции
    TIM22->CR1 |= TIM_CR1_CEN;
    // Добавляем еще и прерывание от растущего фронт от ИК-приемника
    EXTI->RTSR |= IR_RX_PIN;
  }
  else {
    // Сохраняем длительность очередного импульса/паузы в массив 0-го пакета
    *(pIrPkt + irRxIndex++) = TIM22->CNT;
  }
  // Сбрасываем счетчик
  TIM22->CNT = 0;

}

void learnProcess( void ){
  static uint8_t onOffFlag = OFF;

  switch( rxStat ){
    case RX_STAT_0:
      // Приняли начальный НАЧАЛЬНЫЙ пакет
      pIrPkt = irPkt;
      break;
    case RX_STAT_ONOFF:
      if( onOffFlag == OFF ){
        // Принят пакет ВЛЮЧЕНО - проверяем на эдентичность НАЧАЛЬНОМУ
        rxPktCmp( rxStat, 0 );
      }

      break;
    default:
      break;
  }

  rxStat = btn.pressCnt;


}

int8_t rxPktCmp( eRxStat rxSt, uint8_t paramCnt ){
  uint8_t diffCnt = 0;
  uint8_t j = 0;
  uint8_t * k;

  tRxFieldLst *fieldList = pIrField[rxSt] + paramCnt;
  // Если счетчик номера параметра превышен - различия в принятом пакете относительно
  // НАЧАЛЬНОГО считать, но не сохранять.
  // Например: Если уже сохранено для 30 гр.Ц (максимальный номер параметра) - различия считаем,
  // но не сохраняем
  uint8_t prmMaxFlag = paramCnt < paramValCountMax[rxSt];

  for( uint8_t i = 0; i < irRxIndex; i++ ){
    if( ir0Pkt[i] != irPkt[i] ){
      diffCnt++;
      if( prmMaxFlag ){
        // Ищем незаполненную структуру
        for( (k = &((fieldList + j)->fieldNum) ); *k != 0; j++ )
        {}
        if( j > 5 ){
          // Счетчик различающихся полей превысил допустимый - выходим
          diffCnt = -1;
          break;
        }
        *k = i;
        fieldList->fieldDur = irPkt[i];
      }
    }
  }

  return diffCnt;
}
