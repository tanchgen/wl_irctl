/*
 * ir.c
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "string.h"

#include "stm32l0xx.h"
#include "gpio.h"
#include "button.h"
#include "proto.h"
#include "ir.h"

// Массив НАЧАЛЬНОГО пакета неизвестного протокола
uint16_t ir0Pkt[255];

// Массив НЕ НАЧАЛЬНОГО пакета (приятый или отправляемый пакет)
uint16_t irPkt[255];

// Указатель на пакет: НАЧАЛЬНЫЙ, НЕ начальный, отправляемый
uint16_t *pIrPkt;

// Структура изменяемых полей
tRxFieldLst irDiffField[4+120+30+30+30];

// Структура изменяемых полей
tRxFieldLst * pIrDiffField = irDiffField;

// Массив указателей на структуры изменяемых полей
tRxFieldLst * pIrField[ONOFF_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + \
                       FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX];
// Массив количества изменных полей для каждого параметра
uint8_t rxFieldQuant[ONOFF_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + \
                       FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX];

int8_t onOffFlag = -1;

// Признак получения ИК-пакета
uint8_t irRxGetFlag = RESET;
// Индекс следующего поля пакета
uint8_t irRxIndex = 0;
// Состояние (текущий параметр) обучения
eRxStat rxStat;
uint8_t fieldCount;

uint8_t rxEdgeCnt;
uint8_t headerFlag = FALSE;
uint8_t headerFieldCnt = 0;

const uint8_t paramValCountMax[5] = { ONOFF_VAL_COUNT_MAX, TEMP_VAL_COUNT_MAX, MODE_VAL_COUNT_MAX,
                                      FAN_VAL_COUNT_MAX, SWING_VAL_COUNT_MAX };
/* Начальная позиция полей различий для каждого параметра:
 * ON/OFF
 * TEMPERATURE
 * MODE
 * FAN
 * SWING
 */
const uint8_t paramfieldBegin[5] =
{
  0,
  ONOFF_VAL_COUNT_MAX,
  ONOFF_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX,
  ONOFF_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX,
  ONOFF_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + FAN_VAL_COUNT_MAX
};

uint8_t paramValCount = 0;


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

   // Получаем период счета, кратный 38000Гц (несущая частота) ( 4194кГц / 38кГц ) = ~110 :
   TIM22->PSC = (110)-1;
   // Перезагрузка по истечение 100мс
   TIM22->ARR = 10000;
   TIM22->CNT = 9999;
   TIM22->CR1 |= TIM_CR1_CEN;
   // Ждем, пока обновится счетчик
   while( (TIM22->SR & TIM_SR_UIF) == 0 )
   {}
   TIM22->CR1 &= ~TIM_CR1_CEN;
   TIM22->CNT = 0;
   TIM22->SR &= ~TIM_SR_UIF;
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
  IR_RX_PORT->PUPDR = (GPIOA->PUPDR & ~(0x3 << (IR_RX_PIN_NUM * 2))); // | (0x1 << (IR_RX_PIN_NUM * 2));
  IR_RX_PORT->MODER &=  ~(0x3<< (IR_RX_PIN_NUM * 2));

  // Инициализация прерывания от КНОПКИ
  // Select Dio0-Port for Dio0-Pin extended interrupt by writing 0000 in EXTI0
  SYSCFG->EXTICR[IR_RX_PIN_NUM / 4] &= (uint16_t)~(0xF << ((IR_RX_PIN_NUM % 4) * 4));
  SYSCFG->EXTICR[IR_RX_PIN_NUM / 4] |= (uint16_t)( IR_RX_PORT_NUM << ((IR_RX_PIN_NUM % 4) * 4) );

  // Конфигурируем прерывание по падающему фронту (Первый фронт от ИК-приемника)
  EXTI->FTSR |= IR_RX_PIN;
  // Добавляем еще и прерывание от растущего фронт от ИК-приемника
//  EXTI->RTSR |= IR_RX_PIN;
  // ----------- Configure NVIC for Extended Interrupt --------
  NVIC_EnableIRQ( IR_RX_EXTI_IRQn );
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

  uint16_t c = TIM22->CNT;

  rxEdgeCnt++;

  GPIOA->ODR ^= GPIO_Pin_12;

//  if( (irRxIndex != 0) || ((IR_RX_PORT->IDR & IR_RX_PIN) == 0) ){
    // Начало пакета - запускаем счет таймера модуляции
//    TIM22->CR1 |= TIM_CR1_CEN;
//    // Добавляем еще и прерывание от растущего фронт от ИК-приемника
//    EXTI->RTSR |= IR_RX_PIN;
  if( c > 9 ){
    // Сохраняем длительность очередного импульса/паузы в массив 0-го пакета
    *(pIrPkt + irRxIndex++) = c;
  }
  else if( c == 0){
    TIM22->CR1 |= TIM_CR1_CEN;
    EXTI->RTSR |= IR_RX_PIN;
  }
  // Сбрасываем счетчик
  TIM22->CNT = 0x0;

}

uint8_t learnProcess( void ){
  uint8_t prName = PROTO_NONAME;

  if( (rxStat != RX_STAT_0) && ( onOffFlag == ON ) ){
    if( rxStat != btn.pressCnt){
      // Получили пакет с новым параметром (Например: был - температура, стал - вентилятор)
      rxStat = btn.pressCnt;
      // Обнуляем счетчик значения параметра (Температура - 16гр.Ц, вентилятор - мин., и т.д.
      paramValCount = 0;
    }
  }

  switch( rxStat ){
    uint8_t rec = 0;

    case RX_STAT_0:
      // Приняли начальный НАЧАЛЬНЫЙ пакет
      prName = protoDecod( ir0Pkt, irRxIndex );
      pIrPkt = irPkt;
      buzzerShortPulse();
      rxStat = RX_STAT_ONOFF;
      break;
    case RX_STAT_ONOFF:
      switch( onOffFlag ){
        case -1:
          // Сравниваем НУЛЕВОЙ пакет с OFF-пакетом
          if( (rec = rxPktCmp( rxStat, 0 )) == 0 ){
            // Различий не обнаружено - так быть не должно
            // TODO: Обработка ошибки обучения (может длинный зуммер?)
          }
          else if ( rec > 0 ){
            buzzerShortPulse();
            onOffFlag = OFF;
          }
          break;
        case OFF:
          // Сравниваем НУЛЕВОЙ пакет с ON-пакетом
          if( (rec = rxPktCmp( rxStat, 0 )) > 0 ){
            // Различий не обнаружено - так быть не должно
            learnReset();
          }
          else if( rec == 0 ){
            buzzerShortPulse();
            onOffFlag = ON;
          }
          else {
            // Ошибка приема пакета
            learnReset();
          }
          break;
        default:
          break;

      }
      break;
    case RX_STAT_TEMP:
    case RX_STAT_MODE:
    case RX_STAT_FAN:
    case RX_STAT_SWING:
      if( paramValCount < paramValCountMax[rxStat]){
        if( rxPktCmp( rxStat, paramValCount ) < 0){
          // Ошибка приема пакета
          learnReset();
        }
        else {
          // TODO: Проверяем на конец обучения
          buzzerShortPulse();
          // Порядковый номер величины параметра
          paramValCount++;
        }
      }
      break;
    default:
      break;
  }


  return prName;
}

int8_t rxPktCmp( eRxStat rxSt, uint8_t paramCnt ){
  uint8_t diffCnt = 0;
  uint8_t sellNum = paramfieldBegin[rxSt] + paramCnt;

  tRxFieldLst *fieldList;
  // Если счетчик номера параметра превышен - различия в принятом пакете относительно
  // НАЧАЛЬНОГО считать, но не сохранять.
  // Например: Если уже сохранено для 30 гр.Ц (максимальный номер параметра) - различия считаем,
  // но не сохраняем
  uint8_t oldFlag = (pIrField[ sellNum ] == NULL)? RESET: SET;


  if( pIrField[ sellNum ] == NULL ){
    // Сохраняем указатель на изменяемые поля для текущего номера параметров
    pIrField[ sellNum ] = pIrDiffField;
  }
  fieldList = pIrField[ sellNum ];

  for( uint8_t i = 0; i < irRxIndex; i++ ){
    if( irDurCmp( ir0Pkt[i], irPkt[i], 20) ){
      // Несовпадение поля
      diffCnt++;
      // Проверка:
      // Проверка: Измененные поля не дложны затереть
      // ранеее записанные измененные поля других параметров и других значений параметров
      if( oldFlag && (diffCnt > rxFieldQuant[ sellNum ]) && (fieldList != NULL) ){
        // Счетчик ошибок превысил допустимое:  если не прервем - затрем другие записи
        diffCnt = -1;
        goto exit;
      }
      fieldList->fieldNum = i;
      fieldList++->fieldDur = irPkt[i];
      if( fieldList > (irDiffField + sizeof(irDiffField)) ) {
        // Указатель на различающиеся поля превысил допустимый - выходим
        diffCnt = -1;
        goto exit;
      }
    }
  }


  // Указатель для изменяемых полей для следующего номера параметров
  for( ; fieldList->fieldNum != 0; fieldList++ ){
    if( fieldList > (irDiffField + sizeof(irDiffField)) ){
      // Указатель на различающиеся поля превысил допустимый - выходим
      diffCnt = -1;
      goto exit;
    }
  }
  pIrDiffField = fieldList;
  if(onOffFlag != OFF){
    if( diffCnt < 14){
      rxFieldQuant[ sellNum ] = diffCnt;
    }
    else {
      // Слишком большое количество изменений - ошибка пакета
      diffCnt = -1;
      goto exit;
    }

  }
exit:
  return diffCnt;
}

void learnReset( void ){
  uint32_t * p32;

  if( onOffFlag == ON ){
    // Обучение
    rxStat = RX_STAT_0;
    onOffFlag = -1;

    // Очищаем  сохраненные изменяемые поля
    p32 = (uint32_t *)(pIrField[0]);
    for(uint8_t i = 0; i < rxFieldQuant[0]; i++){
      *(p32+i) = 0;
    }
    pIrField[0] = NULL;
    rxFieldQuant[0] = 0;
    pIrDiffField = irDiffField;
  }
  else {
    // Очищаем  сохраненные изменяемые поля
    p32 = (uint32_t *)(pIrField[0]+rxFieldQuant[0]);
    for(uint8_t i = 0; i < (sizeof(irDiffField)/4 - rxFieldQuant[0]); i++){
      *(p32+i) = 0;
    }
    for(uint8_t i = 1; i < (sizeof(pIrField)/4 - 1); i++){
      pIrField[i] = NULL;
    }
    for(uint8_t i = 1; i < (sizeof(rxFieldQuant) - 1); i++){
      rxFieldQuant[i] = 0;
    }
    pIrDiffField = irDiffField + rxFieldQuant[0];
  }

  // обнуляем счетчик нажатий
  btn.pressCnt = 0;
  paramValCount = 0;
  buzzerLongPulse();
}

