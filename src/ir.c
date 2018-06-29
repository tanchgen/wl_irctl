/*
 * ir.c
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "string.h"

#include "stm32l0xx.h"
#include "my_time.h"
#include "gpio.h"
#include "button.h"
#include "proto.h"
#include "ir.h"

// Массив НАЧАЛЬНОГО пакета неизвестного протокола
uint16_t __aligned(4) ir0Pkt[256];

// Массив НЕ НАЧАЛЬНОГО пакета (приятый или отправляемый пакет)
uint16_t __aligned(4) irPkt[256];

// Указатель на пакет: НАЧАЛЬНЫЙ, НЕ начальный, отправляемый
uint16_t *pIrPkt;

// Структура изменяемых полей
tRxFieldLst irDiffField[4+120+30+30+30];

// Структура изменяемых полей
tRxFieldLst * pIrDiffField = irDiffField;

// Массив указателей на структуры изменяемых полей
tRxFieldLst * pIrField[ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + \
                       FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX];
// Массив количества изменных полей для каждого параметра
uint8_t rxFieldQuant[ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + \
                       FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX];

int8_t onOffFlag = -1;

// Признак получения ИК-пакета
uint8_t irRxGetFlag = RESET;
// Индекс следующего поля пакета
uint16_t irRxIndex = 0;
// Состояние (текущий параметр) обучения
eRxStat rxStat;
uint8_t field0Num;                  // Соличество полей в НАЧАЛЬНОМ пакете
uint8_t txFieldCount;               // Счетчик полей, передаваемых по ИК-каналу

uint16_t rxEdgeCnt;
uint8_t headerFlag = FALSE;
uint8_t headerFieldCnt = 0;

const uint8_t paramValCountMax[5] = { ONOFF_VAL_COUNT_MAX, MODE_VAL_COUNT_MAX, TEMP_VAL_COUNT_MAX,
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
  ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX,
  ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX,
  ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + FAN_VAL_COUNT_MAX
};

uint8_t paramValCount = 0;


int8_t rxPktCmp( eRxStat rxSt, uint8_t paramCnt );

// Инициализация таймера несущей ИК-передатчика TIM21
void irCarierTimInit( void ){
  // Включакм тактирование таймера
   RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
   TIM21->PSC = 0;
   // Получаем частоту счета 38000Гц: (110-1)
   TIM21->ARR = 109;
   // Скважность ~ 1/3
   TIM21->CCR1 = (110-36) - 1;
   // Режим PWM1, Активный - низкий уровень
//   TIM21->CCMR1 = (TIM21->CCMR1 & ~(TIM_CCMR1_OC1M)) | (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE;
   TIM21->CCMR1 = (TIM21->CCMR1 & ~(TIM_CCMR1_OC1M)) | (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0) | TIM_CCMR1_OC1PE;
   TIM21->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;
   // Внешняя синхронизация: режим "Gated", ITR1 (TIM22)
   TIM21->SMCR |= (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0) | (TIM_SMCR_TS_0);

//   TIM21->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
   // Конфигурация NVIC для прерывания по таймеру TIM21
//   NVIC_EnableIRQ( TIM21_IRQn );
//   NVIC_SetPriority( TIM21_IRQn, 2 );

   // Выключакм тактирование таймера до востребования
//    RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;
}

// Инициализация таймера модуляции ИК-сигнала TIM2 для обучения
// Таймер подсчитывает длительности имульсов и пауз взодящего/исходящего сигнала в ед. 10мкс
// При приеме (обучении) счетаем в прямом направлении, при передаче - обратный отсчет.
void irModulLearnTimInit( void ){

  // Включакм тактирование таймера
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   // TODO: Настроить отключение таймера в режиме STOP и востановление его по потребности

   // Получаем период счета, кратный 38000Гц (несущая частота) ( 4194кГц / 38кГц ) = ~110 :
   TIM2->PSC = (110)-1;
   // Перезагрузка по истечение 100мс
   TIM2->ARR = 1480;
//   TIM2->CR1 |= TIM_CR1_CEN;
   TIM2->EGR = TIM_EGR_UG;

   // Ждем, пока обновится счетчик
   while( (TIM2->SR & TIM_SR_UIF) == 0 )
   {}
//   TIM2->CR1 &= ~TIM_CR1_CEN;
//   TIM2->CNT = 0;
   TIM2->SR &= ~TIM_SR_UIF;
   // Прерывание по переполнению
   TIM2->DIER |= TIM_DIER_UIE;
   // Конфигурация NVIC для прерывания по таймеру TIM6
   NVIC_EnableIRQ( TIM2_IRQn );
   NVIC_SetPriority( TIM2_IRQn, 1 );

}

// Инициализация таймера модуляции ИК-сигнала TIM22 для передачи
// Таймер подсчитывает длительности имульсов и пауз взодящего/исходящего сигнала в ед. 10мкс
// При приеме (обучении) счетаем в прямом направлении, при передаче - обратный отсчет.
void irModulTxTimInit( void ){

  // Включакм тактирование таймера
   RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
   // TODO: Настроить отключение таймера в режиме STOP и востановление его по потребности

   // Получаем период счета, кратный 38000Гц (несущая частота) ( 4194кГц / 38кГц ) = ~110 :
   TIM22->PSC = (110)-1;

   // OC1REF - как TGRO
   TIM22->CR2 |= TIM_CR2_MMS_2;
   // PWM2
   TIM22->CCMR1 = (TIM22->CCMR1 & ~(TIM_CCMR1_OC1M)) | (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE;
//    TIM22->CCMR1 = (TIM22->CCMR1 & ~(TIM_CCMR1_OC1M)) | (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0) | TIM_CCMR1_OC1PE;
   TIM22->CR1 |= TIM_CR1_ARPE;

//   TIM22->EGR = TIM_EGR_UG;
//   // Ждем, пока обновится счетчик
//   while( (TIM22->SR & TIM_SR_UIF) == 0 )
//   {}
//   TIM22->SR &= ~TIM_SR_UIF;
//
   //  // Прерывание по переполнению
   TIM22->DIER |= TIM_DIER_CC1IE | TIM_DIER_UIE;
   //  // Конфигурация NVIC для прерывания по таймеру TIM22
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

  // Инициализация таймера Модулирующего сигнала для ОБУЧЕНИЯ
  irModulLearnTimInit();

  if( field0Num == 0 ){
    // Обучение еще не проходило (в EEPROM не было записано) ->  начнем обучение с начала
    rxStat = RX_STAT_0;
    pIrPkt = ir0Pkt;
  }

// =========== Для тестирования =====================
// PA11 на вывод данных
//---- Инициализация выхода для: Выход, 2МГц, без подтяжки ---
  GPIOA->OTYPER &= ~(GPIO_Pin_11);
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(0x3 << (11 * 2))) | (0x1 << (11 * 2));
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(0x3 << (11 * 2)));
  GPIOA->MODER = (GPIOA->MODER & ~(0x3<< (11 * 2))) | (0x1 << (11 * 2));
  GPIOA->BSRR |= GPIO_Pin_11;

}

void irRxProcess( void ){
  uint16_t c = TIM2->CNT;

  rxEdgeCnt++;

//  GPIOA->ODR ^= GPIO_Pin_11;

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
#if STOP_EN
    // Выключаем засыпание по ВЫХОДУ ИЗ ПРЕРЫВАНИЯ до окончания приема ИК-пакета
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
#endif

    TIM2->CR1 |= TIM_CR1_CEN;
    EXTI->RTSR |= IR_RX_PIN;
  }
  // Сбрасываем счетчик
  TIM2->CNT = 0x0;
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
            field0Num = irRxIndex;
            // Сохраняем в EEPROM
            memcpy( eeIrProtoBak.fld0Pkt, ir0Pkt, field0Num * 2 );
            eeIrProtoBak.fld0Num = field0Num;
            eeIrProtoBak.protoName = protoName;
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
        if( field0Num != irRxIndex ){
          // Ошибка - Длина принятого пакета отличается от длины НАЧАЛЬНОГО пакета
          learnReset();
        }
        else if( rxPktCmp( rxStat, paramValCount ) < 0){
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
  tRxFieldLst *eeFldList;

  // Если счетчик номера параметра превышен - различия в принятом пакете относительно
  // НАЧАЛЬНОГО считать, но не сохранять.
  // Например: Если уже сохранено для 30 гр.Ц (максимальный номер параметра) - различия считаем,
  // но не сохраняем
  uint8_t oldFlag = (pIrField[ sellNum ] == NULL)? RESET: SET;


  if( pIrField[ sellNum ] == NULL ){
    // Сохраняем указатель на изменяемые поля для текущего номера параметров
    pIrField[ sellNum ] = pIrDiffField;
    // Тоже делаем в BackUp EEPROM
    eeIrProtoBak.pfldDiff[ sellNum ] = pIrDiffField;
  }
  fieldList = pIrField[ sellNum ];

  // Указатель на backup-запись в EEPROM
  eeFldList = eeIrProtoBak.diffField + (fieldList - irDiffField);

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

      // Тоже делаем в BackUp EEPROM
      eeFldList->fieldNum = i;
      eeFldList++->fieldDur = irPkt[i];

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
      // Тоже делаем в BackUp EEPROM
      eeIrProtoBak.fldDiffQuant[ sellNum ] = diffCnt;
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
  uint32_t * eep32;

  if( (onOffFlag != ON) || (btn.longPressCnt > 1) ){
    // Обучение сначала
	field0Num = 0;
    rxStat = RX_STAT_0;
    onOffFlag = -1;
    pIrPkt = ir0Pkt;
    protoName = PROTO_NONAME;

    btn.longPressCnt = 0;

    // Очищаем  сохраненные изменяемые поля
    p32 = (uint32_t *)irDiffField;
    eep32 = (uint32_t *)eeIrProtoBak.diffField;
    for(uint8_t i = 0; i < rxFieldQuant[0]; i++){
      *(p32+i) = 0;
      *(eep32+i) = 0;
    }
    pIrField[0] = NULL;
    rxFieldQuant[0] = 0;
    pIrDiffField = irDiffField;
    // Стираем запись в EEPROM
    eeIrProtoBak.fld0Num = 0;

    // Будет звучать ДВА долгих зуммера
    buzzerLongPulse();
    mDelay(125);
  }
  else {
    // Очищаем  сохраненные изменяемые поля - все, кроме ON/OFF
    p32 = (uint32_t *)(irDiffField + rxFieldQuant[0]);
    eep32 = (uint32_t *)(eeIrProtoBak.diffField + rxFieldQuant[0]);
    for(uint8_t i = 0; i < (sizeof(irDiffField)/4 - rxFieldQuant[0]); i++){
      *(p32+i) = 0;
      *(eep32+i) = 0;
    }

    for(uint8_t i = 1; i < sizeof(rxFieldQuant); i++){
      // Очищаем список указателей на измененные поля
      pIrField[i] = NULL;
      eeIrProtoBak.pfldDiff[i] = NULL;
      // Очищаем список длин последовательностей измененных полей
      rxFieldQuant[i] = 0;
      eeIrProtoBak.fldDiffQuant[i] = 0;
    }
    pIrDiffField = irDiffField + rxFieldQuant[0];
  }
  // Очищаем  сохраненные изменяемые поля

  // обнуляем счетчик нажатий
  btn.pressCnt = 0;
  paramValCount = 0;
  buzzerLongPulse();
}

uint8_t irPktSend( void ){
  uint8_t rec = 0;

  if(field0Num == 0){
    // Еще не прошел обучение - неизвестно, что отправлять
    rec = 1;
    goto exitLabel;
  }
  if( (btn.stat != BTN_OFF) || (state == STAT_BTN_DBNC) ){
    // Кнопка нажата: возможно, идет процесс обучения - параметры протокола могут изменится
    rec = 1;
    goto exitLabel;
  }

  // Выключаем прерывание от КНОПКИ
  EXTI->IMR &= ~(BTN_PIN);

  txFieldCount = 0;

  // Включаем тактирование таймера несущей и запускаем его
  RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
  TIM21->CR1 |= TIM_CR1_CEN;

  // Включаем тактирование таймера модулирующей
  RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;

  // Заполняем длительности полей рульса и паузы
  uint16_t tmp;
  // Заполняем поле пульса
  TIM22->CCR1 = irPkt[txFieldCount++];
  // Заполняем поле паузы (Пауза = ARR - "пульс")
  tmp = TIM22->CCR1;
  if( txFieldCount < field0Num){
   tmp += irPkt[txFieldCount++];
  }
  TIM22->ARR = tmp;

  // Включаем вывод таймера на пин порта
  IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x2 << (IR_TX_PIN_NUM * 2));

#if STOP_EN
  // Выключаем засыпание по Выходу из прерывания до окончания передачи ИК-пакета
  SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
#endif

  // Запускаем таймер модулирующей
  TIM22->CR1 |= TIM_CR1_CEN;
  TIM22->EGR = TIM_EGR_UG;

  // Включаем прерывание от КНОПКИ
  EXTI->IMR |= BTN_PIN;

exitLabel:
  return rec;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
//uint8_t irPktSend( void ){
//
//  if(field0Num == 0){
//    // Последовательность полей длительностей НАЧАЛЬНОГО пакета не заполнена - выходим
//    return 1;
//  }
//
//  // Включаем тактирование таймера несущей
//   RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
//  // Запускем таймер несущей
//  TIM21->CR1 |= TIM_CR1_CEN;
//  TIM21->EGR |= TIM_EGR_UG;
//
//  // Выключаем засыпание по выходу из прерывания до окончания отправки пакета по ИК-каналу
//  uint32_t tmp = SCB->SCR;
//  // Сохраняем засыпание по выходу из прерывания
//  SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
//
//
//  // Включакм тактирование таймера модулирующей
//   RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
//
//   // Длительности: начнем последовательность с первого поля
//   txFieldCount = 0;
//
//  // -------------- НАЧИНАЕМ ПЕРЕДАЧУ --------------------------
//  // Переключаем пин на Таймер модулирующей
//  IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x2 << (IR_TX_PIN_NUM * 2));
//
//  // Заполняем поле пульса
////  TIM22->CCR1 = 0x159;
////  TIM22->ARR = 0xAA + 0x159;
////  txFieldCount = 0;
////  TIM22->CCR1 = irPkt[txFieldCount++];
////  // Заполняем поле паузы (Пауза = ARR - "пульс")
////  tmp = TIM22->CCR1;
////  if( txFieldCount < field0Num){
////    tmp += irPkt[txFieldCount++];
////  }
////  TIM22->ARR = tmp;
//  // Запускем таймер модулирующей
//  TIM22->CR1 |= TIM_CR1_CEN;
////  TIM22->SR &= ~(TIM_SR_UIF | TIM_SR_CC1IF);
//  GPIOA->ODR ^= GPIO_Pin_11;
//  TIM22->EGR = TIM_EGR_UG;
//
//  // Ждем, пока передача не закончится
//  while( TIM22->CR1 & TIM_CR1_CEN ){
//// -----------------------------------------------------------
////    if( TIM22->SR & TIM_SR_UIF){
////      GPIOA->ODR ^= GPIO_Pin_11;
////      txFieldCount += 2;
////      if( txFieldCount >= field0Num ){
////        // Передача закончена - все выключаем
////
////        TIM22->CR1 &= ~TIM_CR1_CEN;
////        TIM22->ARR = ~0;
////      }
////      TIM22->SR &= ~TIM_SR_UIF;
////    }
////    else if( TIM22->SR & TIM_SR_CC1IF){
////      GPIOA->ODR ^= GPIO_Pin_11;
////      // Пульс закончился
////      if( txFieldCount >= field0Num ){
////        // Передача закончена - все выключаем
////        TIM22->CR1 &= ~TIM_CR1_CEN;
////        IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));
////        TIM22->CCR1 = 0;
////        TIM22->ARR = ~0;
////      }
////      TIM22->SR &= ~TIM_SR_CC1IF;
////    }
//
////-------------------------------------------------------
//
////    if( TIM22->SR & TIM_SR_UIF){
////      uint16_t tmp;
////
////      GPIOA->ODR ^= GPIO_Pin_11;
////
////      TIM22->SR &= ~TIM_SR_UIF;
////      // Заполняем поле пульса
////      TIM22->CCR1 = irPkt[txFieldCount++];
////      // Заполняем поле паузы (Пауза = ARR - "пульс")
////      tmp = TIM22->CCR1;
////      if( txFieldCount < field0Num){
////        tmp += irPkt[txFieldCount++];
////      }
////      TIM22->ARR = tmp;
////
////    }
////    else if( TIM22->SR & TIM_SR_CC1IF){
////
////      GPIOA->ODR ^= GPIO_Pin_11;
////
////      // Пульс закончился
////      if( txFieldCount >= field0Num ){
////        // Передача закончена - все выключаем
////        TIM22->CR1 &= ~TIM_CR1_CEN;
////        IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));
////        TIM22->CCR1 = 0;
////        TIM22->ARR = ~0;
////      }
////      TIM22->SR &= ~TIM_SR_CC1IF;
////    }
//  }
//
//  // Выключаем тактирование таймера несущей
//  RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;
//  // Выключаем тактирование таймера модулирующей до следующего использования
//  RCC->APB2ENR &= ~RCC_APB2ENR_TIM22EN;
//
//  // Восстанавливаем засыпание по выходу из прерывания
//  SCB->SCR = tmp;
//
//  return 0;
//}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
