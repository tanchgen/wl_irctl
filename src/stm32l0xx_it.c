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
  if( RTC->ISR & RTC_ISR_ALRAF ){

    uint32_t dr = RTC->DR;
    (void)dr;
    uint32_t tr0 = RTC->TR;
    uint32_t tr = RTC->TR;
    if (tr0 != tr ){
      tr = RTC->TR;
    }

    //Clear ALRAF
    RTC->ISR &= ~RTC_ISR_ALRAF;
//    if( (tr & 0x1) != 0){
//      if((rtc.min % SEND_TOUT) != 0) {
//        sendToutFlag = SET;
//      }
//      // Alarm A interrupt: Каждая вторая секунда
//      uxTime = getRtcTime();
//      if(state == STAT_READY){
//        if( sendToutFlag == SET ){
//          // Периодическое измерение - измеряем все
//          mesureStart();
//        }
//        else {
//          // Измеряем только освещенность
//          lightStart();
//        }
//      }
//    }
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
//#if ! STOP_EN
//  rtcLog[rtcLogCount].ssr = RTC->SSR;
//  rtcLog[rtcLogCount++].state = state;
//  rtcLogCount &= 0x3F;
//#endif
	// Восстанавливаем настройки портов
  restoreContext();

  // Стираем флаг прерывания EXTI
  EXTI->PR &= DIO0_PIN;
  if( rfm.mode == MODE_RX ){
    // Если что-то и приняли, то случайно
    // Опустошаем FIFO
    while( dioRead(DIO_RX_FIFONE) == SET ){
      rfmRegRead( REG_FIFO );
    }
  }
  else if( rfm.mode == MODE_TX ) {
    // Отправили пакет с температурой
  	wutStop();
  	txEnd();
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
  TIM21->SR &= ~TIM_SR_UIF;
  // Переключаем вывод 1->0, 0->1
  IR_TX_PORT->ODR ^= IR_TX_PIN;
}

// Обработчик прерывания таймера модуляции (длительность импульсов и пауз) ИК-передатчика
void TIM22_IRQHandler( void ){
  TIM22->SR &= ~TIM_SR_UIF;
  TIM22->CR1 &= ~TIM_CR1_CEN;
  if( TIM22->CR1 & TIM_CR1_DIR ){
    // Обратный отсчет - передача
  }
  else {
    // Примой счет - прием (обучение)
    // Длительность паузы более 100мс -> пакет принят.
//    buzzerShortPulse();
    // Обнуляем счетчик импульсов и пауз
    GPIOA->ODR ^= GPIO_Pin_12;
    if( protoName == PROTO_NONAME ){
      protoName = learnProcess();
    }
    else {
      buzzerShortPulse();
      mDelay(125);
      buzzerShortPulse();
    }
    irRxGetFlag = SET;
    irRxIndex = 0;
    // Включаем маску внешнего прерывания от ИК-приемника
    EXTI->IMR &= ~IR_RX_PIN;
    // Выждем паузу 250мс до ожидания следующего ИК-пакета
    state = STAT_IR_RX_STOP;
    wutSet(250000);
  }
}
