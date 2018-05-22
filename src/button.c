/*
 * button.c
 *
 *  Created on: 27 апр. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"
#include "gpio.h"
#include "my_time.h"
#include "ir.h"
#include "button.h"

tButton btn;

// Переключение вывода на выключение питания пищалки
// и включения входа прерывания от кнопки
static void buzzerPowerOff( void );
// Переключение вывода на включение питания пищалки
static void buzzerPowerOn( void );


void buttonInit( void ){
  // Инициализация пина

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  //---- Инициализация входа для: Вход, 2МГц, подтяжка ВВЕРХ ---
  BTN_PORT->OTYPER &= ~(BTN_PIN);
  BTN_PORT->OSPEEDR = (GPIOA->OSPEEDR & ~(0x3 << (BTN_PIN_NUM * 2))) | (0x1 << (BTN_PIN_NUM * 2));
  BTN_PORT->PUPDR = (GPIOA->PUPDR & ~(0x3 << (BTN_PIN_NUM * 2)));
  BTN_PORT->MODER &=  ~(0x3<< (BTN_PIN_NUM * 2));

  // Инициализация прерывания от КНОПКИ
  // Select Dio0-Port for Dio0-Pin extended interrupt by writing 0000 in EXTI0
#if (BTN_PIN_NUM < 8)
  SYSCFG->EXTICR[BTN_PIN_NUM / 4] &= (uint16_t)~(0xF << (BTN_PIN_NUM * 4));
  SYSCFG->EXTICR[BTN_PIN_NUM / 4] |= (uint16_t)( BTN_PORT_NUM << (BTN_PIN_NUM * 4) );
#else
  SYSCFG->EXTICR[BTN_PIN_NUM / 4] &= ~(0xFL << ((BTN_PIN_NUM - 8) * 4));
  SYSCFG->EXTICR[BTN_PIN_NUM / 4] |= ( (uint32_t)BTN_PORT_NUM << ((BTN_PIN_NUM - 8) * 4) );
#endif
  // Снимаем маску с бита BTN
  EXTI->IMR |= BTN_PIN;
  // Конфигурируем прерывание по падающему фронту
  EXTI->FTSR |= BTN_PIN;
  EXTI->RTSR |= BTN_PIN;
  // ----------- Configure NVIC for Extended Interrupt --------
  NVIC_EnableIRQ( BTN_EXTI_IRQn );
  NVIC_SetPriority( BTN_EXTI_IRQn, 1 );

  btn.pressCnt = 0;
  btn.stat = 0;
  btn.tOnSec = 0;
  btn.tOnSS = 0;
  btn.tOffSec = 0;
  btn.tOffSS = 0;
}

// Обработка нажатия клавишь
void buttonProcess( uint32_t ut ){
  uint64_t tmpTime;

  if( btn.stat == BTN_ON ){
    //Кнопка нажата
    // Сохраняем время включения кнопки
    btn.tOnSec = ut;
    btn.tOnSS = rtc.ss;

    // Проверяем интервал времени с предыдущего отпускания
    if( btn.tOffSec == 0){
      tmpTime = 0;
    }
    else {
      tmpTime = ( (uint64_t)ut << 8 ) + rtc.ss;
      tmpTime -= ( ((uint64_t)btn.tOffSec << 8 ) + btn.tOffSS);
    }

//    if( tmpTime > 0x1400 ){
//      // Интервал более 20-и секунд - сброс обучения
//      learnReset();
//    }
    // Включаем обработку входа ИК-приемника
    EXTI->IMR |= IR_RX_PIN;

  }
  else {
    //Кнопка отжата
    // Сохраняем время выключения кнопки
    btn.tOffSec = ut;
    btn.tOffSS = rtc.ss;

    // Проверяем интервал времени с предыдущего нажатия
    tmpTime = ( (uint64_t)ut << 8 ) + rtc.ss;
    tmpTime -= ( ((uint64_t)btn.tOnSec << 8 ) + btn.tOnSS);

    if(irRxGetFlag == RESET){
      // ИК-пакет не принимался.
      if( tmpTime > 0x133 ){
        // КНОПКА была нажата более ~1.2 секунды - обнуляем счетчик нажатий
        learnReset();
      }
      else {
        // КНОПКА нажата более 0.125сек и менее 1.2сек - увеличиваем счетчик нажатий
        if( btn.pressCnt < PARAM_NUM_MAX){
          btn.pressCnt++;
          buzzerShortPulse();
        }

      }
    }
    irRxGetFlag = RESET;
  }
}

void buzzerInit( void ){

  // Инициируем вывод
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

  BUZ_PORT->OTYPER &= ~(BUZ_PIN);
  BUZ_PORT->OSPEEDR = (BUZ_PORT->OSPEEDR & ~(0x3 << (BUZ_PIN_NUM * 2))) | (0x1 << (BUZ_PIN_NUM * 2));
  BUZ_PORT->PUPDR = (BUZ_PORT->PUPDR & ~(0x3 << (BUZ_PIN_NUM * 2)));
  BUZ_PORT->MODER = (BUZ_PORT->MODER & ~(0x3 << (BUZ_PIN_NUM * 2))) | (0x1 << (BUZ_PIN_NUM * 2));

  // Включакм тактирование таймера
   RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
   // Получаем частоту счета 4000Гц: (1049-1)
   TIM6->PSC = 1048;
   // Перезагрузка каждый 2-й счет: (2-1)
   TIM6->ARR = 1;
//   // Прерывание по переполнению
//   TIM6->DIER |= TIM_DIER_UIE;
//   // Конфигурация NVIC для прерывания по таймеру TIM6
//   NVIC_EnableIRQ( TIM6_IRQn );
//   NVIC_SetPriority( TIM6_IRQn, 3 );

}

void buzzerShortPulse( void ){
  buzzerPowerOn();
  // Включакм тактирование таймера
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  // Запускаем пищалку
  TIM6->CR1 |= TIM_CR1_CEN;

  // Будет пищать около 70мс
  for( uint8_t i = 0; i< 144; i++ ){
    BUZ_PORT->ODR ^= BUZ_PIN;
    while( (TIM6->SR & TIM_SR_UIF) == 0 )
    {}
    TIM6->SR &= ~TIM_SR_UIF;
  }
  // Останавливаем пищалку
  TIM6->CR1 &= ~TIM_CR1_CEN;
  TIM6->SR &= ~TIM_SR_UIF;
  // Включакм тактирование таймера
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
  buzzerPowerOff();
}

void buzzerLongPulse( void ){

  buzzerPowerOn();
  // Включакм тактирование таймера
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  // Запускаем пищалку
  TIM6->CR1 |= TIM_CR1_CEN;

  // Будет пищать около 300мс
  for( uint16_t i = 0; i< 617; i++ ){
    BUZ_PORT->ODR ^= BUZ_PIN;
    while( (TIM6->SR & TIM_SR_UIF) == 0 )
    {}
    TIM6->SR &= ~TIM_SR_UIF;
  }
  // Останавливаем пищалку
  TIM6->CR1 &= ~TIM_CR1_CEN;
  TIM6->SR &= ~TIM_SR_UIF;
  // Включакм тактирование таймера
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
  buzzerPowerOff();

//  // Включакм тактирование таймера
//  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
//  // Запускаем пищалку
//  TIM6->CR1 |= TIM_CR1_CEN;
//
//  mDelay( 300 );
//  // Останавливаем пищалку
//  TIM6->CR1 &= ~TIM_CR1_CEN;
//  // Включакм тактирование таймера
//  RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
}

static void buzzerPowerOff( void ){
  BTN_PORT->BSRR |= BTN_PIN;

  // Переключаем на вход
  BTN_PORT->MODER &=  ~(0x3<< (BTN_PIN_NUM * 2));
  // Снимаем маску с бита BTN
  EXTI->IMR |= BTN_PIN;

}

// Переключение вывода на включение питания пищалки
static void buzzerPowerOn( void ){
  // Включаем на выход
  BTN_PORT->MODER |=  (0x1<< (BTN_PIN_NUM * 2));

  // Выставляем маску для бита BTN
  EXTI->IMR &= ~BTN_PIN;

  BTN_PORT->BRR |= BTN_PIN;
}
