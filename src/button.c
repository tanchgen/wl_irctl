/*
 * button.c
 *
 *  Created on: 27 апр. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"
#include "gpio.h"
#include "my_time.h"
#include "button.h"

tButton btn;

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

    // Снимаем маску с бита IR_RX
    EXTI->IMR |= IR_RX_PIN;

    // Проверяем интервал времени с предыдущего отпускания
    tmpTime = ( (uint64_t)ut << 8 ) + rtc.ss;
    tmpTime -= ( ((uint64_t)btn.tOffSec << 8 ) + btn.tOffSS);

    if( tmpTime > 0x500 ){
      // Интервал более 5-и секунд - обнуляем счетчик нажатий
      btn.pressCnt = 0;
    }
  }
  else {
    //Кнопка отжата
    // Сохраняем время выключения кнопки
    btn.tOffSec = ut;
    btn.tOffSS = rtc.ss;

    // Проверяем интервал времени с предыдущего нажатия
    tmpTime = ( (uint64_t)ut << 8 ) + rtc.ss;
    tmpTime -= ( ((uint64_t)btn.tOnSec << 8 ) + btn.tOnSS);

    if( tmpTime > 0x333 ){
      // КНОПКА была нажата более ~1.2 секунды - обнуляем счетчик нажатий
      btn.pressCnt = 0;
    }
    else {
    // КНОПКА нажата более 0.125сек и менее 1.2сек - увеличиваем счетчик нажатий
      btn.pressCnt++;
      // Запрещаем прерывание от ИК-приемника
      buzzerShortPulse();
    }

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
   // Прерывание по переполнению
   TIM6->DIER |= TIM_DIER_UIE;
   // Конфигурация NVIC для прерывания по таймеру TIM6
   NVIC_EnableIRQ( TIM6_IRQn );
   NVIC_SetPriority( TIM6_IRQn, 3 );

}

void buzzerShortPulse( void ){
  // Включакм тактирование таймера
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  // Запускаем пищалку
  TIM6->CR1 |= TIM_CR1_CEN;

  mDelay( 70 );
  // Останавливаем пищалку
  TIM6->CR1 &= ~TIM_CR1_CEN;
  TIM6->SR &= ~TIM_SR_UIF;
  // Включакм тактирование таймера
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
}

void buzzerLongPulse( void ){
  // Включакм тактирование таймера
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  // Запускаем пищалку
  TIM6->CR1 |= TIM_CR1_CEN;

  mDelay( 300 );
  // Останавливаем пищалку
  TIM6->CR1 &= ~TIM_CR1_CEN;
  // Включакм тактирование таймера
  RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
}

