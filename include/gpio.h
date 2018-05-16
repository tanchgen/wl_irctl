/*
 * gpio.h
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef INCLUDE_STM32L0XX_LL_GPIO_H_
#define INCLUDE_STM32L0XX_LL_GPIO_H_

#include "stm32l0xx.h"

#define GPIO_Pin_0                      GPIO_BSRR_BS_0 /*!< Select pin 0 */
#define GPIO_Pin_1                      GPIO_BSRR_BS_1 /*!< Select pin 1 */
#define GPIO_Pin_2                      GPIO_BSRR_BS_2 /*!< Select pin 2 */
#define GPIO_Pin_3                      GPIO_BSRR_BS_3 /*!< Select pin 3 */
#define GPIO_Pin_4                      GPIO_BSRR_BS_4 /*!< Select pin 4 */
#define GPIO_Pin_5                      GPIO_BSRR_BS_5 /*!< Select pin 5 */
#define GPIO_Pin_6                      GPIO_BSRR_BS_6 /*!< Select pin 6 */
#define GPIO_Pin_7                      GPIO_BSRR_BS_7 /*!< Select pin 7 */
#define GPIO_Pin_8                      GPIO_BSRR_BS_8 /*!< Select pin 8 */
#define GPIO_Pin_9                      GPIO_BSRR_BS_9 /*!< Select pin 9 */
#define GPIO_Pin_10                     GPIO_BSRR_BS_10 /*!< Select pin 10 */
#define GPIO_Pin_11                     GPIO_BSRR_BS_11 /*!< Select pin 11 */
#define GPIO_Pin_12                     GPIO_BSRR_BS_12 /*!< Select pin 12 */
#define GPIO_Pin_13                     GPIO_BSRR_BS_13 /*!< Select pin 13 */
#define GPIO_Pin_14                     GPIO_BSRR_BS_14 /*!< Select pin 14 */
#define GPIO_Pin_15                     GPIO_BSRR_BS_15 /*!< Select pin 15 */
#define GPIO_Pin_ALL                    (GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1  | GPIO_BSRR_BS_2  | \
                                         GPIO_BSRR_BS_3  | GPIO_BSRR_BS_4  | GPIO_BSRR_BS_5  | \
                                         GPIO_BSRR_BS_6  | GPIO_BSRR_BS_7  | GPIO_BSRR_BS_8  | \
                                         GPIO_BSRR_BS_9  | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11 | \
                                         GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | \
                                         GPIO_BSRR_BS_15) /*!< Select all pins */

// Выводы и порты
// ========================= КНОПКА ====================================
// Номер пина Кнопки
#define BTN_PIN        GPIO_Pin_15
#define BTN_PIN_NUM    15
#define BTN_PORT       GPIOA
#define BTN_PORT_NUM   0

#if (BTN_PIN_NUM > 3)
#define BTN_EXTI_IRQn  EXTI4_15_IRQn
#endif

// ========================= ПЕЩАЛКА ==============================
// Номер пина Пищалки
#define BUZ_PIN        GPIO_Pin_7
#define BUZ_PIN_NUM    7
#define BUZ_PORT       GPIOB
#define BUZ_PORT_NUM   1

// ========================= ИК-ПЕРЕДАТЧИК ==============================
// Номер пина вывода ИК-передатчика
#define IR_TX_PIN        GPIO_Pin_1
#define IR_TX_PIN_NUM    1
#define IR_TX_PORT       GPIOB
#define IR_TX_PORT_NUM   1

// ========================= ИК-ПРИЕМНИК ====================================
// Номер пина ИК-приемника
#define IR_RX_PIN        GPIO_Pin_6
#define IR_RX_PIN_NUM    6
#define IR_RX_PORT       GPIOB
#define IR_RX_PORT_NUM   1

#if (IR_RX_PIN_NUM > 3)
#define IR_RX_EXTI_IRQn  EXTI4_15_IRQn
#endif

#endif /* INCLUDE_STM32L0XX_LL_GPIO_H_ */
