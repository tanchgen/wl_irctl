/*
 * ir.h
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef IR_H_
#define IR_H_

#include "main.h"
#include "gpio.h"

/* Максимальное количествопараметров параметров:
 * 0 - ON/OFF,
 * 1 - MODE,
 * 2 - TEMP,
 * 3 - FAN,
 * 4 - SWING
 */
#define PARAM_NUM_MAX           4

// Состояние обучения
typedef enum {
  RX_STAT_ONOFF,
  RX_STAT_MODE,
  RX_STAT_TEMP,
  RX_STAT_FAN,
  RX_STAT_SWING,
  RX_STAT_0
} eRxStat;

typedef enum {
  PARAM_ONOFF,
  PARAM_MODE,
  PARAM_TEMP,
  PARAM_FAN,
  PARAM_SWING,
  PARAM_CRC
} eParam;

extern int8_t onOffFlag;
extern uint16_t irRxIndex;
extern uint8_t irRxGetFlag;
extern uint8_t paramValCount;
extern uint16_t ir0PktNoname[255];
// Указатель на пакет: НАЧАЛЬНЫЙ, НЕ начальный, отправляемый
extern uint16_t *pIrPkt;
extern uint16_t ir0Pkt[];
extern uint16_t irPkt[];
extern eRxStat rxStat;                      // Стастус (Этап) приема обучающих пакетов
extern const uint8_t paramfieldBegin[];    // Массив Указателей на зону измененных полей для каждого параметра
extern uint8_t rxFieldQuant[];              // Массив количества изменных полей для каждого параметра

extern uint8_t txFieldCount;                // Счетчик полей, передаваемых по ИК-каналу
extern uint8_t field0Num;                  // Соличество полей в НАЧАЛЬНОМ пакете

void irRxInit( void );
void irRxProcess( void );
uint8_t learnProcess( void );
void learnReset( void );
// Таймер модулирующей для обучения
void irModulLearnTimInit( void );
// Таймер модулирующей для передачи
void irModulTxTimInit( void );
uint8_t irPktSend( void );
void irCarierTimInit( void );         // Инициализация таймена несущей ИК-передачи

inline void irTxPinInit( void ){
  // Инициируем вывод ИК
  RCC->IOPENR |= (RCC_IOPENR_GPIOAEN << IR_TX_PORT_NUM);

  // Пин ВВЕРХ
  IR_TX_PORT->BSRR |= IR_TX_PIN;

  // Подтяжка выключена
  IR_TX_PORT->PUPDR = (IR_TX_PORT->PUPDR & ~(0x3 << (IR_TX_PIN_NUM * 2)));
  // Открытый сток
  IR_TX_PORT->OTYPER &= ~(IR_TX_PIN);
  IR_TX_PORT->OSPEEDR = (IR_TX_PORT->OSPEEDR & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));
  IR_TX_PORT->MODER = (IR_TX_PORT->MODER & ~(0x3 << (IR_TX_PIN_NUM * 2))) | (0x1 << (IR_TX_PIN_NUM * 2));
  IR_TX_PORT->AFR[0] = (GPIOA->AFR[0] & ~(0xFL<<(IR_TX_PIN_NUM * 4)));
}

#endif /* IR_H_ */
