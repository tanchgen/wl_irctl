/*
 * ir.h
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef IR_H_
#define IR_H_

#include "main.h"

/* Максимальное количествопараметров параметров:
 * 0 - ON/OFF,
 * 1 - TEMP,
 * 2 - MODE,
 * 3 - FAN,
 * 4 - SWING
 */
#define PARAM_NUM_MAX           4

#define ONOFF_VAL_COUNT_MAX     1
#define TEMP_VAL_COUNT_MAX      15
#define MODE_VAL_COUNT_MAX      5
#define FAN_VAL_COUNT_MAX       5
#define SWING_VAL_COUNT_MAX     5

#define ONOFF_FIELDL_COUNT_MAX     10
#define TEMP_FIELD_COUNT_MAX      20
#define MODE_FIELD_COUNT_MAX      10
#define FAN_FIELD_COUNT_MAX       10
#define SWING_FIELD_COUNT_MAX     10

// Состояние обучения
typedef enum {
  RX_STAT_ONOFF,
  RX_STAT_TEMP,
  RX_STAT_MODE,
  RX_STAT_FAN,
  RX_STAT_SWING,
  RX_STAT_0
} eRxStat;

typedef enum {
  PARAM_ONOFF,
  PARAM_TEMP,
  PARAM_MODE,
  PARAM_FAN,
  PARAM_SWING
} eRxParam;

typedef struct {
  uint8_t fieldNum;         // Порядковый номер поля в пакете
  uint16_t fieldDur;        // Продолжительность поля в 10мкс
} tRxFieldLst;

extern uint8_t irRxIndex;
extern uint8_t irRxGetFlag;
extern uint8_t paramValCount;
extern uint16_t ir0PktNoname[255];
// Указатель на пакет: НАЧАЛЬНЫЙ, НЕ начальный, отправляемый
extern uint16_t *pIrPkt;
extern uint16_t ir0Pkt[];

void irRxInit( void );
void irRxProcess( void );
uint8_t learnProcess( void );
void learnReset( void );
void irModulTimInit( void );

#endif /* IR_H_ */
