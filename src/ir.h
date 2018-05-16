/*
 * ir.h
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef IR_H_
#define IR_H_

#define ONOFF_VAL_COUNT_MAX     1
#define TEMP_VAL_COUNT_MAX      15
#define MODE_VAL_COUNT_MAX      5
#define FAN_VAL_COUNT_MAX       4
#define SWING_VAL_COUNT_MAX     5

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

void irRxInit( void );
void irRxProcess( void );
void learnProcess( void );

#endif /* IR_H_ */
