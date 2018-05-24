/*
 * proto.h
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef PROTO_H_
#define PROTO_H_

#include "main.h"

// Имена доступных протоколов
typedef enum {
  PROTO_SAMSUNG,
  PROTO_AERONIK,
  PROTO_DAIKIN,
  PROTO_PANAS,
  PROTO_MITSU,
  PROTO_FIDJI,
  PROTO_HAIER,
  PROTO_NONAME,     //неизвестный протокол
  PROTO_NUM
} eProtoName;

typedef struct {
  uint8_t fieldType;
  uint8_t fieldLen;
  uint16_t * field;
} tFieldArr;

typedef struct {
  tFieldArr * protoFieldArr;       // Массив полей заголовка
  uint8_t markDur;                // Длительность маркера (импульса)
  uint8_t space0Dur;              // Длительность паузы "0"
  uint8_t space1Dur;              // Длительность паузы "1"
  uint8_t fieldNum;               // Общее количество полей в протоколе
} tProtoDesc;

enum  eFieldType {
  FTYPE_DUR,
  FTYPE_BIT
};

extern eProtoName protoName;

uint8_t protoDecod( uint16_t *pIrPkt, uint8_t len );

// Сравнение длительностей полей в ИК-пакете
inline int8_t irDurCmp( uint16_t dur0, uint16_t dur, uint8_t percent){
  uint16_t tmp;
  if( dur0 > dur){
    tmp = dur0;
    dur0 = dur;
  }
  else {
    tmp = dur;
  }
  // Если разница меньше percent % возвращаем 0 (FALSE), иначе 1 (TRUE)
  // Учитывается точность измерения интервалов: +-1
  return ( (tmp - dur0) > (((dur0 * percent) / 100) + 1) )? TRUE: FALSE;
}

inline uint8_t writeDataBit( uint16_t * fldArr, uint8_t idx, uint8_t bit){
  uint8_t rec = 0;
  uint8_t byteidx = idx / 16;

  *(fldArr + byteidx) |= bit << (idx % 16);

  return rec;
}

#endif /* PROTO_H_ */
