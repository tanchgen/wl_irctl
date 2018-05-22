/*
 * proto.h
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef PROTO_H_
#define PROTO_H_

// Имена доступных протоколов
typedef enum {
  PROTO_NONAME,     //неизвестный протокол
  PROTO_AERONIK,
  PROTO_SAMSUNG,
  PROTO_DAIKIN,
  PROTO_PANAS,
  PROTO_MITSU,
  PROTO_FIDJI,
  PROTO_HAIER,
  PROTO_NUM
} eProtoName;

typedef struct {
  uint8_t headerLen;
  uint16_t * headerfield;
} tHeader;

void decodProto( uint16_t c );


#endif /* PROTO_H_ */
