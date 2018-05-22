/*
 * proto.c
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <proto.h>
#include "stm32l0xx.h"
#include "ir.h"

// Опитсание протокола AERONIK
// Длительности (кол-во периодов несущей) полей заголовка
const uint16_t anikHeader0Field[2] = {343, 172};
// Длительности полей паузы
const uint16_t anikHeader1Field = {172, 172};
const tHeader anikHeader[2] = {
  {2, anikHeader0Field},
  {1, anikHeader1Field}
};


// Декодирование принятого поля
void decodProto( uint16_t c ){
  // Декодирование НАЧАЛЬНОГО пакета

}


