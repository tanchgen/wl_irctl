/*
 * proto.c
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"
#include "ir.h"
#include "proto.h"

#ifndef NULL
#define NULL    ((void *)0)
#endif


int8_t areaFind( tFieldArr * pFldArr, uint8_t *idx );

//
eProtoName protoName = PROTO_NONAME;
// Используемый ИК-протокол
uint8_t protoCnt = PROTO_AERONIK;
// Массив полей данных
uint16_t irPktArr[8];
// Поля заголовка нового протокола
tFieldArr newProtoField = {
    FTYPE_DUR,
    0,
    ir0Pkt
};

// Опитсание протокола AERONIK
// Длительности (кол-во периодов несущей) полей заголовка
uint16_t anikHeader0Field[2] = {344, 172};
// Длительности полей паузы
uint16_t anikMidPauseField[2] = {25, 763};
uint16_t anikMidPause2Field = 25;
// Описание ИК-пакета протокола AERONIK
tFieldArr anikProtoField[6] = {
  {FTYPE_DUR, 2, anikHeader0Field},       // Заголовок: 2 поля, размер - длительности полей, поля в массиве anikHeader0Field
  {FTYPE_BIT, 70, irPktArr},              // Массив данных: 70 полей, размер - 2 поля = 1 бит, начало данных в массиве irPktArr[0]
  {FTYPE_DUR, 2, anikMidPauseField},       // Заголовок 2: 2 поля, размер - длительности полей, поля в массиве anikHeader1Field
  {FTYPE_BIT, 64, irPktArr+3},            // Массив данных: 64 данных, размер - 1 бит, начало данных в массиве irPktArr[3]
  {FTYPE_DUR, 1, &anikMidPause2Field},       // Заголовок 2: 2 поля, размер - длительности полей, поля в массиве anikHeader1Field
  {FTYPE_DUR, 0, NULL}                             // Признак конца пакета
};

// Опитсание протокола SAMSUNG
// Длительности (кол-во периодов несущей) полей заголовка
uint16_t smsgHeader0Field[4] = {25, 667, 112, 340};
// Длительности полей паузы
uint16_t smsgHeader1Field[3] = {112, 112, 763};
// Описание ИК-пакета протокола AERONIK
tFieldArr smsgProtoField[5] = {
  {FTYPE_DUR, 4, smsgHeader0Field},       // Заголовок: 4 поля, размер - длительности полей, поля в массиве smsgHeader0Field
  {FTYPE_BIT, 0, irPktArr},              // Массив данных: 35 данных, размер - 1 бит, начало данных в массиве irPktArr[0]
  {FTYPE_DUR, 3, smsgHeader1Field},       // Заголовок 2: 2 поля, размер - длительности полей, поля в массиве smsgHeader1Field
  {FTYPE_BIT, 0, irPktArr+0},            // Массив данных: 32 данных, размер - 1 бит, начало данных в массиве irPktArr[3]
  {FTYPE_DUR, 0, NULL}                             // Признак конца пакета
};

/*
 * typedef struct {
 *   uint16_t * protoFieldArr;          // Массив полей заголовка
 *   uint8_t markDur;                   // Длительность маркера (импульса)
 *   uint8_t space0Dur;                 // Длительность паузы "0"
 *   uint8_t space1Dur;                 // Длительность паузы "1"
 *   uint8_t fieldNum;                  // Общее количество полей в протоколе
 * } tProtoDesc;
 */
// Описание протоколов
tProtoDesc protoDesc[PROTO_NUM] = {
    {anikProtoField, 25, 21, 61, 0},       // Протокол AERONIK
    {smsgProtoField, 18, 18, 180, 0},      // Протокол SAMSUNG
    {NULL, 0, 0, 0, 0},                    // Протокол Daikin
    {NULL, 0, 0, 0, 0},                    // Протокол Panasonic
    {NULL, 0, 0, 0, 0},                    // Протокол Mitsubishi
    {NULL, 0, 0, 0, 0},                    // Протокол Fidji
    {NULL, 0, 0, 0, 0},          // Протокол Haier
    {&newProtoField, 0, 0, 0, 0},          // Новый протокол
};


// Декодирование принятого поля
uint8_t protoDecod( uint16_t *pIrPkt, uint8_t len ){
  // Декодирование НАЧАЛЬНОГО пакета
  uint8_t protoCnt;
  uint8_t fldCnt;
  int8_t areaNum;

  //
  for( protoCnt = PROTO_AERONIK; (protoCnt < PROTO_NONAME); protoCnt++ ){
    uint8_t i;

    tFieldArr * pPrDsc = protoDesc[protoCnt].protoFieldArr;

    if( pPrDsc == NULL){
      //Данный протокол не описан - переходим к следующему
      continue;
    }

    // Перебираем полученные поля с начала и до конца
    for( i = 0; i < len; i++ ){
      uint16_t c = pIrPkt[i];
      fldCnt = i;
      areaNum = areaFind( pPrDsc, &fldCnt );
      if( areaNum < 0 ){
        // Индекс вышел за границы пакета для данного протокола
        // Переходим к следующему протоколу
        break;
      }
      if( pPrDsc[areaNum].fieldType == FTYPE_DUR ){
        // Поле пришлось на заголовок или промежуточную паузу - проверяем на соответствие
        if( irDurCmp(c, pPrDsc[areaNum].field[fldCnt], 5) ){
          // Очередное поле заголовка не совпадает - начинаем проверку заголовка с начала для следующего протокола
          break;
        }
      }
      else {
        // Поле - данные: декодируем их - 0 или 1. И записываем в соответствующий массив
        if(i & 0x1){
          // Это пауза
          uint8_t bit;
          if( irDurCmp( c, protoDesc[protoCnt].space0Dur, 20 ) == 0 ){
            // Совпало с паузой нуля
            bit = 0;
          }
          else if( irDurCmp( c, protoDesc[protoCnt].space1Dur, 20 ) == 0 ){
            // Совпало с паузой единицы
            bit = 1;
          }
          else {
            // Не совпало ни c нулем, ни с единицей
            // Переходим на неизвестный протокол
            protoCnt = (PROTO_NONAME - 1);
            break;
          }
          writeDataBit( (pPrDsc + areaNum)->field, (fldCnt / 2), bit );
        }
        else {
          // Это импульс (mark)
          if( irDurCmp( c, protoDesc[protoCnt].markDur, 20 ) != 0 ){
            // НЕ совпало с длительностью импульса
            // Переходим на неизвестный протокол
            protoCnt = (PROTO_NONAME - 1);
            break;
          }
        }
      }
    }
    if( i == len ){
      // Проверка закончена - протокол найден
      break;
    }
  }

  return protoCnt;
}

int8_t areaFind( tFieldArr * pFldArr, uint8_t *idx ){
  int8_t areaNum;

  for( areaNum = 0 ; ; areaNum++, pFldArr++ ){
    if( pFldArr->field == NULL ){
      areaNum = -1;
      break;
    }
    if( *idx < pFldArr->fieldLen ){
      break;
    }
    *idx -= pFldArr->fieldLen;
  }
  return areaNum;
}
