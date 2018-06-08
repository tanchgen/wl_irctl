/*
 * proto.c
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <string.h>
#include "stm32l0xx.h"
#include "ir.h"
#include "proto.h"

#ifndef NULL
#define NULL    ((void *)0)
#endif


// Установка параметров для Определенного
void protoDefParamSet( uint16_t * arr, const tParamPos * pprm );
int8_t areaFind( tFieldArr * pFldArr, uint8_t *idx );
uint8_t crcCod( uint16_t * arr, uint8_t len, uint8_t mod );
uint8_t acParamValue( uint8_t param );

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

// Режим работы кондиционера
tAcData acData = {
    ON,             // Включить
    AC_MODE_COOL,           // Охлаждение
    TEMP_23,              // 23гр.Ц
    FAN_SPEED_AUTO,  // Скорость - авто
    SWING_POS_1
};

// ###################################### ОПРЕДЕЛЕННЫЕ ПРОТОКОЛЫ ###################################
//=============== Опитсание протокола AERONIK ============================
// Длительности (кол-во периодов несущей) полей заголовка
uint16_t anikHeader0Field[2] = {344, 172};
// Длительности полей паузы
uint16_t anikMidPauseField[2] = {25, 763};
uint16_t anikMidPause2Field = 52;

// Описание ИК-пакета протокола AERONIK
tFieldArr anikProtoField[6] = {
  {FTYPE_DUR, 2, anikHeader0Field},       // Заголовок: 2 поля, размер - длительности полей, поля в массиве anikHeader0Field
  {FTYPE_BIT, 70, irPktArr},              // Массив данных: 70 полей, размер - 2 поля = 1 бит, начало данных в массиве irPktArr[0]
  {FTYPE_DUR, 2, anikMidPauseField},       // Заголовок 2: 2 поля, размер - длительности полей, поля в массиве anikHeader1Field
  {FTYPE_BIT, 64, irPktArr+3},            // Массив данных: 64 данных, размер - 1 бит, начало данных в массиве irPktArr[3]
  {FTYPE_DUR, 1, &anikMidPause2Field},       // Заголовок 2: 2 поля, размер - длительности полей, поля в массиве anikHeader1Field
  {FTYPE_DUR, 0, NULL}                             // Признак конца пакета
};

// НУЛЕВОЙ пакет AERONIK
const tProtoPkt0 anikPkt0 = { 5, { 0x0709, 0x5040, 0x0002, 0x2000, 0xc000 }};;
const tParamPos anikParams[6] = {
    {0X1, 3},     // ONOFF
    {0x7, 0},     // MODE
    {0xF, 8},     // TEMP
    {0x2, 4},     // FAN
    {0xF, 35},    // SWING
    {0xF, 76}     // CRC
};

const tCrc anikCrc = { 9, 4 };


//============== Опитсание протокола SAMSUNG =============================
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

// НУЛЕВОЙ пакет AERONIK
const tProtoPkt0 smsgPkt0 = { 5, { 0x0709, 0x5040, 0x0002, 0x2000, 0xc000 }};;

const tParamPos smsgParams[5] = {
    {0X1, 3},     // ONOFF
    {0x7, 0},     // MODE
    {0xF, 8},     // TEMP
    {0x2, 4},     // FAN
    {0xF, 16}     // SWING
};

const tCrc smsgCrc = { 4, 8 };

/*
 * typedef struct {
 *   uint16_t * protoFieldArr;          // Массив полей заголовка
 *   uint16_t * protoFieldArr0;         // Массив полей НАЧАЛЬНОГО заголовка
 *   const tParamPos * paramPos;
 *   const tCrc * crc;
 *   uint8_t markDur;                   // Длительность маркера (импульса)
 *   uint8_t space0Dur;                 // Длительность паузы "0"
 *   uint8_t space1Dur;                 // Длительность паузы "1"
 *   uint8_t fieldNum;                  // Общее количество полей в протоколе
 * } tProtoDesc;
 */
// Описание протоколов
tProtoDesc protoDesc[PROTO_NUM] = {
    {anikProtoField, &anikPkt0, anikParams, &anikCrc, 25, 21, 61, 0},       // Протокол AERONIK
    {smsgProtoField, &smsgPkt0, smsgParams, &smsgCrc, 18, 18, 180, 0},      // Протокол SAMSUNG
    {NULL, NULL, NULL, NULL, 0, 0, 0, 0},                    // Протокол Daikin
    {NULL, NULL, NULL, NULL, 0, 0, 0, 0},                    // Протокол Panasonic
    {NULL, NULL, NULL, NULL, 0, 0, 0, 0},                    // Протокол Mitsubishi
    {NULL, NULL, NULL, NULL, 0, 0, 0, 0},                    // Протокол Fidji
    {NULL, NULL, NULL, NULL, 0, 0, 0, 0},          // Протокол Haier
    {&newProtoField, NULL, NULL, NULL, 0, 0, 0, 0},          // Новый протокол
};


/*################################################################################################*/

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



// Установка параметров для Определенного протокола
void protoDefParamSet( uint16_t * arr, const tParamPos * pprm ){
  uint8_t value;
  uint16_t * arrn;
  uint8_t i;

  for( i = 0; i < 6; i++ ){
    value = acParamValue( i );
    arrn = arr + (pprm[i].pos/ 16);
    *arrn = (*arrn & ~(pprm[i].mask << (pprm[i].pos % 16))) | (value << (pprm[i].pos % 16));
  }
}

/* Расчет контрольной суммы для поля данных
 * arr - массив данных
 * mod - длина CRC в битах
 * len - длина расчитываемых данных в длинах "mod":
 * Например: Длина arr - 32бита, mod - 8бит, следовательно, len = 4
 */
uint8_t crcCod( uint16_t * arr, uint8_t len, uint8_t mod ){
  uint8_t crc = 0;
  uint8_t mask = 0;

  mask = (1 << mod) - 1;

  for( uint8_t i = 0; i < len; i++){
    crc += (arr[ i * mod / 16] >> ( (i * mod) % 16) ) & mask;
  }
  while( crc > 0xf ){
    crc = (crc & 0xf) + ((crc & 0xf0) >> 4);
  }

  return crc;
}

// Кодирование пакета для отправки по ИК
void protoPktCod( void ){
  // Формируем сырой пакет для передачи по ИК
  if( protoName == PROTO_NONAME){
    protoNonDefCod();
  }
  else {
    protoDefCod( &protoDesc[protoName] );
  }

}


// Кодирование отправляемого ИК-пакета для Определенного протокола
uint8_t protoDefCod( tProtoDesc * prDesc ){
  uint8_t rec = 0;
  int8_t areaNum;
  uint8_t i;
  uint8_t bitIdx = 0;

  tFieldArr * pField = prDesc->protoFieldArr;

  // Заполняем поле данных пакета
  for( i = 0; i < prDesc->protoFieldArr0->len; i++ ){
    irPktArr[i] = prDesc->protoFieldArr0->arr[i];
  }
  for( ; i < 8; i++ ){
    irPktArr[i] = 0;
  }

  // Формируем массив данных - параметров
  protoDefParamSet( irPktArr, prDesc->paramPos );


  // Формируем Массив сырых данных ( длительностей импульсов и пауз ) пакета

  for( i = 0; i < 255; i++ ) {
    uint8_t pi = i;
    areaNum = areaFind( pField, &pi);
    if( areaNum < 0){
      // Массив заполнен
      break;
    }
    if( pField[areaNum].fieldType == FTYPE_DUR ){
      irPkt[i] = pField[areaNum].field[pi];
    }
    else {
      // Перобразуем битовую кодировку в длительности импульсов и пауз
      if( (i & 0x1) == 0 ){
        // Импульс (Марка)
        irPkt[i] = prDesc->markDur;
      }
      else if( (irPktArr[bitIdx/16] & (1 << (bitIdx % 16)) ) == 0){
        // Пауза бита "0"
        irPkt[i] = prDesc->space0Dur;
        bitIdx++;
        }
      else {
        // Пауза бита "1"
        irPkt[i] = prDesc->space1Dur;
        bitIdx++;
      }
    }
  }


  return rec;
}

void protoNonDefCod( void ){
  // Сначала копируем НАЧАЛЬНЫЙ пакет в массив полей для отправки
  for( uint8_t i = 0; i < irRxIndex; i++ ){
    irPkt[i] = ir0Pkt[i];
  }
  // Теперь переписываем поля в соответсвии с параметрами
  for( eParam i = PARAM_ONOFF; i <= PARAM_SWING; i++ ){
    uint8_t value;
    uint8_t sellNum;

    value = acParamValue( i );
    /* Если какой-то из параметров отличается от параметров НАЧАЛЬНОГО пакета - устанавливаем его и
     * устанавливаем его и следующие НЕ меняем
     */

    if( i == PARAM_ONOFF ){
      i = PARAM_CRC;
    }
    else if ( i == PARAM_MODE ){
      if(value != AC_MODE_COOL){
        i = PARAM_CRC;
      }
    }
    else if( i == PARAM_TEMP ){
      if( value != TEMP_23 ){
        i = PARAM_CRC;
      }
    }
    else if( i == PARAM_FAN ){
      if( value != FAN_SPEED_AUTO ){
        i = PARAM_CRC;
      }
    }

    sellNum = paramfieldBegin[i] + value;

    // Перебираем отличия в полях
    for( uint8_t j = 0; j < rxFieldQuant[sellNum]; j++) {
      // Есть отличия в полях
      irPkt[ (pIrField[sellNum])[j].fieldNum ] = (pIrField[sellNum])[j].fieldDur;
    }

  }
}

// Возвращает значение параметра
uint8_t acParamValue( uint8_t param ){
  uint8_t value;

  switch( param ){
    case   PARAM_ONOFF:
      value = acData.onoff;
      break;
    case   PARAM_MODE:
      value = acData.mode;
      break;
    case   PARAM_TEMP:
      value = acData.temp;
      break;
    case   PARAM_FAN:
      value = acData.fan;
      break;
    case   PARAM_SWING:
      value = acData.swing;
      break;
    case PARAM_CRC:
      if(protoName != PROTO_NONAME ){
        const tCrc * crc = protoDesc[protoName].crc;
        // Только для определенного протокола - Высчитываем CRC
        value = crcCod( irPktArr, crc->len, crc->mod );
      }
      else {
        value = 0;
      }
      break;
    default:
      value = 0;
      break;
  }

  return value;
}

uint8_t irProtoRestore( void ){
  uint8_t fldNum = 0;

  if( (fldNum = eeIrProtoBak.fld0Num) == 0 ){
    // Восстанавливать нечего - ничего не сохранено
    goto restEnd;
  }
  onOffFlag = ON;
  if( (protoName = eeIrProtoBak.protoName) != PROTO_NONAME ){
    // ОПРЕДЕЛЕННЫЙ протокол - больше восстанавливать нечего
    goto restEnd;
  }

  // Восстанавление НЕОПРЕДЕЛЕННОГО протокола
  memcpy( ir0Pkt, eeIrProtoBak.fld0Pkt, fldNum * 2);
  memcpy( irDiffField, eeIrProtoBak.diffField, sizeof(tRxFieldLst) * 214 );

#define SELL_NUM (ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX)
  memcpy( pIrField, eeIrProtoBak.pfldDiff, SELL_NUM * sizeof(tRxFieldLst) );
  memcpy( rxFieldQuant, eeIrProtoBak.fldDiffQuant, SELL_NUM );

restEnd:
  return fldNum;
}
