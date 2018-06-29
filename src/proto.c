/*
 * proto.c
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <string.h>
#include "stm32l0xx.h"
#include "button.h"
#include "ir.h"
#include "proto.h"

#ifndef NULL
#define NULL    ((void *)0)
#endif


// Установка параметров для Определенного
uint8_t protoDefParamSet( uint16_t * arr, const tParamPos * pprm );
int8_t areaFind( tFieldArr * pFldArr, uint8_t *idx );
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

// Режимы работы кондиционера
// -------- по умолчанию --------
tAcData acState = {
    ON,                 // Включить
    AC_MODE_COOL,       // Охлаждение
    TEMP_23,            // 23гр.Ц
    FAN_SPEED_AUTO,     // Скорость - авто
    SWING_POS_AUTO,     // Положение диффузора
    AC_ERR_OK           // Ошибка протокола
};
// ---------- Принятый от центрального устройства --------------------
tAcData rxAcState;

uint8_t volMax[6] = {1, 4, 14, 4, 4, 15};   // Максилмальное значение параметра

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

const uint8_t fanChange[5] = { 0,1,2,3,3 };
const uint8_t swingChange[5] = { 1,2,3,4,5 };

// НУЛЕВОЙ пакет AERONIK
const tProtoPkt0 anikPkt0 = { 5, { 0x0709, 0x5040, 0x0002, 0x2000, 0xc000 }};
const tParamPos anikParams[6] = {
    {0X1, 3, NULL},        // ONOFF
    {0x7, 0, NULL},        // MODE
    {0xF, 8, NULL},        // TEMP
    {0x2, 4, fanChange},        // FAN
    {0x7, (3*16), swingChange},   // SWING
    {0xF, (4*16+12), NULL} // CRC
};


// Функция расчета CRC для протокола AERONIK
uint8_t anikCrc( void ){
  uint8_t crc = 0;
  uint8_t mask = 0;
  const uint8_t module = 4;
  const uint8_t len = 9;

  mask = (1 << module) - 1;

  for( uint8_t i = 0; i < len; i++){
    uint8_t tmp;

    tmp = irPktArr[ i * module / 16] >> ( (i * module) % 16);
    if( i == 1 ){
      // Выключаем FAN из контрольной суммы
      tmp &= 0xC;
    }
    crc += tmp & mask;
  }
  while( crc > 0xf ){
    crc = (crc & 0xf) + ((crc & 0xf0) >> 4);
  }

  return crc;
}


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
const tProtoPkt0 smsgPkt0 = { 5, { 0x0709, 0x5040, 0x0002, 0x2000, 0xc000 }};

const tParamPos smsgParams[5] = {
    {0X1, 3, NULL},     // ONOFF
    {0x7, 0, NULL},     // MODE
    {0xF, 8, NULL},     // TEMP
    {0x2, 4, NULL},     // FAN
    {0xF, 16, NULL}     // SWING
};

// Функция расчета CRC для протокола SAMSUNG
uint8_t smsgCrc( void ){
  uint8_t crc = 0;
//  uint8_t mask = 0;
//  const uint8_t mod = 4;
//  const uint8_t len = 9;
//
//  mask = (1 << mod) - 1;
//
//  for( uint8_t i = 0; i < len; i++){
//    if( i == 3 ){
//      continue;
//    }
//    crc += (irPktArr[ i * mod / 16] >> ( (i * mod) % 16) ) & mask;
//  }
//  while( crc > 0xf ){
//    crc = (crc & 0xf) + ((crc & 0xf0) >> 4);
//  }

  return crc;
}


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
    {anikProtoField, &anikPkt0, anikParams, anikCrc, 25, 21, 61, 0},       // Протокол AERONIK
    {smsgProtoField, &smsgPkt0, smsgParams, smsgCrc, 18, 18, 180, 0},      // Протокол SAMSUNG
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
uint8_t protoDefParamSet( uint16_t * arr, const tParamPos * pprm ){
  uint8_t rec = 0;
  uint8_t value;
  uint16_t * arrn;
  uint8_t i;

  for( i = 0; i < 6; i++ ){
    value = acParamValue( i );
    if(value <= volMax[i]){
      if( pprm[i].paramChange != NULL ){
        value = pprm[i].paramChange[value];
      }
      arrn = arr + (pprm[i].pos/ 16);
      *arrn = (*arrn & ~(pprm[i].mask << (pprm[i].pos % 16))) | (value << (pprm[i].pos % 16));
    }
    else {
      rec = 0;
      acState.err = AC_ERR_PARAM;
      break;
    }
  }
  return rec;
}

// Кодирование пакета для отправки по ИК
uint8_t protoPktCod( void ){
  uint8_t rec = 0;

  if(field0Num == 0){
    // Еще не прошел обучение - неизвестно, что отправлять
    rec = 1;
    goto exitLabel;
  }
  if( (btn.stat != BTN_OFF) || (state == STAT_BTN_DBNC) ){
    // Кнопка нажата: возможно, идет процесс обучения - параметры протокола могут изменится
    rec = 1;
    goto exitLabel;
  }

  // Выключаем прерывание от КНОПКИ
  EXTI->IMR &= ~(BTN_PIN);

  // Формируем сырой пакет для передачи по ИК
  if( protoName == PROTO_NONAME){
    rec = protoNonDefCod();
  }
  else {
    rec = protoDefCod( &protoDesc[protoName] );
  }

  // Включаем прерывание от КНОПКИ
  EXTI->IMR |= BTN_PIN;

exitLabel:
  return rec;
}


// Кодирование отправляемого ИК-пакета для Определенного протокола
uint8_t protoDefCod( tProtoDesc * prDesc ){
  uint8_t rec = 0;
  int8_t areaNum;
  uint8_t i;
//  uint8_t bitIdx = 0;

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
      pi >>= 1; // Один бит состоит из двух полей
      // Перобразуем битовую кодировку в длительности импульсов и пауз
      if( (i & 0x1) == 0 ){
        // Импульс (Марка)
        irPkt[i] = prDesc->markDur;
      }
      else if( (pField[areaNum].field[pi/16] & (1 << (pi % 16)) ) == 0){
        // Пауза бита "0"
        irPkt[i] = prDesc->space0Dur;
//        bitIdx++;
        }
      else {
        // Пауза бита "1"
        irPkt[i] = prDesc->space1Dur;
//        bitIdx++;
      }
    }
  }


  return rec;
}

uint8_t protoNonDefCod( void ){
  uint8_t rec = 0;

  // Сначала копируем НАЧАЛЬНЫЙ пакет в массив полей для отправки
  for( uint8_t i = 0; i < field0Num; i++ ){
    irPkt[i] = ir0Pkt[i];
  }
  // Теперь переписываем поля в соответсвии с параметрами
  for(  uint8_t k = PARAM_ONOFF; k <= PARAM_SWING; k++){
    uint8_t value;
    uint8_t sellNum;

    value = acParamValue( k );
    if( value > volMax[k] ){
      rec = 1;
      break;
    }

    sellNum = paramfieldBegin[k] + value;

    /* Если какой-то из параметров отличается от параметров НАЧАЛЬНОГО пакета - устанавливаем его и
     * устанавливаем его и следующие НЕ меняем
     */

    if( k == PARAM_ONOFF ){
      if( value == OFF){
        k = PARAM_CRC;
      }
      else {
        continue;
      }
    }
    else if ( k == PARAM_MODE ){
      if(value != AC_MODE_COOL){
        k = PARAM_CRC;
      }
    }
    else if( k == PARAM_TEMP ){
      if( value != TEMP_23 ){
        k = PARAM_CRC;
      }
    }
    else if( k == PARAM_FAN ){
      if( value != FAN_SPEED_AUTO ){
        k = PARAM_CRC;
      }
    }

    // Перебираем отличия в полях
    for( uint8_t j = 0; j < rxFieldQuant[sellNum]; j++) {
      // Есть отличия в полях
      irPkt[ (pIrField[sellNum])[j].fieldNum ] = (pIrField[sellNum])[j].fieldDur;
    }
  }

  return rec;

}

// Возвращает значение параметра
uint8_t acParamValue( uint8_t param ){
  uint8_t value;

  switch( param ){
    case   PARAM_ONOFF:
      // Записываем из принятого от центрального устройства в отправляемое по ИК-каналу кондиционеру
      acState.onoff = rxAcState.onoff;
      value = acState.onoff;
      break;
    case   PARAM_MODE:
      // Записываем из принятого от центрального устройства в отправляемое по ИК-каналу кондиционеру
      acState.mode = rxAcState.mode;
      value = acState.mode;
      break;
    case   PARAM_TEMP:
      // Записываем из принятого от центрального устройства в отправляемое по ИК-каналу кондиционеру
      acState.temp = rxAcState.temp;
      value = acState.temp;
      break;
    case   PARAM_FAN:
      // Записываем из принятого от центрального устройства в отправляемое по ИК-каналу кондиционеру
      acState.fan = rxAcState.fan;
      value = acState.fan;
      break;
    case   PARAM_SWING:
      // Записываем из принятого от центрального устройства в отправляемое по ИК-каналу кондиционеру
      acState.swing = rxAcState.swing;
      value = acState.swing;
      break;
    case PARAM_CRC:
      if(protoName != PROTO_NONAME ){
//        const tCrc * crc = protoDesc[protoName].crc;
        // Только для определенного протокола - Высчитываем CRC
//        value = crcCod( irPktArr, crc->len, crc->mod );
        value = protoDesc[protoName].crc();
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
  rxStat = RX_STAT_ONOFF;
  onOffFlag = ON;
  pIrPkt = irPkt;

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
