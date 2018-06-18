/*
 * proto.h
 *
 *  Created on: 22 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef PROTO_H_
#define PROTO_H_

#include "main.h"

#define ONOFF_VAL_COUNT_MAX     2
#define TEMP_VAL_COUNT_MAX      15
#define MODE_VAL_COUNT_MAX      5
#define FAN_VAL_COUNT_MAX       5
#define SWING_VAL_COUNT_MAX     5

#define ONOFF_FIELDL_COUNT_MAX     10
#define TEMP_FIELD_COUNT_MAX      20
#define MODE_FIELD_COUNT_MAX      10
#define FAN_FIELD_COUNT_MAX       10
#define SWING_FIELD_COUNT_MAX     10

// ============== ПАРАМЕТРЫ РАБОТЫ ====================
// Режимы работы
enum eMode{
  AC_MODE_AUTO,
  AC_MODE_COOL,
  AC_MODE_DRY,
  AC_MODE_VENT,
  AC_MODE_HEAT
};

#define TEMP_23     7

// Скорость вентилятора
enum eFanSpeed {
  FAN_SPEED_AUTO,
  FAN_SPEED_1,
  FAN_SPEED_2,
  FAN_SPEED_3,
  FAN_SPEED_4,
};

// Положение диффузора вентилятора
enum eSwingPos {
  SWING_POS_AUTO,
  SWING_POS_1,
  SWING_POS_2,
  SWING_POS_3,
  SWING_POS_4,
};

enum eAcErr{
  AC_ERR_OK = 0,
  AC_ERR_NON_LEARN,
};

// Битовое поле параметров работы кондиционера
typedef struct {
  unsigned int onoff:  1;        // Вкл. / Выкл.
  enum eMode mode:      3;        // Режим работы
  unsigned int temp:   4;        // Температура (гр.Ц - 16): 23гр.Ц --> temp = 7
  enum eFanSpeed fan:   3;        // Скорость вентилятора
  enum eSwingPos swing: 3;        // Положение диффузора
  enum eAcErr   err:    2;        // Ошибка контроллера кондиционера
} tAcData;

// Структура массива полей НАЧАЛЬНОГО заголовка
typedef struct {
  uint8_t len;        // Длина массива в 16-ибитных словах
  uint16_t  arr[5];   // Массив полей НАЧАЛЬНОГО заголовка
} tProtoPkt0;

// ================== ПРОТОКОЛ AIRONIK ===============================

// Имена доступных протоколов
typedef enum {
  PROTO_AERONIK,
  PROTO_SAMSUNG,
  PROTO_DAIKIN,
  PROTO_PANAS,
  PROTO_MITSU,
  PROTO_FIDJI,
  PROTO_HAIER,
  PROTO_NONAME,     //неизвестный протокол
  PROTO_NUM
} eProtoName;

typedef struct {
  uint8_t mask;
  uint8_t pos;
} tParamPos;

typedef struct {
  uint8_t len;
  uint8_t mod;
} tCrc;

// Структура области пакета: заголовок, поле данных, пауза, промежуточный заголовок и т.д.
typedef struct {
  uint8_t fieldType;
  uint8_t fieldLen;
  uint16_t * field;
} tFieldArr;

typedef struct {
  tFieldArr * protoFieldArr;                // Массив полей заголовка
  const tProtoPkt0 * protoFieldArr0;       // Указатель структуры массива полей НАЧАЛЬНОГО заголовка
  const tParamPos * paramPos;
  uint8_t (*crc)( void );
  uint8_t markDur;                // Длительность маркера (импульса)
  uint8_t space0Dur;              // Длительность паузы "0"
  uint8_t space1Dur;              // Длительность паузы "1"
  uint8_t fieldNum;               // Общее количество полей в протоколе
} tProtoDesc;

enum  eFieldType {
  FTYPE_DUR,
  FTYPE_BIT
};

typedef struct {
  uint8_t fieldNum;         // Порядковый номер поля в пакете
  uint16_t fieldDur;        // Продолжительность поля в 10мкс
} tRxFieldLst;

struct eeProtoBak {
  uint16_t fld0Pkt[256];
  // Структура изменяемых полей
  tRxFieldLst diffField[4+120+30+30+30];
  // Массив указателей на структуры изменяемых полей
  tRxFieldLst * pfldDiff[ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + \
                         FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX];
  // Массив количества изменных полей для каждого параметра
  uint8_t fldDiffQuant[ONOFF_VAL_COUNT_MAX + MODE_VAL_COUNT_MAX + TEMP_VAL_COUNT_MAX + \
                         FAN_VAL_COUNT_MAX + SWING_VAL_COUNT_MAX];
  eProtoName protoName;
  uint8_t fld0Num;
};

extern struct eeProtoBak eeIrProtoBak;
extern eProtoName protoName;
extern tProtoDesc protoDesc[];
extern tAcData acData;
extern tRxFieldLst * pIrField[];            // Массив указателей на структуры изменяемых полей
extern tRxFieldLst irDiffField[];           // Список отличающихся от НАЧАЛЬНОГО полей


uint8_t protoDecod( uint16_t *pIrPkt, uint8_t len );
uint8_t protoPktCod( void );
uint8_t protoDefCod( tProtoDesc * prDesc );
void protoNonDefCod( void );
uint8_t irProtoRestore( void );

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
