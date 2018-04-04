/*
 * rfm69.h
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef RFM69_H_
#define RFM69_H_

#include "gpio.h"
#include "my_time.h"

// Определения для входов DIO_RFM
#ifndef __packed
#define __packed __attribute__((packed))
#endif

// Выводы и порты
// Номера пинов DIO
#define DIO0_PIN        GPIO_Pin_0
#define DIO0_PIN_NUM    0
#define DIO0_PORT       GPIOA
#define DIO0_PORT_NUM   0

#define DIO0_EXTI_IRQn  EXTI0_1_IRQn

#define DIO1_PIN        GPIO_Pin_1
#define DIO1_PIN_NUM    1
#define DIO1_PORT       GPIOA

#define DIO2_PIN        GPIO_Pin_2
#define DIO2_PIN_NUM    2
#define DIO2_PORT       GPIOA

#define DIO3_PIN        GPIO_Pin_3
#define DIO3_PIN_NUM    3
#define DIO3_PORT       GPIOA
#define DIO3_PORT_NUM   0
#define DIO3_EXTI_IRQn  EXTI2_3_IRQn

#define DIO4_PIN        GPIO_Pin_6
#define DIO4_PIN_NUM    6
#define DIO4_PORT       GPIOA

#define DIO5_PIN        GPIO_Pin_2
#define DIO5_PIN_NUM    2
#define DIO5_PORT       GPIOB

// Определения для вывода Reset модуля
#define RFM_RST_PIN      GPIO_Pin_8
#define RFM_RST_PIN_NUM  8
#define RFM_RST_PORT     GPIOB

// Выводы по назначению
#define DIO_PAYL_RDY      DIO0
#define DIO_TX_FIFOFULL   DIO1
#define DIO_RX_FIFONE     DIO2
#define DIO_RX_SYNC       DIO3
#define DIO_RX_RSSI       DIO4
#define DIO_MODEREADY     DIO5

#define CONCAT1( x )   (x ## _PIN)
#define CONCAT2( x )   (x ## _PORT)

#define dioRead( a )      ((CONCAT2( a )->IDR) & (CONCAT1( a )))? 1: 0

/* ------------------- Регистры параметров модуля ------------------------------------
 * Забугорные коллеги пишут, что есть 5 правил для выбора параметров в данном модуле.
 * 1. 0.5 <= 2*Fdev/BR <= 10 (MI) стараться понижать
 * 2. BR < 2*RxBw (bit rate) стараться понижать
 * 3. RxBw >= Fdev+BR/2 (полоса приемника стараться понижать)
 * 4. RxBwAfc >= Fdev+BR/2+LOlffset (полоса приема с автоподстройкой)
 * 5. Fdev + BR/2 < 500KHz (максимально что можно установить)
 * Примерный расчет.
 * 433Мгц -> LOoffset = 8.66Khz (+-20ppm crystal)
 * Берем RxBwAfc 41.7 (0b10 / 24 / 3)
 * Актуальная полоса 41.7 - 8.7 = 33КГц (RxBw)
 * Берем MI как 1.5 (3/2), тогда Fdev = BR * 0.75 = BR * 3/4
 * Отсюда 33КГц >= BR * 3/4 + BR/2 имеем 33КГц >= BR*5/4, поэтому BR=33КГц*4/5=26.4 , берем 26Кгц.
 * Fdev = 26000 * 0.75 = 19500
 * Проверка
 * 1. 0.5 <= 2*19.5/26 <=10 Ok
 * 2. 26K < 2*33K Ok
 * 3. 33K >= 19.5K+13K OK
 * 4. 41.7K >= 19.5K+13K + 8.7K Ok
 * 5. 19.5K+13K < 500K Ok
 */

// Адреса регистров RFM69
#define REG_FIFO        (uint8_t)0x00    // доступ к записи/чтению FIFO
#define REG_OPMODE      (uint8_t)0x01    // Режим работы трансивера

#define REG_BR_MSB      (uint8_t)0x03    // Скорость данных старший байт( BitRate = (Fosc=32МГц) / RegBR)
#define REG_BR_LSB      (uint8_t)0x04    // Скорость данных младший байт

#define REG_FDEV_MSB    (uint8_t)0x05    // Девиация частоты старший байт (Fdev = RegFdev * RFstep)
#define REG_FDEV_LSB    (uint8_t)0x06    // Девиация частоты младший байт

#define REG_FRF_MSB     (uint8_t)0x07    // Старший байт величины несущей частоты ( Fнесущ = FRF * 61Гц )
#define REG_FRF_MID     (uint8_t)0x08    // Срединй байт величины несущей частоты
#define REG_FRF_LSB     (uint8_t)0x09    // Младший байт величины несущей частоты

#define REG_RCCAL       (uint8_t)0x0A    // Управление салибровкой RC-генератора
#define REG_PA_LVL      (uint8_t)0x11    // Настройка усилителя передатчика
#define REG_LNA         (uint8_t)0x18    // Настройки LNA
#define REG_RX_BW       (uint8_t)0x19    // BW контроль

#define REG_AFC_BW      (uint8_t)0x1A    //
#define REG_AFCFEI      (uint8_t)0x1E    // Управление AFC-FEI
#define REG_FEI_VAL_MSB (uint8_t)0x21    // Старший байт значения FEI
#define REG_FEI_VAL_LSB (uint8_t)0x22    // Младший байт значения FEI

#define REG_RSSI_CFG    (uint8_t)0x23    // Управление подсчетом Rssi
#define REG_RSSI_VAL    (uint8_t)0x24  // Величина RSSI ( - RssiValue / 2 dBm )
#define REG_DIO_MAP1    (uint8_t)0x25    // Переопределение DIO0-DIO3
#define REG_DIO_MAP2    (uint8_t)0x26    // Переопределение DIO4, DIO5, ...
#define REG_FLAG1   	 (uint8_t)0x27    // Флаги прерываний 1
#define REG_FLAG2   	 (uint8_t)0x28    // Флаги прерываний 2
#define REG_RSSI_THRESH (uint8_t)0x29    // RSSI граница
#define REG_SYNC_CFG    (uint8_t)0x2E    // Конфигурация SYNC-последовательности
#define REG_SYNC1       (uint8_t)0x2F    // Самый старший байт Sync-последовательности (NET ID)
#define REG_SYNC2       (uint8_t)0x30    // Следующий байт Sync-последовательности (NET ID)
#define REG_PACK_CFG    (uint8_t)0x37     // Конфигурация пакета
#define REG_PAYL_LEN    (uint8_t)0x38    // Длина payload (при фиксированной длине = отправляемой/принимаемой, при переменной = максимальной принимаемой)
#define REG_NODE_ADDR   (uint8_t)0x39    // Адрес нода
#define REG_BRDCAST     (uint8_t)0x3A     // Широковещательный адрес
#define REG_FIFO_THRESH (uint8_t)0x3C    // Количество байт в FIFO для начала передачи
#define REG_TEST_DAGC   (uint8_t)0x6F    // Настройка DAGC


#define REG_LNA_ZIN_50    (uint8_t)0x80    // Входное сопротивление = 50 Ом
#define REG_LNA_ZIN_200   (uint8_t)0x00    // Входное сопротивление = 200 Ом
#define REG_LNA_GAIN_AGC  (uint8_t)0x00    // Максимальное усиление определяется AGC
#define REG_LNA_GAIN_HI   (uint8_t)0x01      // Максимальное усиление = макс.
#define REG_LNA_GAIN_6    (uint8_t)0x02      // Максимальное усиление = макс. - 6Дб
#define REG_LNA_GAIN_12   (uint8_t)0x03      // Максимальное усиление = макс. - 12Дб
#define REG_LNA_GAIN_24   (uint8_t)0x04      // Максимальное усиление = макс. - 24Дб
#define REG_LNA_GAIN_36   (uint8_t)0x05      // Максимальное усиление = макс. - 36Дб
#define REG_LNA_GAIN_48   (uint8_t)0x06      // Максимальное усиление = макс. - 48Дб

// =============== Битовые маски регистров ==========================
// RegOpMode
#define REG_OPMODE_SEQOFF     (uint8_t)0x80    // Автоматический секвенсор (1- выкл, 0 - вкл)
#define REG_OPMODE_LISTEN_ON  (uint8_t)0x40    // Режим прослушивания (1 - вкл, 0 - выкл)
#define REG_OPMODE_LISTEN_ABR (uint8_t)0x20    // Прерывание режима прослушивания ( запись 1 - прервать )
#define REG_OPMODE_MODE       (uint8_t)0x1C    // Маска флагов режима работы
#define REG_OPMODE_SLEEP      (uint8_t)0x00    // Sleep-mode
#define REG_OPMODE_STDBY      (uint8_t)0x04    // Standby - режим
#define REG_OPMODE_FS         (uint8_t)0x08    // FS-режим
#define REG_OPMODE_TX         (uint8_t)0x0C    // TX-режим
#define REG_OPMODE_RX         (uint8_t)0x10    // RX-режим

// RegAfcFei
#define REG_AFCFEI_FEI_DONE     (uint8_t)0x40    // FEI посчитан
#define REG_AFCFEI_FEI_START    (uint8_t)0x20    // Старт подсчета FEI
#define REG_AFCFEI_AFC_DONE     (uint8_t)0x10    // AFC посчитан
#define REG_AFCFEI_AFC_AUTO     (uint8_t)0x04    // AFC Автоподсчет
#define REG_AFCFEI_AFC_DONE     (uint8_t)0x10    // AFC посчитан

// RegRssiConfig
#define REG_RSSI_CFG_DONE     (uint8_t)0x02    // Флаг Rssi посчитан
#define REG_RSSI_START        (uint8_t)0x01    // Бит запуска подсчета Rssi

// RegIrqFlag
#define REG_IF1_MODEREADY      (uint8_t)0x80     // Переключение режима закончено
#define REG_IF1_RXREADY        (uint8_t)0x40     // Готовность RX
#define REG_IF1_TXREADY        (uint8_t)0x20     // Готовность TX
#define REG_IF1_PLLLOCK        (uint8_t)0x10     // Готовность PLL (Частота вытавлена)
#define REG_IF1_RSSI           (uint8_t)0x08     // Rssi превысел уровень Rssi Threshold
#define REG_IF1_TOUT           (uint8_t)0x04     // Таймаут RxStart или RssiThresh
#define REG_IF1_AUTO           (uint8_t)0x02     // В авторежиме перешло в промежуточный режим
#define REG_IF1_ADDR           (uint8_t)0x01     // Совпадение Sync-последовательности и адреса нода(если задан)

#define REG_IF2_FIFO_FULL     (uint8_t)0x80     // FIFO полон
#define REG_IF2_FIFO_NE       (uint8_t)0x40     // FIFO НЕ пуст
#define REG_IF2_FIFO_LVL      (uint8_t)0x20     // FIFO заполнен больше, чем FIFO_THRESHOLD
#define REG_IF2_FIFO_OVER     (uint8_t)0x10     // FIFO переполнен
#define REG_IF2_TX_SENT       (uint8_t)0x08     // Пакет отправлен (В режиме TX)
#define REG_IF2_PAYL_RDY      (uint8_t)0x04     // Пакет принят, CRC совпал (В режиме RX)
#define REG_IF2_CRC_OK        (uint8_t)0x02     // CRC совпал (В режиме RX)

// RegRcCal
#define REG_RCCAL_START       (uint8_t)0x80      // Старт калибровки RC-генератора
#define REG_RCCAL_DONE        (uint8_t)0x40      // Завершение калибровки RC-генератора

// RegPacketConfig1
#define REG_PACK_CFG_VAR      (uint8_t)0x80      // Длина пакета данных - переменная
#define REG_PACK_CFG_CRCON    (uint8_t)0x10      // Проверка CRC включена
#define REG_PACK_CFG_CRCCLROFF    (uint8_t)0x08  // При ошибке CRC PayloadReady выставляется и FIFO не стирается
#define REG_PACK_CFG_ADDRBRD  (uint8_t)0x04      // Фильтрация по адресу нода и широковещательному адресу
#define REG_PACK_CFG_ADDR     (uint8_t)0x02      // Фильтрация по адресу нода

#define TX_PWR_0      18     // Мощность передатчика 0 мДб
#define TX_PWR_10      28     // Мощность передатчика -10мДб

// ============== Настройки несущей частоты =======================
#define FR_STEP           (61L)     // Шаг настройки частоты Гц (Fstep = 61.0352727 Гц)

#define CHANN39_20_FREQ     (0x6c6325L)   // (433550000 / Fstep) Начальная частота "нулевого" канала 433Мгц
#define CHANN39_FREQ_STEP   (25000L)   // Разница частоты между каналами 433Мгц
#define CHANN39_NULL_FREQ   (433050000L) // Начальная частота "нулевого" канала 433Мгц
#define CHAN39_MAX        39

#define RF_BAUDRATE       9600
#define RF_BR_MSB         (uint8_t)((((32000000UL << 1) / (RF_BAUDRATE) + 1) >> 1) >> 8)
#define RF_BR_LSB         (uint8_t)(((32000000UL << 1) / RF_BAUDRATE + 1) >> 1)

/* Длитетельность передачи одного пакета:
 * 3байт - Преабула, 2байт - Sync, 1байт - Len, 1байт - nodeAddr, 1байт - msgNum, 1байт - battery, 1байт - temp
 * (11байт = 654мкс, при 19200 бод, 1253мкс, при 9600 бод)
 */
#define TX_DURAT (1260*2)

// Диапазон номеров каналов: 0 - 7  (433050 кГц - 433925 кГц)
#define CHANN8_FREQ_STEP   (100000L)    // Разница частоты между каналами
#define CHANN8_3_FREQ     (0x6c5b45L)   // (433425000 / Fstep) Начальная частота "третьего" канала 433Мгц
#define CHANN8_MAX            7



#define NET_ID            0x0101          // Идентификатор сети
//#define CHANN_DEF         ((NET_ID % 8)+1)   // RF-канал по умолчанию
#define CHANN_DEF         0x03   // RF-канал по умолчанию
#define NODE_ADDR         0x42            // Собственный адрес нода по умолчанию
#define BCRT_ADDR         0x00            // Адрес БКРТ-255
#define BRDCAST_ADDR      0xFF            // Широковещательный адрес

// Режим работы модуля
enum {
  MODE_SLEEP,
  MODE_STDBY,
  MODE_FS,
  MODE_TX,
  MODE_RX,
  MODE_START = 0x0f
};

enum {
  FEI_NOT,
  FEI_START,
  FEI_SET,
};

typedef struct {
  uint8_t channel;
  uint8_t mode;
  uint16_t netId;
  uint8_t nodeAddr;     // Адрес нода распологается в EEPROM
  uint8_t txPwr;
} tRfm;

typedef struct {
  uint8_t cmd;
  tUxTime time;
  uint8_t ms;
} __packed tTimeMsg;

typedef struct {
  uint8_t srcNode;    // Адрес отправителя
	uint8_t sensType;		// Тип конечного устройства
  uint8_t msgNum;     // Номер пакета
  uint8_t batVolt;    // Напряжение батареи питания
  uint16_t Volume;    // Значение сенсора
} __packed tSensMsg;

typedef struct {
  uint8_t cmd;
  uint8_t x8[63];
} __packed tCmdMsg;

typedef union {
  uint8_t u8[64];
  tCmdMsg cmdMsg;
  tTimeMsg timeMsg;
  tSensMsg sensMsg;
} __packed uPayload;


typedef struct {
  uint8_t nodeAddr;       // Адрес получателя пакета
  uint8_t payLen;         // Длина пакета (payload)
  uPayload payLoad;                 // Буфер получаемых от RFM69 данных
#define payBuf       payLoad.u8
#define payCmd       payLoad.cmdMsg.cmd
#define payUtime     payLoad.timeMsg.time
#define payMs        payLoad.timeMsg.ms
#define paySrcNode   payLoad.sensMsg.srcNode
#define payMsgNum    payLoad.sensMsg.msgNum
#define payBat       payLoad.sensMsg.batVolt
#define paySensType  payLoad.sensMsg.sensType
#define payVolume    payLoad.sensMsg.Volume

//  uint8_t bufLen;       // Длина приемного буфера
} __packed tPkt;

extern tPkt pkt;
extern tRfm  rfm;


uint8_t rfmRegRead( uint8_t add_r );
void rfmRegWrite( uint8_t addr, uint8_t data );

// Начальная инициализация трансивера
void rfmInit( void );
// Начальный сброс трансивера
void rfmRst( void );
// Переключение рабочего режима с блокировкой
void rfmSetMode_s( uint8_t opMode );
// Салибровка RC-генератора
void rfmRcCal( void );
void rfmTransmit_s( tPkt * ppkt );
void rfmTransmit( tPkt * ppkt );
void rfmTransmit_Long( uint8_t node, uint8_t *data, uint8_t len );
uint8_t rfmReceive( tPkt *pkt );
void rfmRecvStop( void );
uint16_t channelSearch( uint16_t count );
void rfmChannelSet( uint16_t channel );
void rfmFeiStart( void );
void rfmFeiSet( void );


#endif /* RFM69_H_ */
