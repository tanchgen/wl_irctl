/*
 * rfm69.c
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"

#include "main.h"
#include "spi.h"
#include "rfm69.h"

void rfmFreqSet( uint32_t freq );

tRfm  rfm;
tPkt pkt;            // Структура принятого пакета
//extern uint8_t tmpVal;

// Определение значения частоты трансивера для регистров RFM69HW
#define CHANN_FREQ( c ) (((((int32_t)(c) - 3 ) * CHANN8_FREQ_STEP)) / FR_STEP) + CHANN8_3_FREQ
//#define CHANN_FREQ( channel ) ((((uint32_t)channel * CHANN_FREQ_STEP) + CHANN_NULL_FREQ) / FR_STEP)


static inline void rfDataInit( void );
static inline void rfmRegSetup( void );
static inline void dioInit( void );

// Начальный сброс модуля
void rfmRst( void ){
  //Установить "1" на линии NSS
  RFM_RST_PORT->BSRR |= RFM_RST_PIN;
  mDelay( 1 );
  RFM_RST_PORT->BRR |= RFM_RST_PIN;
  mDelay( 10 );
}

// Чтение значения регистра
uint8_t rfmRegRead( uint8_t addr ){
  uint8_t rxData[2];
  uint8_t txData[2];

  // Выставляем бит "чтение"
  txData[0] = addr & 0x7F;
  // Отправляем адрес
  spiTransRecv_s( txData, rxData, 2 );
  // Считывем значение регистра
  return rxData[1];
}

// Запись значения регистра
void rfmRegWrite( uint8_t addr, uint8_t data ){
  uint8_t rxData[2];
  uint8_t txData[2];

  // Выставляем бит "запись"
  txData[0] = addr | 0x80;
  txData[1] = data;
  // Отправляем адрес
  spiTransRecv_s( txData, rxData, 2 );

}

void rfmInit( void ){
  // Иниализируем интерфейс SPI
  spiInit();

  rfDataInit();
  // Настройка выходов DIO_RFM и прерывания от DIO0 и DIO4 (RSSI)
  dioInit();

  // Начальный сброс модуля
  rfmRst();
  // Установка начальных значений регистров
  rfmRegSetup();
}


// Установка частотного канала
void rfmChannelSet( uint16_t channel ){
  uint32_t chFr;

  // Вычисляем частоту канала
  chFr = CHANN_FREQ( channel );
  rfmFreqSet( chFr );
}

// Установка частоты несущей
void rfmFreqSet( uint32_t freq ){
  uint8_t oldMode;
  uint8_t txBuf[4];
  uint8_t rxBuf[4];

  // Читаем действуюший режим
  oldMode = rfmRegRead( REG_OPMODE ) & REG_OPMODE_MODE;
  if( (oldMode ) == REG_OPMODE_TX ){
    // Если в режиме TX - прерводим в режим RX
    rfmSetMode_s( REG_OPMODE_RX );
  }
  // Формируем буфер передачи (MSB)

  txBuf[0] = REG_FRF_MSB | 0x80;
  txBuf[1] = (uint8_t)(freq >> 16);
  txBuf[2] = (uint8_t)(freq >> 8);
  txBuf[3] = (uint8_t)freq;
  spiTransRecv_s( txBuf, rxBuf, 4 );

  if( (oldMode ) == REG_OPMODE_RX ){
    // Если был в режиме RX - прерводим в режим FS
    rfmSetMode_s( REG_OPMODE_FS );
  }

  rfmSetMode_s( oldMode );
}


// Переключение рабочего режима с блокировкой
// Только для режима ListenOff
void rfmSetMode_s( uint8_t mode ){
	uint8_t rc;

	__disable_irq();

	if( rfm.mode == (mode >> 2) ){
  	goto exit;
  }
  rfmRegWrite( REG_OPMODE, mode );

  while( (rc = dioRead(DIO_MODEREADY)) == 0 )
  {}

  rfm.mode = mode >> 2;
exit:
  __enable_irq();
	return;
}

// Салибровка RC-генератора
void rfmRcCal( void ){
  rfmRegWrite( REG_RCCAL, REG_RCCAL_START );

  // Проверяем/ждем что калибровка завершена
  while( (rfmRegRead(REG_RCCAL) & REG_RCCAL_DONE) == 0 )
  {}

}


// Передача данных в эфир с ожиданием окончания передачи
void rfmTransmit_s( tPkt * ppkt ){
  uint8_t rc;

  EXTI->IMR &= ~DIO0_PIN;
  EXTI->PR &= DIO0_PIN;

  rfmTransmit( ppkt );

  // Ждем, пока закончится передача
  while( (rc=dioRead(DIO_PAYL_RDY)) == 0 )
  {}

  EXTI->IMR |= DIO0_PIN;
}


// Передача данных в эфир
void rfmTransmit( tPkt *pPkt ){
  uint8_t txBuf[67];

  // Формируем пакет для записи в FIFO
  // Заносим адрес FIFO + бит записи: regAddrByte
  txBuf[0] = REG_FIFO | 0x80;
  // Сначала загоняем длину payload (len) + 1 байт (байт адреса нода-получателя): paylLenByte
  txBuf[1] = pPkt->payLen + 1;
  // Загоняем байт адреса нода-получателя: nodeAddrByte
  txBuf[2] = pPkt->nodeAddr;
  for( uint8_t i = 0; i < pPkt->payLen; i++ ){
    txBuf[3+i] = pPkt->payBuf[i];
  }
  // Опустошаем FIFO
  while( dioRead(DIO_RX_FIFONE) == SET ){
    rfmRegRead( REG_FIFO );
  }

  // Записываем все в FIFO (длина = regAddrByte + paylLenByte + nodeAddrByte + len)
  spiTrans_s( txBuf, pPkt->payLen + 3);

  // Отмечаем запуск RFM_TX
#if DEBUG_TIME
	dbgTime.rfmTxStart = mTick;
#endif // DEBUG_TIME

  // Переводим в режим передачи
  if( rfm.mode != MODE_TX ){
    // Переводим в режим TX-mode
    rfmSetMode_s( REG_OPMODE_TX );
  }
}


/* Прием данных из эфира
 * data - приемный буфер
 * len - максимальная длина
 * Возвращает реальное количество принятых байт
 */

uint8_t rfmReceive( tPkt * pkt ){
 uint8_t rc;

  // Проверяем, получен ли пакет полностью
 if( (rc = dioRead(DIO_PAYL_RDY)) == RESET  ){
    return 0;
  }

  // Считываем длину принимаемых данных (payload) (длина пакета - 1 байт адреса)
  pkt->payLen = rfmRegRead( REG_FIFO ) - 1;

  if( pkt->payLen > sizeof(uPayload) ){
  pkt->payLen = sizeof(uPayload);
  }

  // Считываем байт адреса
    pkt->nodeAddr = rfmRegRead( REG_FIFO );
  // Считываем полученные данные
  for(   uint8_t i = 0; i < pkt->payLen; i++ ){
    pkt->payBuf[i] = rfmRegRead( REG_FIFO );
  };

  return pkt->payLen;
}

void rfmRecvStop( void ){
  // Выключаем RFM69
  rfmSetMode_s( REG_OPMODE_SLEEP );
  // Если в FIFO осталось что-то - в мусор...
  while( dioRead(DIO_RX_FIFONE) ){
    rfmRegRead( REG_FIFO );
  }
}

uint16_t channelSearch( uint16_t count ){
  uint16_t ch;
  uint8_t rssi;
  uint8_t bestRssi = 0xFF;
  uint16_t bestCh = 0;

  for( ch = 0; ch < count; ch++ ){
    rfmChannelSet( ch );
    mDelay(3);
    if(dioRead( DIO_RX_RSSI ) != 0){
      if( (rssi = rfmRegRead( REG_RSSI_VAL )) < bestRssi ){
        bestRssi = rssi;
        bestCh = ch;
      }
    }
  }
  rfmChannelSet( bestCh );
  mDelay(3);
  return bestCh;
}

// Инициализация выводов DIO_RFM
static inline void dioInit( void ){

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  //---- Инициализация выводов для RFM_RST: Выход, 400кГц, ОК, без подтяжки ---
  RFM_RST_PORT->OSPEEDR &= ~(0x3 << (RFM_RST_PIN_NUM * 2));
  RFM_RST_PORT->PUPDR &= ~(0x3<< (RFM_RST_PIN_NUM * 2));
  RFM_RST_PORT->PUPDR |= 0x2<< (RFM_RST_PIN_NUM * 2);
  RFM_RST_PORT->MODER = (RFM_RST_PORT->MODER &  ~(0x3<< (RFM_RST_PIN_NUM * 2))) |
                         (0x1<< (RFM_RST_PIN_NUM * 2));


  //---- Инициализация выводов для DIO0 - DIO5 RFM69: вход, 2МГц, без подтяжки

  //Dio0 - PA0, Dio1 - PA1, Dio2 - PA2, Dio3 - PA3, Dio4 - PA6
  GPIOA->OTYPER &= ~(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6);
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(0x3 | (0x3 << (1 * 2)) | (0x3 << (2 * 2)) | (0x3 << (3 * 2)) | (0x3 << (6 * 2)) )) |
        (0x1 | (0x1 << (1 * 2)) | (0x1 << (2 * 2)) | (0x1 << (3 * 2)) | (0x1 << (6 * 2)) );
  GPIOA->PUPDR &= ~(0x3 | (0x3 << (1 * 2)) | (0x3 << (2 * 2)) | (0x3 << (3 * 2)) | (0x3 << (6 * 2)) );
  GPIOA->MODER &= ~(0x3 | (0x3 << (1 * 2)) | (0x3 << (2 * 2)) | (0x3 << (3 * 2)) | (0x3 << (6 * 2)) );

  // Dio5 - PB2
  GPIOB->OTYPER &= ~(GPIO_Pin_2);
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0x3 << (2 * 2))) | (0x1 << (2 * 2));
  GPIOB->PUPDR &= ~(0x3 << (2 * 2));
  GPIOB->MODER &= ~(0x3 << (2 * 2));

  // Инициализация прерывания от DIO0 и DI03
  // Select Dio0-Port for Dio0-Pin extended interrupt by writing 0000 in EXTI0
  SYSCFG->EXTICR[DIO0_PIN_NUM / 4] &= (uint16_t)~(0xF << (DIO0_PIN_NUM * 4));
  // Select Dio0-Port for Dio0-Pin extended interrupt by writing 0000 in EXTI0
  SYSCFG->EXTICR[DIO3_PIN_NUM / 4] &= (uint16_t)~(0xF << (DIO3_PIN_NUM * 4));
  SYSCFG->EXTICR[DIO3_PIN_NUM / 4] |= (uint16_t)( DIO3_PORT_NUM << (DIO3_PIN_NUM * 4) );

  // Configure the corresponding mask bit in the EXTI_IMR register
  EXTI->IMR |= (DIO0_PIN);
  // Configure the Trigger Selection bits of the Interrupt line on rising edge
  EXTI->RTSR |= (DIO0_PIN | DIO3_PIN );
  // ----------- Configure NVIC for Extended Interrupt --------
  NVIC_EnableIRQ( DIO0_EXTI_IRQn );
  NVIC_SetPriority( DIO0_EXTI_IRQn, 1 );

  NVIC_EnableIRQ( DIO3_EXTI_IRQn );
  NVIC_SetPriority( DIO3_EXTI_IRQn, 1 );

}

static inline void rfDataInit( void ){
  uint16_t tmp;

  // Инициализация структуры Rfm
  rfm.mode = MODE_STDBY;
  // Считываем из EEPROM параметры
  if( (tmp = eeBackup.rfmNetId) == 0){
    // В еепром ничего не записанно
    rfm.netId = eeBackup.rfmNetId = NET_ID;
    rfm.channel = eeBackup.rfmChannel = CHANN_DEF;
    rfm.nodeAddr = eeBackup.rfmNodeAddr = NODE_ADDR;
    rfm.txPwr = eeBackup.rfmTxPwr = TX_PWR_10;
  }
  else {
    rfm.netId = tmp;
    rfm.nodeAddr = eeBackup.rfmNodeAddr;
    rfm.channel = eeBackup.rfmChannel;
    rfm.txPwr = ((tmp=eeBackup.rfmTxPwr) > TX_PWR_10)? TX_PWR_10 : tmp;
  }
}

static inline void rfmRegSetup( void ){
  //  Усыпляем радиомодуль
  rfmRegWrite( REG_OPMODE,  REG_OPMODE_SLEEP );
  while( (rfmRegRead(REG_FLAG1) & REG_IF1_MODEREADY) != REG_IF1_MODEREADY)
  {}

  // Калибровка RC-генратора
  rfmRcCal();

  // Настройка DIO:
  // DIO0 - 0b00  // RX- CrcOk, TX- PacketSent
  // DIO1 - 0b01  // RX- FifoFull, TX- FifoFull
  // DIO2 - 0b00  // RX- FifoNotEmpty, TX- FifoNotEmpty
  // // DIO3 - 0b10  // RX- SyncAddr, TX - ----
  // DIO3 - 0b01  // RX- RSSI, TX - ----
  // DIO4 - 0b01  // RX- RSSI, TX - ----
  // DIO5 - 0b11  // RX- ModeReady, TX - ModeReady
  rfmRegWrite( REG_DIO_MAP1, 0x11 );
  rfmRegWrite( REG_DIO_MAP2, 0x77 );

  // -------------- Bitrate ------------------------
    rfmRegWrite( REG_BR_MSB, 0x05 );    // 25000 bit/s
    rfmRegWrite( REG_BR_LSB, 0x00 );    //
    // Настройка девиации частоты
    rfmRegWrite( REG_FDEV_MSB, 0x01 );
    rfmRegWrite( REG_FDEV_LSB, 0x3F );  // 19470 Hz
    // Настройка BW-фильтра
    rfmRegWrite( REG_RX_BW, 0x44 );     // 31300 Hz
    // Настройка AFC Bw
    rfmRegWrite( REG_AFC_BW, 0x8A );    // 100000 Hz

    // rfmRegWrite( REG_AFCFEI, REG_AFCFEI_AFC_AUTO );

    // Установка частоты несущей
    rfmChannelSet( rfm.channel );
    // Настройка усилителя приемника: Вх. = 200 Ом, Усиление - AGC
    rfmRegWrite( REG_LNA, 0x80 );


  // Настройка усилителя передатчика PA0 - выкл, PA1 - вкл.  Мощность - +10 дБм: (-18 + 28)
  rfmRegWrite( REG_PA_LVL, 0x40 | (TX_PWR_10) );
  // Настройка Sync-последовательности: Sync(NetID - вкл), 2 байта, (Net ID = 0x0101)
  rfmRegWrite( REG_SYNC1, (uint8_t)(rfm.netId >> 8) );
  rfmRegWrite( REG_SYNC2, (uint8_t)rfm.netId );
  rfmRegWrite( REG_SYNC_CFG, 0x88 );

  // Запись адреса нода - 0x22 и широковещательный адрес -0xFF
  rfmRegWrite( REG_NODE_ADDR, rfm.nodeAddr );
  rfmRegWrite( REG_BRDCAST, BRDCAST_ADDR );

  // Настройка пакета: Длина пакета - переменная, CRC - вкл, фильтрация адресов: адресс нода + широковещательный
  rfmRegWrite( REG_PACK_CFG, REG_PACK_CFG_VAR | REG_PACK_CFG_CRCON | REG_PACK_CFG_ADDRBRD);

  // Настройка минимальной рабочей границы  RSSI ( -114дБ )
  // rfmRegWrite( REG_RSSI_THRESH, 0xE4 );
  // Настройка минимальной рабочей границы  RSSI ( -108дБ )
  rfmRegWrite( REG_RSSI_THRESH, 0xD8 );

  // Передача начинается сразу по условию: В FIFO есть данные и установлен TX-режим
  rfmRegWrite( REG_FIFO_THRESH, 0x8F );
  // Настройка DAGC
  rfmRegWrite( REG_TEST_DAGC, 0x30 );
}
