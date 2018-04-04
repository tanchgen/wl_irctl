/*
 * spi.h
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32l0xx.h"

void spiInit(void);
// Передача по SPI в блокирующем режима
int8_t spiTrans_s( uint8_t *buf, uint8_t len );
// Прием по SPI в блокирующем режима
int8_t spiRecv_s( uint8_t *buf, uint8_t len );
// Передача с одновременным приемом по SPI в блокирующем режима
int8_t spiTransRecv_s( uint8_t *txBuf, uint8_t *rxBuf, uint8_t len );

#endif /* SPI_H_ */
