/*
 * autocfg.h
 *
 *  Created on: 04 сент. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef AUTOCFG_H_
#define AUTOCFG_H_

#include "my_time.h"

#define ACFG_TX_TOUT_SEC  2
#define ACFG_TX_TOUT_MIN  1
#define RCFG_TOUT         30
#define FCFG_TOUT         300
#define ACFG_COUNT        6

// Таймаут отправки запросов с восстановленной сетевой конфигурацией
extern volatile tUxTime rcfgTout;
// Таймаут отправки запросов с заводской сетевой конфигурацией
extern volatile tUxTime fcfgTout;
// Таймаут отправки запросов с восстановленной сетевой конфигурацией
extern volatile uint32_t cfgCount;

uint8_t acfgRestoreNet( void );
void rfCfg( void );
void cnctProcess( void );

#endif /* AUTOCFG_H_ */
