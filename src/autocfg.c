/*
 * autocfg.c
 *
 *  Created on: 03 сент. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

/*
 * Набор функций для реализации автоматической сетевой конфигурации
 */

#include "stm32l0xx.h"

#include "main.h"
#include "rfm69.h"
#include "node_addr.h"
#include "process.h"
#include "autocfg.h"

// Таймаут отправки запросов с восстановленной сетевой конфигурацией
volatile tUxTime rcfgTout;
// Таймаут отправки запросов с заводской сетевой конфигурацией
volatile tUxTime fcfgTout;
// Таймаут отправки запросов с восстановленной сетевой конфигурацией
volatile uint32_t cfgCount;

void acfgRtcHandler( void ){

	if( RTC->ISR & RTC_ISR_ALRAF ){
      // Alarm B interrupt
    //Clear ALRBF
    RTC->ISR &= ~RTC_ISR_ALRBF;
    uxTime = getRtcTime();
    if(state == STAT_READY){
    	// TODO: Отправка очередного запроса "Установка связи"
    }
    // Стираем флаг прерывания EXTI
    EXTI->PR &= EXTI_PR_PR17;
  }
}

/*
 *  Восстановление сетвой конфигурации из EEPROM или
 *  "Заводские установки", если в EEPROM их нет
 */
uint8_t acfgRestoreNet( void ){
	uint8_t netCfgFlag;
	uint16_t tmp;
	uint8_t tmp2;
	uint8_t tmp3;

  // Считываем из EEPROM параметры
  if( (((tmp = eeBackup.rfmNetId) == 0) || (tmp == NET_ID)) ||
  		((tmp2 = eeBackup.rfmChannel) > CHANN_MAX) ||
  		((tmp3 = eeBackup.rfmNodeAddr) == 0)){
    // В еепром ничего не записанно
    rfm.netId = NET_ID;
    rfm.channel = CHANN_DEF;
    rfm.nodeAddr = NODE_ADDR;
    rfm.txPwr = TX_PWR_10;
    netCfgFlag = FACT_NETCFG;
  }
  else {
    rfm.netId = tmp;
    rfm.channel = tmp2;
    rfm.nodeAddr = tmp3;
    rfm.txPwr = ((tmp = eeBackup.rfmTxPwr) > TX_PWR_10)? TX_PWR_10 : tmp;
    netCfgFlag = WORK_NETCFG;
  }

  return netCfgFlag;
}

void acfgInit( void ){
  if( flags.netCfg == WORK_NETCFG ){
    uxTime = getRtcTime();
    // Устанавливаем границы времени
    rcfgTout = uxTime + RCFG_TOUT;
    fcfgTout = uxTime + FCFG_TOUT;
    cfgCount = ACFG_COUNT;
  }
  else {
    // Безгранично
    rcfgTout = ~0;
    fcfgTout = ~0;
    cfgCount = ~0;
  }
  query = QUERY_CONNECT;

}


// Установка (обновление) сетевой конфигурации
void rfCfg( void ){
  acfgInit();

  // Отправляем первый пакет с запросом соединения
  cnctProcess();
  alrmOn( ALRM_A);
  // Вся работа ведется в прерываниях
  while( flags.connect == FALSE ){
#if STOP_EN
    __WFI();
#endif
  }
}

//
//void sensDataSend( void ){
//  // ---- Формируем пакет данных -----
//  pkt.paySensType = SENS_TYPE_TO;
//  pkt.paySrcNode = rfm.nodeAddr;
//  pkt.payMsgNum = 1;
//  pkt.payBat = sensData.bat;
//// XXX: Автоматическая настройка сетевой конфигурации
//  if(pkt.payMsgType == MSG_TYPE_QUERY){
//    pkt.payData = query;
//  }
//  else {
//    pkt.payData = sensData.volume;
//  }
//
//  // Передаем заполненую при измерении запись
//  pkt.nodeAddr = BCRT_ADDR;
//  // Длина payload = 1(nodeAddr) + 1(msgNum) + 1(bat) + 2(temp)
//  pkt.payLen = sizeof(tSensMsg);
//
//  rfmTransmit( &pkt );
//  // Таймаут до окончания передачи
//  wutSet( TX_DURAT*10 );
//  state = STAT_TX_START;
//}



void cnctProcess( void ){

  if( flags.netCfg == WORK_NETCFG ){
    // Установка соединение с рабочей конфигурацией
    if( uxTime > rcfgTout ){
      /* Закончился цикл попыток установки соединения с рабочей конфигурацией
       * Переходим на заводскую конфигурацию
       */
      rfmNetCfg( FACT_NETCFG );
      rfmNetSetup();
      fcfgTout = uxTime + FCFG_TOUT;

#if DEBUG_PIN
  // Преключаем дебажный вывод
  GPIOA->ODR ^= GPIO_Pin_11;
  mDelay(5);
  GPIOA->ODR ^= GPIO_Pin_11;
  mDelay(5);
  GPIOA->ODR ^= GPIO_Pin_11;
#endif

    }
  }
  else {
    // Устанавливка соединение с заводской конфигурацией
    if( uxTime > fcfgTout ){
      /* Закончился цикл попыток установки соединения с рабочей конфигурацией
       * По любому переходим на рабочую конфигурацию
       */
      rfmNetCfg( WORK_NETCFG );
      rfmNetSetup();
      if( --cfgCount == 0){
        // Закончились попытки установки связи - переходим в обычный рабочий режим
        connOk();
        return;
      }
      else {
        // Возвращаемся к попытке установки соединения с рабочей конфигурацией
        rcfgTout = uxTime + RCFG_TOUT;

#if DEBUG_PIN
  // Преключаем дебажный вывод
  GPIOA->ODR ^= GPIO_Pin_11;
  mDelay(5);
  GPIOA->ODR ^= GPIO_Pin_11;
  mDelay(5);
  GPIOA->ODR ^= GPIO_Pin_11;
#endif

      }
    }
  }

  pkt.payMsgType = MSG_TYPE_QUERY;

  csmaRun();

}

