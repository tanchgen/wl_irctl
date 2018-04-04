/*
 * my_time.h
 *
 *  Created on: 31 окт. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef MY_TIME_H_
#define MY_TIME_H_

#include <stdint.h>
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx.h"

#define ALARM_UPDATE_TOUT   100

#define ALRM_A      0   // НЕ МЕНЯТЬ !!!!
#define ALRM_B      1   // НЕ МЕНЯТЬ !!!!

#define RTCCLOCK     32768

typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t ss;     // Доля секунды = (PREDIV_S - SS) / (PREDIV_S + 1)
} tTime;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
} tDate;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t ss;     // Доля секунды = (PREDIV_S - SS) / (PREDIV_S + 1)
} tRtc;

#define TIMEZONE_MSK      (+3)

  // DEF: standard signed format
  // UNDEF: non-standard unsigned format
  #define _XT_SIGNED

#ifdef  _XT_SIGNED
  typedef int32_t                           tUxTime;
#else
  typedef uint32                          tUxTime;
#endif

extern volatile tRtc rtc;
extern volatile tUxTime uxTime;
extern volatile uint8_t sendToutFlag;

extern __IO uint32_t myTick;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void rtcInit(void);
void timeInit( void );
// Получение системного мремени
uint32_t getTick( void );
tUxTime xTm2Utime( volatile tRtc * prtc );

void timersProcess( void );

void timersHandler( void );
void mDelay( uint32_t del );

void setRtcTime( tUxTime xtime );
tUxTime getRtcTime( void );
void setAlrm( tUxTime xtime, uint8_t alrm );
void correctAlrm( uint8_t alrm );
void wutStop( void );
void wutSet( uint32_t us );


#endif /* UNIX_TIME_H_ */
