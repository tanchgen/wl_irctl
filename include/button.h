/*
 * button.h
 *
 *  Created on: 27 апр. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef BUTTON_H_
#define BUTTON_H_

typedef enum {
  BTN_ON = 0,           // Нажатие кнопки - на выводе низкий уровень
  BTN_OFF = 1,           // Отжатие кнопки - на выводе высокий уровень
} eBtn;

typedef struct {
  eBtn stat;            // Состояние кнопки
  uint32_t tOnSec;      // Время нажатия - секунды
  uint32_t tOnSS;       // Время нажатия - субсекунды
  uint32_t tOffSec;     // Время отжатия - секунды
  uint32_t tOffSS;      // Время нажатия - субсекунды
  uint8_t pressCnt;     // Счетчик нажатий
  uint8_t longPressCnt; // Счетчик долгих нажатий
} tButton;

extern tButton btn;

void buttonInit( void );
void buttonProcess( uint32_t ut );
void buzzerInit( void );
void buzzerShortPulse( void );
void buzzerLongPulse( void );

#endif /* BUTTON_H_ */
