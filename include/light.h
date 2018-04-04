/*
 * light.h
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef THERMO_H_
#define THERMO_H_

#include "stm32l0xx.h"
#include "gpio.h"

// Датчик освещенности BH1750

#define VH1750_ADDR     0x23      // I2C-Адрес датчика при ADDR = LOW без учета сдвига под R/NW-бит
//#define VH1750_ADDR     0x5с      // I2C-Адрес датчика при ADDR = HIGH без учета сдвига под R/NW-бит

// Код операции
#define	VH1750_POWER_DOWN 		(uint8_t)0x00	// Режим Power_Down
#define	VH1750_POWER_ON 			(uint8_t)0x01	// Режим Power_On
#define	VH1750_RESET					(uint8_t)0x07	// Сброс регистра данных
#define	VH1750_CONT_HI				(uint8_t)0x10	// Запуск постоянного измерения в Hi (1люкс	x 120мс)
#define	VH1750_CONT_HI2			(uint8_t)0x11	// Запуск постоянного измерения в Hi2 (0.5люкс	x 120мс)
#define	VH1750_CONT_LO				(uint8_t)0x13	// Запуск постоянного измерения в Lo (4люкс	x 16мс)
#define	VH1750_OSH_HI				(uint8_t)0x20	// Запуск разового измерения в Hi (1люкс	x 120мс)
#define	VH1750_OSH_HI2				(uint8_t)0x21	// Запуск разового измерения в Hi2 (0.5люкс	x 120мс)
#define	VH1750_OSH_LO				(uint8_t)0x23	// Запуск разового измерения в Lo (4люкс	x 16мс)
#define	VH1750_TIME_HI				(uint8_t)0x40 // Маска изменения длительности замера (старший байт)
#define	VH1750_TIME_LO				(uint8_t)0x40 // Маска изменения длительности замера (младший байт)

#define CMD_MESURE    VH1750_OSH_HI
#define CONV_TOUT     180000  // Задание времени преобразования: HI-RESOL 180 мс, LO-RESOL 24 мс


// Выводы и порты
// Номера пинов DIO
#define DVI_PIN        GPIO_Pin_5
#define DVI_PIN_NUM    5
#define DVI_PORT       GPIOB
#define DVI_PORT_NUM   1

void lightInit( void );
uint8_t lightEnd( void );

// Останов BH1750VHI
void vh1750Stop( void );
// Запуск замера освещенности
void lightStart( void );
// Считывание измеренной  освещенности
uint16_t vh1750LsRead( void );

void lightIrqHandler( void );

#endif /* THERMO_H_ */
