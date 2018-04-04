/*
 * i2c.c
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx_ll_i2c.h"

#include "main.h"
#include "my_time.h"
#include "light.h"

static uint8_t vh1750Write( uint8_t op );
static uint32_t vh1750Read( uint8_t *rxBuffer );

static inline void i2cGpioInit(void) {
  // SCL - PB6, SDA - PB7
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

  // PullUp for I2C signals
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) \
                 | (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
  // Open drain for I2C signals
  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
  // AF1 for I2C signals
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~((0x0FL << ( 6 * 4 )) | (0x0FL << ( 7 * 4 )))) \
                  | (1 << ( 6 * 4 )) | (1 << (7 * 4));
  //Select AF mode (10) on PB6 and PB7
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7)) \
                 | (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
}


static inline void i2cInit( void ){
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  // Use APBCLK for I2C CLK
  RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;

  // Сначала отключаем I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
  // Расчитанно из STM32CubeMX
  I2C1->TIMINGR = 0x00000004L;

  // Адрес ведомого
  I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_SADD) | (VH1750_ADDR << 1);
#define I2C1_OWN_ADDRESS (0x00)
  I2C2->OAR1 = (I2C1_OWN_ADDRESS<<1);


  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

//  NVIC_SetPriority(I2C1_IRQn, 0);
//  NVIC_EnableIRQ(I2C1_IRQn);
}

void lightInit( void ){
  i2cGpioInit();
  i2cInit();
  // Инициализация вывода DVI
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

  //---- Инициализация выводов для RFM_RST: Выход, 400кГц, ОК, без подтяжки ---
  DVI_PORT->OSPEEDR &= ~(0x3 << (DVI_PIN_NUM * 2));
  DVI_PORT->PUPDR &= ~(0x3<< (DVI_PIN_NUM * 2));
  DVI_PORT->MODER = (DVI_PORT->MODER &  ~(0x3<< (DVI_PIN_NUM * 2))) | (0x1<< (DVI_PIN_NUM * 2));
//  // Включаем DVI
//  DVI_PORT->BSRR |= DVI_PIN;
//  vh1750Write( VH1750_POWER_DOWN );

  vh1750Stop();
  // Ждем >1мкс Для тактовой частоты 4192МГц
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

void vh1750Stop( void ){

  // Выключаем VH1750
  DVI_PORT->BRR |= DVI_PIN;
	// Подтягиваем вывод вниз
  DVI_PORT->PUPDR |= 0x2<< (DVI_PIN_NUM * 2);
  // Переключаем в режим входа
  DVI_PORT->MODER = (DVI_PORT->MODER ^ (0x1<< (DVI_PIN_NUM * 2)));

  // Отмечаем запуск измерения
#if DEBUG_TIME
	dbgTime.thermoStart = mTick;
#endif // DEBUG_TIME

}

void lightStart( void ){

  // Включаем VH1750
  DVI_PORT->BSRR |= DVI_PIN;
  // Переключаем в режим выхода
  DVI_PORT->MODER = (DVI_PORT->MODER ^ (0x1<< (DVI_PIN_NUM * 2)));
  // Выключаем подтяжку вывода
  DVI_PORT->PUPDR &= ~(0x2<< (DVI_PIN_NUM * 2));

  // Отмечаем запуск измерения
#if DEBUG_TIME
	dbgTime.thermoStart = mTick;
#endif // DEBUG_TIME

  // Отправляем команду начать измерение
  if( vh1750Write( CMD_MESURE ) != 1){
  	flags.sensErr = SET;
  	sensData.volume = 0;
  }
  else {
  	flags.sensErr = RESET;
  }
  state = STAT_L_MESUR;

  // Уснуть на время преобразования, мкс макс: Hi-Resolution - 180мс, Lo-Resolution - 24мс
  wutSet( CONV_TOUT );
}

// Читаем освещенность
uint16_t vh1750LsRead( void ){
  union{
    uint8_t u8[2];
    uint16_t u16;
  } lsBuf;

  if( vh1750Read( lsBuf.u8 ) != 0) {
    lsBuf.u16 = 0;
    flags.sensErr = SET;
  }
  else {
    // Пересчет значения: коэфф. = 1/1.2
    lsBuf.u16 = lsBuf.u16 * 10 / 12;
    flags.sensErr = RESET;
  }

  // Отмечаем останов измерения
#if DEBUG_TIME
	dbgTime.thermoEnd = mTick;
#endif // DEBUG_TIME

  return ( (uint16_t)lsBuf.u16 );
}


// Прием двух байт от VH1750
static uint32_t vh1750Read( uint8_t *rxBuffer ) {
  uint8_t numRxBytes = 2;

  // I2C_CR2_NBYTES = 2
  I2C1->CR2 = (2 << 16) | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | (VH1750_ADDR << 1);

  I2C1->CR1 |= I2C_CR1_PE;
  I2C1->CR2 |= I2C_CR2_START;

  while(I2C1->CR2 & I2C_CR2_START);

  for( uint32_t i = 0; i < 0x0000001F; i++) {

    if (I2C1->ISR & I2C_ISR_RXNE) {
      // Device acknowledged and we must send the next byte
      if (numRxBytes > 0){
        rxBuffer[--numRxBytes] = I2C1->RXDR;
      }
      i = 0;
    }

    if( (I2C1->ISR & I2C_ISR_BUSY) == 0) {
      break;
    }

  }
  // Выключаем I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
  return numRxBytes;
}


uint8_t lightEnd( void ){
  if( flags.sensErr == 0){
    sensData.volume = vh1750LsRead();
  }
  flags.sensCplt = SET;
  state = STAT_L_CPLT;

  vh1750Stop();
  return flags.sensErr;
}


static uint8_t vh1750Write( uint8_t op ){
  uint8_t numTxBytes = 0;

  // I2C_CR2_NBYTES = 1, Slave address = VH1750_ADDR, AUTOEND - On
  I2C1->CR2 = (1 << 16) | (VH1750_ADDR << 1) | I2C_CR2_AUTOEND;

  I2C1->CR1 |= I2C_CR1_PE;
  I2C1->CR2 |= I2C_CR2_START;
  while(I2C1->CR2 & I2C_CR2_START);

  // Цикл передачи num байт в I2C
  // i - индекс таймаута...
  for( uint32_t i = 0; i < 0x0000001F; i++) {
    if (I2C1->ISR & I2C_ISR_NACKF) {
      // Was not acknowledged, disable device and exit
      break;
    }

    if (I2C1->ISR & I2C_ISR_TC) {
      // Передачу закончили - выходим
      break;
    }

    if (I2C1->ISR & I2C_ISR_TXIS) {
      // Device acknowledged and we must send the next byte
      if (numTxBytes == 0){
        I2C1->TXDR = op;
        numTxBytes++;
      }
      // Сбрасываем таймаут
      i = 0;
    }
  }
  I2C1->CR1 &= ~I2C_CR1_PE;
  return numTxBytes;
}

void thermoIrqHandler( void ){
  __NOP();
}

#if 0
static void thermoErrHandler( void ){
  flags.sensErr = SET;
  sensData.volume = 0xFF00;
  // Сброс I2C
  I2C1->CR1 &= ~I2C_CR1_PE;
  // APB2 clock == system clock
  __NOP(); __NOP(); __NOP();
  I2C1->CR1 |= I2C_CR1_PE;
}
#endif
