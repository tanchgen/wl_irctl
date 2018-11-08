//**************************************************
//* Разработано: "НИЛ АП",ООО, г. Таганрог.        *
//* Программирование: Танчин Г.В.                  *
//* Дата последней редакции программы: 29.06.2018г.*
//**************************************************
/*
 * main.c
 *
 *  Created on: 12 мая 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "my_time.h"
#include "bat.h"
#include "spi.h"
#include "rfm69.h"
#include "process.h"
#include "button.h"
#include "ir.h"
#include "ir_proto.h"
#include "main.h"

//volatile uint32_t mTick;

EEMEM tEeBackup eeBackup;     // Структура сохраняемых в EEPROM параметров
//tEeBackup eeBackup;         // Структура сохраняемых в EEPROM параметров

EEMEM __aligned(4) struct eeProtoBak eeIrProtoBak;


//EEMEM uint32_t wfiFaultCount;   // Для тестирования: Счетчик секунд работы не в состоянии STOP

volatile tDriveData driveData;    // Структура измеряемых датчиком параметров
volatile eState state;          // Состояние машины
volatile tFlags flags;          // Флаги состояний системы

// Места восстановления и сохранения состояния GPIO перед остановом
uint32_t GPIOA_MODER;
uint32_t GPIOB_MODER;
uint32_t GPIOC_MODER;

uint8_t risCount = 0;
uint8_t failCount = 0;

uint8_t pausePulse;

#if DEBUG_TIME

tDbgTime dbgTime;

#endif // DEBUG_TIME

/* Private function prototypes -----------------------------------------------*/
static inline void mainInit( void );
static inline void sysClockInit(void);
static inline void startPwrInit( void );
static inline void eepromUnlock( void );


void (*saveCtx)(void);
void (*restCtx)(void);

void acCtrlTest( void );

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  irTxPinInit();
  mainInit();
  sysClockInit();

/* Сначала конфигурируем необходимый минимум для того, чтобы усыпить контроллер,
 * а затем по нажатию КНОПКИ разбудить, настроить ВСЕ и включить в рабочее состояние
 */
  startPwrInit();
  // Разлочили EEPROM
  eepromUnlock();

  rtcStartInit();
  buttonInit();
  buzzerInit();

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  field0Num = irProtoRestore();

// ==============================================================================
  irRxInit();
  // Включаем обработку входа ИК-приемника
  EXTI->IMR |= IR_RX_PIN;

  saveCtx();
// Засыпаем до нажатия на кнопку
  while( btn.tOnSec == 0 ) {
#if STOP_EN
    __WFI();
#endif
  }
  restCtx();

  // Ждем определения протокола (обучения)
  while( field0Num == 0 )
  {}

  // Инициализация таймера Несущей сигнала для ПЕРЕДАЧИ по ИК
  irCarierTimInit();
  // Инициализация таймера Модулирующего сигнала для ПЕРЕДАЧИ по ИК
  irModulTxTimInit();


// ################## ДЛЯ ТЕСТИРОВАНИЯ ################################
//  acCtrlTest();
//
//  while(1){
//    // Включаем кондиционер
//    rxAcState.onoff = ON;
//    protoPktCod();
//    irPktSend();
//    mDelay(20000);
//    rxAcState.onoff = OFF;
//    irPktSend();
//    buzzerLongPulse();
//    mDelay(120000);
//  }
// ####################################################################

  rfmInit();
  batInit();

  // XXX: ---------- Автоматическая настройка сетевой конфигурации -------------
    // Первый запуск измерений
    mesure();
    // Начальная настройка RTC
    rtcStartInit();
    flags.netCfg = acfgRestoreNet();
    rfmInit();
    pwrStartInit();

  #if DEBUG_PIN
    GPIOA->ODR ^= GPIO_Pin_11;
  #endif
  //
  //  // ---- Формируем пакет данных -----
  //  pkt.payCmd = CMD_SENS_SEND;
  //  pkt.paySensType = SENS_TYPE_TO;
  //  pkt.paySrcNode = rfm.nodeAddr;
  //  pkt.payMsgNum = 1;
  //  pkt.payBat = sensData.bat;
  //  pkt.payData = sensData.volume;
  //
  //  // Передаем заполненую при измерении запись
  //  pkt.nodeAddr = BCRT_ADDR;
  //  // Длина payload = 1(nodeAddr) + 1(msgNum) + 1(bat) + 2(temp)
  //  pkt.payLen = sizeof(tSensMsg);
  //
  //  rfmTransmit_s( &pkt );

    rfCfg();

  // ---------- Конец автоматической настройки сетевой конфигурации ------------

  // В рабочем режиме включаем будильник
  timeInit();
#if STOP_EN
  // В рабочем режиме включаем засыпание по выходу из прерывания
  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
#endif
//  // Запустили измерения
//  mesure();
//  // Передаем состояние, как инициализационный пакет
//  csmaRun();

//  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE3) | GPIO_MODER_MODE3_0;
  saveCtx();
#if STOP_EN
  __WFI();
#endif

//  restoreContext();
  while (1){
//  	GPIOB->ODR ^= GPIO_Pin_3;
#if STOP_EN
//    wfiFaultCount++;
#endif
    mDelay(1000);
  }
}

static inline void mainInit( void ){
  // Power registry ON
  RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
  FLASH->ACR |= FLASH_ACR_PRE_READ;
  // Выключаем перрывание EXTI от I2C1, USART1
  EXTI->IMR &= ~(EXTI_IMR_IM23 | EXTI_IMR_IM25);
}


static inline void sysClockInit(void){
  // MSI range 4194 kHz
  RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | RCC_ICSCR_MSIRANGE_6;
  SysTick_Config( 4194 );

  SysTick->CTRL &= ~( SysTick_CTRL_TICKINT_Msk ); // no systick interrupt

//  // SysTick_IRQn interrupt configuration
//  NVIC_EnableIRQ( SysTick_IRQn );
//  NVIC_SetPriority(SysTick_IRQn, 0);
}

static inline void startPwrInit( void ){
  // Power range 3
	while( (PWR->CSR & PWR_CSR_VOSF) != 0 )
	{}
  PWR->CR |= PWR_CR_VOS;
	while( (PWR->CSR & PWR_CSR_VOSF) != 0 )
	{}
  //------------------- Stop mode config -------------------------
  // Stop mode
  PWR->CR &= ~PWR_CR_PDDS;
  // Clear PWR_CSR_WUF
  PWR->CR |= PWR_CR_CWUF;
  // MSI clock wakeup enable
  RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
  // Выключаем VREFIN при остановке + Быстрое просыпание:
  // не ждем, пока восстановится VREFIN, проверяем только при запуске АЦП
  PWR->CR |= PWR_CR_ULP | PWR_CR_FWU | PWR_CR_LPSDSR;
  // Interrupt-only Wakeup, DeepSleep enable, SleepOnExit enable
  // Начальная инициализация до включения контроллера
  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk;
//#if STOP_EN
//  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk;
//#else
//
//#endif // STOP_EN
  saveCtx = saveCntext;
  restCtx = restoreCntext;
}

static inline void eepromUnlock( void ){
  while ((FLASH->SR & FLASH_SR_BSY) != 0)
  {}
  if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0){
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
  }
}


/*******************************************************************************
* Function:			Idd_RestoreContext
* Author:				Matt Mielke
* Desctription:   This function uses the following global variables:
*               GPIOA_MODER, GPIOB_MODER, and GPIOC_MODER. Assuming the function
*               Idd_SaveContext was called before this one, the GPIOx_MODER
*               registers will be restored to their original values, taking the
*               GPIO pins out of analog mode. Interrupts should not be enabled
*               while this function is executing, which is why code is added at
*               the beginning and end to disable and reenable interrupts if they
*               weren't already disabled when this function was called.
* Date:         09-23-16
*******************************************************************************/
void restoreCntext(void){
#if STOP_EN
	// disable interrupts if they weren't already disabled
	__disable_irq();
		// Enable GPIO clocks
		RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

		GPIOA->MODER = GPIOA_MODER; // dummy write

		// Restore the previous mode of the I/O pins
		GPIOA->MODER = GPIOA_MODER;
		GPIOB->MODER = GPIOB_MODER;
	// enable interrupts if they were enabled before this function was called
	__enable_irq();
#endif //STOP_EN
}

void restoreCntext0(void)
{}

/*******************************************************************************
* Function:			Idd_SaveContext
* Author:				Matt Mielke
* Desctription:   In this function, the state of the GPIOx_MODER registers are
*               save into global variables GPIOA_MODER, GPIOB_MODER, and
*               GPIOC_MODER. The MODER registers are all then configured to
*               analog mode. This will prevent the GPIO pins from
*               consuming any current, not including the external interrupts.
*               Interrupts should not be enabled while this function is
*               executing which is why code is added at the beginning and end
*               to disable and reenable interrupts if they weren't already
*               disabled when this function was called.
* Date:         09-23-16
*******************************************************************************/
void saveCntext(void){
#if STOP_EN

	// disable interrupts
	__disable_irq();

		// Enable GPIO clocks
		RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

		GPIOA_MODER = GPIOA->MODER;  // dummy read

		// Save the current mode of the I/O pins
		GPIOA_MODER = GPIOA->MODER;
		GPIOB_MODER = GPIOB->MODER;
		GPIOC_MODER = GPIOC->MODER;

		// Configure GPIO port pins in Analog Input mode
		// PA0 - DIO0 interrupt
		GPIOA->MODER = 0x2BFF300C;
		GPIOB->MODER |= 0xFFFFCF3F;

		// Disable GPIO clocks
		RCC->IOPENR &= ~( RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN );
	__enable_irq();
#endif // STOP_EN
}

void saveCntext0(void)
{}

void acCtrlTest( void ){
  rxAcState = acState;
  for(uint8_t i = 0; i < 32; i++ ){
    switch (i){
      case 0:
        // Включаем кондиционер
        rxAcState.onoff = OFF;
        break;
      case 1:
        // Выключаем кондиционер
        rxAcState.onoff = ON;
        break;
      case 2 ... 6:
        // Перебераем режимы MODE
        rxAcState.mode = i - 2;
        break;
      case 7 ... 21:
        rxAcState.mode = 1;
        // Перебераем температуру 16-30 гр.Ц
        rxAcState.temp = i - 7;
        break;
      case 22 ... 25:
        // Восстанавливаем температуру 23 гр.Ц
        rxAcState.temp = 7;
        // Перебераем скорость вентилятора
        rxAcState.fan = i - 22;
        break;
      case 27 ... 31:
        // Перебераем скорость вентилятора
        rxAcState.fan = 0;
        // Перебераем положение задвижки
        rxAcState.swing = i - 27;
        break;
      default:
        break;
    }
    // Отправляем пакет на ИК
    protoPktCod();
    irPktSend();
    mDelay(5000);
  }
}
// ----------------------------------------------------------------------------
