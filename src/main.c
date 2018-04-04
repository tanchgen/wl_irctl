// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "my_time.h"
#include "bat.h"
#include "light.h"
#include "spi.h"
#include "rfm69.h"
#include "process.h"
#include "main.h"

//volatile uint32_t mTick;

EEMEM tEeBackup eeBackup;     // Структура сохраняемых в EEPROM параметров
//tEeBackup eeBackup;             // Структура сохраняемых в EEPROM параметров

EEMEM uint32_t wfiFaultCount;   // Для тестирования: Счетчик секунд работы не в состоянии STOP

volatile tSensData sensData;    // Структура измеряемых датчиком параметров
volatile eState state;          // Состояние машины
volatile tFlags flags;          // Флаги состояний системы

// Места восстановления и сохранения состояния GPIO перед остановом
uint32_t GPIOA_MODER;
uint32_t GPIOB_MODER;
uint32_t GPIOC_MODER;

//extern uint8_t regBuf[];

#if DEBUG_TIME

tDbgTime dbgTime;

#endif // DEBUG_TIME

/* Private function prototypes -----------------------------------------------*/
static inline void mainInit( void );
static inline void sysClockInit(void);
static inline void pwrInit( void );
static inline void eepromUnlock( void );

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  mainInit();
  sysClockInit();
  // Разлочили EEPROM
  eepromUnlock();

  batInit();
  lightInit();
  rfmInit();

  pwrInit();
  timeInit();
  // Запустили измерения
  mesureStart();
//  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE3) | GPIO_MODER_MODE3_0;
  saveContext();
#if STOP_EN
  __WFI();
#endif

//  restoreContext();
  // Infinite loop
  while (1){
//  	GPIOB->ODR ^= GPIO_Pin_3;
#if STOP_EN
    wfiFaultCount++;
#endif
    mDelay(1000);
  }
  // Infinite loop, never return.
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

static inline void pwrInit( void ){
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
#if STOP_EN
  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk;
#else
  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk;// | SCB_SCR_SLEEPONEXIT_Msk;
#endif // STOP_EN
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
void restoreContext(void){
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
void saveContext(void){
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
		GPIOA->MODER = 0xEBFF30FC;
		GPIOB->MODER |= 0xFFFFF3FF;

		// Disable GPIO clocks
		RCC->IOPENR &= ~( RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN );
	__enable_irq();
#endif // STOP_EN
}


// ----------------------------------------------------------------------------
