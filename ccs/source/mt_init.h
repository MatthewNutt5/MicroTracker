/* 
 * mt_init.h - Header file for MicroTracker initialization routines.
 */

#ifndef mt_init_include
#define mt_init_include

#include <stdint.h>
#include <ti/devices/msp/msp.h>

#define SW1 ((uint32_t) 1 << 31) // PA31
#define SYNC_B ((uint32_t) 1 << 16) // PA16
#define SCK ((uint32_t) 1 << 17) // PA17
#define PICO ((uint32_t) 1 << 18) // PA18



/*
 * Initializes clocks and BOR.
 */
void InitializeProcessor(void);

/*
 * Initializes GPIOs for SPI, buttons, and bit-banging.
 */
void InitializeGPIO(void);

/*
 * Initializes Timer into repeated countdown mode, with interrupts.
 * The timer uses a 1 MHz clock, so the frequency is 1M / (load+1).
 */
void InitializeTimerInterrupt(GPTIMER_Regs *Timer, uint32_t load);

/*
 * Initializes Timer into repeated countdown mode.
 * The timer uses a 1 MHz clock, so the frequency is 1M / (load+1).
 */
void InitializeTimerOscillator(GPTIMER_Regs *Timer, uint32_t load);

/*
 * Initializes SPI1 for output.
 */
//void InitializeSPI1(void);

/*
 * Writes a 16-bit SPI message to the onboard DAC.
 */
void writeDAC(uint16_t message);



#endif // mt_init_include
