/*
 * mt_init.c - Source file for MicroTracker initialization routines.
 */

#include "mt_init.h"
#include <ti/devices/msp/msp.h>
#include "mt_low.h"



void InitializeProcessor(void) {

    SYSCTL->SOCLOCK.BORTHRESHOLD = SYSCTL_SYSSTATUS_BORCURTHRESHOLD_BORMIN; // brownout generates a reset

    update_reg(&SYSCTL->SOCLOCK.MCLKCFG, (uint32_t) 0x0, SYSCTL_MCLKCFG_MDIV_MASK); // disable MCLK Divider
    update_reg(&SYSCTL->SOCLOCK.MCLKCFG, (uint32_t) SYSCTL_MCLKCFG_UDIV_NODIVIDE, SYSCTL_MCLKCFG_UDIV_MASK); // set UPCLK divider
    update_reg(&SYSCTL->SOCLOCK.MCLKCFG, (uint32_t) SYSCTL_MCLKCFG_USEMFTICK_ENABLE, SYSCTL_MCLKCFG_USEMFTICK_MASK); // enable MFCLK
    update_reg(&SYSCTL->SOCLOCK.SYSOSCCFG, SYSCTL_SYSOSCCFG_FREQ_SYSOSCBASE, SYSCTL_SYSOSCCFG_FREQ_MASK); // set SYSOSC to 32 MHz

}



void InitializeGPIO(void) {

    GPIOA->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR | GPIO_RSTCTL_RESETASSERT_ASSERT);
    GPIOA->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);

    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

    // initialize SPI1 connections
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM39)] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM39_PF_SPI1_SCLK; // SPI1_SCLK on PA17
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM40)] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM40_PF_SPI1_PICO; // SPI1_PICO on PA18
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM58)] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM58_PF_SPI1_CS1_POCI1; // SPI1_CS on PB27

    // initialize GPIO input connections
    uint32_t input_configuration = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE |
            ((uint32_t) 0x00000001) | // GPIO function is always MUX entry 1
            IOMUX_PINCM_INV_DISABLE | // do not invert logic
            IOMUX_PINCM_PIPU_DISABLE | IOMUX_PINCM_PIPD_DISABLE | // no pullup/pulldown
            IOMUX_PINCM_HYSTEN_DISABLE | // disable input pin hysteresis
            IOMUX_PINCM_WUEN_DISABLE;  // wake-up disable

    IOMUX->SECCFG.PINCM[(IOMUX_PINCM6)] = input_configuration; // "Enter" button on PA31

    // initialize GPIO output connections
    uint32_t output_configuration = IOMUX_PINCM_PC_CONNECTED | ((uint32_t) 0x00000001);

    IOMUX->SECCFG.PINCM[(IOMUX_PINCM38)] = output_configuration; // ~SYNC on PA16
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM39)] = output_configuration; // SCK on PA17
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM40)] = output_configuration; // PICO on PA18
    GPIOA->DOUTCLR31_0 = (SYNC_B | SCK | PICO);
    GPIOA->DOESET31_0 = (SYNC_B | SCK | PICO);

    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

}



void InitializeTimerInterrupt(GPTIMER_Regs *Timer, uint32_t load) {

    Timer->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR | GPIO_RSTCTL_RESETASSERT_ASSERT);
    Timer->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

    // configure clocking for the module
    Timer->CLKSEL = GPTIMER_CLKSEL_MFCLK_SEL_ENABLE; // mid-freq clock, 8 MHz, 1% accuracy
    Timer->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_8; // divide by 8 --> 1 MHz

    // configure module logic
    // CM_DOWN - count down mode; REPEAT_1 - keep going when we reach zero;
    // CVAE_LDVAL - when we get to zero, reload LOAD value; EN_DISABLED - start with timer disabled
    Timer->COUNTERREGS.CTRCTL =
            (GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1 | GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_EN_DISABLED);
    Timer->COUNTERREGS.LOAD = load; // period is LOAD+1
    Timer->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED); // enable counter

    Timer->CPU_INT.IMASK |= GPTIMER_CPU_INT_IMASK_Z_SET; // enable timer interrupt when we reach 0
    Timer->PDBGCTL = GPTIMER_PDBGCTL_SOFT_IMMEDIATE; // stop immediately on debug freeze

    Timer->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED; // enable clocking to the module

}



void InitializeTimerOscillator(GPTIMER_Regs *Timer, uint32_t load) {

    Timer->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR | GPIO_RSTCTL_RESETASSERT_ASSERT);
    Timer->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

    // configure clocking for the module
    Timer->CLKSEL = GPTIMER_CLKSEL_MFCLK_SEL_ENABLE; // mid-freq clock, 8 MHz, 1% accuracy
    Timer->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_8; // divide by 8 --> 1 MHz

    // configure module logic
    // CM_DOWN - count down mode; REPEAT_1 - keep going when we reach zero;
    // CVAE_LDVAL - when we get to zero, reload LOAD value; EN_DISABLED - start with timer disabled
    Timer->COUNTERREGS.CTRCTL =
            (GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1 | GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_EN_DISABLED);
    Timer->COUNTERREGS.LOAD = load; // period is LOAD+1
    Timer->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED); // enable counter

    Timer->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED; // enable clocking to the module

}



//void InitializeSPI1(void) {
//
//    SPI1->GPRCM.RSTCTL = (SPI_RSTCTL_KEY_UNLOCK_W | SPI_RSTCTL_RESETSTKYCLR_CLR | SPI_RSTCTL_RESETASSERT_ASSERT);
//    SPI1->GPRCM.PWREN = (SPI_PWREN_KEY_UNLOCK_W | SPI_PWREN_ENABLE_ENABLE);
//    delay_cycles(POWER_STARTUP_DELAY); // delay to enable SPI to turn on and reset
//
//    // configure clocking for SPI1
//    SPI1->CLKSEL = (uint32_t) SPI_CLKSEL_SYSCLK_SEL_ENABLE; // use the SYSOSC
//    SPI1->CLKDIV = (uint32_t) SPI_CLKDIV_RATIO_DIV_BY_1; // no division
//
//    // configure the module
//    SPI1->CTL0 = SPI_CTL0_SPO_LOW | SPI_CTL0_SPH_SECOND | // clock edges and phases for data
//            SPI_CTL0_FRF_MIRCOWIRE | // Microwire format
//            SPI_CTL0_DSS_DSS_16 | // set data size to 16 bits
//            SPI_CTL0_CSSEL_CSSEL_1; // chip-select 1
//
//    SPI1->CTL1 = SPI_CTL1_CP_ENABLE | // microcontroller is CONTROLLER
//            SPI_CTL1_PREN_DISABLE | SPI_CTL1_PTEN_DISABLE | // disable parity on RX and TX
//            SPI_CTL1_MSB_ENABLE; // bit order is MSB first
//
//    //SPI1->CS1_POCI1 |= ((uint32_t) 1 << 26);
//
//    /* Configure Controller mode */
//    /*
//     * Set the bit rate clock divider to generate the serial output clock
//     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
//     *     500000 = (32000000)/((1 + 31) * 2)
//     */
//
//    SPI1->CLKCTL = 31;
//
//    // set RX and TX FIFO threshold levels
//    SPI1->IFLS = SPI_IFLS_RXIFLSEL_LEVEL_1 | // Make the FIFO just one word big
//            SPI_IFLS_TXIFLSEL_LVL_EMPTY;     // Trigger an interrupt when the FIFO is empty
//
//    // enable Transmit FIFO interrupt
//    SPI1->CPU_INT.IMASK |= SPI_CPU_INT_IMASK_TX_SET;
//
//    // enable module
//    SPI1->CTL1 |= SPI_CTL1_ENABLE_ENABLE;
//
//}



void writeDAC(uint16_t message) {

    // initialize message w/ falling edge of ~SYNC
    GPIOA->DOUTSET31_0 = (SYNC_B | SCK);
    GPIOA->DOUTCLR31_0 = SYNC_B;

    // update each bit on PICO, then have SCK fall
    for (int i = 15; i != -1; i--) {
        if (message & (1 << i)) {
            GPIOA->DOUT31_0 |= (PICO | SCK);
        } else {
            GPIOA->DOUT31_0 = (GPIOA->DOUT31_0 & ~PICO) | SCK;
        }
        GPIOA->DOUTCLR31_0 = SCK;
    }

}
