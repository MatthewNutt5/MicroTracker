/*
 * simon_setup.h - Source file for simon project boilerplate
 */

#include <mt_setup.h>



void delay_cycles(uint32_t cycles)
{
    /* this is a scratch register for the compiler to use */
    uint32_t scratch;

    /* There will be a 2 cycle delay here to fetch & decode instructions
     * if branch and linking to this function */

    /* Subtract 2 net cycles for constant offset: +2 cycles for entry jump,
     * +2 cycles for exit, -1 cycle for a shorter loop cycle on the last loop,
     * -1 for this instruction */

    __asm volatile(
#ifdef __GNUC__
        ".syntax unified\n\t"
#endif
        "SUBS %0, %[numCycles], #2; \n"
        "%=: \n\t"
        "SUBS %0, %0, #4; \n\t"
        "NOP; \n\t"
        "BHS  %=b;" /* branches back to the label defined above if number > 0 */
        /* Return: 2 cycles */
        : "=&r"(scratch)
        : [ numCycles ] "r"(cycles));
}



void InitializeProcessor(void) {

    SYSCTL->SOCLOCK.BORTHRESHOLD = SYSCTL_SYSSTATUS_BORCURTHRESHOLD_BORMIN; // Brownout generates a reset.

    update_reg(&SYSCTL->SOCLOCK.MCLKCFG, (uint32_t) SYSCTL_MCLKCFG_UDIV_NODIVIDE, SYSCTL_MCLKCFG_UDIV_MASK); // Set UPCLK divider
    update_reg(&SYSCTL->SOCLOCK.SYSOSCCFG, SYSCTL_SYSOSCCFG_FREQ_SYSOSCBASE, SYSCTL_SYSOSCCFG_FREQ_MASK); // Set SYSOSC to 32 MHz

    // Disable MCLK Divider
    update_reg(&SYSCTL->SOCLOCK.MCLKCFG, (uint32_t) 0x0, SYSCTL_MCLKCFG_MDIV_MASK);

}



void InitializeGPIO(void) {

    GPIOA->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR | GPIO_RSTCTL_RESETASSERT_ASSERT);
    GPIOA->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);

    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

    // Initialize SPI1 connections
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM39)] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM39_PF_SPI1_SCLK; // SPI1_SCLK on PA17
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM40)] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM40_PF_SPI1_PICO; // SPI1_PICO on PA18
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM58)] = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM58_PF_SPI1_CS1_POCI1; // SPI1_CS on PB27

    // Initialize GPIO input connections
    uint32_t input_configuration = IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE |
            ((uint32_t) 0x00000001) | // GPIO function is always MUX entry 1
            IOMUX_PINCM_INV_DISABLE | // do not invert logic
            IOMUX_PINCM_PIPU_DISABLE | IOMUX_PINCM_PIPD_DISABLE | // no pullup/pulldown
            IOMUX_PINCM_HYSTEN_DISABLE | // disable input pin hysteresis
            IOMUX_PINCM_WUEN_DISABLE;  // wake-up disable

    IOMUX->SECCFG.PINCM[(IOMUX_PINCM6)] = input_configuration; // "Enter" button on PA31

    // Initialize GPIO output connections
    uint32_t output_configuration = IOMUX_PINCM_PC_CONNECTED | ((uint32_t) 0x00000001);

    IOMUX->SECCFG.PINCM[(IOMUX_PINCM38)] = output_configuration; // ~SYNC on PA16
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM39)] = output_configuration; // SCK on PA17
    IOMUX->SECCFG.PINCM[(IOMUX_PINCM40)] = output_configuration; // PICO on PA18
    GPIOA->DOUTCLR31_0 = (SYNC_B | SCK | PICO);
    GPIOA->DOESET31_0 = (SYNC_B | SCK | PICO);

    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

}



void InitializeTimerG0(void) {

    TIMG0->GPRCM.RSTCTL = (GPIO_RSTCTL_KEY_UNLOCK_W | GPIO_RSTCTL_RESETSTKYCLR_CLR | GPIO_RSTCTL_RESETASSERT_ASSERT);
    TIMG0->GPRCM.PWREN = (GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE);
    delay_cycles(POWER_STARTUP_DELAY); // delay to enable GPIO to turn on and reset

    // Configure clocking for Timer Module
    TIMG0->CLKSEL = GPTIMER_CLKSEL_LFCLK_SEL_ENABLE;
    TIMG0->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_1;

    /* This will configure what happens when we count down to zero - we'll set counter to LOAD value */

    TIMG0->COUNTERREGS.CC_01[0] = 0; // Count down to zero (value in CC0), then reload
    TIMG0->COUNTERREGS.CCCTL_01[0] = GPTIMER_CCCTL_01_ACOND_TIMCLK; // Set timer to advance on TIMCLK ticks

    // CM_DOWN - count down mode; REPEAT_1 - keep going when we reach zero; CVAE_LDVAL - when we get to zero, reload LOAD value;
    // EN_DISABLED - Start with timer disabled
    TIMG0->COUNTERREGS.CTRCTL =
        (( GPTIMER_CTRCTL_CVAE_LDVAL | GPTIMER_CTRCTL_CM_DOWN | GPTIMER_CTRCTL_REPEAT_REPEAT_1) | GPTIMER_CTRCTL_EN_DISABLED);

    // Enable timer interrupt when we reach 0
    TIMG0->CPU_INT.IMASK |= GPTIMER_CPU_INT_IMASK_Z_SET;

    TIMG0->PDBGCTL = GPTIMER_PDBGCTL_SOFT_IMMEDIATE;

    TIMG0->COMMONREGS.CCLKCTL = (GPTIMER_CCLKCTL_CLKEN_ENABLED);

}



//void InitializeSPI1(void) {
//
//    SPI1->GPRCM.RSTCTL = (SPI_RSTCTL_KEY_UNLOCK_W | SPI_RSTCTL_RESETSTKYCLR_CLR | SPI_RSTCTL_RESETASSERT_ASSERT);
//    SPI1->GPRCM.PWREN = (SPI_PWREN_KEY_UNLOCK_W | SPI_PWREN_ENABLE_ENABLE);
//    delay_cycles(POWER_STARTUP_DELAY); // delay to enable SPI to turn on and reset
//
//    // Configure clocking for SPI1
//    SPI1->CLKSEL = (uint32_t) SPI_CLKSEL_SYSCLK_SEL_ENABLE; // use the SYSOSC
//    SPI1->CLKDIV = (uint32_t) SPI_CLKDIV_RATIO_DIV_BY_1; // actually 0x0, which is going to be default, but here for completeness
//
//    // Configure the module
//    SPI1->CTL0 = SPI_CTL0_SPO_LOW | SPI_CTL0_SPH_SECOND | // Clock edges and phases for data
//            SPI_CTL0_FRF_MIRCOWIRE | // Microwire format
//            SPI_CTL0_DSS_DSS_16 | // Set data size to 16 bits
//            SPI_CTL0_CSSEL_CSSEL_1; // Chip-select 1
//
//    SPI1->CTL1 = SPI_CTL1_CP_ENABLE | // Microcontroller is CONTROLLER
//            SPI_CTL1_PREN_DISABLE | SPI_CTL1_PTEN_DISABLE | // Disable parity on RX and TX
//            SPI_CTL1_MSB_ENABLE; // Bit order is MSB first
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
//    /* Set RX and TX FIFO threshold levels */
//    SPI1->IFLS = SPI_IFLS_RXIFLSEL_LEVEL_1 | // Make the FIFO just one word big
//            SPI_IFLS_TXIFLSEL_LVL_EMPTY;     // Trigger an interrupt when the FIFO is empty
//
//    /* Enable Transmit FIFO interrupt */
//    SPI1->CPU_INT.IMASK |= SPI_CPU_INT_IMASK_TX_SET;
//
//    /* Enable module */
//    SPI1->CTL1 |= SPI_CTL1_ENABLE_ENABLE;
//
//}



void writeDAC(uint16_t message) {

    // Initialize message w/ falling edge of ~SYNC
    GPIOA->DOUTSET31_0 = (SYNC_B | SCK);
    GPIOA->DOUTCLR31_0 = SYNC_B;

    // Update each bit on PICO, then have SCK fall
    for (int i = 15; i != -1; i--) {
        GPIOA->DOUTSET31_0 = SCK;
        if (message & (1 << i)) {
            GPIOA->DOUTSET31_0 = PICO;
        } else {
            GPIOA->DOUTCLR31_0 = PICO;
        }
        GPIOA->DOUTCLR31_0 = SCK;
    }

}



/*
 *
 * This code is a reproduction of standard TI code


 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
