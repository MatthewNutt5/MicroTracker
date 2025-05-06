#include <mt_setup.h>
#include <ti/devices/msp/msp.h>



// SPI stuff
//uint16_t *txPacket;
uint16_t lowPacket = 0x2000;
uint16_t highPacket = 0x2FF0;
//int transmissionComplete = 0; // flag for SPI ISR wakeup
int timerTicked = 0; // flag for timer ISR wakeup
//int idx = 0;
//int message_len = sizeof(lowPacket) / sizeof(lowPacket[0]);

// Enum for FSM
enum current_state_enum {
    OFF,
    ON
};



int main(void)
{
    InitializeProcessor();
    InitializeGPIO();
//    InitializeSPI1();

    InitializeTimerG0();

    NVIC_EnableIRQ(TIMG0_INT_IRQn); // enable the timer interrupt
    TIMG0->COUNTERREGS.LOAD = 163; // set timer: approx 10ms
    TIMG0->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);

    enum current_state_enum next_state;
    next_state = OFF;

    while (1) { // this loop will execute once per timer interrupt
        // Begin FSM ===========================================================
        switch (next_state) {

        case OFF:
            if ( (GPIOA->DIN31_0 & SW1) != SW1 ) {
                writeDAC(highPacket);
                next_state = ON;
            }

            break;

        case ON:
            writeDAC(lowPacket);
            next_state = OFF;

            break;

        default:
            next_state = OFF;

            break;

        }
        // End FSM =============================================================

        // Set up SPI message
//        GPIOA->DOUTSET31_0 = SYNC_B; // set ~SYNC high for first dummy bits
//        NVIC_ClearPendingIRQ(SPI1_INT_IRQn);
//        NVIC_EnableIRQ(SPI1_INT_IRQn);
//        transmissionComplete = 0; // reset flag
//        idx = 1; // reset pointer to point at the second element of the SPI message
//        SPI1->TXDATA = *txPacket; // This will start TX ISR running.
        // It will stop itself at the end of the message, and disable SPI interrupts.

        while (!timerTicked) // Wait for timer wake up
            __WFI();

        timerTicked = 0; // reset timer interrupt flag
    }
}

//void SPI1_IRQHandler(void)
//{
//    switch (SPI1->CPU_INT.IIDX) {
//        case SPI_CPU_INT_IIDX_STAT_TX_EVT: // SPI interrupt index for transmit FIFO
//            SPI1->TXDATA = txPacket[idx];
//            idx++;
//            if (idx == message_len) {
//                GPIOA->DOUTCLR31_0 = SYNC_B; // set ~SYNC low after first dummy bits
//                transmissionComplete = 1;
//                NVIC_DisableIRQ(SPI1_INT_IRQn);
//            }
//            break;
//        default:
//            break;
//    }
//}

void TIMG0_IRQHandler(void)
{
    // This wakes up the processor!

    switch (TIMG0->CPU_INT.IIDX) {
        case GPTIMER_CPU_INT_IIDX_STAT_Z: // Counted down to zero event.
            timerTicked = 1; // set a flag so we can know what woke us up.
            break;
        default:
            break;
    }
}

/*
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
