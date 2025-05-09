/*
 * mt.c - Main source file for MicroTracker.
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <ti/devices/msp/msp.h>
#include "mt_setup.h"



// DAC control
uint16_t blankPacketL = 0x1000; // Fill middle 8 bits w/ amplitude
uint16_t blankPacketR = 0x5000; // "

#define MIDLINE ((uint8_t) 128)
#define AMP ((int32_t) (INT_MAX / 2) - 1)

// SPI stuff
//uint16_t *txPacket;

//int transmissionComplete = 0; // flag for SPI ISR wakeup
int timerTicked = 0; // flag for timer ISR wakeup
//int idx = 0;
//int message_len = sizeof(lowPacket) / sizeof(lowPacket[0]);

// Enum for FSM
enum current_state_enum {
    OFF,
    ON
};
enum current_state_enum next_state;

bool channel1_bool;
int32_t channel1_value;

bool channel2_bool;
int32_t channel2_value;

uint8_t mixed_value;

int TIMG6_cross;
int TIMG7_cross;



int main(void)
{
    InitializeProcessor();
    InitializeGPIO();
    InitializeTimerG0();
    InitializeTimerG6();
    InitializeTimerG7();

    TIMG6_cross = (TIMG6->COUNTERREGS.LOAD + 1) / 2;
    TIMG7_cross = (TIMG7->COUNTERREGS.LOAD + 1) / 2;

//    InitializeSPI1();

    NVIC_EnableIRQ(TIMG0_INT_IRQn); // enable the timer interrupt
    TIMG0->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);
    TIMG6->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);
    TIMG7->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);

    next_state = OFF;

    while (1) { // this loop will execute once per TIMG0 interrupt

        channel1_bool = (TIMG6->COUNTERREGS.CTR < TIMG6_cross);
        if (channel1_bool)
            channel1_value = AMP;
        else
            channel1_value = -AMP;

        channel2_bool = (TIMG7->COUNTERREGS.CTR < TIMG7_cross);
        if (channel2_bool)
            channel2_value = AMP;
        else
            channel2_value = -AMP;

        mixed_value = ((channel1_value + channel2_value) >> 24) + MIDLINE;

        if ( (GPIOA->DIN31_0 & SW1) != SW1 ) {
            writeDAC(blankPacketL | (mixed_value << 4));
            writeDAC(blankPacketR | (mixed_value << 4));
        } else {
            writeDAC(blankPacketL | (MIDLINE << 4));
            writeDAC(blankPacketR | (MIDLINE << 4));
        }


        /*
        // Begin FSM ===========================================================
        switch (next_state) {

        case OFF:
            if ( (GPIOA->DIN31_0 & SW1) != SW1 ) {
                writeDAC(highPacketL);
                writeDAC(highPacketR);
                next_state = ON;
            }

            break;

        case ON:
            writeDAC(lowPacketL);
            writeDAC(lowPacketR);
            next_state = OFF;

            break;

        default:
            next_state = OFF;

            break;

        }
        // End FSM =============================================================
        */

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
    switch (TIMG0->CPU_INT.IIDX) {
        case GPTIMER_CPU_INT_IIDX_STAT_Z: // counted down to zero event
            timerTicked = 1; // set a flag so we can know what woke us up
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
