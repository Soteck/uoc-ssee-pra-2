/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// https://www.ti.com/lit/an/slaa795/slaa795.pdf?ts=1636035210603

/*----------------------------------------------------------------------------*/

#include <stdlib.h>

#include "FreeRTOS.h"
#include "timer_a.h"
#include "gpio.h"
#include "interrupt.h"

#include "Encoder.h"

/*----------------------------------------------------------------------------*/

#define ENCODER_LEFT_A_GPIO_PORT        ( GPIO_PORT_P10 )
#define ENCODER_LEFT_A_GPIO_PIN         ( GPIO_PIN5 )
#define ENCODER_LEFT_A_GPIO_MODE        ( GPIO_PRIMARY_MODULE_FUNCTION )

#define ENCODER_LEFT_B_GPIO_PORT        ( GPIO_PORT_P5 )
#define ENCODER_LEFT_B_GPIO_PIN         ( GPIO_PIN2 )
#define ENCODER_LEFT_B_GPIO_MODE        ( GPIO_PRIMARY_MODULE_FUNCTION )

/*----------------------------------------------------------------------------*/

#define ENCODER_RIGHT_A_GPIO_PORT       ( GPIO_PORT_P10 )
#define ENCODER_RIGHT_A_GPIO_PIN        ( GPIO_PIN4 )
#define ENCODER_RIGHT_A_GPIO_MODE       ( GPIO_PRIMARY_MODULE_FUNCTION )

#define ENCODER_RIGHT_B_GPIO_PORT       ( GPIO_PORT_P5 )
#define ENCODER_RIGHT_B_GPIO_PIN        ( GPIO_PIN0 )
#define ENCODER_RIGHT_B_GPIO_MODE       ( GPIO_PRIMARY_MODULE_FUNCTION )

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

static int32_t left_motor_cur = 0;
static int32_t right_motor_cur = 0;



static BaseType_t callbacks_attached = pdFALSE;


static float r_callback_counts_up = 0;
static float r_callback_counts_down = 0;
static float l_callback_counts_up = 0;
static float l_callback_counts_down = 0;

static void (*right_callback_fn)(int32_t);
static void (*left_callback_fn)(int32_t);

/*----------------------------------------------------------------------------*/

void EncoderInit(void) {
    // Configure GPIO for left encoder
    GPIO_setAsInputPin(ENCODER_LEFT_A_GPIO_PORT, ENCODER_LEFT_A_GPIO_PIN);
    GPIO_setAsInputPin(ENCODER_LEFT_B_GPIO_PORT, ENCODER_LEFT_B_GPIO_PIN);

    // Configure GPIO for right encoder
    GPIO_setAsInputPin(ENCODER_RIGHT_A_GPIO_PORT, ENCODER_RIGHT_A_GPIO_PIN);
    GPIO_setAsInputPin(ENCODER_RIGHT_B_GPIO_PORT, ENCODER_RIGHT_B_GPIO_PIN);

    // Configure GPIO interrupts for left encoder
    GPIO_interruptEdgeSelect(ENCODER_LEFT_B_GPIO_PORT, ENCODER_LEFT_B_GPIO_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(ENCODER_LEFT_B_GPIO_PORT, ENCODER_LEFT_B_GPIO_PIN);
    GPIO_enableInterrupt(ENCODER_LEFT_B_GPIO_PORT, ENCODER_LEFT_B_GPIO_PIN);

    // Configure GPIO interrupts for right encoder
    GPIO_interruptEdgeSelect(ENCODER_RIGHT_B_GPIO_PORT, ENCODER_RIGHT_B_GPIO_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(ENCODER_RIGHT_B_GPIO_PORT, ENCODER_RIGHT_B_GPIO_PIN);
    GPIO_enableInterrupt(ENCODER_RIGHT_B_GPIO_PORT, ENCODER_RIGHT_B_GPIO_PIN);

    // Enable GPIO interrupts
    Interrupt_setPriority(INT_PORT5, 0xFF);
    Interrupt_enableInterrupt(INT_PORT5);
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

void SETUP_THRESHOLD_CALLBACK(
        float l_rpm,
        float r_rpm,
        void (*left_callback_fnr)(int32_t),
        void (*right_callback_fnr)(int32_t)
        ){
    l_callback_counts_up = COUNTS_PER_REVOLUTION * l_rpm;
    l_callback_counts_down = COUNTS_PER_REVOLUTION_INV * l_rpm;
    r_callback_counts_up = COUNTS_PER_REVOLUTION * r_rpm;
    r_callback_counts_down = COUNTS_PER_REVOLUTION_INV * r_rpm;
    callbacks_attached = pdTRUE;
    left_callback_fn = left_callback_fnr;
    right_callback_fn = right_callback_fnr;
}

void REMOVE_CALLBACKS(){
    callbacks_attached = pdFALSE;
    left_motor_cur = 0;
    right_motor_cur = 0;
}


/*----------------------------------------------------------------------------*/

/* GPIO ISR */
void PORT5_IRQHandler(void)
{
    uint32_t status;
    uint8_t value;

    // Read and clear GPIO interrupt
    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    // Account for left motor interrupts
    if (status & ENCODER_LEFT_B_GPIO_PIN) {
        value = GPIO_getInputPinValue(ENCODER_LEFT_A_GPIO_PORT, ENCODER_LEFT_A_GPIO_PIN);
        if (value > 0) {
            left_motor_cur--;
        }
        else {
            left_motor_cur++;
        }
    }

    // Account for right motor interrupts
    if (status & ENCODER_RIGHT_B_GPIO_PIN) {
        value = GPIO_getInputPinValue(ENCODER_RIGHT_A_GPIO_PORT, ENCODER_RIGHT_A_GPIO_PIN);
        if (value > 0) {
            right_motor_cur--;
        }
        else {
            right_motor_cur++;
        }
    }

    if(callbacks_attached == pdTRUE){
        if (left_motor_cur > l_callback_counts_up || left_motor_cur < l_callback_counts_down){
            left_callback_fn(left_motor_cur);
            left_motor_cur = 0;
        }
        if (right_motor_cur > r_callback_counts_up || right_motor_cur < r_callback_counts_down){
            right_callback_fn(right_motor_cur);
            right_motor_cur = 0;
        }

    }


}
