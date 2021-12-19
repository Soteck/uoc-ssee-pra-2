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

static int32_t EncoderElapsedCounts(encoder_e encoder);

/*----------------------------------------------------------------------------*/

static int32_t left_motor_cur = 0;
static int32_t left_motor_old = 0;
static int32_t right_motor_cur = 0;
static int32_t right_motor_old = 0;

static float left_rev_count = 0;
static float right_rev_count = 0;

static uint16_t counts_callback_trigger = 0;
static void (*counts_callback_fn);

static BaseType_t callbacks_attached = pdFALSE;
static void (*right_rotation_callback_fn)(void);
static void (*left_rotation_callback_fn)(void);

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
    Interrupt_enableInterrupt(INT_PORT5);
}

/*----------------------------------------------------------------------------*/

void EncoderGetSpeed(encoder_e encoder, uint32_t elapsed_ms, float * distance_mm, float * speed_mm_s) {
    const float wheel_length_mm = 3.14159f * WHEEL_DIAMETER_MM;
    const float counts_per_rev  = COUNTS_PER_REVOLUTION;

    int32_t elapsed_counts;
    float revolutions;

    elapsed_counts = EncoderElapsedCounts(encoder);
    revolutions    = elapsed_counts / counts_per_rev;
    *distance_mm   = revolutions *  wheel_length_mm;
    *speed_mm_s    = 1000.0f * (*distance_mm) / (float) elapsed_ms;
}

/*----------------------------------------------------------------------------*/

static int32_t EncoderElapsedCounts(encoder_e encoder) {
    int32_t current;
    int32_t previous;
    int32_t counts;

    switch (encoder) {
        case ENCODER_LEFT:
            current = left_motor_cur;
            previous = left_motor_old;
            left_motor_old = current;
            break;
        case ENCODER_RIGHT:
            current = right_motor_cur;
            previous = right_motor_old;
            right_motor_old = current;
            break;
        default:
            while(1);
    }

    // We have rolled over
    if (previous > current) {
        counts = previous - current;
    } else {
        counts = current - previous;
    }

    return counts;
}

/*----------------------------------------------------------------------------*/

void AddCallBackForRotationCount(uint16_t rotationsNumber, void (*callback)(uint8_t)){
    counts_callback_trigger = rotationsNumber * COUNTS_PER_REVOLUTION;
    counts_callback_fn = callback;
}

void INIT_REV_CALLLBACK(
        void (*left_callback_fn)(void),
        void (*right_callback_fn)(void)
){
    callbacks_attached = pdTRUE;
    left_rotation_callback_fn = left_callback_fn;
    right_rotation_callback_fn = right_callback_fn;
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
            left_rev_count--;
        }
        else {
            left_motor_cur++;
            left_rev_count++;
        }
    }

    // Account for right motor interrupts
    if (status & ENCODER_RIGHT_B_GPIO_PIN) {
        value = GPIO_getInputPinValue(ENCODER_RIGHT_A_GPIO_PORT, ENCODER_RIGHT_A_GPIO_PIN);
        if (value > 0) {
            right_motor_cur--;
            right_rev_count--;
        }
        else {
            right_motor_cur++;
            right_rev_count++;
        }
    }

    // Si las revoluciones llegan a una vuelta hacia adelante o hacia atrás y el callback esta seteado, lo llamamos
    if(right_rev_count != 0 &&
            (right_rev_count > COUNTS_PER_REVOLUTION || right_rev_count < COUNTS_PER_REVOLUTION_INV)){
        right_rev_count = 0;
        if(callbacks_attached == pdTRUE){
            right_rotation_callback_fn();
        }
    }
    if(left_rev_count != 0 &&
            (left_rev_count > COUNTS_PER_REVOLUTION || left_rev_count < COUNTS_PER_REVOLUTION_INV)){
        left_rev_count = 0;
        if(callbacks_attached == pdTRUE){
            left_rotation_callback_fn();
        }
    }

}
