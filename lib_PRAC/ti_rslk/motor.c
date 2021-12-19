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

/*----------------------------------------------------------------------------*/

#include "motor.h"
#include "gpio.h"
#include "timer_a.h"

/*----------------------------------------------------------------------------*/

/*
 * TIMER_A is sourced from SMCLK, which is configured to operate from DCO at 40 MHz (see msp432_launchpad_board.c, line 217-223)
 * TIMER_A period was originally set to 32000, meaning that the PWM is reset every 32000 counts and thus it operates at 40000000/32000 = 1250 Hz = 1.25 kHz
 * Ideally, the PWM should operate between 5-20 kHz, otherwise the ripple can be noticeable, so we set the period to 2000
 * Alternatively, one can divide the clock source using the MOTOR_LEFT_PWM_CLK_DIV property so that the period is suitable
 * The MOTOR_LEFT_PWM_CLK_MODE is set to TIMER_A_OUTPUTMODE_TOGGLE, meaning that when the CCRx counts to the value it will toggle the output
 * (see https://www.ti.com/lit/ug/slau400f/slau400f.pdf?ts=1635986366328)
 *
 */

/*
 * nSLEEP: if LOW disables chip
 * PH: if HIGH reverse, if LOW forward
 */

/*----------------------------------------------------------------------------*/

#define MOTOR_LEFT_SLP_PORT         ( GPIO_PORT_P3 )
#define MOTOR_LEFT_SLP_PIN          ( GPIO_PIN7 )

#define MOTOR_LEFT_DIR_PORT         ( GPIO_PORT_P5 )
#define MOTOR_LEFT_DIR_PIN          ( GPIO_PIN4 )

#define MOTOR_LEFT_PWM_GPIO_PORT    ( GPIO_PORT_P2 )
#define MOTOR_LEFT_PWM_GPIO_PIN     ( GPIO_PIN7 )
#define MOTOR_LEFT_PWM_GPIO_MODE    ( GPIO_PRIMARY_MODULE_FUNCTION )

#define MOTOR_LEFT_PWM_CLK_TIMER    ( TIMER_A0_BASE )
#define MOTOR_LEFT_PWM_CLK_SRC      ( TIMER_A_CLOCKSOURCE_SMCLK )
#define MOTOR_LEFT_PWM_CLK_DIV      ( TIMER_A_CLOCKSOURCE_DIVIDER_1 )
#define MOTOR_LEFT_PWM_CLK_PERIOD   ( 2000 )
#define MOTOR_LEFT_PWM_CLK_REG      ( TIMER_A_CAPTURECOMPARE_REGISTER_4 )
#define MOTOR_LEFT_PWM_CLK_MODE     ( TIMER_A_OUTPUTMODE_TOGGLE_SET )

/*----------------------------------------------------------------------------*/

#define MOTOR_RIGHT_SLP_PORT        ( GPIO_PORT_P3 )
#define MOTOR_RIGHT_SLP_PIN         ( GPIO_PIN6 )

#define MOTOR_RIGHT_DIR_PORT        ( GPIO_PORT_P5 )
#define MOTOR_RIGHT_DIR_PIN         ( GPIO_PIN5 )

#define MOTOR_RIGHT_PWM_GPIO_PORT   ( GPIO_PORT_P2 )
#define MOTOR_RIGHT_PWM_GPIO_PIN    ( GPIO_PIN6 )
#define MOTOR_RIGHT_PWM_GPIO_MODE   ( GPIO_PRIMARY_MODULE_FUNCTION )

#define MOTOR_RIGHT_PWM_CLK_TIMER   ( TIMER_A0_BASE )
#define MOTOR_RIGHT_PWM_CLK_SRC     ( TIMER_A_CLOCKSOURCE_SMCLK )
#define MOTOR_RIGHT_PWM_CLK_DIV     ( TIMER_A_CLOCKSOURCE_DIVIDER_1 )
#define MOTOR_RIGHT_PWM_CLK_PERIOD  ( 2000 )
#define MOTOR_RIGHT_PWM_CLK_REG     ( TIMER_A_CAPTURECOMPARE_REGISTER_3 )
#define MOTOR_RIGHT_PWM_CLK_MODE    ( TIMER_A_OUTPUTMODE_TOGGLE_SET )

/*----------------------------------------------------------------------------*/

static Timer_A_PWMConfig rightMotorPWMConfig = { MOTOR_RIGHT_PWM_CLK_SRC,
                                                 MOTOR_RIGHT_PWM_CLK_DIV,
                                                 MOTOR_RIGHT_PWM_CLK_PERIOD, // Period
                                                 MOTOR_RIGHT_PWM_CLK_REG,
                                                 MOTOR_RIGHT_PWM_CLK_MODE,
                                                 0}; // Duty cycle

static Timer_A_PWMConfig leftMotorPWMConfig = { MOTOR_LEFT_PWM_CLK_SRC,
                                                MOTOR_LEFT_PWM_CLK_DIV,
                                                MOTOR_LEFT_PWM_CLK_PERIOD,
                                                MOTOR_LEFT_PWM_CLK_REG,
                                                MOTOR_LEFT_PWM_CLK_MODE,
                                                0};

/*----------------------------------------------------------------------------*/

/*
 * Initialize the motors
 */
void MotorInit(void) {
    // Init SLP GPIOs
    GPIO_setAsOutputPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    GPIO_setAsOutputPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);

    // By default sleep is LOW, so the device is in sleep mode
    GPIO_setOutputLowOnPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    GPIO_setOutputLowOnPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);

    // Init DIR GPIOs
    GPIO_setAsOutputPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    GPIO_setAsOutputPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);

    // By default dir is LOW, so the motors go forward
    GPIO_setOutputLowOnPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    GPIO_setOutputLowOnPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);

    // Init PWM GPIOs
    GPIO_setAsPeripheralModuleFunctionOutputPin(MOTOR_LEFT_PWM_GPIO_PORT, MOTOR_LEFT_PWM_GPIO_PIN, MOTOR_LEFT_PWM_GPIO_MODE);
    GPIO_setAsPeripheralModuleFunctionOutputPin(MOTOR_RIGHT_PWM_GPIO_PORT, MOTOR_RIGHT_PWM_GPIO_PIN, MOTOR_RIGHT_PWM_GPIO_MODE);

    // Init PWM Timers
    Timer_A_generatePWM(MOTOR_LEFT_PWM_CLK_TIMER, &leftMotorPWMConfig);
    Timer_A_generatePWM(MOTOR_RIGHT_PWM_CLK_TIMER, &rightMotorPWMConfig);
}

/*----------------------------------------------------------------------------*/

/*
 * Start the motors
 * motor: The motor to actuate on (MOTOR_RIGHT, MOTOR_LEFT or MOTOR_BOTH)
 */
void MotorStart(motor_e motor) {
    // Turn ON right motor
    if ((motor & MOTOR_RIGHT) == MOTOR_RIGHT) {
        GPIO_setOutputHighOnPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);
    }

    // Turn ON left motor
    if ((motor & MOTOR_LEFT) == MOTOR_LEFT) {
        GPIO_setOutputHighOnPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    }
}

/*----------------------------------------------------------------------------*/

/*
 * Stop the motors
 * motor: The motor to actuate on (MOTOR_RIGHT, MOTOR_LEFT or MOTOR_BOTH)
 */
void MotorStop(motor_e motor) {
    // Turn OFF right motor
    if ((motor & MOTOR_RIGHT) == MOTOR_RIGHT) {
        GPIO_setOutputLowOnPin(MOTOR_RIGHT_SLP_PORT, MOTOR_RIGHT_SLP_PIN);
    }

    // Turn OFF left motor
    if ((motor & MOTOR_LEFT) == MOTOR_LEFT) {
        GPIO_setOutputLowOnPin(MOTOR_LEFT_SLP_PORT, MOTOR_LEFT_SLP_PIN);
    }
}

/*----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*/

/*
 * Configure the motors
 * motor:     The motor to actuate on (MOTOR_RIGHT, MOTOR_LEFT or MOTOR_BOTH)
 * motor_dir: The direction to spin (MOTOR_FORWARD or MOTOR_REVERSE
 * motor_pwm: The PWM value to set the motor (between 0 and 100)
 */
void MotorConfigure(motor_e motor, motor_dir_e motor_dir, uint8_t motor_pwm) {
    // Check PWM values
    if (motor_pwm > 100) {
        motor_pwm = 100;
    }

    // Calculate PWM
    uint16_t duty_cycle = motor_pwm * MOTOR_RIGHT_PWM_CLK_PERIOD / 100;

    // Set PWM on right motor with proper direction
    if ((motor & MOTOR_RIGHT) == MOTOR_RIGHT) {
        if (motor_dir == MOTOR_DIR_FORWARD) {
            GPIO_setOutputLowOnPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
        } else if (motor_dir == MOTOR_DIR_BACKWARD) {
            GPIO_setOutputHighOnPin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
        }

        rightMotorPWMConfig.dutyCycle = duty_cycle;

        Timer_A_generatePWM(MOTOR_RIGHT_PWM_CLK_TIMER, &rightMotorPWMConfig);
    }

    // Set PWM on left motor
    if ((motor & MOTOR_LEFT) == MOTOR_LEFT) {
        if (motor_dir == MOTOR_DIR_FORWARD) {
            GPIO_setOutputLowOnPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
        } else if (motor_dir == MOTOR_DIR_BACKWARD) {
            GPIO_setOutputHighOnPin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
        }

        leftMotorPWMConfig.dutyCycle = duty_cycle;

        Timer_A_generatePWM(MOTOR_LEFT_PWM_CLK_TIMER, &leftMotorPWMConfig);
    }
}

/*----------------------------------------------------------------------------*/
