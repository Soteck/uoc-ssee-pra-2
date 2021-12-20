/*
 * EncodedMotorDriver.c
 * Este driver esta hecho para desacoplar las aplicaciones del uso combinado de los motores con los encoders
 * De este modo, una aplicacion que haga uso de ellos por separado podrá reutilizar código
 *
 *  Created on: 19 dic. 2021
 *      Author: Guillem
 */





#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>


/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "portmacro.h"


/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "Encoder.h"
#include "motor.h"
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/

static int32_t left_rev_count = 0;
static int32_t right_rev_count = 0;
static uint8_t left_rev_order = 0;
static uint8_t right_rev_order = 0;
static BaseType_t checkDecimal = pdFALSE;

static void (*end_roll_callback)(void);
static BaseType_t end_roll_callback_set = pdFALSE;

static BaseType_t r_motor_running = pdFALSE;
static BaseType_t l_motor_running = pdFALSE;
static uint8_t motors_pwm = 0;



void GetMotorStatus(BaseType_t * point_l_motor, BaseType_t * point_r_motor, uint8_t * point_pwm){
    *point_l_motor = l_motor_running;
    *point_r_motor = r_motor_running;
    *point_pwm = motors_pwm;

}
static void TestCallback(){
    if(end_roll_callback_set == pdTRUE && l_motor_running == pdFALSE && r_motor_running == pdFALSE){
        end_roll_callback();
    }
}

static void StopLeft(){
    MotorStop(MOTOR_LEFT);
    l_motor_running = pdFALSE;
}
static void StartLeft(){
    MotorStart(MOTOR_LEFT);
    l_motor_running = pdTRUE;
}
static void StopRight(){
    MotorStop(MOTOR_RIGHT);
    r_motor_running = pdFALSE;
    TestCallback();
}
static void StartRight(){
    MotorStart(MOTOR_RIGHT);
    r_motor_running = pdTRUE;
    TestCallback();
}

// No necesitamos implementar secciones criticas porque al estar dentro de un ISR no puede interumpirse
static void LeftRevCallBack() {
    left_rev_count++;
    if(!checkDecimal && left_rev_count >= left_rev_order){
        StopLeft();
    }
}
static void RightRevCallBack() {
    right_rev_count++;
    if(!checkDecimal &&  right_rev_count >= right_rev_order){
        StopRight();
    }
}
static void LeftDecimalRevCallBack() {
    if(left_rev_count >= left_rev_order){
        StopLeft();
    }
}
static void RightDecimalRevCallBack() {
    if(right_rev_count >= right_rev_order){
        StopRight();
    }
}



void EncodedMotorRoll(float left_revolutions, float right_revolutions, uint8_t motor_pwm){
    motors_pwm = motor_pwm;
    motor_dir_e left_direction = MOTOR_DIR_FORWARD;
    motor_dir_e right_direction = MOTOR_DIR_FORWARD;
    if(left_revolutions < 0){
        left_direction = MOTOR_DIR_BACKWARD;
        left_revolutions = left_revolutions * -1;
    }
    if(right_revolutions < 0){
        right_direction = MOTOR_DIR_BACKWARD;
        right_revolutions = right_revolutions * -1;
    }

    left_rev_order = (uint8_t) left_revolutions;
    right_rev_order = (uint8_t) right_revolutions;
    float l_decimal = left_revolutions - left_rev_order;
    float r_decimal = right_revolutions - right_rev_order;


    INIT_REV_CALLLBACK(LeftRevCallBack, RightRevCallBack);
    if(l_decimal > 0 || r_decimal > 0){
        checkDecimal = pdTRUE;
        INIT_DECIMAL_REV_CALLBACK(l_decimal, r_decimal, LeftDecimalRevCallBack, RightDecimalRevCallBack);
    }

    if(left_revolutions != 0){
        MotorConfigure(MOTOR_LEFT, left_direction, motor_pwm);
        StartLeft();
    }
    if(left_revolutions != 0){
        MotorConfigure(MOTOR_RIGHT, right_direction, motor_pwm);
        StartRight();
    }
}

void EncodeedMotorCancelRoll(){
    left_rev_order = 0;
    right_rev_order = 0;
    StopLeft();
    StopRight();
}



/*----------------------------------------------------------------------------*/




/*----------------------------------------------------------------------------*/

void EncodedMotorInit( void (*end_callback_fn)(void)){
    MotorInit();
    EncoderInit();
    if(end_callback_fn){
        end_roll_callback_set = pdTRUE;
        end_roll_callback = end_callback_fn;
    }
}
void EncodedMotorStart(motor_e motor){
    MotorStart(motor);
}

void EncodedMotorGetSpeed(encoder_e encoder, uint32_t elapsed_ms, float * distance_mm, float * speed_mm_ms){
    EncoderGetSpeed(encoder, elapsed_ms, distance_mm, speed_mm_ms);
}
