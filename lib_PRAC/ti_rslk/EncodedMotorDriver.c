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

#include "Encoder.h"
#include "motor.h"

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/

static int32_t left_rev_count = 0;
static int32_t right_rev_count = 0;
static float left_rev_order = 0;
static float right_rev_order = 0;



static void RightRevCallBack() {
    right_rev_count++;
    if(right_rev_count >= right_rev_order){
        MotorStop(MOTOR_RIGHT);
    }
}

static void LeftRevCallBack() {
    left_rev_count++;
    if(left_rev_count >= left_rev_order){
        MotorStop(MOTOR_LEFT);
    }
}

void EncodedMotorRoll(float left_revolutions, float right_revolutions, uint8_t motor_pwm){
    left_rev_order = left_revolutions;
    right_rev_order = right_revolutions;
    motor_dir_e left_direction = MOTOR_DIR_FORWARD;
    motor_dir_e right_direction = MOTOR_DIR_FORWARD;
    if(left_revolutions < 0){
        left_direction = MOTOR_DIR_BACKWARD;
        left_rev_order = left_revolutions * -1;
    }
    if(right_revolutions < 0){
        right_direction = MOTOR_DIR_BACKWARD;
        right_rev_order = right_revolutions * -1;
    }
    INIT_REV_CALLLBACK(LeftRevCallBack, RightRevCallBack);
    if(left_revolutions != 0){
        MotorConfigure(MOTOR_LEFT, left_direction, motor_pwm);
    }
    if(left_revolutions != 0){
        MotorConfigure(MOTOR_RIGHT, right_direction, motor_pwm);
    }
}

void EncodeedMotorCancelRoll(){
    left_rev_order = 0;
    right_rev_order = 0;
    MotorStop(MOTOR_BOTH);
}

void EncodedMotorInit(){
    MotorInit();
    EncoderInit();
}
void EncodedMotorStart(motor_e motor){
    MotorStart(motor);
}

void EncodedMotorGetSpeed(encoder_e encoder, uint32_t elapsed_ms, float * distance_mm, float * speed_mm_ms){
    EncoderGetSpeed(encoder, elapsed_ms, distance_mm, speed_mm_ms);
}
