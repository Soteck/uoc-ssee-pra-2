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
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"


/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "Encoder.h"
#include "motor.h"
#include "EncodedMotorDriver.h"
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/

static void (*end_roll_callback)(void);
static void (*roll_interrupt_callback)(void);
static BaseType_t end_roll_callback_set = pdFALSE;
static BaseType_t roll_interrupt_callback_set = pdFALSE;

static BaseType_t r_motor_running = pdFALSE;
static BaseType_t l_motor_running = pdFALSE;
static uint8_t motors_pwm = 0;

QueueHandle_t queueReporting;

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
    TestCallback();
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
}

static void SendQueue(int32_t r_ticks, int32_t l_ticks){
    reporting_message action;
    if(queueReporting){
        if(r_ticks){
            action.r_ticks = r_ticks;
        }
        if(l_ticks){
            action.l_ticks = l_ticks;
        }
        xQueueSendFromISR(queueReporting, (void *)&action, NULL);
    }
}

// No necesitamos implementar secciones criticas porque al estar dentro de un ISR no puede interumpirse
static void LeftCallBack(int32_t ticks) {
    StopLeft();
}
static void RightCallBack(int32_t ticks) {
    StopRight();
    SendQueue(ticks, NULL);
}


void EncodeedMotorCancelRoll(){

    StopLeft();
    StopRight();

    if(roll_interrupt_callback_set == pdTRUE){
        roll_interrupt_callback();
    }
}


void EncodedMotorRoll(float left_revolutions, float right_revolutions, uint8_t motor_pwm){
    EncodeedMotorCancelRoll();
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


    SETUP_THRESHOLD_CALLBACK(left_revolutions, right_revolutions, LeftCallBack, RightCallBack);


    if(left_revolutions != 0){
        MotorConfigure(MOTOR_LEFT, left_direction, motor_pwm);
        StartLeft();
    }
    if(left_revolutions != 0){
        MotorConfigure(MOTOR_RIGHT, right_direction, motor_pwm);
        StartRight();
    }
}



/*----------------------------------------------------------------------------*/




/*----------------------------------------------------------------------------*/

// Esta hecho para poderle pasar una cola en la cual va a poner mensajes de estados de motores, por si se quiere auditar
void EncodedMotorInit(
        void (*end_callback_fn)(void),
        void (*interrupted_callback_fn)(void),
       QueueHandle_t * queueReportingR

){
    MotorInit();
    EncoderInit();
    if(end_callback_fn){
        end_roll_callback_set = pdTRUE;
        end_roll_callback = end_callback_fn;
    }
    if(interrupted_callback_fn){
        roll_interrupt_callback_set = pdTRUE;
        roll_interrupt_callback = interrupted_callback_fn;
    }
    if(queueReporting){
        queueReporting = queueReportingR;
    }
}
void EncodedMotorStart(){
    MotorStart(MOTOR_BOTH);
}
