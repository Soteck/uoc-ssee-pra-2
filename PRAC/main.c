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

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>


/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"


/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "BumpInt.h"
#include "driverlib.h"
#include "uart_driver.h"
#include "EncodedMotorDriver.h"
#include "motor.h"
#include "Encoder.h"

/*----------------------------------------------------------------------------*/

#define TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define BUMP_TASK_PRIORITY          ( tskIDLE_PRIORITY + 3 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define TASK_STACK_SIZE             ( 1024 )
#define HEARTBEAT_STACK_SIZE        ( 128 )

#define HEART_BEAT_ON_MS            ( 10 )
#define HEART_BEAT_OFF_MS           ( 990 )

#define PWM_VALUE                   ( 25 )

//Pongo un valor relativamente alto por si el robot choca con cualquier cosa, tenga un segundo de debounce
#define DEBOUNCING_MS               ( 1000 )



#define QUEUE_SIZE                  ( 50 )

#define TX_UART_MESSAGE_LENGTH      ( 128 )


#define TIME_WINDOW_MILIS            ( 2000 )

#define MOTOR_BREAK_TIME             ( 500 )

#define pdTICKS_TO_MS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) 1000 ) / ( TickType_t ) configTICK_RATE_HZ ) )

/*----------------------------------------------------------------------------*/
typedef enum {
    TEXT = 0,
    JSON = 1,
    JSPUSH = 2,
    CSV = 3
} uart_format;

typedef enum {
    AV_3 = 0,
    VR_90 = 1,
    AV_5 = 2,
    VL_90 = 3,
    RE_1 = 4,
    VR_180 = 5
} command_enum;


typedef struct{
    float r_revs;
    float l_revs;
    uint8_t pwm;
}action_message;


// Tasks
static void CommandsTask(void *pvParameters);
static void MotorTask(void *pvParameters);
static void SensingTask(void *pvParameters);
// callbacks & functions


//Task sync tools and variables
SemaphoreHandle_t xBumperReceived;
SemaphoreHandle_t xActionDone;
QueueHandle_t xQueueActions;
QueueHandle_t xQueueReporting;
//Constants
const TickType_t bumperDEBOUNCE_DELAY = pdMS_TO_TICKS(DEBOUNCING_MS);
const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS(500);
const uint8_t UART_FORMAT = TEXT;
const uint8_t TIME_WINDOW_SECONDS = TIME_WINDOW_MILIS/1000;

const float ROBOT_DIAMETER_MM = DISTANCIA_ENTRE_EJES_MM * MATH_PI;

const float WHEEL_LENGTH_MM = MATH_PI * WHEEL_DIAMETER_MM;
//Nos podemos saltar las multiplicaciones por pi ya que estarian en cada parte de la division
const float REVS_TO_360 = DISTANCIA_ENTRE_EJES_MM/WHEEL_DIAMETER_MM;

const uint8_t N_REGULAR_ACTIONS = 4;
const uint8_t N_BUMP_HIT_ACTIONS = 2;


/*----------------------------------------------------------------------------*/

static void BuildCommand(command_enum command, action_message * action){
    char message[TX_UART_MESSAGE_LENGTH];
    switch(command){
    case AV_3:
        //Avanzar 3 vueltas
        (*action).l_revs = 3;
        (*action).r_revs = 3;
        break;
    case VR_90:
        //Girar 90º a la derecha
        (*action).l_revs = REVS_TO_360/4;
        (*action).r_revs = REVS_TO_360/-4;
        break;
    case AV_5:
        //Avanzar 5 vueltas
        (*action).l_revs = 5;
        (*action).r_revs = 5;
        break;
    case VL_90:
        //Girar 90º a la izquierda
        (*action).l_revs = REVS_TO_360/-4;
        (*action).r_revs = REVS_TO_360/4;
        break;
    case RE_1:
        //Retroceder 1 vuelta
        (*action).l_revs = -1;
        (*action).r_revs = -1;
        break;
    case VR_180:
        //Girar 180º a la derecha
        (*action).l_revs = REVS_TO_360/2;
        (*action).r_revs = REVS_TO_360/-2;
        break;
    default:
        sprintf(message, "Unknown command %d. \n\r", command);
        uart_print(message);
        break;
    }

}


static void BuildAction(int8_t action_index, action_message * action){
    (*action).pwm =  PWM_VALUE;
    char message[TX_UART_MESSAGE_LENGTH];
    command_enum command;
    switch(action_index){
    case 1:
        command = AV_3;
        break;
    case 2:
        command = VR_90;
        break;
    case 3:
        command = AV_5;
        break;
    case 4:
        command = VL_90;
        break;
    case 5:
        command = AV_3;
        break;
    case -1:
        command = RE_1;
        break;
    case -2:
        command = VR_180;
        break;
    default:
        sprintf(message, "Unknown action %d. \n\r", action_index);
        uart_print(message);
        break;
    }
    BuildCommand(command, action);

}

static void CommandsTask(void *pvParameters) {
    uint8_t action_index = 1;
    action_message action;
    for (;;) {

        BuildAction(action_index, &action);

        xQueueSend(xQueueActions, (void *)&action, xMaxExpectedBlockTime);

        vTaskDelay(pdMS_TO_TICKS(TIME_WINDOW_MILIS));

        action_index++;
        if(action_index > N_REGULAR_ACTIONS){
            vTaskSuspend( NULL );
        }
    }
}


static void MotorTask(void *pvParameters) {
    BaseType_t r_motor_isrunning, l_motor_isrunning;
    uint8_t motors_pwm_status;

    action_message commandToReceive;


    xSemaphoreGive(xActionDone);
    for (;;) {
        GetMotorStatus(&l_motor_isrunning, &r_motor_isrunning, &motors_pwm_status);
        if(l_motor_isrunning == pdFALSE && r_motor_isrunning == pdFALSE){

            if (xSemaphoreTake(xActionDone, xMaxExpectedBlockTime) == pdPASS){

                if (xQueueReceive(xQueueActions, (void *)&commandToReceive, xMaxExpectedBlockTime) == pdTRUE) {
                    //Demos un respiro a los motores
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_BREAK_TIME));
                    EncodedMotorRoll(commandToReceive.l_revs, commandToReceive.r_revs, commandToReceive.pwm);
                }

                xSemaphoreGive(xActionDone);

            }

        }
    }

}


static void SensingTask(void *pvParameters) {
    action_message action;
    while(1){
        if (xSemaphoreTake(xBumperReceived, xMaxExpectedBlockTime) == pdPASS){
            //uint8_t status = BumpInt_Read();

            EncodeedMotorCancelRoll();
            if(xSemaphoreTake(xActionDone, xMaxExpectedBlockTime) == pdPASS){
                for(int8_t i = (N_BUMP_HIT_ACTIONS * -1); i < 0; i++){

                    BuildAction(i, &action);

                    xQueueSendToFront(xQueueActions, (void *)&action, xMaxExpectedBlockTime);
                }
                xSemaphoreGive(xActionDone);
            }
            vTaskDelay( bumperDEBOUNCE_DELAY );
            xSemaphoreTake(xBumperReceived, 0);
        }
    }
}


/*----------------------------------------------------------------------------*/
void BumperCallBack(uint8_t bumpers){
    xSemaphoreGiveFromISR(xBumperReceived, NULL);
}
void MotorTaskEndCallback(){
    xSemaphoreGiveFromISR(xActionDone, NULL);
}
void MotorTaskInterruptedCallback(){
    xSemaphoreGive(xActionDone);
}

int main(int argc, char** argv)
{
    int32_t retVal = -1;

    // Initialize semaphores, queues,...
    xQueueActions = xQueueCreate( QUEUE_SIZE, sizeof(action_message) );
    if(!xQueueActions){
        led_on(MSP432_LAUNCHPAD_LED_RED);
        while(1);
    }
    xBumperReceived = xSemaphoreCreateBinary();
    if(!xBumperReceived){
        led_on(MSP432_LAUNCHPAD_LED_RED);
        while(1);
    }
    xActionDone = xSemaphoreCreateBinary();
    if(!xActionDone){
        led_on(MSP432_LAUNCHPAD_LED_RED);
        while(1);
    }


    /* Initialize the board and peripherals */
    board_init();
    // No auditamos el estado de los motores
    EncodedMotorInit( MotorTaskEndCallback, MotorTaskInterruptedCallback, NULL );
    uart_init(NULL);
    BumpInt_Init(BumperCallBack);


    if ( true ) {  //can be used to check the existence of FreeRTOS sync tools
        EncodedMotorStart();

        /* Create CommandsTask task */
        retVal = xTaskCreate(CommandsTask, "CommandsTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }
//        EncodedMotorRoll(20, 20, 25);
        /* Create Motor task */
        retVal = xTaskCreate(MotorTask, "MotorTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create Sensing task */
        retVal = xTaskCreate(SensingTask, "SensingTask", TASK_STACK_SIZE, NULL, BUMP_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }


        /* Start the task scheduler */
        vTaskStartScheduler();
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
