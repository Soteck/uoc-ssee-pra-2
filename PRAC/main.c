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
#include "motor.h"
#include "uart_driver.h"
#include "encoder.h"


/*----------------------------------------------------------------------------*/

#define TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define TASK_STACK_SIZE             ( 1024 )
#define HEARTBEAT_STACK_SIZE        ( 128 )

#define HEART_BEAT_ON_MS            ( 10 )
#define HEART_BEAT_OFF_MS           ( 990 )
#define PWM_INCREMENTAL_RATE        ( 5 )
#define PWM_START_VALUE             ( 0 )
#define PWM_MAX_VALUE               ( 100 )
#define DEBOUNCING_MS               ( 20 )

#define QUEUE_SIZE                  ( 50 )

#define TX_UART_MESSAGE_LENGTH      ( 128 )


#define TIME_WINDOW_MILIS            ( 5000 )

#define pdTICKS_TO_MS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) 1000 ) / ( TickType_t ) configTICK_RATE_HZ ) )

/*----------------------------------------------------------------------------*/
typedef enum {
    TEXT = 0,
    JSON = 1,
    JSPUSH = 2,
    CSV = 3
} uart_format;


typedef struct{
    motor_e motor;
    motor_dir_e dir;
    uint8_t pwm;
}queue_message;

// Tasks
static void HeartBeatTask(void *pvParameters);
static void CommandsTask(void *pvParameters);
static void MotorTask(void *pvParameters);
static void SpeedMeasure(void *pvParameters);
static void formatAndPrintMessage(float left_distance_mm, float left_speed_mm_s, float right_distance_mm, float right_speed_mm_s, float left_median_speed, float right_median_speed, uint8_t pwm);

// callbacks & functions


//Task sync tools and variables
QueueHandle_t xQueueActions;
QueueHandle_t xQueueMeasurements;
//Constants
const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS(500);
const uint8_t UART_FORMAT = TEXT;
const uint8_t TIME_WINDOW_SECONDS = TIME_WINDOW_MILIS/1000;


/*----------------------------------------------------------------------------*/


static void HeartBeatTask(void *pvParameters){
    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_ON_MS) );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_OFF_MS) );
    }
}

static void CommandsTask(void *pvParameters) {
    uint8_t pwm = PWM_START_VALUE;

    for (;;) {


        queue_message commandToSendL;
        commandToSendL.motor = MOTOR_LEFT;
        commandToSendL.dir = MOTOR_DIR_FORWARD;
        commandToSendL.pwm = pwm;
        queue_message commandToSendR;
        commandToSendR.motor = MOTOR_RIGHT;
        commandToSendR.dir = MOTOR_DIR_FORWARD;
        commandToSendR.pwm = pwm;
        xQueueSend(xQueueActions, (void *)&commandToSendL, xMaxExpectedBlockTime);
        xQueueSend(xQueueActions, (void *)&commandToSendR, xMaxExpectedBlockTime);
        xQueueSend(xQueueMeasurements, (void *)&commandToSendR, xMaxExpectedBlockTime);

        if(pwm == PWM_MAX_VALUE){
            pwm = PWM_START_VALUE;
        }else{
            pwm+= PWM_INCREMENTAL_RATE;
        }


        vTaskDelay(pdMS_TO_TICKS(TIME_WINDOW_MILIS));
    }

}


static void MotorTask(void *pvParameters) {
    queue_message commandToReceive;

    MotorStart(MOTOR_BOTH);

    MotorConfigure(MOTOR_RIGHT, MOTOR_DIR_FORWARD, 0);
    MotorConfigure(MOTOR_LEFT, MOTOR_DIR_FORWARD, 0);

    for (;;) {

        if (xQueueReceive(xQueueActions, (void *)&commandToReceive, xMaxExpectedBlockTime) == pdTRUE) {
            MotorConfigure(commandToReceive.motor, commandToReceive.dir, commandToReceive.pwm);

        }
    }

}

static void SpeedMeasure(void *pvParameters) {

    TickType_t previous_ticks, current_ticks, elapsed_ticks;
    uint32_t elapsed_ms;
    float left_distance_mm, left_speed_mm_s, right_distance_mm, right_speed_mm_s, left_median_speed, right_median_speed;

    // Wait for motors to start
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Get previous ticks
    previous_ticks = xTaskGetTickCount();

    uart_print("Initiating speed measurements... \n\r");
    if(UART_FORMAT == CSV){
        uart_print("R Distance,R Speed,L Distance,L Speed,Duty cycle \n\r");
    }

    for (;;) {
        // Get current ticks

        current_ticks = xTaskGetTickCount();

        // Calculate elapsed milliseconds and update variables
        elapsed_ticks = current_ticks - previous_ticks;
        previous_ticks = current_ticks;
        elapsed_ms = pdTICKS_TO_MS(elapsed_ticks);

        // Calculate wheel speed for left and right motors
        EncoderGetSpeed(ENCODER_LEFT, elapsed_ms, &left_distance_mm, &left_speed_mm_s);
        left_median_speed = left_distance_mm / TIME_WINDOW_SECONDS;
        EncoderGetSpeed(ENCODER_RIGHT, elapsed_ms, &right_distance_mm, &right_speed_mm_s);
        right_median_speed = right_distance_mm / TIME_WINDOW_SECONDS;

        formatAndPrintMessage(
                left_distance_mm,
                left_speed_mm_s,
                right_distance_mm,
                right_speed_mm_s,
                left_median_speed,
                right_median_speed,
                0);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

}

static void formatAndPrintMessage(
        float left_distance_mm,
        float left_speed_mm_s,
        float right_distance_mm,
        float right_speed_mm_s,
        float left_median_speed,
        float right_median_speed,
        uint8_t pwm
    ){
    char message[TX_UART_MESSAGE_LENGTH];

    switch(UART_FORMAT){
    case TEXT:
        sprintf(message, "R Distance: %.1f, R speed: %.1f, L Distance: %.1f, L Speed: %.1f, Duty cycle: %d. \n\r", right_distance_mm, right_median_speed, left_distance_mm, left_median_speed, pwm);
        break;
    case JSPUSH:
        sprintf(message, "data.push({r: {d: %.1f, s: %.1f}, l: {d: %.1f, s: %.1f}, d: %d}); \n\r", right_distance_mm, right_median_speed, left_distance_mm, left_median_speed, pwm);
        break;
    case JSON:
        sprintf(message, "{r: {d: %.1f, s: %.1f}, l: {d: %.1f, s: %.1f}, d: %d} \n\r", right_distance_mm, right_median_speed, left_distance_mm, left_median_speed, pwm);
        break;
    case CSV:
        sprintf(message, "%.1f,%.1f,%.1f,%.1f,%d \n\r", right_distance_mm, right_median_speed, left_distance_mm, left_median_speed, pwm);
        break;
    }

    // send via UART
    uart_print(message);
}



/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal = -1;

    // Initialize semaphores, queues,...
    xQueueActions = xQueueCreate( QUEUE_SIZE, sizeof(queue_message) );
    xQueueMeasurements = xQueueCreate( QUEUE_SIZE, sizeof(queue_message) );


    /* Initialize the board and peripherals */
    board_init();
    MotorInit();
    EncoderInit();
    uart_init(NULL);
    BumpInt_Init(NULL);


    if ( true ) {  //can be used to check the existence of FreeRTOS sync tools
        MotorStart(MOTOR_BOTH);
        MotorRoll(10, -10, 50);
        /* Create HeartBeat task */
        retVal = xTaskCreate(HeartBeatTask, "HeartBeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create CommandsTask task */
//        retVal = xTaskCreate(CommandsTask, "CommandsTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create Motor task */
//        retVal = xTaskCreate(MotorTask, "MotorTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Create SpeedMeasure task */
        retVal = xTaskCreate(SpeedMeasure, "SpeedMeasure", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
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
