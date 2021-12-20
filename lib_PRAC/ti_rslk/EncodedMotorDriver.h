
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>


typedef struct{
    int32_t r_ticks;
    int32_t l_ticks;
}reporting_message;

void EncodedMotorRoll(float left_revolutions, float right_revolutions, uint8_t motor_pwm);
void EncodeedMotorCancelRoll();
void EncodedMotorInit( void (*end_callback_fn)(void),  void (*interrupted_callback_fn)(void), QueueHandle_t *  queueReportingR);
void EncodedMotorStart();


void GetMotorStatus(BaseType_t * point_l_motor, BaseType_t * point_r_motor, uint8_t * point_pwm);
