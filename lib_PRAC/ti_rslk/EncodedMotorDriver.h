
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <MotorConstants.h>
#include <EncoderConstants.h>

void EncodedMotorRoll(float left_revolutions, float right_revolutions, uint8_t motor_pwm);
void EncodeedMotorCancelRoll();
void EncodedMotorInit( void (*end_callback_fn)(void),  void (*interrupted_callback_fn)(void));
void EncodedMotorStart(motor_e motor);


void EncodedMotorGetSpeed(encoder_e encoder, uint32_t elapsed_ms, float * distance_mm, float * speed_mm_ms);
void GetMotorStatus(BaseType_t * point_l_motor, BaseType_t * point_r_motor, uint8_t * point_pwm);
