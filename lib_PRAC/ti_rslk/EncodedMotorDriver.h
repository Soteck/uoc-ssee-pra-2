
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <MotorConstants.h>
#include <EncoderConstants.h>

void EncodedMotorRoll(float left_revolutions, float right_revolutions, uint8_t motor_pwm);
void EncodeedMotorCancelRoll();
void EncodedMotorInit();
void EncodedMotorStart(motor_e motor);


void EncodedMotorGetSpeed(encoder_e encoder, uint32_t elapsed_ms, float * distance_mm, float * speed_mm_ms);
