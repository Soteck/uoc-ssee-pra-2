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


#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "EncoderConstants.h"

/*----------------------------------------------------------------------------*/

#define COUNTS_PER_REVOLUTION           ( 360.0f )
#define COUNTS_PER_REVOLUTION_INV       ( -360.0f ) // Setting that as constant will prevent a lot of cpu cycles

/*----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------*/

typedef void (*encoder_callback_fn)(void);

/*----------------------------------------------------------------------------*/

void EncoderInit(void);
void EncoderGetSpeed(encoder_e encoder, uint32_t elapsed_ms, float * distance_mm, float * speed_mm_ms);
void AddCallBackForRotationCount(uint16_t rotationsNumber, void (*callback)(uint8_t));
void INIT_REV_CALLLBACK( void (*left_callback_fn)(void), void (*right_callback_fn)(void));
void INIT_DECIMAL_REV_CALLBACK( float l_rpm_fraction, float r_rpm_fraction, void (*left_callback_fn)(void), void (*right_callback_fn)(void));
void REMOVE_DECIMAL_REV_CALLBACK();

/*----------------------------------------------------------------------------*/
