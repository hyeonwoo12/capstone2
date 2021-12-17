/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/hwpark/capstone2/px4-firmware/msg/actuator_test.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define ACTUATOR_TEST_ACTION_RELEASE_CONTROL 0
#define ACTUATOR_TEST_ACTION_DO_CONTROL 1
#define ACTUATOR_TEST_FUNCTION_MOTOR1 101
#define ACTUATOR_TEST_MAX_NUM_MOTORS 8
#define ACTUATOR_TEST_FUNCTION_SERVO1 201
#define ACTUATOR_TEST_MAX_NUM_SERVOS 8
#define ACTUATOR_TEST_ORB_QUEUE_LENGTH 8

#endif


#ifdef __cplusplus
struct __EXPORT actuator_test_s {
#else
struct actuator_test_s {
#endif
	uint64_t timestamp;
	float value;
	uint32_t timeout_ms;
	uint16_t function;
	uint8_t action;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t ACTION_RELEASE_CONTROL = 0;
	static constexpr uint8_t ACTION_DO_CONTROL = 1;
	static constexpr uint8_t FUNCTION_MOTOR1 = 101;
	static constexpr uint8_t MAX_NUM_MOTORS = 8;
	static constexpr uint8_t FUNCTION_SERVO1 = 201;
	static constexpr uint8_t MAX_NUM_SERVOS = 8;
	static constexpr uint8_t ORB_QUEUE_LENGTH = 8;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(actuator_test);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const actuator_test_s& message);
#endif
