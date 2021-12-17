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

/* Auto-generated by genmsg_cpp from file /home/hwpark/capstone2/px4-firmware/msg/commander_state.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define COMMANDER_STATE_MAIN_STATE_MANUAL 0
#define COMMANDER_STATE_MAIN_STATE_ALTCTL 1
#define COMMANDER_STATE_MAIN_STATE_POSCTL 2
#define COMMANDER_STATE_MAIN_STATE_AUTO_MISSION 3
#define COMMANDER_STATE_MAIN_STATE_AUTO_LOITER 4
#define COMMANDER_STATE_MAIN_STATE_AUTO_RTL 5
#define COMMANDER_STATE_MAIN_STATE_ACRO 6
#define COMMANDER_STATE_MAIN_STATE_OFFBOARD 7
#define COMMANDER_STATE_MAIN_STATE_STAB 8
#define COMMANDER_STATE_MAIN_STATE_AUTO_TAKEOFF 10
#define COMMANDER_STATE_MAIN_STATE_AUTO_LAND 11
#define COMMANDER_STATE_MAIN_STATE_AUTO_FOLLOW_TARGET 12
#define COMMANDER_STATE_MAIN_STATE_AUTO_PRECLAND 13
#define COMMANDER_STATE_MAIN_STATE_ORBIT 14
#define COMMANDER_STATE_MAIN_STATE_MAX 15

#endif


#ifdef __cplusplus
struct __EXPORT commander_state_s {
#else
struct commander_state_s {
#endif
	uint64_t timestamp;
	uint16_t main_state_changes;
	uint8_t main_state;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t MAIN_STATE_MANUAL = 0;
	static constexpr uint8_t MAIN_STATE_ALTCTL = 1;
	static constexpr uint8_t MAIN_STATE_POSCTL = 2;
	static constexpr uint8_t MAIN_STATE_AUTO_MISSION = 3;
	static constexpr uint8_t MAIN_STATE_AUTO_LOITER = 4;
	static constexpr uint8_t MAIN_STATE_AUTO_RTL = 5;
	static constexpr uint8_t MAIN_STATE_ACRO = 6;
	static constexpr uint8_t MAIN_STATE_OFFBOARD = 7;
	static constexpr uint8_t MAIN_STATE_STAB = 8;
	static constexpr uint8_t MAIN_STATE_AUTO_TAKEOFF = 10;
	static constexpr uint8_t MAIN_STATE_AUTO_LAND = 11;
	static constexpr uint8_t MAIN_STATE_AUTO_FOLLOW_TARGET = 12;
	static constexpr uint8_t MAIN_STATE_AUTO_PRECLAND = 13;
	static constexpr uint8_t MAIN_STATE_ORBIT = 14;
	static constexpr uint8_t MAIN_STATE_MAX = 15;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(commander_state);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const commander_state_s& message);
#endif
