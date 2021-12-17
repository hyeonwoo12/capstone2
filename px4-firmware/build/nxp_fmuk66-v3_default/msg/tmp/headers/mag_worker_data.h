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

/* Auto-generated by genmsg_cpp from file /home/hwpark/capstone2/px4-firmware/msg/mag_worker_data.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define MAG_WORKER_DATA_MAX_MAGS 4

#endif


#ifdef __cplusplus
struct __EXPORT mag_worker_data_s {
#else
struct mag_worker_data_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	uint64_t calibration_interval_perside_us;
	uint32_t done_count;
	uint32_t calibration_points_perside;
	uint32_t calibration_counter_total[4];
	float x[4];
	float y[4];
	float z[4];
	bool side_data_collected[4];
	uint8_t _padding0[4]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t MAX_MAGS = 4;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(mag_worker_data);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const mag_worker_data_s& message);
#endif
