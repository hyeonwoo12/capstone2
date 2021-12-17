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

/* Auto-generated by genmsg_cpp from file /home/hwpark/capstone2/px4-firmware/msg/internal_combustion_engine_status.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define INTERNAL_COMBUSTION_ENGINE_STATUS_STATE_STOPPED 0
#define INTERNAL_COMBUSTION_ENGINE_STATUS_STATE_STARTING 1
#define INTERNAL_COMBUSTION_ENGINE_STATUS_STATE_RUNNING 2
#define INTERNAL_COMBUSTION_ENGINE_STATUS_STATE_FAULT 3
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_GENERAL_ERROR 1
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED 2
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR 4
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_TEMPERATURE_SUPPORTED 8
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_TEMPERATURE_BELOW_NOMINAL 16
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_TEMPERATURE_ABOVE_NOMINAL 32
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_TEMPERATURE_OVERHEATING 64
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL 128
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_FUEL_PRESSURE_SUPPORTED 256
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_FUEL_PRESSURE_BELOW_NOMINAL 512
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_FUEL_PRESSURE_ABOVE_NOMINAL 1024
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_DETONATION_SUPPORTED 2048
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_DETONATION_OBSERVED 4096
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_MISFIRE_SUPPORTED 8192
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_MISFIRE_OBSERVED 16384
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_OIL_PRESSURE_SUPPORTED 32768
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_OIL_PRESSURE_BELOW_NOMINAL 65536
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_OIL_PRESSURE_ABOVE_NOMINAL 131072
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_DEBRIS_SUPPORTED 262144
#define INTERNAL_COMBUSTION_ENGINE_STATUS_FLAG_DEBRIS_DETECTED 524288
#define INTERNAL_COMBUSTION_ENGINE_STATUS_SPARK_PLUG_SINGLE 0
#define INTERNAL_COMBUSTION_ENGINE_STATUS_SPARK_PLUG_FIRST_ACTIVE 1
#define INTERNAL_COMBUSTION_ENGINE_STATUS_SPARK_PLUG_SECOND_ACTIVE 2
#define INTERNAL_COMBUSTION_ENGINE_STATUS_SPARK_PLUG_BOTH_ACTIVE 3

#endif


#ifdef __cplusplus
struct __EXPORT internal_combustion_engine_status_s {
#else
struct internal_combustion_engine_status_s {
#endif
	uint64_t timestamp;
	uint32_t flags;
	uint32_t engine_speed_rpm;
	float spark_dwell_time_ms;
	float atmospheric_pressure_kpa;
	float intake_manifold_pressure_kpa;
	float intake_manifold_temperature;
	float coolant_temperature;
	float oil_pressure;
	float oil_temperature;
	float fuel_pressure;
	float fuel_consumption_rate_cm3pm;
	float estimated_consumed_fuel_volume_cm3;
	float ignition_timing_deg;
	float injection_time_ms;
	float cylinder_head_temperature;
	float exhaust_gas_temperature;
	float lambda_coefficient;
	uint8_t state;
	uint8_t engine_load_percent;
	uint8_t throttle_position_percent;
	uint8_t ecu_index;
	uint8_t spark_plug_usage;
	uint8_t _padding0[7]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t STATE_STOPPED = 0;
	static constexpr uint8_t STATE_STARTING = 1;
	static constexpr uint8_t STATE_RUNNING = 2;
	static constexpr uint8_t STATE_FAULT = 3;
	static constexpr uint32_t FLAG_GENERAL_ERROR = 1;
	static constexpr uint32_t FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED = 2;
	static constexpr uint32_t FLAG_CRANKSHAFT_SENSOR_ERROR = 4;
	static constexpr uint32_t FLAG_TEMPERATURE_SUPPORTED = 8;
	static constexpr uint32_t FLAG_TEMPERATURE_BELOW_NOMINAL = 16;
	static constexpr uint32_t FLAG_TEMPERATURE_ABOVE_NOMINAL = 32;
	static constexpr uint32_t FLAG_TEMPERATURE_OVERHEATING = 64;
	static constexpr uint32_t FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL = 128;
	static constexpr uint32_t FLAG_FUEL_PRESSURE_SUPPORTED = 256;
	static constexpr uint32_t FLAG_FUEL_PRESSURE_BELOW_NOMINAL = 512;
	static constexpr uint32_t FLAG_FUEL_PRESSURE_ABOVE_NOMINAL = 1024;
	static constexpr uint32_t FLAG_DETONATION_SUPPORTED = 2048;
	static constexpr uint32_t FLAG_DETONATION_OBSERVED = 4096;
	static constexpr uint32_t FLAG_MISFIRE_SUPPORTED = 8192;
	static constexpr uint32_t FLAG_MISFIRE_OBSERVED = 16384;
	static constexpr uint32_t FLAG_OIL_PRESSURE_SUPPORTED = 32768;
	static constexpr uint32_t FLAG_OIL_PRESSURE_BELOW_NOMINAL = 65536;
	static constexpr uint32_t FLAG_OIL_PRESSURE_ABOVE_NOMINAL = 131072;
	static constexpr uint32_t FLAG_DEBRIS_SUPPORTED = 262144;
	static constexpr uint32_t FLAG_DEBRIS_DETECTED = 524288;
	static constexpr uint8_t SPARK_PLUG_SINGLE = 0;
	static constexpr uint8_t SPARK_PLUG_FIRST_ACTIVE = 1;
	static constexpr uint8_t SPARK_PLUG_SECOND_ACTIVE = 2;
	static constexpr uint8_t SPARK_PLUG_BOTH_ACTIVE = 3;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(internal_combustion_engine_status);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const internal_combustion_engine_status_s& message);
#endif
