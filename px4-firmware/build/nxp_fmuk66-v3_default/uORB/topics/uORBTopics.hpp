/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{191};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	action_request = 0,
	actuator_armed = 1,
	actuator_controls = 2,
	actuator_controls_0 = 3,
	actuator_controls_1 = 4,
	actuator_controls_2 = 5,
	actuator_controls_3 = 6,
	actuator_controls_status = 7,
	actuator_controls_status_0 = 8,
	actuator_controls_status_1 = 9,
	actuator_controls_virtual_fw = 10,
	actuator_controls_virtual_mc = 11,
	actuator_motors = 12,
	actuator_outputs = 13,
	actuator_servos = 14,
	actuator_test = 15,
	adc_report = 16,
	airspeed = 17,
	airspeed_validated = 18,
	airspeed_wind = 19,
	autotune_attitude_control_status = 20,
	battery_status = 21,
	camera_capture = 22,
	camera_status = 23,
	camera_trigger = 24,
	cellular_status = 25,
	collision_constraints = 26,
	collision_report = 27,
	commander_state = 28,
	control_allocator_status = 29,
	cpuload = 30,
	debug_array = 31,
	debug_key_value = 32,
	debug_value = 33,
	debug_vect = 34,
	differential_pressure = 35,
	distance_sensor = 36,
	ekf2_timestamps = 37,
	ekf_gps_drift = 38,
	esc_report = 39,
	esc_status = 40,
	estimator_attitude = 41,
	estimator_baro_bias = 42,
	estimator_event_flags = 43,
	estimator_global_position = 44,
	estimator_innovation_test_ratios = 45,
	estimator_innovation_variances = 46,
	estimator_innovations = 47,
	estimator_local_position = 48,
	estimator_odometry = 49,
	estimator_optical_flow_vel = 50,
	estimator_selector_status = 51,
	estimator_sensor_bias = 52,
	estimator_states = 53,
	estimator_status = 54,
	estimator_status_flags = 55,
	estimator_visual_odometry_aligned = 56,
	estimator_wind = 57,
	event = 58,
	failure_detector_status = 59,
	follow_target = 60,
	fw_virtual_attitude_setpoint = 61,
	generator_status = 62,
	geofence_result = 63,
	gimbal_device_attitude_status = 64,
	gimbal_device_information = 65,
	gimbal_device_set_attitude = 66,
	gimbal_manager_information = 67,
	gimbal_manager_set_attitude = 68,
	gimbal_manager_set_manual_control = 69,
	gimbal_manager_status = 70,
	gps_dump = 71,
	gps_inject_data = 72,
	heater_status = 73,
	home_position = 74,
	hover_thrust_estimate = 75,
	input_rc = 76,
	internal_combustion_engine_status = 77,
	iridiumsbd_status = 78,
	irlock_report = 79,
	landing_gear = 80,
	landing_target_innovations = 81,
	landing_target_pose = 82,
	led_control = 83,
	log_message = 84,
	logger_status = 85,
	mag_worker_data = 86,
	magnetometer_bias_estimate = 87,
	manual_control_input = 88,
	manual_control_setpoint = 89,
	manual_control_switches = 90,
	mavlink_log = 91,
	mc_virtual_attitude_setpoint = 92,
	mission = 93,
	mission_result = 94,
	mount_orientation = 95,
	navigator_mission_item = 96,
	obstacle_distance = 97,
	obstacle_distance_fused = 98,
	offboard_control_mode = 99,
	onboard_computer_status = 100,
	optical_flow = 101,
	orbit_status = 102,
	parameter_update = 103,
	ping = 104,
	position_controller_landing_status = 105,
	position_controller_status = 106,
	position_setpoint = 107,
	position_setpoint_triplet = 108,
	power_button_state = 109,
	power_monitor = 110,
	pwm_input = 111,
	px4io_status = 112,
	radio_status = 113,
	rate_ctrl_status = 114,
	rc_channels = 115,
	rc_parameter_map = 116,
	rpm = 117,
	rtl_time_estimate = 118,
	safety = 119,
	satellite_info = 120,
	sensor_accel = 121,
	sensor_accel_fifo = 122,
	sensor_baro = 123,
	sensor_combined = 124,
	sensor_correction = 125,
	sensor_gps = 126,
	sensor_gyro = 127,
	sensor_gyro_fft = 128,
	sensor_gyro_fifo = 129,
	sensor_mag = 130,
	sensor_preflight_mag = 131,
	sensor_selection = 132,
	sensors_status_imu = 133,
	system_power = 134,
	takeoff_status = 135,
	task_stack_info = 136,
	tecs_status = 137,
	telemetry_status = 138,
	test_motor = 139,
	timesync = 140,
	timesync_status = 141,
	trajectory_bezier = 142,
	trajectory_setpoint = 143,
	trajectory_waypoint = 144,
	transponder_report = 145,
	tune_control = 146,
	uavcan_parameter_request = 147,
	uavcan_parameter_value = 148,
	ulog_stream = 149,
	ulog_stream_ack = 150,
	vehicle_acceleration = 151,
	vehicle_air_data = 152,
	vehicle_angular_acceleration = 153,
	vehicle_angular_acceleration_setpoint = 154,
	vehicle_angular_velocity = 155,
	vehicle_angular_velocity_groundtruth = 156,
	vehicle_attitude = 157,
	vehicle_attitude_groundtruth = 158,
	vehicle_attitude_setpoint = 159,
	vehicle_command = 160,
	vehicle_command_ack = 161,
	vehicle_constraints = 162,
	vehicle_control_mode = 163,
	vehicle_global_position = 164,
	vehicle_global_position_groundtruth = 165,
	vehicle_gps_position = 166,
	vehicle_imu = 167,
	vehicle_imu_status = 168,
	vehicle_land_detected = 169,
	vehicle_local_position = 170,
	vehicle_local_position_groundtruth = 171,
	vehicle_local_position_setpoint = 172,
	vehicle_magnetometer = 173,
	vehicle_mocap_odometry = 174,
	vehicle_odometry = 175,
	vehicle_rates_setpoint = 176,
	vehicle_roi = 177,
	vehicle_status = 178,
	vehicle_status_flags = 179,
	vehicle_thrust_setpoint = 180,
	vehicle_torque_setpoint = 181,
	vehicle_trajectory_bezier = 182,
	vehicle_trajectory_waypoint = 183,
	vehicle_trajectory_waypoint_desired = 184,
	vehicle_vision_attitude = 185,
	vehicle_visual_odometry = 186,
	vtol_vehicle_status = 187,
	wheel_encoders = 188,
	wind = 189,
	yaw_estimator_status = 190,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
