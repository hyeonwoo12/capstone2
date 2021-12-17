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

/* Auto-generated by genmsg_cpp from file /home/hwpark/capstone2/px4-firmware/msg/estimator_status.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_estimator_status_fields[] = "\x89 timestamp;\x89 timestamp_sample;\x8a[3] vibe;\x8a[3] output_tracking_error;\x88 control_mode_flags;\x88 filter_fault_flags;\x8a pos_horiz_accuracy;\x8a pos_vert_accuracy;\x8a mag_test_ratio;\x8a vel_test_ratio;\x8a pos_test_ratio;\x8a hgt_test_ratio;\x8a tas_test_ratio;\x8a hagl_test_ratio;\x8a beta_test_ratio;\x8a time_slip;\x88 accel_device_id;\x88 gyro_device_id;\x88 baro_device_id;\x88 mag_device_id;\x87 gps_check_fail_flags;\x87 innovation_check_flags;\x87 solution_status_flags;\x86 reset_count_vel_ne;\x86 reset_count_vel_d;\x86 reset_count_pos_ne;\x86 reset_count_pod_d;\x86 reset_count_quat;\x8c pre_flt_fail_innov_heading;\x8c pre_flt_fail_innov_vel_horiz;\x8c pre_flt_fail_innov_vel_vert;\x8c pre_flt_fail_innov_height;\x8c pre_flt_fail_mag_field_disturbed;\x86 health_flags;\x86 timeout_flags;\x86[6] _padding0;";

ORB_DEFINE(estimator_status, struct estimator_status_s, 122, __orb_estimator_status_fields, static_cast<uint8_t>(ORB_ID::estimator_status));


void print_message(const orb_metadata *meta, const estimator_status_s& message)
{
	if (sizeof(message) != meta->o_size) {
		printf("unexpected message size for %s: %zu != %i\n", meta->o_name, sizeof(message), meta->o_size);
		return;
	}
	orb_print_message_internal(meta, &message, true);
}
