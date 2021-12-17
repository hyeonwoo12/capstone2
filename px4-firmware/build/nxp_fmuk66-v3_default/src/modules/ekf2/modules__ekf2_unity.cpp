/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file airspeed_fusion.cpp
 * airspeed fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::fuseAirspeed()
{
	const float &vn = _state.vel(0); // Velocity in north direction
	const float &ve = _state.vel(1); // Velocity in east direction
	const float &vd = _state.vel(2); // Velocity in downwards direction
	const float &vwn = _state.wind_vel(0); // Wind speed in north direction
	const float &vwe = _state.wind_vel(1); // Wind speed in east direction

	// Variance for true airspeed measurement - (m/sec)^2
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			       math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_is_wind_dead_reckoning;

	// Intermediate variables
	const float HK0 = vn - vwn;
	const float HK1 = ve - vwe;
	const float HK2 = ecl::powf(HK0, 2) + ecl::powf(HK1, 2) + ecl::powf(vd, 2);
	const float v_tas_pred = sqrtf(HK2); // predicted airspeed

	//const float HK3 = powf(HK2, -1.0F/2.0F);
	if (v_tas_pred < 1.0f) {
		// calculation can be badly conditioned for very low airspeed values so don't fuse this time
		return;
	}

	const float HK3 = 1.0f / v_tas_pred;
	const float HK4 = HK0*HK3;
	const float HK5 = HK1*HK3;
	const float HK6 = 1.0F/HK2;
	const float HK7 = HK0*P(4,6) - HK0*P(6,22) + HK1*P(5,6) - HK1*P(6,23) + P(6,6)*vd;
	const float HK8 = HK1*P(5,23);
	const float HK9 = HK0*P(4,5) - HK0*P(5,22) + HK1*P(5,5) - HK8 + P(5,6)*vd;
	const float HK10 = HK1*HK6;
	const float HK11 = HK0*P(4,22);
	const float HK12 = HK0*P(4,4) - HK1*P(4,23) + HK1*P(4,5) - HK11 + P(4,6)*vd;
	const float HK13 = HK0*HK6;
	const float HK14 = -HK0*P(22,23) + HK0*P(4,23) - HK1*P(23,23) + HK8 + P(6,23)*vd;
	const float HK15 = -HK0*P(22,22) - HK1*P(22,23) + HK1*P(5,22) + HK11 + P(6,22)*vd;
	//const float HK16 = HK3/(-HK10*HK14 + HK10*HK9 + HK12*HK13 - HK13*HK15 + HK6*HK7*vd + R_TAS);

	// innovation variance - check for badly conditioned calculation
	_airspeed_innov_var = (-HK10 * HK14 + HK10 * HK9 + HK12 * HK13 - HK13 * HK15 + HK6 * HK7 * vd + R_TAS);

	if (_airspeed_innov_var < R_TAS) { //
		// Reset the estimator covariance matrix
		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindUsingAirspeed();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string);

		_fault_status.flags.bad_airspeed = true;

		return;
	}

	const float HK16 = HK3 / _airspeed_innov_var;
	_fault_status.flags.bad_airspeed = false;

	// Observation Jacobians
	SparseVector24f<4,5,6,22,23> Hfusion;
	Hfusion.at<4>() = HK4;
	Hfusion.at<5>() = HK5;
	Hfusion.at<6>() = HK3*vd;
	Hfusion.at<22>() = -HK4;
	Hfusion.at<23>() = -HK5;

	Vector24f Kfusion; // Kalman gain vector

	if (!update_wind_only) {
		// we have no other source of aiding, so use airspeed measurements to correct states
		for (unsigned row = 0; row <= 3; row++) {
			Kfusion(row) = HK16*(HK0*P(4,row) - HK0*P(row,22) + HK1*P(5,row) - HK1*P(row,23) + P(6,row)*vd);
		}

		Kfusion(4) = HK12*HK16;
		Kfusion(5) = HK16*HK9;
		Kfusion(6) = HK16*HK7;

		for (unsigned row = 7; row <= 21; row++) {
			Kfusion(row) = HK16*(HK0*P(4,row) - HK0*P(row,22) + HK1*P(5,row) - HK1*P(row,23) + P(6,row)*vd);
		}
	}

	Kfusion(22) = HK15*HK16;
	Kfusion(23) = HK14*HK16;

	// Calculate measurement innovation
	_airspeed_innov = v_tas_pred - _airspeed_sample_delayed.true_airspeed;

	// Compute the ratio of innovation to gate size
	_tas_test_ratio = sq(_airspeed_innov) / (sq(fmaxf(_params.tas_innov_gate, 1.0f)) * _airspeed_innov_var);

	// If the innovation consistency check fails then don't fuse the sample and indicate bad airspeed health
	if (_tas_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_airspeed = true;
		return;

	} else {
		_innov_check_fail_status.flags.reject_airspeed = false;
	}

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, _airspeed_innov);

	_fault_status.flags.bad_airspeed = !is_fused;

	if (is_fused) {
		_time_last_arsp_fuse = _time_last_imu;
	}
}

float Ekf::getTrueAirspeed() const
{
	return (_state.vel - Vector3f(_state.wind_vel(0), _state.wind_vel(1), 0.f)).norm();
}

void Ekf::resetWind()
{
	if (_control_status.flags.fuse_aspd) {
		resetWindUsingAirspeed();

	} else {
		resetWindToZero();
	}
}

/*
 * Reset the wind states using the current airspeed measurement, ground relative nav velocity, yaw angle and assumption of zero sideslip
*/
void Ekf::resetWindUsingAirspeed()
{
	const float euler_yaw = shouldUse321RotationSequence(_R_to_earth)
				? getEuler321Yaw(_state.quat_nominal)
				: getEuler312Yaw(_state.quat_nominal);

	// estimate wind using zero sideslip assumption and airspeed measurement if airspeed available
	_state.wind_vel(0) = _state.vel(0) - _airspeed_sample_delayed.true_airspeed * cosf(euler_yaw);
	_state.wind_vel(1) = _state.vel(1) - _airspeed_sample_delayed.true_airspeed * sinf(euler_yaw);

	resetWindCovarianceUsingAirspeed();

	_time_last_arsp_fuse = _time_last_imu;
}

void Ekf::resetWindToZero()
{
	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file baro_bias_estimator.cpp
 *
 * @author Mathieu Bresciani 	<mathieu@auterion.com>
 */

#include "baro_bias_estimator.hpp"

void BaroBiasEstimator::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	_state_var += _process_var * dt * dt;
	constrainStateVar();

	if (dt > FLT_EPSILON && fabsf(_dt - dt) > 0.001f) {
		_signed_innov_test_ratio_lpf.setParameters(dt, _lpf_time_constant);
		_dt = dt;
	}
}

void BaroBiasEstimator::constrainStateVar()
{
	_state_var = math::constrain(_state_var, 1e-8f, _state_var_max);
}

void BaroBiasEstimator::fuseBias(const float measurement, const float measurement_var)
{
	const float innov_var = _state_var + measurement_var;
	const float innov = measurement - _state;
	const float K = _state_var / innov_var;
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);

	if (isTestRatioPassing(innov_test_ratio)) {
		updateState(K, innov);
		updateStateCovariance(K);

	}

	if (isLargeOffsetDetected()) {
		// A bias in the state has been detected by the innovation
		// sequence check.
		bumpStateVariance();
	}

	const float signed_innov_test_ratio = matrix::sign(innov) * innov_test_ratio;
	_signed_innov_test_ratio_lpf.update(math::constrain(signed_innov_test_ratio, -1.f, 1.f));

	_status = packStatus(innov, innov_var, innov_test_ratio);
}

inline float BaroBiasEstimator::computeInnovTestRatio(const float innov, const float innov_var) const
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}

inline bool BaroBiasEstimator::isTestRatioPassing(const float innov_test_ratio) const
{
	return innov_test_ratio < 1.f;
}

inline void BaroBiasEstimator::updateState(const float K, const float innov)
{
	_state = math::constrain(_state + K * innov, -_state_max, _state_max);
}

inline void BaroBiasEstimator::updateStateCovariance(const float K)
{
	_state_var -= K * _state_var;
	constrainStateVar();
}

inline bool BaroBiasEstimator::isLargeOffsetDetected() const
{
	return fabsf(_signed_innov_test_ratio_lpf.getState()) > 0.2f;
}

inline void BaroBiasEstimator::bumpStateVariance()
{
	_state_var += _process_var_boost_gain * _process_var * _dt * _dt;
}

inline BaroBiasEstimator::status BaroBiasEstimator::packStatus(const float innov, const float innov_var,
		const float innov_test_ratio) const
{
	// Send back status for logging
	status ret{};
	ret.bias = _state;
	ret.bias_var = _state_var;
	ret.innov = innov;
	ret.innov_var = innov_var;
	ret.innov_test_ratio = innov_test_ratio;

	return ret;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes()
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		const Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(math::radians(3.0f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = nullptr;

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";

			} else {
				height_source = "unknown";

			}

			if (height_source) {
				ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
					 (unsigned long long)_imu_sample_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);
			}
		}
	}

	// check for intermittent data (before pop_first_older_than)
	const baroSample &baro_init = _baro_buffer.get_newest();
	_baro_hgt_faulty = !isRecent(baro_init.time_us, 2 * BARO_MAX_INTERVAL);

	const gpsSample &gps_init = _gps_buffer.get_newest();
	_gps_hgt_intermittent = !isRecent(gps_init.time_us, 2 * GPS_MAX_INTERVAL);

	// check for arrival of new sensor data at the fusion time horizon
	_time_prev_gps_us = _gps_sample_delayed.time_us;
	_gps_data_ready = _gps_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed);
	_mag_data_ready = _mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed);

	if (_mag_data_ready) {
		_mag_lpf.update(_mag_sample_delayed.mag);

		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z && (_params.mag_declination_source & MASK_USE_GEO_DECL) && (_NED_origin_initialised
				|| PX4_ISFINITE(_mag_declination_gps))) {

			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			_mag_sample_delayed.mag(2) = calculate_synthetic_mag_z_measurement(_mag_sample_delayed.mag, mag_earth_pred);
			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}
	}

	_delta_time_baro_us = _baro_sample_delayed.time_us;
	_baro_data_ready = _baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);

	// if we have a new baro sample save the delta time between this sample and the last sample which is
	// used below for baro offset calculations
	if (_baro_data_ready) {
		_delta_time_baro_us = _baro_sample_delayed.time_us - _delta_time_baro_us;
	}

	{
		// Get range data from buffer and check validity
		const bool is_rng_data_ready = _range_buffer.pop_first_older_than(_imu_sample_delayed.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(is_rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(_imu_sample_delayed.time_us, _R_to_earth);
	}

	if (_range_sensor.isDataHealthy()) {
		// correct the range data for position offset relative to the IMU
		const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
		_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());
	}

	// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
	// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
	// in this case we need to empty the buffer
	if (!_flow_data_ready || !_control_status.flags.opt_flow) {
		_flow_data_ready = _flow_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed);
	}

	// check if we should fuse flow data for terrain estimation
	if (!_flow_for_terrain_data_ready && _flow_data_ready && _control_status.flags.in_air) {
		// TODO: WARNING, _flow_data_ready can be modified in controlOpticalFlowFusion
		// due to some checks failing
		// only fuse flow for terrain if range data hasn't been fused for 5 seconds
		_flow_for_terrain_data_ready = isTimedOut(_time_last_hagl_fuse, (uint64_t)5E6);
		// only fuse flow for terrain if the main filter is not fusing flow and we are using gps
		_flow_for_terrain_data_ready &= (!_control_status.flags.opt_flow && _control_status.flags.gps);
	}

	_ev_data_ready = _ext_vision_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed);
	_tas_data_ready = _airspeed_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed);

	// check for height sensor timeouts and reset and change sensor if necessary
	controlHeightSensorTimeouts();

	// control use of observations for aiding
	controlMagFusion();
	controlOpticalFlowFusion();
	controlGpsFusion();
	controlAirDataFusion();
	controlBetaFusion();
	controlDragFusion();
	controlHeightFusion();

	// Additional data odoemtery data from an external estimator can be fused.
	controlExternalVisionFusion();

	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	update_deadreckoning_status();
}

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {

		if (_inhibit_ev_yaw_use) {
			stopEvYawFusion();
		}

		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if ((_params.fusion_mode & MASK_ROTATE_EV) && ((_params.fusion_mode & MASK_USE_EVPOS)
				|| (_params.fusion_mode & MASK_USE_EVVEL)) && !_control_status.flags.ev_yaw) {

			// rotate EV measurements into the EKF Navigation frame
			calcExtVisRotMat();
		}

		// external vision aiding selection logic
		if (_control_status.flags.tilt_align && _control_status.flags.yaw_align) {

			// check for a external vision measurement that has fallen behind the fusion time horizon
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				// turn on use of external vision measurements for position
				if (_params.fusion_mode & MASK_USE_EVPOS && !_control_status.flags.ev_pos) {
					startEvPosFusion();
				}

				// turn on use of external vision measurements for velocity
				if (_params.fusion_mode & MASK_USE_EVVEL && !_control_status.flags.ev_vel) {
					startEvVelFusion();
				}
			}
		}

		// external vision yaw aiding selection logic
		if (!_inhibit_ev_yaw_use && (_params.fusion_mode & MASK_USE_EVYAW) && !_control_status.flags.ev_yaw
		    && _control_status.flags.tilt_align) {

			// don't start using EV data unless data is arriving frequently
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				if (resetYawToEv()) {
					_control_status.flags.yaw_align = true;
					startEvYawFusion();
				}
			}
		}

		// determine if we should use the horizontal position observations
		if (_control_status.flags.ev_pos) {

			Vector3f ev_pos_obs_var;
			Vector2f ev_pos_innov_gates;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_ev_sample_delayed.pos -= pos_offset_earth;

			// Use an incremental position fusion method for EV position data if GPS is also used
			if (_params.fusion_mode & MASK_USE_GPS) {
				_fuse_hpos_as_odom = true;

			} else {
				_fuse_hpos_as_odom = false;
			}

			if (_fuse_hpos_as_odom) {
				if (!_hpos_prev_available) {
					// no previous observation available to calculate position change
					_hpos_prev_available = true;

				} else {
					// calculate the change in position since the last measurement
					Vector3f ev_delta_pos = _ev_sample_delayed.pos - _pos_meas_prev;

					// rotate measurement into body frame is required when fusing with GPS
					ev_delta_pos = _R_ev_to_ekf * ev_delta_pos;

					// use the change in position since the last measurement
					_ev_pos_innov(0) = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
					_ev_pos_innov(1) = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

					// observation 1-STD error, incremental pos observation is expected to have more uncertainty
					Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
					ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
					ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));
				}

				// record observation and estimate for use next time
				_pos_meas_prev = _ev_sample_delayed.pos;
				_hpos_pred_prev = _state.pos.xy();

			} else {
				// use the absolute position
				Vector3f ev_pos_meas = _ev_sample_delayed.pos;
				Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);

				if (_params.fusion_mode & MASK_ROTATE_EV) {
					ev_pos_meas = _R_ev_to_ekf * ev_pos_meas;
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
				}

				_ev_pos_innov(0) = _state.pos(0) - ev_pos_meas(0);
				_ev_pos_innov(1) = _state.pos(1) - ev_pos_meas(1);

				ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.01f));
				ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.01f));

				// check if we have been deadreckoning too long
				if (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)) {
					// only reset velocity if we have no another source of aiding constraining it
					if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
					    isTimedOut(_time_last_hor_vel_fuse, (uint64_t)1E6)) {
						resetVelocity();
					}

					resetHorizontalPosition();
				}
			}

			// innovation gate size
			ev_pos_innov_gates(0) = fmaxf(_params.ev_pos_innov_gate, 1.0f);

			fuseHorizontalPosition(_ev_pos_innov, ev_pos_innov_gates, ev_pos_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio);
		}

		// determine if we should use the velocity observations
		if (_control_status.flags.ev_vel) {

			Vector2f ev_vel_innov_gates;

			_last_vel_obs = getVisionVelocityInEkfFrame();
			_ev_vel_innov = _state.vel - _last_vel_obs;

			// check if we have been deadreckoning too long
			if (isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)) {
				// only reset velocity if we have no another source of aiding constraining it
				if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
				    isTimedOut(_time_last_hor_pos_fuse, (uint64_t)1E6)) {
					resetVelocity();
				}
			}

			_last_vel_obs_var = matrix::max(getVisionVelocityVarianceInEkfFrame(), sq(0.05f));

			ev_vel_innov_gates.setAll(fmaxf(_params.ev_vel_innov_gate, 1.0f));

			fuseHorizontalVelocity(_ev_vel_innov, ev_vel_innov_gates, _last_vel_obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
			fuseVerticalVelocity(_ev_vel_innov, ev_vel_innov_gates, _last_vel_obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
		}

		// determine if we should use the yaw observation
		if (_control_status.flags.ev_yaw) {
			fuseHeading();
		}

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvFusion();
		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlOpticalFlowFusion()
{
	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		updateOnGroundMotionForOpticalFlowChecks();

	} else {
		resetOnGroundMotionForOpticalFlowChecks();
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	_imu_del_ang_of += _imu_sample_delayed.delta_ang - _state.delta_ang_bias;
	_delta_time_of += _imu_sample_delayed.delta_ang_dt;

	if (_flow_data_ready) {
		const bool is_quality_good = (_flow_sample_delayed.quality >= _params.flow_qual_min);
		const bool is_magnitude_good = !_flow_sample_delayed.flow_xy_rad.longerThan(_flow_sample_delayed.dt * _flow_max_rate);
		const bool is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);

		const float delta_time_min = fmaxf(0.7f * _delta_time_of, 0.001f);
		const float delta_time_max = fminf(1.3f * _delta_time_of, 0.2f);
		const bool is_delta_time_good = _flow_sample_delayed.dt >= delta_time_min && _flow_sample_delayed.dt <= delta_time_max;
		const bool is_body_rate_comp_available = calcOptFlowBodyRateComp();

		if (is_quality_good
		    && is_magnitude_good
		    && is_tilt_good
		    && is_body_rate_comp_available
		    && is_delta_time_good) {
			// compensate for body motion to give a LOS rate
			_flow_compensated_XY_rad = _flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_xyz.xy();

		} else if (!_control_status.flags.in_air) {

			if (!is_delta_time_good) {
				// handle special case of SITL and PX4Flow where dt is forced to
				// zero when the quaity is 0
				_flow_sample_delayed.dt = delta_time_min;
			}

			// don't allow invalid flow gyro_xyz to propagate
			if (!is_body_rate_comp_available) {
				if (!PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(1)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(2))) {
					_flow_sample_delayed.gyro_xyz.zero();
				}
			}

			// when on the ground with poor flow quality,
			// assume zero ground relative velocity and LOS rate
			_flow_compensated_XY_rad.setZero();

		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
			_flow_for_terrain_data_ready = false; // TODO: find a better place
		}
	}

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_data_ready) {
		// Inhibit flow use if motion is un-suitable or we have good quality GPS
		// Apply hysteresis to prevent rapid mode switching
		const float gps_err_norm_lim = _control_status.flags.opt_flow ? 0.7f : 1.0f;

		// Check if we are in-air and require optical flow to control position drift
		const bool is_flow_required = _control_status.flags.in_air
					      && (_is_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
						  || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)
						  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad


		// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
		const bool preflight_motion_not_ok = !_control_status.flags.in_air
						     && ((_imu_sample_delayed.time_us > (_time_good_motion_us + (uint64_t)1E5))
								     || (_imu_sample_delayed.time_us < (_time_bad_motion_us + (uint64_t)5E6)));
		const bool flight_condition_not_ok = _control_status.flags.in_air && !isTerrainEstimateValid();

		_inhibit_flow_use = ((preflight_motion_not_ok || flight_condition_not_ok) && !is_flow_required)
				    || !_control_status.flags.tilt_align;

		// Handle cases where we are using optical flow but we should not use it anymore
		if (_control_status.flags.opt_flow) {
			if (!(_params.fusion_mode & MASK_USE_OF)
			    || _inhibit_flow_use) {

				stopFlowFusion();
				return;
			}
		}

		// optical flow fusion mode selection logic
		if ((_params.fusion_mode & MASK_USE_OF) // optical flow has been selected by the user
		    && !_control_status.flags.opt_flow // we are not yet using flow data
		    && !_inhibit_flow_use) {
			// If the heading is valid and use is not inhibited , start using optical flow aiding
			if (_control_status.flags.yaw_align) {
				// set the flag and reset the fusion timeout
				_control_status.flags.opt_flow = true;
				_time_last_of_fuse = _time_last_imu;

				// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
				const bool flow_aid_only = !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow);

				if (flow_aid_only) {
					resetVelocity();
					resetHorizontalPosition();
				}
			}
		}

		if (_control_status.flags.opt_flow) {
			// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
			if (_imu_sample_delayed.time_us > (_flow_sample_delayed.time_us - uint32_t(1e6f * _flow_sample_delayed.dt) / 2)) {
				// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
				// but use a relaxed time criteria to enable it to coast through bad range finder data
				if (isRecent(_time_last_hagl_fuse, (uint64_t)10e6)) {
					fuseOptFlow();
					_last_known_posNE = _state.pos.xy();
				}

				_flow_data_ready = false;
			}

			// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
			if (isTimedOut(_time_last_of_fuse, _params.reset_timeout_max)
			    && !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {

				resetVelocity();
				resetHorizontalPosition();
			}
		}

	} else if (_control_status.flags.opt_flow
		   && (_imu_sample_delayed.time_us >  _flow_sample_delayed.time_us + (uint64_t)10e6)) {

		stopFlowFusion();
	}
}

void Ekf::updateOnGroundMotionForOpticalFlowChecks()
{
	// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
	const float accel_norm = _accel_vec_filt.norm();

	const bool motion_is_excessive = ((accel_norm > (CONSTANTS_ONE_G * 1.5f)) // upper g limit
					  || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // lower g limit
					  || (_ang_rate_magnitude_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					  || (_R_to_earth(2, 2) < cosf(math::radians(30.0f)))); // tilted excessively

	if (motion_is_excessive) {
		_time_bad_motion_us = _imu_sample_delayed.time_us;

	} else {
		_time_good_motion_us = _imu_sample_delayed.time_us;
	}
}

void Ekf::resetOnGroundMotionForOpticalFlowChecks()
{
	_time_bad_motion_us = 0;
	_time_good_motion_us = _imu_sample_delayed.time_us;
}

void Ekf::controlGpsYawFusion(bool gps_checks_passing, bool gps_checks_failing)
{
	if (!(_params.fusion_mode & MASK_USE_GPSYAW)
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(_gps_sample_delayed.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = !gps_checks_failing;

		const bool is_gps_yaw_data_intermittent = !isRecent(_time_last_gps_yaw_data, 2 * GPS_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& gps_checks_passing
				&& !is_gps_yaw_data_intermittent
				&& !_gps_hgt_intermittent;

		_time_last_gps_yaw_data = _time_last_imu;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw();

				const bool is_fusion_failing = isTimedOut(_time_last_gps_yaw_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps();

						if (_control_status.flags.in_air) {
							_nb_gps_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						_control_status.flags.gps_yaw_fault = true;
						stopGpsYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					}

					// TODO: should we give a new reset credit when the fusion does not fail for some time?
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				startGpsYawFusion();

				if (_control_status.flags.gps_yaw) {
					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw && isTimedOut(_time_last_gps_yaw_data, _params.reset_timeout_max)) {
		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}

	// Before takeoff, we do not want to continue to rely on the current heading
	// if we had to stop the fusion
	if (!_control_status.flags.in_air
	    && !_control_status.flags.gps_yaw
	    && _control_status_prev.flags.gps_yaw) {
		_control_status.flags.yaw_align = false;
	}
}

void Ekf::controlHeightSensorTimeouts()
{
	/*
	 * Handle the case where we have not fused height measurements recently and
	 * uncertainty exceeds the max allowable. Reset using the best available height
	 * measurement source, continue using it after the reset and declare the current
	 * source failed if we have switched.
	*/

	checkVerticalAccelerationHealth();

	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	// in vision hgt mode check for vision data
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, (uint64_t)5e6);

	if (hgt_fusion_timeout || continuous_bad_accel_hgt) {

		bool request_height_reset = false;
		const char *failing_height_source = nullptr;
		const char *new_height_source = nullptr;

		if (_control_status.flags.baro_hgt) {
			// check if GPS height is available
			const gpsSample &gps_init = _gps_buffer.get_newest();
			const bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			// check for inertial sensing errors in the last BADACC_PROBATION seconds
			const bool prev_bad_vert_accel = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

			// reset to GPS if adequate GPS data is available and the timeout cannot be blamed on IMU data
			const bool reset_to_gps = !_gps_hgt_intermittent &&
						  ((gps_hgt_accurate && !prev_bad_vert_accel) || _baro_hgt_faulty);

			if (reset_to_gps) {
				// set height sensor health
				_baro_hgt_faulty = true;

				startGpsHgtFusion();

				request_height_reset = true;
				failing_height_source = "baro";
				new_height_source = "gps";

			} else if (!_baro_hgt_faulty) {
				request_height_reset = true;
				failing_height_source = "baro";
				new_height_source = "baro";
			}

		} else if (_control_status.flags.gps_hgt) {
			// check if GPS height is available
			const gpsSample &gps_init = _gps_buffer.get_newest();
			const bool gps_hgt_accurate = (gps_init.vacc < _params.req_vacc);

			// check the baro height source for consistency and freshness
			const baroSample &baro_init = _baro_buffer.get_newest();
			const float baro_innov = _state.pos(2) - (_hgt_sensor_offset - baro_init.hgt + _baro_hgt_offset);
			const bool baro_data_consistent = fabsf(baro_innov) < (sq(_params.baro_noise) + P(9, 9)) * sq(_params.baro_innov_gate);

			// if baro data is acceptable and GPS data is inaccurate, reset height to baro
			const bool reset_to_baro = !_baro_hgt_faulty &&
						   ((baro_data_consistent && !gps_hgt_accurate) || _gps_hgt_intermittent);

			if (reset_to_baro) {
				startBaroHgtFusion();

				request_height_reset = true;
				failing_height_source = "gps";
				new_height_source = "baro";

			} else if (!_gps_hgt_intermittent) {
				request_height_reset = true;
				failing_height_source = "gps";
				new_height_source = "gps";
			}

		} else if (_control_status.flags.rng_hgt) {

			if (_range_sensor.isHealthy()) {
				request_height_reset = true;
				failing_height_source = "rng";
				new_height_source = "rng";

			} else if (!_baro_hgt_faulty) {
				startBaroHgtFusion();

				request_height_reset = true;
				failing_height_source = "rng";
				new_height_source = "baro";
			}

		} else if (_control_status.flags.ev_hgt) {
			// check if vision data is available
			const extVisionSample &ev_init = _ext_vision_buffer.get_newest();
			const bool ev_data_available = isRecent(ev_init.time_us, 2 * EV_MAX_INTERVAL);

			if (ev_data_available) {
				request_height_reset = true;
				failing_height_source = "ev";
				new_height_source = "ev";

			} else if (_range_sensor.isHealthy()) {
				// Fallback to rangefinder data if available
				setControlRangeHeight();
				request_height_reset = true;
				failing_height_source = "ev";
				new_height_source = "rng";

			} else if (!_baro_hgt_faulty) {
				startBaroHgtFusion();

				request_height_reset = true;
				failing_height_source = "ev";
				new_height_source = "baro";
			}
		}

		if (failing_height_source && new_height_source) {
			_warning_events.flags.height_sensor_timeout = true;
			ECL_WARN("%s hgt timeout - reset to %s", failing_height_source, new_height_source);
		}

		// Reset vertical position and velocity states to the last measurement
		if (request_height_reset) {
			resetHeight();
			// Reset the timout timer
			_time_last_hgt_fuse = _time_last_imu;
		}
	}
}

void Ekf::checkVerticalAccelerationHealth()
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.
	// Don't use stale innovation data.
	bool is_inertial_nav_falling = false;
	bool are_vertical_pos_and_vel_independant = false;

	if (isRecent(_vert_pos_fuse_attempt_time_us, 1000000)) {
		if (isRecent(_vert_vel_fuse_time_us, 1000000)) {
			// If vertical position and velocity come from independent sensors then we can
			// trust them more if they disagree with the IMU, but need to check that they agree
			const bool using_gps_for_both = _control_status.flags.gps_hgt && _control_status.flags.gps;
			const bool using_ev_for_both = _control_status.flags.ev_hgt && _control_status.flags.ev_vel;
			are_vertical_pos_and_vel_independant = !(using_gps_for_both || using_ev_for_both);
			is_inertial_nav_falling |= _vert_vel_innov_ratio > _params.vert_innov_test_lim && _vert_pos_innov_ratio > 0.0f;
			is_inertial_nav_falling |= _vert_pos_innov_ratio > _params.vert_innov_test_lim && _vert_vel_innov_ratio > 0.0f;

		} else {
			// only height sensing available
			is_inertial_nav_falling = _vert_pos_innov_ratio > _params.vert_innov_test_lim;
		}
	}

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1000 / FILTER_UPDATE_PERIOD_MS;
	const bool is_clipping = _imu_sample_delayed.delta_vel_clipping[0] ||
				 _imu_sample_delayed.delta_vel_clipping[1] ||
				 _imu_sample_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

	} else if (_clip_counter > 0) {
		_clip_counter--;
	}

	const bool is_clipping_frequently = _clip_counter > 0;

	// if vertical velocity and position are independent and agree, then do not require evidence of clipping if
	// innovations are large
	const bool bad_vert_accel = (are_vertical_pos_and_vel_independant || is_clipping_frequently) && is_inertial_nav_falling;

	if (bad_vert_accel) {
		_time_bad_vert_accel =  _time_last_imu;

	} else {
		_time_good_vert_accel = _time_last_imu;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

void Ekf::controlHeightFusion()
{
	checkRangeAidSuitability();
	const bool do_range_aid = (_params.range_aid == 1) && isRangeAidSuitable();

	bool fuse_height = false;

	switch (_params.vdist_sensor_type) {
	default:
		ECL_ERR("Invalid hgt mode: %d", _params.vdist_sensor_type);

	// FALLTHROUGH
	case VDIST_SENSOR_BARO:
		if (do_range_aid && _range_sensor.isDataHealthy()) {
			setControlRangeHeight();
			fuse_height = true;

			// we have just switched to using range finder, calculate height sensor offset such that current
			// measurement matches our current height estimate
			if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
				_hgt_sensor_offset = _terrain_vpos;
			}

		} else if (!do_range_aid && _baro_data_ready && !_baro_hgt_faulty) {
			startBaroHgtFusion();
			fuse_height = true;

		} else if (_control_status.flags.gps_hgt && _gps_data_ready && !_gps_hgt_intermittent) {
			// switch to gps if there was a reset to gps
			fuse_height = true;

			// we have just switched to using gps height, calculate height sensor offset such that current
			// measurement matches our current height estimate
			if (_control_status_prev.flags.gps_hgt != _control_status.flags.gps_hgt) {
				_hgt_sensor_offset = _gps_sample_delayed.hgt - _gps_alt_ref + _state.pos(2);
			}
		}

		break;

	case VDIST_SENSOR_RANGE:
		if (_range_sensor.isDataHealthy()) {
			setControlRangeHeight();
			fuse_height = true;

			if (_control_status_prev.flags.rng_hgt != _control_status.flags.rng_hgt) {
				// we have just switched to using range finder, calculate height sensor offset such that current
				// measurement matches our current height estimate
				// use the parameter rng_gnd_clearance if on ground to avoid a noisy offset initialization (e.g. sonar)
				if (_control_status.flags.in_air && isTerrainEstimateValid()) {
					_hgt_sensor_offset = _terrain_vpos;

				} else if (_control_status.flags.in_air) {
					_hgt_sensor_offset = _range_sensor.getDistBottom() + _state.pos(2);

				} else {
					_hgt_sensor_offset = _params.rng_gnd_clearance;
				}
			}

		} else if (_control_status.flags.baro_hgt && _baro_data_ready && !_baro_hgt_faulty) {
			// fuse baro data if there was a reset to baro
			fuse_height = true;
		}

		break;

	case VDIST_SENSOR_GPS:

		// NOTE: emergency fallback due to extended loss of currently selected sensor data or failure
		// to pass innovation cinsistency checks is handled elsewhere in Ekf::controlHeightSensorTimeouts.
		// Do switching between GPS and rangefinder if using range finder as a height source when close
		// to ground and moving slowly. Also handle switch back from emergency Baro sensor when GPS recovers.
		if (!_control_status_prev.flags.rng_hgt && do_range_aid && _range_sensor.isDataHealthy()) {
			setControlRangeHeight();

			// we have just switched to using range finder, calculate height sensor offset such that current
			// measurement matches our current height estimate
			_hgt_sensor_offset = _terrain_vpos;

		} else if (_control_status_prev.flags.rng_hgt && !do_range_aid) {
			// must stop using range finder so find another sensor now
			if (!_gps_hgt_intermittent && _gps_checks_passed) {
				// GPS quality OK
				startGpsHgtFusion();

			} else if (!_baro_hgt_faulty) {
				// Use baro as a fallback
				startBaroHgtFusion();
			}

		} else if (_control_status.flags.baro_hgt && !do_range_aid && !_gps_hgt_intermittent && _gps_checks_passed) {
			// In baro fallback mode and GPS has recovered so start using it
			startGpsHgtFusion();
		}

		if (_control_status.flags.gps_hgt && _gps_data_ready) {
			fuse_height = true;

		} else if (_control_status.flags.rng_hgt && _range_sensor.isDataHealthy()) {
			fuse_height = true;

		} else if (_control_status.flags.baro_hgt && _baro_data_ready && !_baro_hgt_faulty) {
			fuse_height = true;
		}

		break;

	case VDIST_SENSOR_EV:

		// don't start using EV data unless data is arriving frequently, do not reset if pref mode was height
		if (!_control_status.flags.ev_hgt && isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
			fuse_height = true;
			setControlEVHeight();

			if (!_control_status_prev.flags.rng_hgt) {
				resetHeight();
			}
		}

		if (_control_status.flags.ev_hgt && _ev_data_ready) {
			fuse_height = true;

		} else if (_control_status.flags.rng_hgt && _range_sensor.isDataHealthy()) {
			fuse_height = true;

		} else if (_control_status.flags.baro_hgt && _baro_data_ready && !_baro_hgt_faulty) {
			fuse_height = true;
		}

		break;
	}

	updateBaroHgtBias();
	updateBaroHgtOffset();

	if (_control_status.flags.rng_hgt
	    && isTimedOut(_time_last_hgt_fuse, 2 * RNG_MAX_INTERVAL)
	    && !_range_sensor.isDataHealthy()
	    && _range_sensor.isRegularlySendingData()
	    && !_control_status.flags.in_air) {

		// If we are supposed to be using range finder data as the primary height sensor, have missed or rejected measurements
		// and are on the ground, then synthesise a measurement at the expected on ground value
		_range_sensor.setRange(_params.rng_gnd_clearance);
		_range_sensor.setDataReadiness(true);
		_range_sensor.setValidity(true); // bypass the checks

		fuse_height = true;
	}

	if (fuse_height) {
		if (_control_status.flags.baro_hgt) {
			Vector2f baro_hgt_innov_gate;
			Vector3f baro_hgt_obs_var;

			// vertical position innovation - baro measurement has opposite sign to earth z axis
			const float unbiased_baro = _baro_sample_delayed.hgt - _baro_b_est.getBias();
			_baro_hgt_innov(2) = _state.pos(2) + unbiased_baro - _baro_hgt_offset;
			// observation variance - user parameter defined
			baro_hgt_obs_var(2) = sq(fmaxf(_params.baro_noise, 0.01f));
			// innovation gate size
			baro_hgt_innov_gate(1) = fmaxf(_params.baro_innov_gate, 1.0f);

			// Compensate for positive static pressure transients (negative vertical position innovations)
			// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
			const float deadzone_start = 0.0f;
			const float deadzone_end = deadzone_start + _params.gnd_effect_deadzone;

			if (_control_status.flags.gnd_effect) {
				if (_baro_hgt_innov(2) < -deadzone_start) {
					if (_baro_hgt_innov(2) <= -deadzone_end) {
						_baro_hgt_innov(2) += deadzone_end;

					} else {
						_baro_hgt_innov(2) = -deadzone_start;
					}
				}
			}

			// fuse height information
			fuseVerticalPosition(_baro_hgt_innov, baro_hgt_innov_gate,
					     baro_hgt_obs_var, _baro_hgt_innov_var, _baro_hgt_test_ratio);

		} else if (_control_status.flags.gps_hgt) {
			Vector2f gps_hgt_innov_gate;
			Vector3f gps_hgt_obs_var;
			// vertical position innovation - gps measurement has opposite sign to earth z axis
			_gps_pos_innov(2) = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _hgt_sensor_offset;
			gps_hgt_obs_var(2) = getGpsHeightVariance();
			// innovation gate size
			gps_hgt_innov_gate(1) = fmaxf(_params.baro_innov_gate, 1.0f);
			// fuse height information
			fuseVerticalPosition(_gps_pos_innov, gps_hgt_innov_gate,
					     gps_hgt_obs_var, _gps_pos_innov_var, _gps_pos_test_ratio);

		} else if (_control_status.flags.rng_hgt) {
			Vector2f rng_hgt_innov_gate;
			Vector3f rng_hgt_obs_var;
			// use range finder with tilt correction
			_rng_hgt_innov(2) = _state.pos(2) - (-math::max(_range_sensor.getDistBottom(),
							     _params.rng_gnd_clearance)) - _hgt_sensor_offset;
			// observation variance - user parameter defined
			rng_hgt_obs_var(2) = fmaxf(sq(_params.range_noise)
						   + sq(_params.range_noise_scaler * _range_sensor.getDistBottom()), 0.01f);
			// innovation gate size
			rng_hgt_innov_gate(1) = fmaxf(_params.range_innov_gate, 1.0f);
			// fuse height information
			fuseVerticalPosition(_rng_hgt_innov, rng_hgt_innov_gate,
					     rng_hgt_obs_var, _rng_hgt_innov_var, _rng_hgt_test_ratio);

		} else if (_control_status.flags.ev_hgt) {
			Vector2f ev_hgt_innov_gate;
			Vector3f ev_hgt_obs_var;
			// calculate the innovation assuming the external vision observation is in local NED frame
			_ev_pos_innov(2) = _state.pos(2) - _ev_sample_delayed.pos(2);
			// observation variance - defined externally
			ev_hgt_obs_var(2) = fmaxf(_ev_sample_delayed.posVar(2), sq(0.01f));
			// innovation gate size
			ev_hgt_innov_gate(1) = fmaxf(_params.ev_pos_innov_gate, 1.0f);
			// fuse height information
			fuseVerticalPosition(_ev_pos_innov, ev_hgt_innov_gate,
					     ev_hgt_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio);
		}
	}
}

void Ekf::checkRangeAidSuitability()
{
	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		const float range_hagl = _terrain_vpos - _state.pos(2);
		const float range_hagl_max = _is_range_aid_suitable ? _params.max_hagl_for_range_aid : (_params.max_hagl_for_range_aid * 0.7f);
		const bool is_in_range = range_hagl < range_hagl_max;

		const float hagl_test_ratio = (_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var));
		const bool is_hagl_stable = _is_range_aid_suitable ? (hagl_test_ratio < 1.f) : (hagl_test_ratio < 0.01f);

		if (isHorizontalAidingActive()) {
			const float max_vel = _is_range_aid_suitable ? _params.max_vel_for_range_aid : (_params.max_vel_for_range_aid * 0.7f);
			const bool is_below_max_speed = !_state.vel.xy().longerThan(max_vel);

			_is_range_aid_suitable = is_in_range && is_hagl_stable && is_below_max_speed;

		} else {
			_is_range_aid_suitable = is_in_range && is_hagl_stable;
		}

	} else {
		_is_range_aid_suitable = false;
	}
}

void Ekf::controlAirDataFusion()
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_time_last_beta_fuse, (uint64_t)10e6);

	if (_using_synthetic_position || (airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & MASK_USE_DRAG))) {
		_control_status.flags.wind = false;
	}

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_tas_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.in_air && _control_status.flags.fixed_wing && !_using_synthetic_position;
		const bool is_airspeed_significant = _airspeed_sample_delayed.true_airspeed > _params.arsp_thr;
		const bool starting_conditions_passing = continuing_conditions_passing && is_airspeed_significant;

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed();
				}

				const bool is_fusion_failing = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			startAirspeedFusion();
		}

	} else if (_control_status.flags.fuse_aspd && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us > (uint64_t) 1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::controlBetaFusion()
{
	if (_using_synthetic_position) {
		return;
	}

	// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fuson had been enabled externally:
	const bool beta_fusion_time_triggered = isTimedOut(_time_last_beta_fuse, _params.beta_avg_ft_us);

	if (beta_fusion_time_triggered &&
	    _control_status.flags.fuse_beta &&
	    _control_status.flags.in_air) {
		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timeout timers to prevent repeated resets
			_time_last_beta_fuse = _time_last_imu;
			resetWind();
		}

		fuseSideslip();
	}
}

void Ekf::controlDragFusion()
{
	if ((_params.fusion_mode & MASK_USE_DRAG) &&
	    !_using_synthetic_position &&
	    _control_status.flags.in_air &&
	    !_mag_inhibit_yaw_reset_req) {

		if (!_control_status.flags.wind) {
			// reset the wind states and covariances when starting drag accel fusion
			_control_status.flags.wind = true;
			resetWind();

		} else if (_drag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_drag_sample_delayed)) {
			fuseDrag();
		}

	}
}

void Ekf::controlFakePosFusion()
{
	// if we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// Coincide fake measurements with baro data for efficiency with a minimum fusion rate of 5Hz

	if (!isHorizontalAidingActive()
	    && !(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta)) {

		// We now need to use a synthetic position observation to prevent unconstrained drift of the INS states.
		_using_synthetic_position = true;

		// Fuse synthetic position observations every 200msec
		if (isTimedOut(_time_last_fake_pos, (uint64_t)2e5)) {

			// Reset position and velocity states if we re-commence this aiding method
			if (isTimedOut(_time_last_fake_pos, (uint64_t)4e5)) {
				_last_known_posNE = _state.pos.xy();
				resetHorizontalPosition();
				resetVelocity();
				_fuse_hpos_as_odom = false;

				if (_time_last_fake_pos != 0) {
					_warning_events.flags.stopping_navigation = true;
					ECL_WARN("stopping navigation");
				}

			}

			_time_last_fake_pos = _time_last_imu;

			Vector3f fake_pos_obs_var;

			if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise));

			} else if (_control_status.flags.vehicle_at_rest) {
				// Accelerate tilt fine alignment by fusing more
				// aggressively when the vehicle is at rest
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.1f);

			} else {
				fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.5f);
			}

			_gps_pos_innov.xy() = Vector2f(_state.pos) - _last_known_posNE;

			const Vector2f fake_pos_innov_gate(3.0f, 3.0f);

			fuseHorizontalPosition(_gps_pos_innov, fake_pos_innov_gate, fake_pos_obs_var,
					       _gps_pos_innov_var, _gps_pos_test_ratio, true);
		}

	} else {
		_using_synthetic_position = false;
	}

}

void Ekf::controlAuxVelFusion()
{
	const bool data_ready = _auxvel_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_auxvel_sample_delayed);

	if (data_ready && isHorizontalAidingActive()) {

		const Vector2f aux_vel_innov_gate(_params.auxvel_gate, _params.auxvel_gate);

		_last_vel_obs = _auxvel_sample_delayed.vel;
		_aux_vel_innov = _state.vel - _last_vel_obs;
		_last_vel_obs_var = _aux_vel_innov_var;

		fuseHorizontalVelocity(_aux_vel_innov, aux_vel_innov_gate, _auxvel_sample_delayed.velVar,
				       _aux_vel_innov_var, _aux_vel_test_ratio);

		// Can be enabled after bit for this is added to EKF_AID_MASK
		// fuseVerticalVelocity(_aux_vel_innov, aux_vel_innov_gate, _auxvel_sample_delayed.velVar,
		//		_aux_vel_innov_var, _aux_vel_test_ratio);

	}
}


bool Ekf::isVelStateAlignedWithObs() const
{
	/* Do sanity check to see if the innovation failures is likely caused by a yaw angle error
	 * by measuring the angle between the velocity estimate and the last velocity observation
	 * Only use those vectors if their norm if they are larger than 4 times their noise standard deviation
	 */
	const float vel_obs_xy_norm_sq = _last_vel_obs.xy().norm_squared();
	const float vel_state_xy_norm_sq = _state.vel.xy().norm_squared();

	const float vel_obs_threshold_sq = fmaxf(sq(4.f) * (_last_vel_obs_var(0) + _last_vel_obs_var(1)), sq(0.4f));
	const float vel_state_threshold_sq = fmaxf(sq(4.f) * (P(4, 4) + P(5, 5)), sq(0.4f));

	if (vel_obs_xy_norm_sq > vel_obs_threshold_sq && vel_state_xy_norm_sq > vel_state_threshold_sq) {
		const float obs_dot_vel = Vector2f(_last_vel_obs).dot(_state.vel.xy());
		const float cos_sq = sq(obs_dot_vel) / (vel_state_xy_norm_sq * vel_obs_xy_norm_sq);

		if (cos_sq < sq(cosf(math::radians(25.f))) || obs_dot_vel < 0.f) {
			// The angle between the observation and the velocity estimate is greater than 25 degrees
			return false;
		}
	}

	return true;
}

bool Ekf::hasHorizontalAidingTimedOut() const
{
	return isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_delpos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_of_fuse, _params.reset_timeout_max);
}

void Ekf::processVelPosResetRequest()
{
	if (_velpos_reset_request) {
		resetVelocity();
		resetHorizontalPosition();
		_velpos_reset_request = false;

		// Reset the timeout counters
		_time_last_hor_pos_fuse = _time_last_imu;
		_time_last_hor_vel_fuse = _time_last_imu;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file covariance.cpp
 * Contains functions for initialising, predicting and updating the state
 * covariance matrix
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include "ekf.h"
#include "utils.hpp"

#include <math.h>
#include <mathlib/mathlib.h>

// Sets initial values for the covariance matrix
// Do not call before quaternion states have been initialised
void Ekf::initialiseCovariance()
{
	P.zero();

	_delta_angle_bias_var_accum.setZero();
	_delta_vel_bias_var_accum.setZero();

	const float dt = FILTER_UPDATE_PERIOD_S;

	resetQuatCov();

	// velocity
	P(4,4) = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	P(5,5) = P(4,4);
	P(6,6) = sq(1.5f) * P(4,4);

	// position
	P(7,7) = sq(fmaxf(_params.gps_pos_noise, 0.01f));
	P(8,8) = P(7,7);

	if (_control_status.flags.rng_hgt) {
		P(9,9) = sq(fmaxf(_params.range_noise, 0.01f));

	} else if (_control_status.flags.gps_hgt) {
		P(9,9) = getGpsHeightVariance();

	} else {
		P(9,9) = sq(fmaxf(_params.baro_noise, 0.01f));
	}

	// gyro bias
	P(10,10) = sq(_params.switch_on_gyro_bias * dt);
	P(11,11) = P(10,10);
	P(12,12) = P(10,10);

	// accel bias
	_prev_dvel_bias_var(0) = P(13,13) = sq(_params.switch_on_accel_bias * dt);
	_prev_dvel_bias_var(1) = P(14,14) = P(13,13);
	_prev_dvel_bias_var(2) = P(15,15) = P(13,13);

	resetMagCov();

	// wind
	P(22,22) = sq(_params.initial_wind_uncertainty);
	P(23,23) = P(22,22);

}

void Ekf::predictCovariance()
{
	// assign intermediate state variables
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	const float &dax = _imu_sample_delayed.delta_ang(0);
	const float &day = _imu_sample_delayed.delta_ang(1);
	const float &daz = _imu_sample_delayed.delta_ang(2);

	const float &dvx = _imu_sample_delayed.delta_vel(0);
	const float &dvy = _imu_sample_delayed.delta_vel(1);
	const float &dvz = _imu_sample_delayed.delta_vel(2);

	const float &dax_b = _state.delta_ang_bias(0);
	const float &day_b = _state.delta_ang_bias(1);
	const float &daz_b = _state.delta_ang_bias(2);

	const float &dvx_b = _state.delta_vel_bias(0);
	const float &dvy_b = _state.delta_vel_bias(1);
	const float &dvz_b = _state.delta_vel_bias(2);

	// Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	const float dt = FILTER_UPDATE_PERIOD_S;
	const float dt_inv = 1.0f / dt;

	// convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	const float d_ang_bias_sig = dt * dt * math::constrain(_params.gyro_bias_p_noise, 0.0f, 1.0f);

	// convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	const float d_vel_bias_sig = dt * dt * math::constrain(_params.accel_bias_p_noise, 0.0f, 1.0f);

	// inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	// xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
	const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
	const float beta = 1.0f - alpha;
	_ang_rate_magnitude_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_ang.norm(), beta * _ang_rate_magnitude_filt);
	_accel_magnitude_filt = fmaxf(dt_inv * _imu_sample_delayed.delta_vel.norm(), beta * _accel_magnitude_filt);
	_accel_vec_filt = alpha * dt_inv * _imu_sample_delayed.delta_vel + beta * _accel_vec_filt;

	const bool is_manoeuvre_level_high = _ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim
					     || _accel_magnitude_filt > _params.acc_bias_learn_acc_lim;

	const bool do_inhibit_all_axes = (_params.fusion_mode & MASK_INHIBIT_ACC_BIAS)
					 || is_manoeuvre_level_high
					 || _fault_status.flags.bad_acc_vertical;

	for (unsigned stateIndex = 13; stateIndex <= 15; stateIndex++) {
		const unsigned index = stateIndex - 13;

		const bool do_inhibit_axis = do_inhibit_all_axes || _imu_sample_delayed.delta_vel_clipping[index];

		if (do_inhibit_axis) {
			// store the bias state variances to be reinstated later
			if (!_accel_bias_inhibit[index]) {
				_prev_dvel_bias_var(index) = P(stateIndex, stateIndex);
				_accel_bias_inhibit[index] = true;
			}

		} else {
			if (_accel_bias_inhibit[index]) {
				// reinstate the bias state variances
				P(stateIndex, stateIndex) = _prev_dvel_bias_var(index);
				_accel_bias_inhibit[index] = false;
			}
		}
	}

	// Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_I_sig;

	if (_control_status.flags.mag_3D && (P(16, 16) + P(17, 17) + P(18, 18)) < 0.1f) {
		mag_I_sig = dt * math::constrain(_params.mage_p_noise, 0.0f, 1.0f);

	} else {
		mag_I_sig = 0.0f;
	}

	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	float mag_B_sig;

	if (_control_status.flags.mag_3D && (P(19, 19) + P(20, 20) + P(21, 21)) < 0.1f) {
		mag_B_sig = dt * math::constrain(_params.magb_p_noise, 0.0f, 1.0f);

	} else {
		mag_B_sig = 0.0f;
	}

	float wind_vel_sig;

	// Calculate low pass filtered height rate
	float alpha_height_rate_lpf = 0.1f * dt; // 10 seconds time constant
	_height_rate_lpf = _height_rate_lpf * (1.0f - alpha_height_rate_lpf) + _state.vel(2) * alpha_height_rate_lpf;

	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	if (_control_status.flags.wind && (P(22,22) + P(23,23)) < sq(_params.initial_wind_uncertainty)) {
		wind_vel_sig = dt * math::constrain(_params.wind_vel_p_noise, 0.0f, 1.0f) * (1.0f + _params.wind_vel_p_noise_scaler * fabsf(_height_rate_lpf));

	} else {
		wind_vel_sig = 0.0f;
	}

	// compute noise variance for stationary processes
	Vector24f process_noise;

	// Construct the process noise variance diagonal for those states with a stationary process model
	// These are kinematic states and their error growth is controlled separately by the IMU noise variances

	// delta angle bias states
	process_noise.slice<3, 1>(10, 0) = sq(d_ang_bias_sig);
	// delta_velocity bias states
	process_noise.slice<3, 1>(13, 0) = sq(d_vel_bias_sig);
	// earth frame magnetic field states
	process_noise.slice<3, 1>(16, 0) = sq(mag_I_sig);
	// body frame magnetic field states
	process_noise.slice<3, 1>(19, 0) = sq(mag_B_sig);
	// wind velocity states
	process_noise.slice<2, 1>(22, 0) = sq(wind_vel_sig);

	// assign IMU noise variances
	// inputs to the system are 3 delta angles and 3 delta velocities
	float gyro_noise = math::constrain(_params.gyro_noise, 0.0f, 1.0f);
	const float daxVar = sq(dt * gyro_noise);
	const float dayVar = daxVar;
	const float dazVar = daxVar;

	float accel_noise = math::constrain(_params.accel_noise, 0.0f, 1.0f);

	if (_fault_status.flags.bad_acc_vertical) {
		// Increase accelerometer process noise if bad accel data is detected. Measurement errors due to
		// vibration induced clipping commonly reach an equivalent 0.5g offset.
		accel_noise = BADACC_BIAS_PNOISE;
	}

	float dvxVar, dvyVar, dvzVar;
	dvxVar = dvyVar = dvzVar = sq(dt * accel_noise);

	// Accelerometer Clipping
	_fault_status.flags.bad_acc_clipping = false; // reset flag

	// delta velocity X: increase process noise if sample contained any X axis clipping
	if (_imu_sample_delayed.delta_vel_clipping[0]) {
		dvxVar = sq(dt * BADACC_BIAS_PNOISE);
		_fault_status.flags.bad_acc_clipping = true;
	}

	// delta velocity Y: increase process noise if sample contained any Y axis clipping
	if (_imu_sample_delayed.delta_vel_clipping[1]) {
		dvyVar = sq(dt * BADACC_BIAS_PNOISE);
		_fault_status.flags.bad_acc_clipping = true;
	}

	// delta velocity Z: increase process noise if sample contained any Z axis clipping
	if (_imu_sample_delayed.delta_vel_clipping[2]) {
		dvzVar = sq(dt * BADACC_BIAS_PNOISE);
		_fault_status.flags.bad_acc_clipping = true;
	}

	// predict the covariance
	// equations generated using EKF/python/ekf_derivation/main.py

	// intermediate calculations
	const float PS0 = ecl::powf(q1, 2);
	const float PS1 = 0.25F*daxVar;
	const float PS2 = ecl::powf(q2, 2);
	const float PS3 = 0.25F*dayVar;
	const float PS4 = ecl::powf(q3, 2);
	const float PS5 = 0.25F*dazVar;
	const float PS6 = 0.5F*q1;
	const float PS7 = 0.5F*q2;
	const float PS8 = P(10,11)*PS7;
	const float PS9 = 0.5F*q3;
	const float PS10 = P(10,12)*PS9;
	const float PS11 = 0.5F*dax - 0.5F*dax_b;
	const float PS12 = 0.5F*day - 0.5F*day_b;
	const float PS13 = 0.5F*daz - 0.5F*daz_b;
	const float PS14 = P(0,10) - P(1,10)*PS11 + P(10,10)*PS6 - P(2,10)*PS12 - P(3,10)*PS13 + PS10 + PS8;
	const float PS15 = P(10,11)*PS6;
	const float PS16 = P(11,12)*PS9;
	const float PS17 = P(0,11) - P(1,11)*PS11 + P(11,11)*PS7 - P(2,11)*PS12 - P(3,11)*PS13 + PS15 + PS16;
	const float PS18 = P(10,12)*PS6;
	const float PS19 = P(11,12)*PS7;
	const float PS20 = P(0,12) - P(1,12)*PS11 + P(12,12)*PS9 - P(2,12)*PS12 - P(3,12)*PS13 + PS18 + PS19;
	const float PS21 = P(1,2)*PS12;
	const float PS22 = -P(1,3)*PS13;
	const float PS23 = P(0,1) - P(1,1)*PS11 + P(1,10)*PS6 + P(1,11)*PS7 + P(1,12)*PS9 - PS21 + PS22;
	const float PS24 = -P(1,2)*PS11;
	const float PS25 = P(2,3)*PS13;
	const float PS26 = P(0,2) + P(2,10)*PS6 + P(2,11)*PS7 + P(2,12)*PS9 - P(2,2)*PS12 + PS24 - PS25;
	const float PS27 = P(1,3)*PS11;
	const float PS28 = -P(2,3)*PS12;
	const float PS29 = P(0,3) + P(3,10)*PS6 + P(3,11)*PS7 + P(3,12)*PS9 - P(3,3)*PS13 - PS27 + PS28;
	const float PS30 = P(0,1)*PS11;
	const float PS31 = P(0,2)*PS12;
	const float PS32 = P(0,3)*PS13;
	const float PS33 = P(0,0) + P(0,10)*PS6 + P(0,11)*PS7 + P(0,12)*PS9 - PS30 - PS31 - PS32;
	const float PS34 = 0.5F*q0;
	const float PS35 = q2*q3;
	const float PS36 = q0*q1;
	const float PS37 = q1*q3;
	const float PS38 = q0*q2;
	const float PS39 = q1*q2;
	const float PS40 = q0*q3;
	const float PS41 = 2*PS2;
	const float PS42 = 2*PS4 - 1;
	const float PS43 = PS41 + PS42;
	const float PS44 = P(0,13) - P(1,13)*PS11 + P(10,13)*PS6 + P(11,13)*PS7 + P(12,13)*PS9 - P(2,13)*PS12 - P(3,13)*PS13;
	const float PS45 = PS37 + PS38;
	const float PS46 = P(0,15) - P(1,15)*PS11 + P(10,15)*PS6 + P(11,15)*PS7 + P(12,15)*PS9 - P(2,15)*PS12 - P(3,15)*PS13;
	const float PS47 = 2*PS46;
	const float PS48 = dvy - dvy_b;
	const float PS49 = PS48*q0;
	const float PS50 = dvz - dvz_b;
	const float PS51 = PS50*q1;
	const float PS52 = dvx - dvx_b;
	const float PS53 = PS52*q3;
	const float PS54 = PS49 - PS51 + 2*PS53;
	const float PS55 = 2*PS29;
	const float PS56 = -PS39 + PS40;
	const float PS57 = P(0,14) - P(1,14)*PS11 + P(10,14)*PS6 + P(11,14)*PS7 + P(12,14)*PS9 - P(2,14)*PS12 - P(3,14)*PS13;
	const float PS58 = 2*PS57;
	const float PS59 = PS48*q2;
	const float PS60 = PS50*q3;
	const float PS61 = PS59 + PS60;
	const float PS62 = 2*PS23;
	const float PS63 = PS50*q2;
	const float PS64 = PS48*q3;
	const float PS65 = -PS64;
	const float PS66 = PS63 + PS65;
	const float PS67 = 2*PS33;
	const float PS68 = PS50*q0;
	const float PS69 = PS48*q1;
	const float PS70 = PS52*q2;
	const float PS71 = PS68 + PS69 - 2*PS70;
	const float PS72 = 2*PS26;
	const float PS73 = P(0,4) - P(1,4)*PS11 - P(2,4)*PS12 - P(3,4)*PS13 + P(4,10)*PS6 + P(4,11)*PS7 + P(4,12)*PS9;
	const float PS74 = 2*PS0;
	const float PS75 = PS42 + PS74;
	const float PS76 = PS39 + PS40;
	const float PS77 = 2*PS44;
	const float PS78 = PS51 - PS53;
	const float PS79 = -PS70;
	const float PS80 = PS68 + 2*PS69 + PS79;
	const float PS81 = -PS35 + PS36;
	const float PS82 = PS52*q1;
	const float PS83 = PS60 + PS82;
	const float PS84 = PS52*q0;
	const float PS85 = PS63 - 2*PS64 + PS84;
	const float PS86 = P(0,5) - P(1,5)*PS11 - P(2,5)*PS12 - P(3,5)*PS13 + P(5,10)*PS6 + P(5,11)*PS7 + P(5,12)*PS9;
	const float PS87 = PS41 + PS74 - 1;
	const float PS88 = PS35 + PS36;
	const float PS89 = 2*PS63 + PS65 + PS84;
	const float PS90 = -PS37 + PS38;
	const float PS91 = PS59 + PS82;
	const float PS92 = PS69 + PS79;
	const float PS93 = PS49 - 2*PS51 + PS53;
	const float PS94 = P(0,6) - P(1,6)*PS11 - P(2,6)*PS12 - P(3,6)*PS13 + P(6,10)*PS6 + P(6,11)*PS7 + P(6,12)*PS9;
	const float PS95 = ecl::powf(q0, 2);
	const float PS96 = -P(10,11)*PS34;
	const float PS97 = P(0,11)*PS11 + P(1,11) + P(11,11)*PS9 + P(2,11)*PS13 - P(3,11)*PS12 - PS19 + PS96;
	const float PS98 = P(0,2)*PS13;
	const float PS99 = P(0,3)*PS12;
	const float PS100 = P(0,0)*PS11 + P(0,1) - P(0,10)*PS34 + P(0,11)*PS9 - P(0,12)*PS7 + PS98 - PS99;
	const float PS101 = P(0,2)*PS11;
	const float PS102 = P(1,2) - P(2,10)*PS34 + P(2,11)*PS9 - P(2,12)*PS7 + P(2,2)*PS13 + PS101 + PS28;
	const float PS103 = P(10,11)*PS9;
	const float PS104 = P(10,12)*PS7;
	const float PS105 = P(0,10)*PS11 + P(1,10) - P(10,10)*PS34 + P(2,10)*PS13 - P(3,10)*PS12 + PS103 - PS104;
	const float PS106 = -P(10,12)*PS34;
	const float PS107 = P(0,12)*PS11 + P(1,12) - P(12,12)*PS7 + P(2,12)*PS13 - P(3,12)*PS12 + PS106 + PS16;
	const float PS108 = P(0,3)*PS11;
	const float PS109 = P(1,3) - P(3,10)*PS34 + P(3,11)*PS9 - P(3,12)*PS7 - P(3,3)*PS12 + PS108 + PS25;
	const float PS110 = P(1,2)*PS13;
	const float PS111 = P(1,3)*PS12;
	const float PS112 = P(1,1) - P(1,10)*PS34 + P(1,11)*PS9 - P(1,12)*PS7 + PS110 - PS111 + PS30;
	const float PS113 = P(0,13)*PS11 + P(1,13) - P(10,13)*PS34 + P(11,13)*PS9 - P(12,13)*PS7 + P(2,13)*PS13 - P(3,13)*PS12;
	const float PS114 = P(0,15)*PS11 + P(1,15) - P(10,15)*PS34 + P(11,15)*PS9 - P(12,15)*PS7 + P(2,15)*PS13 - P(3,15)*PS12;
	const float PS115 = 2*PS114;
	const float PS116 = 2*PS109;
	const float PS117 = P(0,14)*PS11 + P(1,14) - P(10,14)*PS34 + P(11,14)*PS9 - P(12,14)*PS7 + P(2,14)*PS13 - P(3,14)*PS12;
	const float PS118 = 2*PS117;
	const float PS119 = 2*PS112;
	const float PS120 = 2*PS100;
	const float PS121 = 2*PS102;
	const float PS122 = P(0,4)*PS11 + P(1,4) + P(2,4)*PS13 - P(3,4)*PS12 - P(4,10)*PS34 + P(4,11)*PS9 - P(4,12)*PS7;
	const float PS123 = 2*PS113;
	const float PS124 = P(0,5)*PS11 + P(1,5) + P(2,5)*PS13 - P(3,5)*PS12 - P(5,10)*PS34 + P(5,11)*PS9 - P(5,12)*PS7;
	const float PS125 = P(0,6)*PS11 + P(1,6) + P(2,6)*PS13 - P(3,6)*PS12 - P(6,10)*PS34 + P(6,11)*PS9 - P(6,12)*PS7;
	const float PS126 = -P(11,12)*PS34;
	const float PS127 = P(0,12)*PS12 - P(1,12)*PS13 + P(12,12)*PS6 + P(2,12) + P(3,12)*PS11 - PS10 + PS126;
	const float PS128 = P(2,3) - P(3,10)*PS9 - P(3,11)*PS34 + P(3,12)*PS6 + P(3,3)*PS11 + PS22 + PS99;
	const float PS129 = P(0,1)*PS13;
	const float PS130 = P(0,0)*PS12 - P(0,10)*PS9 - P(0,11)*PS34 + P(0,12)*PS6 + P(0,2) + PS108 - PS129;
	const float PS131 = P(11,12)*PS6;
	const float PS132 = P(0,11)*PS12 - P(1,11)*PS13 - P(11,11)*PS34 + P(2,11) + P(3,11)*PS11 - PS103 + PS131;
	const float PS133 = P(0,10)*PS12 - P(1,10)*PS13 - P(10,10)*PS9 + P(2,10) + P(3,10)*PS11 + PS18 + PS96;
	const float PS134 = P(0,1)*PS12;
	const float PS135 = -P(1,1)*PS13 - P(1,10)*PS9 - P(1,11)*PS34 + P(1,12)*PS6 + P(1,2) + PS134 + PS27;
	const float PS136 = P(2,3)*PS11;
	const float PS137 = -P(2,10)*PS9 - P(2,11)*PS34 + P(2,12)*PS6 + P(2,2) - PS110 + PS136 + PS31;
	const float PS138 = P(0,13)*PS12 - P(1,13)*PS13 - P(10,13)*PS9 - P(11,13)*PS34 + P(12,13)*PS6 + P(2,13) + P(3,13)*PS11;
	const float PS139 = P(0,15)*PS12 - P(1,15)*PS13 - P(10,15)*PS9 - P(11,15)*PS34 + P(12,15)*PS6 + P(2,15) + P(3,15)*PS11;
	const float PS140 = 2*PS139;
	const float PS141 = 2*PS128;
	const float PS142 = P(0,14)*PS12 - P(1,14)*PS13 - P(10,14)*PS9 - P(11,14)*PS34 + P(12,14)*PS6 + P(2,14) + P(3,14)*PS11;
	const float PS143 = 2*PS142;
	const float PS144 = 2*PS135;
	const float PS145 = 2*PS130;
	const float PS146 = 2*PS137;
	const float PS147 = P(0,4)*PS12 - P(1,4)*PS13 + P(2,4) + P(3,4)*PS11 - P(4,10)*PS9 - P(4,11)*PS34 + P(4,12)*PS6;
	const float PS148 = 2*PS138;
	const float PS149 = P(0,5)*PS12 - P(1,5)*PS13 + P(2,5) + P(3,5)*PS11 - P(5,10)*PS9 - P(5,11)*PS34 + P(5,12)*PS6;
	const float PS150 = P(0,6)*PS12 - P(1,6)*PS13 + P(2,6) + P(3,6)*PS11 - P(6,10)*PS9 - P(6,11)*PS34 + P(6,12)*PS6;
	const float PS151 = P(0,10)*PS13 + P(1,10)*PS12 + P(10,10)*PS7 - P(2,10)*PS11 + P(3,10) + PS106 - PS15;
	const float PS152 = P(1,1)*PS12 + P(1,10)*PS7 - P(1,11)*PS6 - P(1,12)*PS34 + P(1,3) + PS129 + PS24;
	const float PS153 = P(0,0)*PS13 + P(0,10)*PS7 - P(0,11)*PS6 - P(0,12)*PS34 + P(0,3) - PS101 + PS134;
	const float PS154 = P(0,12)*PS13 + P(1,12)*PS12 - P(12,12)*PS34 - P(2,12)*PS11 + P(3,12) + PS104 - PS131;
	const float PS155 = P(0,11)*PS13 + P(1,11)*PS12 - P(11,11)*PS6 - P(2,11)*PS11 + P(3,11) + PS126 + PS8;
	const float PS156 = P(2,10)*PS7 - P(2,11)*PS6 - P(2,12)*PS34 - P(2,2)*PS11 + P(2,3) + PS21 + PS98;
	const float PS157 = P(3,10)*PS7 - P(3,11)*PS6 - P(3,12)*PS34 + P(3,3) + PS111 - PS136 + PS32;
	const float PS158 = P(0,13)*PS13 + P(1,13)*PS12 + P(10,13)*PS7 - P(11,13)*PS6 - P(12,13)*PS34 - P(2,13)*PS11 + P(3,13);
	const float PS159 = P(0,15)*PS13 + P(1,15)*PS12 + P(10,15)*PS7 - P(11,15)*PS6 - P(12,15)*PS34 - P(2,15)*PS11 + P(3,15);
	const float PS160 = 2*PS159;
	const float PS161 = 2*PS157;
	const float PS162 = P(0,14)*PS13 + P(1,14)*PS12 + P(10,14)*PS7 - P(11,14)*PS6 - P(12,14)*PS34 - P(2,14)*PS11 + P(3,14);
	const float PS163 = 2*PS162;
	const float PS164 = 2*PS152;
	const float PS165 = 2*PS153;
	const float PS166 = 2*PS156;
	const float PS167 = P(0,4)*PS13 + P(1,4)*PS12 - P(2,4)*PS11 + P(3,4) + P(4,10)*PS7 - P(4,11)*PS6 - P(4,12)*PS34;
	const float PS168 = 2*PS158;
	const float PS169 = P(0,5)*PS13 + P(1,5)*PS12 - P(2,5)*PS11 + P(3,5) + P(5,10)*PS7 - P(5,11)*PS6 - P(5,12)*PS34;
	const float PS170 = P(0,6)*PS13 + P(1,6)*PS12 - P(2,6)*PS11 + P(3,6) + P(6,10)*PS7 - P(6,11)*PS6 - P(6,12)*PS34;
	const float PS171 = 2*PS45;
	const float PS172 = 2*PS56;
	const float PS173 = 2*PS61;
	const float PS174 = 2*PS66;
	const float PS175 = 2*PS71;
	const float PS176 = 2*PS54;
	const float PS177 = P(0,13)*PS174 + P(1,13)*PS173 + P(13,13)*PS43 + P(13,14)*PS172 - P(13,15)*PS171 + P(2,13)*PS175 - P(3,13)*PS176 + P(4,13);
	const float PS178 = P(0,15)*PS174 + P(1,15)*PS173 + P(13,15)*PS43 + P(14,15)*PS172 - P(15,15)*PS171 + P(2,15)*PS175 - P(3,15)*PS176 + P(4,15);
	const float PS179 = P(0,3)*PS174 + P(1,3)*PS173 + P(2,3)*PS175 + P(3,13)*PS43 + P(3,14)*PS172 - P(3,15)*PS171 - P(3,3)*PS176 + P(3,4);
	const float PS180 = P(0,14)*PS174 + P(1,14)*PS173 + P(13,14)*PS43 + P(14,14)*PS172 - P(14,15)*PS171 + P(2,14)*PS175 - P(3,14)*PS176 + P(4,14);
	const float PS181 = P(0,1)*PS174 + P(1,1)*PS173 + P(1,13)*PS43 + P(1,14)*PS172 - P(1,15)*PS171 + P(1,2)*PS175 - P(1,3)*PS176 + P(1,4);
	const float PS182 = P(0,0)*PS174 + P(0,1)*PS173 + P(0,13)*PS43 + P(0,14)*PS172 - P(0,15)*PS171 + P(0,2)*PS175 - P(0,3)*PS176 + P(0,4);
	const float PS183 = P(0,2)*PS174 + P(1,2)*PS173 + P(2,13)*PS43 + P(2,14)*PS172 - P(2,15)*PS171 + P(2,2)*PS175 - P(2,3)*PS176 + P(2,4);
	const float PS184 = 4*dvyVar;
	const float PS185 = 4*dvzVar;
	const float PS186 = P(0,4)*PS174 + P(1,4)*PS173 + P(2,4)*PS175 - P(3,4)*PS176 + P(4,13)*PS43 + P(4,14)*PS172 - P(4,15)*PS171 + P(4,4);
	const float PS187 = 2*PS177;
	const float PS188 = 2*PS182;
	const float PS189 = 2*PS181;
	const float PS190 = 2*PS81;
	const float PS191 = 2*PS183;
	const float PS192 = 2*PS179;
	const float PS193 = 2*PS76;
	const float PS194 = PS43*dvxVar;
	const float PS195 = PS75*dvyVar;
	const float PS196 = P(0,5)*PS174 + P(1,5)*PS173 + P(2,5)*PS175 - P(3,5)*PS176 + P(4,5) + P(5,13)*PS43 + P(5,14)*PS172 - P(5,15)*PS171;
	const float PS197 = 2*PS88;
	const float PS198 = PS87*dvzVar;
	const float PS199 = 2*PS90;
	const float PS200 = P(0,6)*PS174 + P(1,6)*PS173 + P(2,6)*PS175 - P(3,6)*PS176 + P(4,6) + P(6,13)*PS43 + P(6,14)*PS172 - P(6,15)*PS171;
	const float PS201 = 2*PS83;
	const float PS202 = 2*PS78;
	const float PS203 = 2*PS85;
	const float PS204 = 2*PS80;
	const float PS205 = -P(0,14)*PS202 - P(1,14)*PS204 - P(13,14)*PS193 + P(14,14)*PS75 + P(14,15)*PS190 + P(2,14)*PS201 + P(3,14)*PS203 + P(5,14);
	const float PS206 = -P(0,13)*PS202 - P(1,13)*PS204 - P(13,13)*PS193 + P(13,14)*PS75 + P(13,15)*PS190 + P(2,13)*PS201 + P(3,13)*PS203 + P(5,13);
	const float PS207 = -P(0,0)*PS202 - P(0,1)*PS204 - P(0,13)*PS193 + P(0,14)*PS75 + P(0,15)*PS190 + P(0,2)*PS201 + P(0,3)*PS203 + P(0,5);
	const float PS208 = -P(0,1)*PS202 - P(1,1)*PS204 - P(1,13)*PS193 + P(1,14)*PS75 + P(1,15)*PS190 + P(1,2)*PS201 + P(1,3)*PS203 + P(1,5);
	const float PS209 = -P(0,15)*PS202 - P(1,15)*PS204 - P(13,15)*PS193 + P(14,15)*PS75 + P(15,15)*PS190 + P(2,15)*PS201 + P(3,15)*PS203 + P(5,15);
	const float PS210 = -P(0,2)*PS202 - P(1,2)*PS204 - P(2,13)*PS193 + P(2,14)*PS75 + P(2,15)*PS190 + P(2,2)*PS201 + P(2,3)*PS203 + P(2,5);
	const float PS211 = -P(0,3)*PS202 - P(1,3)*PS204 + P(2,3)*PS201 - P(3,13)*PS193 + P(3,14)*PS75 + P(3,15)*PS190 + P(3,3)*PS203 + P(3,5);
	const float PS212 = 4*dvxVar;
	const float PS213 = -P(0,5)*PS202 - P(1,5)*PS204 + P(2,5)*PS201 + P(3,5)*PS203 - P(5,13)*PS193 + P(5,14)*PS75 + P(5,15)*PS190 + P(5,5);
	const float PS214 = 2*PS89;
	const float PS215 = 2*PS91;
	const float PS216 = 2*PS92;
	const float PS217 = 2*PS93;
	const float PS218 = -P(0,6)*PS202 - P(1,6)*PS204 + P(2,6)*PS201 + P(3,6)*PS203 + P(5,6) - P(6,13)*PS193 + P(6,14)*PS75 + P(6,15)*PS190;
	const float PS219 = P(0,15)*PS216 + P(1,15)*PS217 + P(13,15)*PS199 - P(14,15)*PS197 + P(15,15)*PS87 - P(2,15)*PS214 + P(3,15)*PS215 + P(6,15);
	const float PS220 = P(0,14)*PS216 + P(1,14)*PS217 + P(13,14)*PS199 - P(14,14)*PS197 + P(14,15)*PS87 - P(2,14)*PS214 + P(3,14)*PS215 + P(6,14);
	const float PS221 = P(0,13)*PS216 + P(1,13)*PS217 + P(13,13)*PS199 - P(13,14)*PS197 + P(13,15)*PS87 - P(2,13)*PS214 + P(3,13)*PS215 + P(6,13);
	const float PS222 = P(0,6)*PS216 + P(1,6)*PS217 - P(2,6)*PS214 + P(3,6)*PS215 + P(6,13)*PS199 - P(6,14)*PS197 + P(6,15)*PS87 + P(6,6);


	// covariance update
	SquareMatrix24f nextP;

	// calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states

	nextP(0,0) = PS0*PS1 - PS11*PS23 - PS12*PS26 - PS13*PS29 + PS14*PS6 + PS17*PS7 + PS2*PS3 + PS20*PS9 + PS33 + PS4*PS5;
	nextP(0,1) = -PS1*PS36 + PS11*PS33 - PS12*PS29 + PS13*PS26 - PS14*PS34 + PS17*PS9 - PS20*PS7 + PS23 + PS3*PS35 - PS35*PS5;
	nextP(1,1) = PS1*PS95 + PS100*PS11 + PS102*PS13 - PS105*PS34 - PS107*PS7 - PS109*PS12 + PS112 + PS2*PS5 + PS3*PS4 + PS9*PS97;
	nextP(0,2) = -PS1*PS37 + PS11*PS29 + PS12*PS33 - PS13*PS23 - PS14*PS9 - PS17*PS34 + PS20*PS6 + PS26 - PS3*PS38 + PS37*PS5;
	nextP(1,2) = PS1*PS40 + PS100*PS12 + PS102 - PS105*PS9 + PS107*PS6 + PS109*PS11 - PS112*PS13 - PS3*PS40 - PS34*PS97 - PS39*PS5;
	nextP(2,2) = PS0*PS5 + PS1*PS4 + PS11*PS128 + PS12*PS130 + PS127*PS6 - PS13*PS135 - PS132*PS34 - PS133*PS9 + PS137 + PS3*PS95;
	nextP(0,3) = PS1*PS39 - PS11*PS26 + PS12*PS23 + PS13*PS33 + PS14*PS7 - PS17*PS6 - PS20*PS34 + PS29 - PS3*PS39 - PS40*PS5;
	nextP(1,3) = -PS1*PS38 + PS100*PS13 - PS102*PS11 + PS105*PS7 - PS107*PS34 + PS109 + PS112*PS12 - PS3*PS37 + PS38*PS5 - PS6*PS97;
	nextP(2,3) = -PS1*PS35 - PS11*PS137 + PS12*PS135 - PS127*PS34 + PS128 + PS13*PS130 - PS132*PS6 + PS133*PS7 + PS3*PS36 - PS36*PS5;
	nextP(3,3) = PS0*PS3 + PS1*PS2 - PS11*PS156 + PS12*PS152 + PS13*PS153 + PS151*PS7 - PS154*PS34 - PS155*PS6 + PS157 + PS5*PS95;
	nextP(0,4) = PS43*PS44 - PS45*PS47 - PS54*PS55 + PS56*PS58 + PS61*PS62 + PS66*PS67 + PS71*PS72 + PS73;
	nextP(1,4) = PS113*PS43 - PS115*PS45 - PS116*PS54 + PS118*PS56 + PS119*PS61 + PS120*PS66 + PS121*PS71 + PS122;
	nextP(2,4) = PS138*PS43 - PS140*PS45 - PS141*PS54 + PS143*PS56 + PS144*PS61 + PS145*PS66 + PS146*PS71 + PS147;
	nextP(3,4) = PS158*PS43 - PS160*PS45 - PS161*PS54 + PS163*PS56 + PS164*PS61 + PS165*PS66 + PS166*PS71 + PS167;
	nextP(4,4) = -PS171*PS178 + PS172*PS180 + PS173*PS181 + PS174*PS182 + PS175*PS183 - PS176*PS179 + PS177*PS43 + PS184*ecl::powf(PS56, 2) + PS185*ecl::powf(PS45, 2) + PS186 + ecl::powf(PS43, 2)*dvxVar;
	nextP(0,5) = PS47*PS81 + PS55*PS85 + PS57*PS75 - PS62*PS80 - PS67*PS78 + PS72*PS83 - PS76*PS77 + PS86;
	nextP(1,5) = PS115*PS81 + PS116*PS85 + PS117*PS75 - PS119*PS80 - PS120*PS78 + PS121*PS83 - PS123*PS76 + PS124;
	nextP(2,5) = PS140*PS81 + PS141*PS85 + PS142*PS75 - PS144*PS80 - PS145*PS78 + PS146*PS83 - PS148*PS76 + PS149;
	nextP(3,5) = PS160*PS81 + PS161*PS85 + PS162*PS75 - PS164*PS80 - PS165*PS78 + PS166*PS83 - PS168*PS76 + PS169;
	nextP(4,5) = PS172*PS195 + PS178*PS190 + PS180*PS75 - PS185*PS45*PS81 - PS187*PS76 - PS188*PS78 - PS189*PS80 + PS191*PS83 + PS192*PS85 - PS193*PS194 + PS196;
	nextP(5,5) = PS185*ecl::powf(PS81, 2) + PS190*PS209 - PS193*PS206 + PS201*PS210 - PS202*PS207 + PS203*PS211 - PS204*PS208 + PS205*PS75 + PS212*ecl::powf(PS76, 2) + PS213 + ecl::powf(PS75, 2)*dvyVar;
	nextP(0,6) = PS46*PS87 + PS55*PS91 - PS58*PS88 + PS62*PS93 + PS67*PS92 - PS72*PS89 + PS77*PS90 + PS94;
	nextP(1,6) = PS114*PS87 + PS116*PS91 - PS118*PS88 + PS119*PS93 + PS120*PS92 - PS121*PS89 + PS123*PS90 + PS125;
	nextP(2,6) = PS139*PS87 + PS141*PS91 - PS143*PS88 + PS144*PS93 + PS145*PS92 - PS146*PS89 + PS148*PS90 + PS150;
	nextP(3,6) = PS159*PS87 + PS161*PS91 - PS163*PS88 + PS164*PS93 + PS165*PS92 - PS166*PS89 + PS168*PS90 + PS170;
	nextP(4,6) = -PS171*PS198 + PS178*PS87 - PS180*PS197 - PS184*PS56*PS88 + PS187*PS90 + PS188*PS92 + PS189*PS93 - PS191*PS89 + PS192*PS91 + PS194*PS199 + PS200;
	nextP(5,6) = PS190*PS198 - PS195*PS197 - PS197*PS205 + PS199*PS206 + PS207*PS216 + PS208*PS217 + PS209*PS87 - PS210*PS214 + PS211*PS215 - PS212*PS76*PS90 + PS218;
	nextP(6,6) = PS184*ecl::powf(PS88, 2) - PS197*PS220 + PS199*PS221 + PS212*ecl::powf(PS90, 2) - PS214*(P(0,2)*PS216 + P(1,2)*PS217 + P(2,13)*PS199 - P(2,14)*PS197 + P(2,15)*PS87 - P(2,2)*PS214 + P(2,3)*PS215 + P(2,6)) + PS215*(P(0,3)*PS216 + P(1,3)*PS217 - P(2,3)*PS214 + P(3,13)*PS199 - P(3,14)*PS197 + P(3,15)*PS87 + P(3,3)*PS215 + P(3,6)) + PS216*(P(0,0)*PS216 + P(0,1)*PS217 + P(0,13)*PS199 - P(0,14)*PS197 + P(0,15)*PS87 - P(0,2)*PS214 + P(0,3)*PS215 + P(0,6)) + PS217*(P(0,1)*PS216 + P(1,1)*PS217 + P(1,13)*PS199 - P(1,14)*PS197 + P(1,15)*PS87 - P(1,2)*PS214 + P(1,3)*PS215 + P(1,6)) + PS219*PS87 + PS222 + ecl::powf(PS87, 2)*dvzVar;
	nextP(0,7) = P(0,7) - P(1,7)*PS11 - P(2,7)*PS12 - P(3,7)*PS13 + P(7,10)*PS6 + P(7,11)*PS7 + P(7,12)*PS9 + PS73*dt;
	nextP(1,7) = P(0,7)*PS11 + P(1,7) + P(2,7)*PS13 - P(3,7)*PS12 - P(7,10)*PS34 + P(7,11)*PS9 - P(7,12)*PS7 + PS122*dt;
	nextP(2,7) = P(0,7)*PS12 - P(1,7)*PS13 + P(2,7) + P(3,7)*PS11 - P(7,10)*PS9 - P(7,11)*PS34 + P(7,12)*PS6 + PS147*dt;
	nextP(3,7) = P(0,7)*PS13 + P(1,7)*PS12 - P(2,7)*PS11 + P(3,7) + P(7,10)*PS7 - P(7,11)*PS6 - P(7,12)*PS34 + PS167*dt;
	nextP(4,7) = P(0,7)*PS174 + P(1,7)*PS173 + P(2,7)*PS175 - P(3,7)*PS176 + P(4,7) + P(7,13)*PS43 + P(7,14)*PS172 - P(7,15)*PS171 + PS186*dt;
	nextP(5,7) = -P(0,7)*PS202 - P(1,7)*PS204 + P(2,7)*PS201 + P(3,7)*PS203 + P(5,7) - P(7,13)*PS193 + P(7,14)*PS75 + P(7,15)*PS190 + dt*(-P(0,4)*PS202 - P(1,4)*PS204 + P(2,4)*PS201 + P(3,4)*PS203 - P(4,13)*PS193 + P(4,14)*PS75 + P(4,15)*PS190 + P(4,5));
	nextP(6,7) = P(0,7)*PS216 + P(1,7)*PS217 - P(2,7)*PS214 + P(3,7)*PS215 + P(6,7) + P(7,13)*PS199 - P(7,14)*PS197 + P(7,15)*PS87 + dt*(P(0,4)*PS216 + P(1,4)*PS217 - P(2,4)*PS214 + P(3,4)*PS215 + P(4,13)*PS199 - P(4,14)*PS197 + P(4,15)*PS87 + P(4,6));
	nextP(7,7) = P(4,7)*dt + P(7,7) + dt*(P(4,4)*dt + P(4,7));
	nextP(0,8) = P(0,8) - P(1,8)*PS11 - P(2,8)*PS12 - P(3,8)*PS13 + P(8,10)*PS6 + P(8,11)*PS7 + P(8,12)*PS9 + PS86*dt;
	nextP(1,8) = P(0,8)*PS11 + P(1,8) + P(2,8)*PS13 - P(3,8)*PS12 - P(8,10)*PS34 + P(8,11)*PS9 - P(8,12)*PS7 + PS124*dt;
	nextP(2,8) = P(0,8)*PS12 - P(1,8)*PS13 + P(2,8) + P(3,8)*PS11 - P(8,10)*PS9 - P(8,11)*PS34 + P(8,12)*PS6 + PS149*dt;
	nextP(3,8) = P(0,8)*PS13 + P(1,8)*PS12 - P(2,8)*PS11 + P(3,8) + P(8,10)*PS7 - P(8,11)*PS6 - P(8,12)*PS34 + PS169*dt;
	nextP(4,8) = P(0,8)*PS174 + P(1,8)*PS173 + P(2,8)*PS175 - P(3,8)*PS176 + P(4,8) + P(8,13)*PS43 + P(8,14)*PS172 - P(8,15)*PS171 + PS196*dt;
	nextP(5,8) = -P(0,8)*PS202 - P(1,8)*PS204 + P(2,8)*PS201 + P(3,8)*PS203 + P(5,8) - P(8,13)*PS193 + P(8,14)*PS75 + P(8,15)*PS190 + PS213*dt;
	nextP(6,8) = P(0,8)*PS216 + P(1,8)*PS217 - P(2,8)*PS214 + P(3,8)*PS215 + P(6,8) + P(8,13)*PS199 - P(8,14)*PS197 + P(8,15)*PS87 + dt*(P(0,5)*PS216 + P(1,5)*PS217 - P(2,5)*PS214 + P(3,5)*PS215 + P(5,13)*PS199 - P(5,14)*PS197 + P(5,15)*PS87 + P(5,6));
	nextP(7,8) = P(4,8)*dt + P(7,8) + dt*(P(4,5)*dt + P(5,7));
	nextP(8,8) = P(5,8)*dt + P(8,8) + dt*(P(5,5)*dt + P(5,8));
	nextP(0,9) = P(0,9) - P(1,9)*PS11 - P(2,9)*PS12 - P(3,9)*PS13 + P(9,10)*PS6 + P(9,11)*PS7 + P(9,12)*PS9 + PS94*dt;
	nextP(1,9) = P(0,9)*PS11 + P(1,9) + P(2,9)*PS13 - P(3,9)*PS12 - P(9,10)*PS34 + P(9,11)*PS9 - P(9,12)*PS7 + PS125*dt;
	nextP(2,9) = P(0,9)*PS12 - P(1,9)*PS13 + P(2,9) + P(3,9)*PS11 - P(9,10)*PS9 - P(9,11)*PS34 + P(9,12)*PS6 + PS150*dt;
	nextP(3,9) = P(0,9)*PS13 + P(1,9)*PS12 - P(2,9)*PS11 + P(3,9) + P(9,10)*PS7 - P(9,11)*PS6 - P(9,12)*PS34 + PS170*dt;
	nextP(4,9) = P(0,9)*PS174 + P(1,9)*PS173 + P(2,9)*PS175 - P(3,9)*PS176 + P(4,9) + P(9,13)*PS43 + P(9,14)*PS172 - P(9,15)*PS171 + PS200*dt;
	nextP(5,9) = -P(0,9)*PS202 - P(1,9)*PS204 + P(2,9)*PS201 + P(3,9)*PS203 + P(5,9) - P(9,13)*PS193 + P(9,14)*PS75 + P(9,15)*PS190 + PS218*dt;
	nextP(6,9) = P(0,9)*PS216 + P(1,9)*PS217 - P(2,9)*PS214 + P(3,9)*PS215 + P(6,9) + P(9,13)*PS199 - P(9,14)*PS197 + P(9,15)*PS87 + PS222*dt;
	nextP(7,9) = P(4,9)*dt + P(7,9) + dt*(P(4,6)*dt + P(6,7));
	nextP(8,9) = P(5,9)*dt + P(8,9) + dt*(P(5,6)*dt + P(6,8));
	nextP(9,9) = P(6,9)*dt + P(9,9) + dt*(P(6,6)*dt + P(6,9));
	nextP(0,10) = PS14;
	nextP(1,10) = PS105;
	nextP(2,10) = PS133;
	nextP(3,10) = PS151;
	nextP(4,10) = P(0,10)*PS174 + P(1,10)*PS173 + P(10,13)*PS43 + P(10,14)*PS172 - P(10,15)*PS171 + P(2,10)*PS175 - P(3,10)*PS176 + P(4,10);
	nextP(5,10) = -P(0,10)*PS202 - P(1,10)*PS204 - P(10,13)*PS193 + P(10,14)*PS75 + P(10,15)*PS190 + P(2,10)*PS201 + P(3,10)*PS203 + P(5,10);
	nextP(6,10) = P(0,10)*PS216 + P(1,10)*PS217 + P(10,13)*PS199 - P(10,14)*PS197 + P(10,15)*PS87 - P(2,10)*PS214 + P(3,10)*PS215 + P(6,10);
	nextP(7,10) = P(4,10)*dt + P(7,10);
	nextP(8,10) = P(5,10)*dt + P(8,10);
	nextP(9,10) = P(6,10)*dt + P(9,10);
	nextP(10,10) = P(10,10);
	nextP(0,11) = PS17;
	nextP(1,11) = PS97;
	nextP(2,11) = PS132;
	nextP(3,11) = PS155;
	nextP(4,11) = P(0,11)*PS174 + P(1,11)*PS173 + P(11,13)*PS43 + P(11,14)*PS172 - P(11,15)*PS171 + P(2,11)*PS175 - P(3,11)*PS176 + P(4,11);
	nextP(5,11) = -P(0,11)*PS202 - P(1,11)*PS204 - P(11,13)*PS193 + P(11,14)*PS75 + P(11,15)*PS190 + P(2,11)*PS201 + P(3,11)*PS203 + P(5,11);
	nextP(6,11) = P(0,11)*PS216 + P(1,11)*PS217 + P(11,13)*PS199 - P(11,14)*PS197 + P(11,15)*PS87 - P(2,11)*PS214 + P(3,11)*PS215 + P(6,11);
	nextP(7,11) = P(4,11)*dt + P(7,11);
	nextP(8,11) = P(5,11)*dt + P(8,11);
	nextP(9,11) = P(6,11)*dt + P(9,11);
	nextP(10,11) = P(10,11);
	nextP(11,11) = P(11,11);
	nextP(0,12) = PS20;
	nextP(1,12) = PS107;
	nextP(2,12) = PS127;
	nextP(3,12) = PS154;
	nextP(4,12) = P(0,12)*PS174 + P(1,12)*PS173 + P(12,13)*PS43 + P(12,14)*PS172 - P(12,15)*PS171 + P(2,12)*PS175 - P(3,12)*PS176 + P(4,12);
	nextP(5,12) = -P(0,12)*PS202 - P(1,12)*PS204 - P(12,13)*PS193 + P(12,14)*PS75 + P(12,15)*PS190 + P(2,12)*PS201 + P(3,12)*PS203 + P(5,12);
	nextP(6,12) = P(0,12)*PS216 + P(1,12)*PS217 + P(12,13)*PS199 - P(12,14)*PS197 + P(12,15)*PS87 - P(2,12)*PS214 + P(3,12)*PS215 + P(6,12);
	nextP(7,12) = P(4,12)*dt + P(7,12);
	nextP(8,12) = P(5,12)*dt + P(8,12);
	nextP(9,12) = P(6,12)*dt + P(9,12);
	nextP(10,12) = P(10,12);
	nextP(11,12) = P(11,12);
	nextP(12,12) = P(12,12);

	// process noise contribution for delta angle states can be very small compared to
	// the variances, therefore use algorithm to minimise numerical error
	for (unsigned i = 10; i <= 12; i++) {
		const int index = i - 10;
		nextP(i, i) = kahanSummation(nextP(i, i), process_noise(i), _delta_angle_bias_var_accum(index));
	}

	if (!_accel_bias_inhibit[0]) {
		// calculate variances and upper diagonal covariances for IMU X axis delta velocity bias state
		nextP(0,13) = PS44;
		nextP(1,13) = PS113;
		nextP(2,13) = PS138;
		nextP(3,13) = PS158;
		nextP(4,13) = PS177;
		nextP(5,13) = PS206;
		nextP(6,13) = PS221;
		nextP(7,13) = P(4,13)*dt + P(7,13);
		nextP(8,13) = P(5,13)*dt + P(8,13);
		nextP(9,13) = P(6,13)*dt + P(9,13);
		nextP(10,13) = P(10,13);
		nextP(11,13) = P(11,13);
		nextP(12,13) = P(12,13);
		nextP(13,13) = P(13,13);

		// add process noise that is not from the IMU
		// process noise contribution for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		nextP(13, 13) = kahanSummation(nextP(13, 13), process_noise(13), _delta_vel_bias_var_accum(0));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(13, _prev_dvel_bias_var(0));
		_delta_vel_bias_var_accum(0) = 0.f;

	}

	if (!_accel_bias_inhibit[1]) {
		// calculate variances and upper diagonal covariances for IMU Y axis delta velocity bias state

		nextP(0,14) = PS57;
		nextP(1,14) = PS117;
		nextP(2,14) = PS142;
		nextP(3,14) = PS162;
		nextP(4,14) = PS180;
		nextP(5,14) = PS205;
		nextP(6,14) = PS220;
		nextP(7,14) = P(4,14)*dt + P(7,14);
		nextP(8,14) = P(5,14)*dt + P(8,14);
		nextP(9,14) = P(6,14)*dt + P(9,14);
		nextP(10,14) = P(10,14);
		nextP(11,14) = P(11,14);
		nextP(12,14) = P(12,14);
		nextP(13,14) = P(13,14);
		nextP(14,14) = P(14,14);

		// add process noise that is not from the IMU
		// process noise contribution for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		nextP(14, 14) = kahanSummation(nextP(14, 14), process_noise(14), _delta_vel_bias_var_accum(1));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(14, _prev_dvel_bias_var(1));
		_delta_vel_bias_var_accum(1) = 0.f;

	}

	if (!_accel_bias_inhibit[2]) {
		// calculate variances and upper diagonal covariances for IMU Z axis delta velocity bias state
		nextP(0,15) = PS46;
		nextP(1,15) = PS114;
		nextP(2,15) = PS139;
		nextP(3,15) = PS159;
		nextP(4,15) = PS178;
		nextP(5,15) = PS209;
		nextP(6,15) = PS219;
		nextP(7,15) = P(4,15)*dt + P(7,15);
		nextP(8,15) = P(5,15)*dt + P(8,15);
		nextP(9,15) = P(6,15)*dt + P(9,15);
		nextP(10,15) = P(10,15);
		nextP(11,15) = P(11,15);
		nextP(12,15) = P(12,15);
		nextP(13,15) = P(13,15);
		nextP(14,15) = P(14,15);
		nextP(15,15) = P(15,15);

		// add process noise that is not from the IMU
		// process noise contribution for delta velocity states can be very small compared to
		// the variances, therefore use algorithm to minimise numerical error
		nextP(15, 15) = kahanSummation(nextP(15, 15), process_noise(15), _delta_vel_bias_var_accum(2));

	} else {
		nextP.uncorrelateCovarianceSetVariance<1>(15, _prev_dvel_bias_var(2));
		_delta_vel_bias_var_accum(2) = 0.f;
	}

	// Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
	if (_control_status.flags.mag_3D) {
		// calculate variances and upper diagonal covariances for earth and body magnetic field states


		nextP(0,16) = P(0,16) - P(1,16)*PS11 + P(10,16)*PS6 + P(11,16)*PS7 + P(12,16)*PS9 - P(2,16)*PS12 - P(3,16)*PS13;
		nextP(1,16) = P(0,16)*PS11 + P(1,16) - P(10,16)*PS34 + P(11,16)*PS9 - P(12,16)*PS7 + P(2,16)*PS13 - P(3,16)*PS12;
		nextP(2,16) = P(0,16)*PS12 - P(1,16)*PS13 - P(10,16)*PS9 - P(11,16)*PS34 + P(12,16)*PS6 + P(2,16) + P(3,16)*PS11;
		nextP(3,16) = P(0,16)*PS13 + P(1,16)*PS12 + P(10,16)*PS7 - P(11,16)*PS6 - P(12,16)*PS34 - P(2,16)*PS11 + P(3,16);
		nextP(4,16) = P(0,16)*PS174 + P(1,16)*PS173 + P(13,16)*PS43 + P(14,16)*PS172 - P(15,16)*PS171 + P(2,16)*PS175 - P(3,16)*PS176 + P(4,16);
		nextP(5,16) = -P(0,16)*PS202 - P(1,16)*PS204 - P(13,16)*PS193 + P(14,16)*PS75 + P(15,16)*PS190 + P(2,16)*PS201 + P(3,16)*PS203 + P(5,16);
		nextP(6,16) = P(0,16)*PS216 + P(1,16)*PS217 + P(13,16)*PS199 - P(14,16)*PS197 + P(15,16)*PS87 - P(2,16)*PS214 + P(3,16)*PS215 + P(6,16);
		nextP(7,16) = P(4,16)*dt + P(7,16);
		nextP(8,16) = P(5,16)*dt + P(8,16);
		nextP(9,16) = P(6,16)*dt + P(9,16);
		nextP(10,16) = P(10,16);
		nextP(11,16) = P(11,16);
		nextP(12,16) = P(12,16);
		nextP(13,16) = P(13,16);
		nextP(14,16) = P(14,16);
		nextP(15,16) = P(15,16);
		nextP(16,16) = P(16,16);
		nextP(0,17) = P(0,17) - P(1,17)*PS11 + P(10,17)*PS6 + P(11,17)*PS7 + P(12,17)*PS9 - P(2,17)*PS12 - P(3,17)*PS13;
		nextP(1,17) = P(0,17)*PS11 + P(1,17) - P(10,17)*PS34 + P(11,17)*PS9 - P(12,17)*PS7 + P(2,17)*PS13 - P(3,17)*PS12;
		nextP(2,17) = P(0,17)*PS12 - P(1,17)*PS13 - P(10,17)*PS9 - P(11,17)*PS34 + P(12,17)*PS6 + P(2,17) + P(3,17)*PS11;
		nextP(3,17) = P(0,17)*PS13 + P(1,17)*PS12 + P(10,17)*PS7 - P(11,17)*PS6 - P(12,17)*PS34 - P(2,17)*PS11 + P(3,17);
		nextP(4,17) = P(0,17)*PS174 + P(1,17)*PS173 + P(13,17)*PS43 + P(14,17)*PS172 - P(15,17)*PS171 + P(2,17)*PS175 - P(3,17)*PS176 + P(4,17);
		nextP(5,17) = -P(0,17)*PS202 - P(1,17)*PS204 - P(13,17)*PS193 + P(14,17)*PS75 + P(15,17)*PS190 + P(2,17)*PS201 + P(3,17)*PS203 + P(5,17);
		nextP(6,17) = P(0,17)*PS216 + P(1,17)*PS217 + P(13,17)*PS199 - P(14,17)*PS197 + P(15,17)*PS87 - P(2,17)*PS214 + P(3,17)*PS215 + P(6,17);
		nextP(7,17) = P(4,17)*dt + P(7,17);
		nextP(8,17) = P(5,17)*dt + P(8,17);
		nextP(9,17) = P(6,17)*dt + P(9,17);
		nextP(10,17) = P(10,17);
		nextP(11,17) = P(11,17);
		nextP(12,17) = P(12,17);
		nextP(13,17) = P(13,17);
		nextP(14,17) = P(14,17);
		nextP(15,17) = P(15,17);
		nextP(16,17) = P(16,17);
		nextP(17,17) = P(17,17);
		nextP(0,18) = P(0,18) - P(1,18)*PS11 + P(10,18)*PS6 + P(11,18)*PS7 + P(12,18)*PS9 - P(2,18)*PS12 - P(3,18)*PS13;
		nextP(1,18) = P(0,18)*PS11 + P(1,18) - P(10,18)*PS34 + P(11,18)*PS9 - P(12,18)*PS7 + P(2,18)*PS13 - P(3,18)*PS12;
		nextP(2,18) = P(0,18)*PS12 - P(1,18)*PS13 - P(10,18)*PS9 - P(11,18)*PS34 + P(12,18)*PS6 + P(2,18) + P(3,18)*PS11;
		nextP(3,18) = P(0,18)*PS13 + P(1,18)*PS12 + P(10,18)*PS7 - P(11,18)*PS6 - P(12,18)*PS34 - P(2,18)*PS11 + P(3,18);
		nextP(4,18) = P(0,18)*PS174 + P(1,18)*PS173 + P(13,18)*PS43 + P(14,18)*PS172 - P(15,18)*PS171 + P(2,18)*PS175 - P(3,18)*PS176 + P(4,18);
		nextP(5,18) = -P(0,18)*PS202 - P(1,18)*PS204 - P(13,18)*PS193 + P(14,18)*PS75 + P(15,18)*PS190 + P(2,18)*PS201 + P(3,18)*PS203 + P(5,18);
		nextP(6,18) = P(0,18)*PS216 + P(1,18)*PS217 + P(13,18)*PS199 - P(14,18)*PS197 + P(15,18)*PS87 - P(2,18)*PS214 + P(3,18)*PS215 + P(6,18);
		nextP(7,18) = P(4,18)*dt + P(7,18);
		nextP(8,18) = P(5,18)*dt + P(8,18);
		nextP(9,18) = P(6,18)*dt + P(9,18);
		nextP(10,18) = P(10,18);
		nextP(11,18) = P(11,18);
		nextP(12,18) = P(12,18);
		nextP(13,18) = P(13,18);
		nextP(14,18) = P(14,18);
		nextP(15,18) = P(15,18);
		nextP(16,18) = P(16,18);
		nextP(17,18) = P(17,18);
		nextP(18,18) = P(18,18);
		nextP(0,19) = P(0,19) - P(1,19)*PS11 + P(10,19)*PS6 + P(11,19)*PS7 + P(12,19)*PS9 - P(2,19)*PS12 - P(3,19)*PS13;
		nextP(1,19) = P(0,19)*PS11 + P(1,19) - P(10,19)*PS34 + P(11,19)*PS9 - P(12,19)*PS7 + P(2,19)*PS13 - P(3,19)*PS12;
		nextP(2,19) = P(0,19)*PS12 - P(1,19)*PS13 - P(10,19)*PS9 - P(11,19)*PS34 + P(12,19)*PS6 + P(2,19) + P(3,19)*PS11;
		nextP(3,19) = P(0,19)*PS13 + P(1,19)*PS12 + P(10,19)*PS7 - P(11,19)*PS6 - P(12,19)*PS34 - P(2,19)*PS11 + P(3,19);
		nextP(4,19) = P(0,19)*PS174 + P(1,19)*PS173 + P(13,19)*PS43 + P(14,19)*PS172 - P(15,19)*PS171 + P(2,19)*PS175 - P(3,19)*PS176 + P(4,19);
		nextP(5,19) = -P(0,19)*PS202 - P(1,19)*PS204 - P(13,19)*PS193 + P(14,19)*PS75 + P(15,19)*PS190 + P(2,19)*PS201 + P(3,19)*PS203 + P(5,19);
		nextP(6,19) = P(0,19)*PS216 + P(1,19)*PS217 + P(13,19)*PS199 - P(14,19)*PS197 + P(15,19)*PS87 - P(2,19)*PS214 + P(3,19)*PS215 + P(6,19);
		nextP(7,19) = P(4,19)*dt + P(7,19);
		nextP(8,19) = P(5,19)*dt + P(8,19);
		nextP(9,19) = P(6,19)*dt + P(9,19);
		nextP(10,19) = P(10,19);
		nextP(11,19) = P(11,19);
		nextP(12,19) = P(12,19);
		nextP(13,19) = P(13,19);
		nextP(14,19) = P(14,19);
		nextP(15,19) = P(15,19);
		nextP(16,19) = P(16,19);
		nextP(17,19) = P(17,19);
		nextP(18,19) = P(18,19);
		nextP(19,19) = P(19,19);
		nextP(0,20) = P(0,20) - P(1,20)*PS11 + P(10,20)*PS6 + P(11,20)*PS7 + P(12,20)*PS9 - P(2,20)*PS12 - P(3,20)*PS13;
		nextP(1,20) = P(0,20)*PS11 + P(1,20) - P(10,20)*PS34 + P(11,20)*PS9 - P(12,20)*PS7 + P(2,20)*PS13 - P(3,20)*PS12;
		nextP(2,20) = P(0,20)*PS12 - P(1,20)*PS13 - P(10,20)*PS9 - P(11,20)*PS34 + P(12,20)*PS6 + P(2,20) + P(3,20)*PS11;
		nextP(3,20) = P(0,20)*PS13 + P(1,20)*PS12 + P(10,20)*PS7 - P(11,20)*PS6 - P(12,20)*PS34 - P(2,20)*PS11 + P(3,20);
		nextP(4,20) = P(0,20)*PS174 + P(1,20)*PS173 + P(13,20)*PS43 + P(14,20)*PS172 - P(15,20)*PS171 + P(2,20)*PS175 - P(3,20)*PS176 + P(4,20);
		nextP(5,20) = -P(0,20)*PS202 - P(1,20)*PS204 - P(13,20)*PS193 + P(14,20)*PS75 + P(15,20)*PS190 + P(2,20)*PS201 + P(3,20)*PS203 + P(5,20);
		nextP(6,20) = P(0,20)*PS216 + P(1,20)*PS217 + P(13,20)*PS199 - P(14,20)*PS197 + P(15,20)*PS87 - P(2,20)*PS214 + P(3,20)*PS215 + P(6,20);
		nextP(7,20) = P(4,20)*dt + P(7,20);
		nextP(8,20) = P(5,20)*dt + P(8,20);
		nextP(9,20) = P(6,20)*dt + P(9,20);
		nextP(10,20) = P(10,20);
		nextP(11,20) = P(11,20);
		nextP(12,20) = P(12,20);
		nextP(13,20) = P(13,20);
		nextP(14,20) = P(14,20);
		nextP(15,20) = P(15,20);
		nextP(16,20) = P(16,20);
		nextP(17,20) = P(17,20);
		nextP(18,20) = P(18,20);
		nextP(19,20) = P(19,20);
		nextP(20,20) = P(20,20);
		nextP(0,21) = P(0,21) - P(1,21)*PS11 + P(10,21)*PS6 + P(11,21)*PS7 + P(12,21)*PS9 - P(2,21)*PS12 - P(3,21)*PS13;
		nextP(1,21) = P(0,21)*PS11 + P(1,21) - P(10,21)*PS34 + P(11,21)*PS9 - P(12,21)*PS7 + P(2,21)*PS13 - P(3,21)*PS12;
		nextP(2,21) = P(0,21)*PS12 - P(1,21)*PS13 - P(10,21)*PS9 - P(11,21)*PS34 + P(12,21)*PS6 + P(2,21) + P(3,21)*PS11;
		nextP(3,21) = P(0,21)*PS13 + P(1,21)*PS12 + P(10,21)*PS7 - P(11,21)*PS6 - P(12,21)*PS34 - P(2,21)*PS11 + P(3,21);
		nextP(4,21) = P(0,21)*PS174 + P(1,21)*PS173 + P(13,21)*PS43 + P(14,21)*PS172 - P(15,21)*PS171 + P(2,21)*PS175 - P(3,21)*PS176 + P(4,21);
		nextP(5,21) = -P(0,21)*PS202 - P(1,21)*PS204 - P(13,21)*PS193 + P(14,21)*PS75 + P(15,21)*PS190 + P(2,21)*PS201 + P(3,21)*PS203 + P(5,21);
		nextP(6,21) = P(0,21)*PS216 + P(1,21)*PS217 + P(13,21)*PS199 - P(14,21)*PS197 + P(15,21)*PS87 - P(2,21)*PS214 + P(3,21)*PS215 + P(6,21);
		nextP(7,21) = P(4,21)*dt + P(7,21);
		nextP(8,21) = P(5,21)*dt + P(8,21);
		nextP(9,21) = P(6,21)*dt + P(9,21);
		nextP(10,21) = P(10,21);
		nextP(11,21) = P(11,21);
		nextP(12,21) = P(12,21);
		nextP(13,21) = P(13,21);
		nextP(14,21) = P(14,21);
		nextP(15,21) = P(15,21);
		nextP(16,21) = P(16,21);
		nextP(17,21) = P(17,21);
		nextP(18,21) = P(18,21);
		nextP(19,21) = P(19,21);
		nextP(20,21) = P(20,21);
		nextP(21,21) = P(21,21);

		// add process noise that is not from the IMU
		for (unsigned i = 16; i <= 21; i++) {
			nextP(i, i) += process_noise(i);
		}

	}

	// Don't do covariance prediction on wind states unless we are using them
	if (_control_status.flags.wind) {

		// calculate variances and upper diagonal covariances for wind states

		nextP(0,22) = P(0,22) - P(1,22)*PS11 + P(10,22)*PS6 + P(11,22)*PS7 + P(12,22)*PS9 - P(2,22)*PS12 - P(3,22)*PS13;
		nextP(1,22) = P(0,22)*PS11 + P(1,22) - P(10,22)*PS34 + P(11,22)*PS9 - P(12,22)*PS7 + P(2,22)*PS13 - P(3,22)*PS12;
		nextP(2,22) = P(0,22)*PS12 - P(1,22)*PS13 - P(10,22)*PS9 - P(11,22)*PS34 + P(12,22)*PS6 + P(2,22) + P(3,22)*PS11;
		nextP(3,22) = P(0,22)*PS13 + P(1,22)*PS12 + P(10,22)*PS7 - P(11,22)*PS6 - P(12,22)*PS34 - P(2,22)*PS11 + P(3,22);
		nextP(4,22) = P(0,22)*PS174 + P(1,22)*PS173 + P(13,22)*PS43 + P(14,22)*PS172 - P(15,22)*PS171 + P(2,22)*PS175 - P(3,22)*PS176 + P(4,22);
		nextP(5,22) = -P(0,22)*PS202 - P(1,22)*PS204 - P(13,22)*PS193 + P(14,22)*PS75 + P(15,22)*PS190 + P(2,22)*PS201 + P(3,22)*PS203 + P(5,22);
		nextP(6,22) = P(0,22)*PS216 + P(1,22)*PS217 + P(13,22)*PS199 - P(14,22)*PS197 + P(15,22)*PS87 - P(2,22)*PS214 + P(3,22)*PS215 + P(6,22);
		nextP(7,22) = P(4,22)*dt + P(7,22);
		nextP(8,22) = P(5,22)*dt + P(8,22);
		nextP(9,22) = P(6,22)*dt + P(9,22);
		nextP(10,22) = P(10,22);
		nextP(11,22) = P(11,22);
		nextP(12,22) = P(12,22);
		nextP(13,22) = P(13,22);
		nextP(14,22) = P(14,22);
		nextP(15,22) = P(15,22);
		nextP(16,22) = P(16,22);
		nextP(17,22) = P(17,22);
		nextP(18,22) = P(18,22);
		nextP(19,22) = P(19,22);
		nextP(20,22) = P(20,22);
		nextP(21,22) = P(21,22);
		nextP(22,22) = P(22,22);
		nextP(0,23) = P(0,23) - P(1,23)*PS11 + P(10,23)*PS6 + P(11,23)*PS7 + P(12,23)*PS9 - P(2,23)*PS12 - P(3,23)*PS13;
		nextP(1,23) = P(0,23)*PS11 + P(1,23) - P(10,23)*PS34 + P(11,23)*PS9 - P(12,23)*PS7 + P(2,23)*PS13 - P(3,23)*PS12;
		nextP(2,23) = P(0,23)*PS12 - P(1,23)*PS13 - P(10,23)*PS9 - P(11,23)*PS34 + P(12,23)*PS6 + P(2,23) + P(3,23)*PS11;
		nextP(3,23) = P(0,23)*PS13 + P(1,23)*PS12 + P(10,23)*PS7 - P(11,23)*PS6 - P(12,23)*PS34 - P(2,23)*PS11 + P(3,23);
		nextP(4,23) = P(0,23)*PS174 + P(1,23)*PS173 + P(13,23)*PS43 + P(14,23)*PS172 - P(15,23)*PS171 + P(2,23)*PS175 - P(3,23)*PS176 + P(4,23);
		nextP(5,23) = -P(0,23)*PS202 - P(1,23)*PS204 - P(13,23)*PS193 + P(14,23)*PS75 + P(15,23)*PS190 + P(2,23)*PS201 + P(3,23)*PS203 + P(5,23);
		nextP(6,23) = P(0,23)*PS216 + P(1,23)*PS217 + P(13,23)*PS199 - P(14,23)*PS197 + P(15,23)*PS87 - P(2,23)*PS214 + P(3,23)*PS215 + P(6,23);
		nextP(7,23) = P(4,23)*dt + P(7,23);
		nextP(8,23) = P(5,23)*dt + P(8,23);
		nextP(9,23) = P(6,23)*dt + P(9,23);
		nextP(10,23) = P(10,23);
		nextP(11,23) = P(11,23);
		nextP(12,23) = P(12,23);
		nextP(13,23) = P(13,23);
		nextP(14,23) = P(14,23);
		nextP(15,23) = P(15,23);
		nextP(16,23) = P(16,23);
		nextP(17,23) = P(17,23);
		nextP(18,23) = P(18,23);
		nextP(19,23) = P(19,23);
		nextP(20,23) = P(20,23);
		nextP(21,23) = P(21,23);
		nextP(22,23) = P(22,23);
		nextP(23,23) = P(23,23);

		// add process noise that is not from the IMU
		for (unsigned i = 22; i <= 23; i++) {
			nextP(i, i) += process_noise(i);
		}

	}

	// stop position covariance growth if our total position variance reaches 100m
	// this can happen if we lose gps for some time
	if ((P(7, 7) + P(8, 8)) > 1e4f) {
		for (uint8_t i = 7; i <= 8; i++) {
			for (uint8_t j = 0; j < _k_num_states; j++) {
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			}
		}
	}

	// covariance matrix is symmetrical, so copy upper half to lower half
	for (unsigned row = 1; row < _k_num_states; row++) {
		for (unsigned column = 0 ; column < row; column++) {
			P(row, column) = P(column, row) = nextP(column, row);
		}
	}

	// copy variances (diagonals)
	for (unsigned i = 0; i < _k_num_states; i++) {
		P(i, i) = nextP(i, i);
	}

	// fix gross errors in the covariance matrix and ensure rows and
	// columns for un-used states are zero
	fixCovarianceErrors(false);

}

void Ekf::fixCovarianceErrors(bool force_symmetry)
{
	// NOTE: This limiting is a last resort and should not be relied on
	// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
	// and set corresponding entries in Q to zero when states exceed 50% of the limit
	// Covariance diagonal limits. Use same values for states which
	// belong to the same group (e.g. vel_x, vel_y, vel_z)
	float P_lim[8] = {};
	P_lim[0] = 1.0f;		// quaternion max var
	P_lim[1] = 1e6f;		// velocity max var
	P_lim[2] = 1e6f;		// positiion max var
	P_lim[3] = 1.0f;		// gyro bias max var
	P_lim[4] = 1.0f;		// delta velocity z bias max var
	P_lim[5] = 1.0f;		// earth mag field max var
	P_lim[6] = 1.0f;		// body mag field max var
	P_lim[7] = 1e6f;		// wind max var

	for (int i = 0; i <= 3; i++) {
		// quaternion states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[0]);
	}

	for (int i = 4; i <= 6; i++) {
		// NED velocity states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[1]);
	}

	for (int i = 7; i <= 9; i++) {
		// NED position states
		P(i, i) = math::constrain(P(i, i), 1e-6f, P_lim[2]);
	}

	for (int i = 10; i <= 12; i++) {
		// gyro bias states
		P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[3]);
	}

	// force symmetry on the quaternion, velocity and position state covariances
	if (force_symmetry) {
		P.makeRowColSymmetric<13>(0);
	}

	// the following states are optional and are deactivated when not required
	// by ensuring the corresponding covariance matrix values are kept at zero

	// accelerometer bias states
	if (!_accel_bias_inhibit[0] || !_accel_bias_inhibit[1] || !_accel_bias_inhibit[2]) {
		// Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		const float minSafeStateVar = 1e-9f;
		float maxStateVar = minSafeStateVar;
		bool resetRequired = false;

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_accel_bias_inhibit[stateIndex - 13]) {
				// Skip the check for the inhibited axis
				continue;
			}

			if (P(stateIndex, stateIndex) > maxStateVar) {
				maxStateVar = P(stateIndex, stateIndex);

			} else if (P(stateIndex, stateIndex) < minSafeStateVar) {
				resetRequired = true;
			}
		}

		// To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
		// not exceed 100 and the minimum variance must not fall below the target minimum
		// Also limit variance to a maximum equivalent to a 0.1g uncertainty
		const float minStateVarTarget = 5E-8f;
		float minAllowedStateVar = fmaxf(0.01f * maxStateVar, minStateVarTarget);

		for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
			if (_accel_bias_inhibit[stateIndex - 13]) {
				// Skip the check for the inhibited axis
				continue;
			}

			P(stateIndex, stateIndex) = math::constrain(P(stateIndex, stateIndex), minAllowedStateVar,
						    sq(0.1f * CONSTANTS_ONE_G * _dt_ekf_avg));
		}

		// If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) {
			P.uncorrelateCovariance<3>(13);
		}

		// Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
		// calculate accel bias term aligned with the gravity vector
		const float dVel_bias_lim = 0.9f * _params.acc_bias_lim * _dt_ekf_avg;
		const float down_dvel_bias = _state.delta_vel_bias.dot(Vector3f(_R_to_earth.row(2)));

		// check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bool bad_acc_bias = (fabsf(down_dvel_bias) > dVel_bias_lim
				     && ((down_dvel_bias * _gps_vel_innov(2) < 0.0f && _control_status.flags.gps)
					 || (down_dvel_bias * _ev_vel_innov(2) < 0.0f && _control_status.flags.ev_vel))
				     && ((down_dvel_bias * _gps_pos_innov(2) < 0.0f && _control_status.flags.gps_hgt)
					 || (down_dvel_bias * _baro_hgt_innov(2) < 0.0f && _control_status.flags.baro_hgt)
					 || (down_dvel_bias * _rng_hgt_innov(2) < 0.0f && _control_status.flags.rng_hgt)
					 || (down_dvel_bias * _ev_pos_innov(2) < 0.0f && _control_status.flags.ev_hgt)));

		// record the pass/fail
		if (!bad_acc_bias) {
			_fault_status.flags.bad_acc_bias = false;
			_time_acc_bias_check = _time_last_imu;

		} else {
			_fault_status.flags.bad_acc_bias = true;
		}

		// if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		// the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(_time_acc_bias_check, (uint64_t)7e6)) {

			P.uncorrelateCovariance<3>(13);

			_time_acc_bias_check = _time_last_imu;
			_fault_status.flags.bad_acc_bias = false;
			_warning_events.flags.invalid_accel_bias_cov_reset = true;
			ECL_WARN("invalid accel bias - covariance reset");

		} else if (force_symmetry) {
			// ensure the covariance values are symmetrical
			P.makeRowColSymmetric<3>(13);
		}

	}

	// magnetic field states
	if (!_control_status.flags.mag_3D) {
		zeroMagCov();

	} else {
		// constrain variances
		for (int i = 16; i <= 18; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[5]);
		}

		for (int i = 19; i <= 21; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[6]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<3>(16);
			P.makeRowColSymmetric<3>(19);
		}

	}

	// wind velocity states
	if (!_control_status.flags.wind) {
		P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

	} else {
		// constrain variances
		for (int i = 22; i <= 23; i++) {
			P(i, i) = math::constrain(P(i, i), 0.0f, P_lim[7]);
		}

		// force symmetry
		if (force_symmetry) {
			P.makeRowColSymmetric<2>(22);
		}
	}
}

// if the covariance correction will result in a negative variance, then
// the covariance matrix is unhealthy and must be corrected
bool Ekf::checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP)
{
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);
			healthy = false;
		}
	}

	return healthy;
}

void Ekf::resetMagRelatedCovariances()
{
	resetQuatCov();
	resetMagCov();
}

void Ekf::resetQuatCov()
{
	zeroQuatCov();

	// define the initial angle uncertainty as variances for a rotation vector
	Vector3f rot_vec_var;
	rot_vec_var.setAll(sq(_params.initial_tilt_err));

	initialiseQuatCovariances(rot_vec_var);
}

void Ekf::zeroQuatCov()
{
	P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
	P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);
}

void Ekf::resetMagCov()
{
	// reset the corresponding rows and columns in the covariance matrix and
	// set the variances on the magnetic field states to the measurement variance
	clearMagCov();

	P.uncorrelateCovarianceSetVariance<3>(16, sq(_params.mag_noise));
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

	if (!_control_status.flags.mag_3D) {
		// save covariance data for re-use when auto-switching between heading and 3-axis fusion
		// if already in 3-axis fusion mode, the covariances are automatically saved when switching out
		// of this mode
		saveMagCovData();
	}
}

void Ekf::clearMagCov()
{
	zeroMagCov();
	_mag_decl_cov_reset = false;
}

void Ekf::zeroMagCov()
{
	P.uncorrelateCovarianceSetVariance<3>(16, 0.0f);
	P.uncorrelateCovarianceSetVariance<3>(19, 0.0f);
}

void Ekf::resetZDeltaAngBiasCov()
{
	const float init_delta_ang_bias_var = sq(_params.switch_on_gyro_bias * _dt_ekf_avg);

	P.uncorrelateCovarianceSetVariance<1>(12, init_delta_ang_bias_var);
}

void Ekf::resetWindCovarianceUsingAirspeed()
{
	// Derived using EKF/matlab/scripts/Inertial Nav EKF/wind_cov.py
	// TODO: explicitly include the sideslip angle in the derivation
	const float euler_yaw = getEuler321Yaw(_state.quat_nominal);
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));
	constexpr float initial_sideslip_uncertainty = math::radians(15.0f);
	const float initial_wind_var_body_y = sq(_airspeed_sample_delayed.true_airspeed * sinf(initial_sideslip_uncertainty));
	constexpr float R_yaw = sq(math::radians(10.0f));

	const float cos_yaw = cosf(euler_yaw);
	const float sin_yaw = sinf(euler_yaw);

	// rotate wind velocity into earth frame aligned with vehicle yaw
	const float Wx = _state.wind_vel(0) * cos_yaw + _state.wind_vel(1) * sin_yaw;
	const float Wy = -_state.wind_vel(0) * sin_yaw + _state.wind_vel(1) * cos_yaw;

	// it is safer to remove all existing correlations to other states at this time
	P.uncorrelateCovarianceSetVariance<2>(22, 0.0f);

	P(22, 22) = R_TAS * sq(cos_yaw) + R_yaw * sq(-Wx * sin_yaw - Wy * cos_yaw) + initial_wind_var_body_y * sq(sin_yaw);
	P(22, 23) = R_TAS * sin_yaw * cos_yaw + R_yaw * (-Wx * sin_yaw - Wy * cos_yaw) * (Wx * cos_yaw - Wy * sin_yaw) -
		    initial_wind_var_body_y * sin_yaw * cos_yaw;
	P(23, 22) = P(22, 23);
	P(23, 23) = R_TAS * sq(sin_yaw) + R_yaw * sq(Wx * cos_yaw - Wy * sin_yaw) + initial_wind_var_body_y * sq(cos_yaw);

	// Now add the variance due to uncertainty in vehicle velocity that was used to calculate the initial wind speed
	P(22, 22) += P(4, 4);
	P(23, 23) += P(5, 5);
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file drag_fusion.cpp
 * Body frame drag fusion methods used for multi-rotor wind estimation.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::fuseDrag()
{
	SparseVector24f<0,1,2,3,4,5,6,22,23> Hfusion;  // Observation Jacobians
	Vector24f Kfusion; // Kalman gain vector

	const float R_ACC = fmaxf(_params.drag_noise, 0.5f); // observation noise variance in specific force drag (m/sec**2)**2
	const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

	// correct rotor momentum drag for increase in required rotor mass flow with altitude
	// obtained from momentum disc theory
	const float mcoef_corrrected = _params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

	// drag model parameters
	const bool using_bcoef_x = _params.bcoef_x > 1.0f;
	const bool using_bcoef_y = _params.bcoef_y > 1.0f;
	const bool using_mcoef   = _params.mcoef   > 0.001f;

	if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
		return;
	}

	// get latest estimated orientation
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float &vn = _state.vel(0);
	const float &ve = _state.vel(1);
	const float &vd = _state.vel(2);

	// get latest wind velocity in earth frame
	const float &vwn = _state.wind_vel(0);
	const float &vwe = _state.wind_vel(1);

	// predicted specific forces
	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(vn - vwn, ve - vwe, vd);
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);
	const Vector3f rel_wind_body = earth_to_body * rel_wind_earth;

	// perform sequential fusion of XY specific forces
	for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
		// measured drag acceleration corrected for sensor bias
		const float mea_acc = _drag_sample_delayed.accelXY(axis_index)  - _state.delta_vel_bias(axis_index) / _dt_ekf_avg;

		// predicted drag force sign is opposite to predicted wind relative velocity
		const float drag_sign = (rel_wind_body(axis_index) >= 0.f) ? -1.f : 1.f;

		// Drag is modelled as an arbitrary combination of bluff body drag that proportional to
		// equivalent airspeed squared, and rotor momentum drag that is proportional to true airspeed
		// parallel to the rotor disc and mass flow through the rotor disc.
		float pred_acc = 0.0f; // predicted drag acceleration

		if (axis_index == 0) {
			float Kacc; // Derivative of specific force wrt airspeed

			if (using_mcoef && using_bcoef_x) {
				// Use a combination of bluff body and propeller momentum drag
				const float bcoef_inv = 1.0f / _params.bcoef_x;
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = 0.5 * rho * bcoef_inv * airspeed**2 + mcoef_corrrected * airspeed
				const float airspeed = (_params.bcoef_x  / rho) * (- mcoef_corrrected  + sqrtf(sq(mcoef_corrrected) + 2.0f * rho * bcoef_inv * fabsf(mea_acc)));
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed + mcoef_corrrected);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(0)) * drag_sign - rel_wind_body(0) * mcoef_corrrected;

			} else if (using_mcoef) {
				// Use propeller momentum drag only
				Kacc = fmaxf(1e-1f, mcoef_corrrected);
				pred_acc = - rel_wind_body(0) * mcoef_corrrected;

			} else if (using_bcoef_x) {
				// Use bluff body drag only
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = (0.5 * rho / _params.bcoef_x) * airspeed**2
				const float airspeed = sqrtf((2.0f * _params.bcoef_x  * fabsf(mea_acc)) / rho);
				const float bcoef_inv = 1.0f / _params.bcoef_x;
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(0)) * drag_sign;

			} else {
				// skip this axis
				continue;
			}

			// intermediate variables
			const float HK0 = vn - vwn;
			const float HK1 = ve - vwe;
			const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
			const float HK3 = 2*Kacc;
			const float HK4 = HK0*q1 + HK1*q2 + q3*vd;
			const float HK5 = HK0*q2 - HK1*q1 + q0*vd;
			const float HK6 = -HK0*q3 + HK1*q0 + q1*vd;
			const float HK7 = ecl::powf(q0, 2) + ecl::powf(q1, 2) - ecl::powf(q2, 2) - ecl::powf(q3, 2);
			const float HK8 = HK7*Kacc;
			const float HK9 = q0*q3 + q1*q2;
			const float HK10 = HK3*HK9;
			const float HK11 = q0*q2 - q1*q3;
			const float HK12 = 2*HK9;
			const float HK13 = 2*HK11;
			const float HK14 = 2*HK4;
			const float HK15 = 2*HK2;
			const float HK16 = 2*HK5;
			const float HK17 = 2*HK6;
			const float HK18 = -HK12*P(0,23) + HK12*P(0,5) - HK13*P(0,6) + HK14*P(0,1) + HK15*P(0,0) - HK16*P(0,2) + HK17*P(0,3) - HK7*P(0,22) + HK7*P(0,4);
			const float HK19 = HK12*P(5,23);
			const float HK20 = -HK12*P(23,23) - HK13*P(6,23) + HK14*P(1,23) + HK15*P(0,23) - HK16*P(2,23) + HK17*P(3,23) + HK19 - HK7*P(22,23) + HK7*P(4,23);
			const float HK21 = ecl::powf(Kacc, 2);
			const float HK22 = HK12*HK21;
			const float HK23 = HK12*P(5,5) - HK13*P(5,6) + HK14*P(1,5) + HK15*P(0,5) - HK16*P(2,5) + HK17*P(3,5) - HK19 + HK7*P(4,5) - HK7*P(5,22);
			const float HK24 = HK12*P(5,6) - HK12*P(6,23) - HK13*P(6,6) + HK14*P(1,6) + HK15*P(0,6) - HK16*P(2,6) + HK17*P(3,6) + HK7*P(4,6) - HK7*P(6,22);
			const float HK25 = HK7*P(4,22);
			const float HK26 = -HK12*P(4,23) + HK12*P(4,5) - HK13*P(4,6) + HK14*P(1,4) + HK15*P(0,4) - HK16*P(2,4) + HK17*P(3,4) - HK25 + HK7*P(4,4);
			const float HK27 = HK21*HK7;
			const float HK28 = -HK12*P(22,23) + HK12*P(5,22) - HK13*P(6,22) + HK14*P(1,22) + HK15*P(0,22) - HK16*P(2,22) + HK17*P(3,22) + HK25 - HK7*P(22,22);
			const float HK29 = -HK12*P(1,23) + HK12*P(1,5) - HK13*P(1,6) + HK14*P(1,1) + HK15*P(0,1) - HK16*P(1,2) + HK17*P(1,3) - HK7*P(1,22) + HK7*P(1,4);
			const float HK30 = -HK12*P(2,23) + HK12*P(2,5) - HK13*P(2,6) + HK14*P(1,2) + HK15*P(0,2) - HK16*P(2,2) + HK17*P(2,3) - HK7*P(2,22) + HK7*P(2,4);
			const float HK31 = -HK12*P(3,23) + HK12*P(3,5) - HK13*P(3,6) + HK14*P(1,3) + HK15*P(0,3) - HK16*P(2,3) + HK17*P(3,3) - HK7*P(3,22) + HK7*P(3,4);
			//const float HK32 = Kacc/(-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);

			// calculate innovation variance and exit if badly conditioned
			_drag_innov_var(0) = (-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);
			if (_drag_innov_var(0) < R_ACC) {
				return;
			}

			const float HK32 = Kacc / _drag_innov_var(0);

			// Observation Jacobians
			Hfusion.at<0>() = -HK2*HK3;
			Hfusion.at<1>() = -HK3*HK4;
			Hfusion.at<2>() = HK3*HK5;
			Hfusion.at<3>() = -HK3*HK6;
			Hfusion.at<4>() = -HK8;
			Hfusion.at<5>() = -HK10;
			Hfusion.at<6>() = HK11*HK3;
			Hfusion.at<22>() = HK8;
			Hfusion.at<23>() = HK10;

			// Kalman gains
			// Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
			// Kfusion(0) = -HK18*HK32;
			// Kfusion(1) = -HK29*HK32;
			// Kfusion(2) = -HK30*HK32;
			// Kfusion(3) = -HK31*HK32;
			// Kfusion(4) = -HK26*HK32;
			// Kfusion(5) = -HK23*HK32;
			// Kfusion(6) = -HK24*HK32;
			// Kfusion(7) = -HK32*(HK12*P(5,7) - HK12*P(7,23) - HK13*P(6,7) + HK14*P(1,7) + HK15*P(0,7) - HK16*P(2,7) + HK17*P(3,7) + HK7*P(4,7) - HK7*P(7,22));
			// Kfusion(8) = -HK32*(HK12*P(5,8) - HK12*P(8,23) - HK13*P(6,8) + HK14*P(1,8) + HK15*P(0,8) - HK16*P(2,8) + HK17*P(3,8) + HK7*P(4,8) - HK7*P(8,22));
			// Kfusion(9) = -HK32*(HK12*P(5,9) - HK12*P(9,23) - HK13*P(6,9) + HK14*P(1,9) + HK15*P(0,9) - HK16*P(2,9) + HK17*P(3,9) + HK7*P(4,9) - HK7*P(9,22));
			// Kfusion(10) = -HK32*(-HK12*P(10,23) + HK12*P(5,10) - HK13*P(6,10) + HK14*P(1,10) + HK15*P(0,10) - HK16*P(2,10) + HK17*P(3,10) - HK7*P(10,22) + HK7*P(4,10));
			// Kfusion(11) = -HK32*(-HK12*P(11,23) + HK12*P(5,11) - HK13*P(6,11) + HK14*P(1,11) + HK15*P(0,11) - HK16*P(2,11) + HK17*P(3,11) - HK7*P(11,22) + HK7*P(4,11));
			// Kfusion(12) = -HK32*(-HK12*P(12,23) + HK12*P(5,12) - HK13*P(6,12) + HK14*P(1,12) + HK15*P(0,12) - HK16*P(2,12) + HK17*P(3,12) - HK7*P(12,22) + HK7*P(4,12));
			// Kfusion(13) = -HK32*(-HK12*P(13,23) + HK12*P(5,13) - HK13*P(6,13) + HK14*P(1,13) + HK15*P(0,13) - HK16*P(2,13) + HK17*P(3,13) - HK7*P(13,22) + HK7*P(4,13));
			// Kfusion(14) = -HK32*(-HK12*P(14,23) + HK12*P(5,14) - HK13*P(6,14) + HK14*P(1,14) + HK15*P(0,14) - HK16*P(2,14) + HK17*P(3,14) - HK7*P(14,22) + HK7*P(4,14));
			// Kfusion(15) = -HK32*(-HK12*P(15,23) + HK12*P(5,15) - HK13*P(6,15) + HK14*P(1,15) + HK15*P(0,15) - HK16*P(2,15) + HK17*P(3,15) - HK7*P(15,22) + HK7*P(4,15));
			// Kfusion(16) = -HK32*(-HK12*P(16,23) + HK12*P(5,16) - HK13*P(6,16) + HK14*P(1,16) + HK15*P(0,16) - HK16*P(2,16) + HK17*P(3,16) - HK7*P(16,22) + HK7*P(4,16));
			// Kfusion(17) = -HK32*(-HK12*P(17,23) + HK12*P(5,17) - HK13*P(6,17) + HK14*P(1,17) + HK15*P(0,17) - HK16*P(2,17) + HK17*P(3,17) - HK7*P(17,22) + HK7*P(4,17));
			// Kfusion(18) = -HK32*(-HK12*P(18,23) + HK12*P(5,18) - HK13*P(6,18) + HK14*P(1,18) + HK15*P(0,18) - HK16*P(2,18) + HK17*P(3,18) - HK7*P(18,22) + HK7*P(4,18));
			// Kfusion(19) = -HK32*(-HK12*P(19,23) + HK12*P(5,19) - HK13*P(6,19) + HK14*P(1,19) + HK15*P(0,19) - HK16*P(2,19) + HK17*P(3,19) - HK7*P(19,22) + HK7*P(4,19));
			// Kfusion(20) = -HK32*(-HK12*P(20,23) + HK12*P(5,20) - HK13*P(6,20) + HK14*P(1,20) + HK15*P(0,20) - HK16*P(2,20) + HK17*P(3,20) - HK7*P(20,22) + HK7*P(4,20));
			// Kfusion(21) = -HK32*(-HK12*P(21,23) + HK12*P(5,21) - HK13*P(6,21) + HK14*P(1,21) + HK15*P(0,21) - HK16*P(2,21) + HK17*P(3,21) - HK7*P(21,22) + HK7*P(4,21));
			Kfusion(22) = -HK28*HK32;
			Kfusion(23) = -HK20*HK32;


		} else if (axis_index == 1) {
			float Kacc; // Derivative of specific force wrt airspeed

			if (using_mcoef && using_bcoef_y) {
				// Use a combination of bluff body and propeller momentum drag
				const float bcoef_inv = 1.0f / _params.bcoef_y;
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = 0.5 * rho * bcoef_inv * airspeed**2 + mcoef_corrrected * airspeed
				const float airspeed = (_params.bcoef_y  / rho) * (- mcoef_corrrected  + sqrtf(sq(mcoef_corrrected) + 2.0f * rho * bcoef_inv * fabsf(mea_acc)));
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed + mcoef_corrrected);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(1)) * drag_sign - rel_wind_body(1) * mcoef_corrrected;

			} else if (using_mcoef) {
				// Use propeller momentum drag only
				Kacc = fmaxf(1e-1f, mcoef_corrrected);
				pred_acc = - rel_wind_body(1) * mcoef_corrrected;

			} else if (using_bcoef_y) {
				// Use bluff body drag only
				// The airspeed used for linearisation is calculated from the measured acceleration by solving the following quadratic
				// mea_acc = (0.5 * rho / _params.bcoef_y) * airspeed**2
				const float airspeed = sqrtf((2.0f * _params.bcoef_y * fabsf(mea_acc)) / rho);
				const float bcoef_inv = 1.0f / _params.bcoef_y;
				Kacc = fmaxf(1e-1f, rho * bcoef_inv * airspeed);
				pred_acc = 0.5f * bcoef_inv * rho * sq(rel_wind_body(1)) * drag_sign;

			} else {
				// nothing more to do
				return;
			}

			// intermediate variables
			const float HK0 = ve - vwe;
			const float HK1 = vn - vwn;
			const float HK2 = HK0*q0 - HK1*q3 + q1*vd;
			const float HK3 = 2*Kacc;
			const float HK4 = -HK0*q1 + HK1*q2 + q0*vd;
			const float HK5 = HK0*q2 + HK1*q1 + q3*vd;
			const float HK6 = HK0*q3 + HK1*q0 - q2*vd;
			const float HK7 = q0*q3 - q1*q2;
			const float HK8 = HK3*HK7;
			const float HK9 = ecl::powf(q0, 2) - ecl::powf(q1, 2) + ecl::powf(q2, 2) - ecl::powf(q3, 2);
			const float HK10 = HK9*Kacc;
			const float HK11 = q0*q1 + q2*q3;
			const float HK12 = 2*HK11;
			const float HK13 = 2*HK7;
			const float HK14 = 2*HK5;
			const float HK15 = 2*HK2;
			const float HK16 = 2*HK4;
			const float HK17 = 2*HK6;
			const float HK18 = HK12*P(0,6) + HK13*P(0,22) - HK13*P(0,4) + HK14*P(0,2) + HK15*P(0,0) + HK16*P(0,1) - HK17*P(0,3) - HK9*P(0,23) + HK9*P(0,5);
			const float HK19 = ecl::powf(Kacc, 2);
			const float HK20 = HK12*P(6,6) - HK13*P(4,6) + HK13*P(6,22) + HK14*P(2,6) + HK15*P(0,6) + HK16*P(1,6) - HK17*P(3,6) + HK9*P(5,6) - HK9*P(6,23);
			const float HK21 = HK13*P(4,22);
			const float HK22 = HK12*P(6,22) + HK13*P(22,22) + HK14*P(2,22) + HK15*P(0,22) + HK16*P(1,22) - HK17*P(3,22) - HK21 - HK9*P(22,23) + HK9*P(5,22);
			const float HK23 = HK13*HK19;
			const float HK24 = HK12*P(4,6) - HK13*P(4,4) + HK14*P(2,4) + HK15*P(0,4) + HK16*P(1,4) - HK17*P(3,4) + HK21 - HK9*P(4,23) + HK9*P(4,5);
			const float HK25 = HK9*P(5,23);
			const float HK26 = HK12*P(5,6) - HK13*P(4,5) + HK13*P(5,22) + HK14*P(2,5) + HK15*P(0,5) + HK16*P(1,5) - HK17*P(3,5) - HK25 + HK9*P(5,5);
			const float HK27 = HK19*HK9;
			const float HK28 = HK12*P(6,23) + HK13*P(22,23) - HK13*P(4,23) + HK14*P(2,23) + HK15*P(0,23) + HK16*P(1,23) - HK17*P(3,23) + HK25 - HK9*P(23,23);
			const float HK29 = HK12*P(2,6) + HK13*P(2,22) - HK13*P(2,4) + HK14*P(2,2) + HK15*P(0,2) + HK16*P(1,2) - HK17*P(2,3) - HK9*P(2,23) + HK9*P(2,5);
			const float HK30 = HK12*P(1,6) + HK13*P(1,22) - HK13*P(1,4) + HK14*P(1,2) + HK15*P(0,1) + HK16*P(1,1) - HK17*P(1,3) - HK9*P(1,23) + HK9*P(1,5);
			const float HK31 = HK12*P(3,6) + HK13*P(3,22) - HK13*P(3,4) + HK14*P(2,3) + HK15*P(0,3) + HK16*P(1,3) - HK17*P(3,3) - HK9*P(3,23) + HK9*P(3,5);
			// const float HK32 = Kacc/(HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);

			_drag_innov_var(1) = (HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);
			if (_drag_innov_var(1) < R_ACC) {
				// calculation is badly conditioned
				return;
			}

			const float HK32 = Kacc / _drag_innov_var(1);

			// Observation Jacobians
			Hfusion.at<0>() = -HK2*HK3;
			Hfusion.at<1>() = -HK3*HK4;
			Hfusion.at<2>() = -HK3*HK5;
			Hfusion.at<3>() = HK3*HK6;
			Hfusion.at<4>() = HK8;
			Hfusion.at<5>() = -HK10;
			Hfusion.at<6>() = -HK11*HK3;
			Hfusion.at<22>() = -HK8;
			Hfusion.at<23>() = HK10;

			// Kalman gains
			// Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
			// Kfusion(0) = -HK18*HK32;
			// Kfusion(1) = -HK30*HK32;
			// Kfusion(2) = -HK29*HK32;
			// Kfusion(3) = -HK31*HK32;
			// Kfusion(4) = -HK24*HK32;
			// Kfusion(5) = -HK26*HK32;
			// Kfusion(6) = -HK20*HK32;
			// Kfusion(7) = -HK32*(HK12*P(6,7) - HK13*P(4,7) + HK13*P(7,22) + HK14*P(2,7) + HK15*P(0,7) + HK16*P(1,7) - HK17*P(3,7) + HK9*P(5,7) - HK9*P(7,23));
			// Kfusion(8) = -HK32*(HK12*P(6,8) - HK13*P(4,8) + HK13*P(8,22) + HK14*P(2,8) + HK15*P(0,8) + HK16*P(1,8) - HK17*P(3,8) + HK9*P(5,8) - HK9*P(8,23));
			// Kfusion(9) = -HK32*(HK12*P(6,9) - HK13*P(4,9) + HK13*P(9,22) + HK14*P(2,9) + HK15*P(0,9) + HK16*P(1,9) - HK17*P(3,9) + HK9*P(5,9) - HK9*P(9,23));
			// Kfusion(10) = -HK32*(HK12*P(6,10) + HK13*P(10,22) - HK13*P(4,10) + HK14*P(2,10) + HK15*P(0,10) + HK16*P(1,10) - HK17*P(3,10) - HK9*P(10,23) + HK9*P(5,10));
			// Kfusion(11) = -HK32*(HK12*P(6,11) + HK13*P(11,22) - HK13*P(4,11) + HK14*P(2,11) + HK15*P(0,11) + HK16*P(1,11) - HK17*P(3,11) - HK9*P(11,23) + HK9*P(5,11));
			// Kfusion(12) = -HK32*(HK12*P(6,12) + HK13*P(12,22) - HK13*P(4,12) + HK14*P(2,12) + HK15*P(0,12) + HK16*P(1,12) - HK17*P(3,12) - HK9*P(12,23) + HK9*P(5,12));
			// Kfusion(13) = -HK32*(HK12*P(6,13) + HK13*P(13,22) - HK13*P(4,13) + HK14*P(2,13) + HK15*P(0,13) + HK16*P(1,13) - HK17*P(3,13) - HK9*P(13,23) + HK9*P(5,13));
			// Kfusion(14) = -HK32*(HK12*P(6,14) + HK13*P(14,22) - HK13*P(4,14) + HK14*P(2,14) + HK15*P(0,14) + HK16*P(1,14) - HK17*P(3,14) - HK9*P(14,23) + HK9*P(5,14));
			// Kfusion(15) = -HK32*(HK12*P(6,15) + HK13*P(15,22) - HK13*P(4,15) + HK14*P(2,15) + HK15*P(0,15) + HK16*P(1,15) - HK17*P(3,15) - HK9*P(15,23) + HK9*P(5,15));
			// Kfusion(16) = -HK32*(HK12*P(6,16) + HK13*P(16,22) - HK13*P(4,16) + HK14*P(2,16) + HK15*P(0,16) + HK16*P(1,16) - HK17*P(3,16) - HK9*P(16,23) + HK9*P(5,16));
			// Kfusion(17) = -HK32*(HK12*P(6,17) + HK13*P(17,22) - HK13*P(4,17) + HK14*P(2,17) + HK15*P(0,17) + HK16*P(1,17) - HK17*P(3,17) - HK9*P(17,23) + HK9*P(5,17));
			// Kfusion(18) = -HK32*(HK12*P(6,18) + HK13*P(18,22) - HK13*P(4,18) + HK14*P(2,18) + HK15*P(0,18) + HK16*P(1,18) - HK17*P(3,18) - HK9*P(18,23) + HK9*P(5,18));
			// Kfusion(19) = -HK32*(HK12*P(6,19) + HK13*P(19,22) - HK13*P(4,19) + HK14*P(2,19) + HK15*P(0,19) + HK16*P(1,19) - HK17*P(3,19) - HK9*P(19,23) + HK9*P(5,19));
			// Kfusion(20) = -HK32*(HK12*P(6,20) + HK13*P(20,22) - HK13*P(4,20) + HK14*P(2,20) + HK15*P(0,20) + HK16*P(1,20) - HK17*P(3,20) - HK9*P(20,23) + HK9*P(5,20));
			// Kfusion(21) = -HK32*(HK12*P(6,21) + HK13*P(21,22) - HK13*P(4,21) + HK14*P(2,21) + HK15*P(0,21) + HK16*P(1,21) - HK17*P(3,21) - HK9*P(21,23) + HK9*P(5,21));
			Kfusion(22) = -HK22*HK32;
			Kfusion(23) = -HK28*HK32;

		}

		// Apply an innovation consistency check with a 5 Sigma threshold
		_drag_innov(axis_index) = pred_acc - mea_acc;
		_drag_test_ratio(axis_index) = sq(_drag_innov(axis_index)) / (sq(5.0f) * _drag_innov_var(axis_index));

		// if the innovation consistency check fails then don't fuse the sample
		if (_drag_test_ratio(axis_index) <= 1.0f) {
			measurementUpdate(Kfusion, Hfusion, _drag_innov(axis_index));
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	_state.vel.setZero();
	_state.pos.setZero();
	_state.delta_ang_bias.setZero();
	_state.delta_vel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setIdentity();

	// TODO: who resets the output buffer content?
	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal.setIdentity();

	_delta_angle_corr.setZero();

	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_ang_rate_delayed_raw.zero();

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_prev_dvel_bias_var.zero();

	resetGpsDriftCheckFilters();
}

bool Ekf::update()
{
	bool updated = false;

	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		// perform state and covariance prediction for the main filter
		predictState();
		predictCovariance();

		// control fusion of observation data
		controlFusionModes();

		// run a separate filter for terrain estimation
		runTerrainEstimator();

		updated = true;

		// run EKF-GSF yaw estimator
		runYawEKFGSF();
	}

	// the output observer always runs
	// Use full rate IMU data at the current time horizon
	calculateOutputStates(_newest_high_rate_imu_sample);

	return updated;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	// protect against zero data
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	// Sum the magnetometer measurements
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		if (_mag_sample_delayed.time_us != 0) {
			if (_mag_counter == 0) {
				_mag_lpf.reset(_mag_sample_delayed.mag);

			} else {
				_mag_lpf.update(_mag_sample_delayed.mag);
			}

			_mag_counter++;
		}
	}

	// accumulate enough height measurements to be confident in the quality of the data
	if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		if (_baro_sample_delayed.time_us != 0) {
			if (_baro_counter == 0) {
				_baro_hgt_offset = _baro_sample_delayed.hgt;

			} else {
				_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
			}

			_baro_counter++;
		}
	}

	if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		if (_mag_counter < _obs_buffer_length) {
			// not enough mag samples accumulated
			return false;
		}
	}

	if (_baro_counter < _obs_buffer_length) {
		// not enough baro samples accumulated
		return false;
	}

	// we use baro height initially and switch to GPS/range/EV finder later when it passes checks.
	setControlBaroHeight();

	if (!initialiseTilt()) {
		return false;
	}

	// calculate the initial magnetic field and yaw alignment
	// but do not mark the yaw alignement complete as it needs to be
	// reset once the leveling phase is done
	resetMagHeading(_mag_lpf.getState(), false, false);

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// update the yaw angle variance using the variance of the measurement
	if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		// using magnetic heading tuning parameter
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}

	// try to initialise the terrain estimator
	_terrain_initialised = initHagl();

	// reset the essential fusion timeout counters
	_time_last_hgt_fuse = _time_last_imu;
	_time_last_hor_pos_fuse = _time_last_imu;
	_time_last_delpos_fuse = _time_last_imu;
	_time_last_hor_vel_fuse = _time_last_imu;
	_time_last_hagl_fuse = _time_last_imu;
	_time_last_flow_terrain_fuse = _time_last_imu;
	_time_last_of_fuse = _time_last_imu;

	// reset the output predictor state history to match the EKF initial values
	alignOutputFilter();

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
	const Vector3f gravity_in_body = _accel_lpf.getState().normalized();
	const float pitch = asinf(gravity_in_body(0));
	const float roll = atan2f(-gravity_in_body(1), -gravity_in_body(2));

	_state.quat_nominal = Quatf{Eulerf{roll, pitch, 0.0f}};
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState()
{
	// apply imu bias corrections
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.delta_ang_bias;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();

	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.delta_vel_bias;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - _imu_sample_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	input = math::constrain(input, 0.5f * FILTER_UPDATE_PERIOD_S, 2.0f * FILTER_UPDATE_PERIOD_S);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect angainst possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (_imu_sample_delayed.delta_ang_dt > 0.25f * FILTER_UPDATE_PERIOD_S) {
		_ang_rate_delayed_raw = _imu_sample_delayed.delta_ang / _imu_sample_delayed.delta_ang_dt;
	}

}

/*
 * Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
 * Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
 * Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
 * current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian:
 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void Ekf::calculateOutputStates(const imuSample &imu)
{
	// Use full rate IMU data at the current time horizon

	// correct delta angles for bias offsets
	const float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle(imu.delta_ang - _state.delta_ang_bias * dt_scale_correction + _delta_angle_corr);

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = delta_angle.dot(Vector3f(_R_to_earth_now.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu.delta_ang_dt;

	const Quatf dq(AxisAnglef{delta_angle});

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.time_us = imu.time_us;
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// the quaternions must always be normalised after modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// correct delta velocity for bias offsets
	const Vector3f delta_vel_body{imu.delta_vel - _state.delta_vel_bias * dt_scale_correction};

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_earth{_R_to_earth_now * delta_vel_body};

	// correct for measured acceleration due to gravity
	delta_vel_earth(2) += CONSTANTS_ONE_G * imu.delta_vel_dt;

	// calculate the earth frame velocity derivatives
	if (imu.delta_vel_dt > 1e-4f) {
		_vel_deriv = delta_vel_earth * (1.0f / imu.delta_vel_dt);
	}

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last(_output_new.vel);

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (imu.delta_vel_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += imu.delta_vel_dt;

	// correct velocity for IMU offset
	if (imu.delta_ang_dt > 1e-4f) {
		// calculate the average angular rate across the last IMU update
		const Vector3f ang_rate = imu.delta_ang * (1.0f / imu.delta_ang_dt);

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = ang_rate % _params.imu_pos_body;

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_output_vert_buffer.push(_output_vert_new);

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		// TODO: there is no guarantee that data is at delayed fusion horizon
		//       Shouldnt we use pop_first_older_than?
		const outputSample &output_delayed = _output_buffer.get_oldest();
		const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		const Quatf q_error((_state.quat_nominal.inversed() * output_delayed.quat_nominal).normalized());

		// convert the quaternion delta to a delta angle
		const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

		const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		const float time_delay = fmaxf((imu.time_us - _imu_sample_delayed.time_us) * 1e-6f, _dt_imu_avg);
		const float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		_delta_angle_corr = delta_ang_error * att_gain;
		_output_tracking_error(0) = delta_ang_error.norm();

		/*
		 * Loop through the output filter state history and apply the corrections to the velocity and position states.
		 * This method is too expensive to use for the attitude states due to the quaternion operations required
		 * but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
		 * to be used and reduces tracking error relative to EKF states.
		 */

		// Complementary filter gains
		const float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.0f);
		const float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.0f);

		// calculate down velocity and position tracking errors
		const float vert_vel_err = (_state.vel(2) - output_vert_delayed.vert_vel);
		const float vert_vel_integ_err = (_state.pos(2) - output_vert_delayed.vert_vel_integ);

		// calculate a velocity correction that will be applied to the output state history
		// using a PD feedback tuned to a 5% overshoot
		const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

		applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

		// calculate velocity and position tracking errors
		const Vector3f vel_err(_state.vel - output_delayed.vel);
		const Vector3f pos_err(_state.pos - output_delayed.pos);

		_output_tracking_error(1) = vel_err.norm();
		_output_tracking_error(2) = pos_err.norm();

		// calculate a velocity correction that will be applied to the output state history
		_vel_err_integ += vel_err;
		const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

		// calculate a position correction that will be applied to the output state history
		_pos_err_integ += pos_err;
		const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

		applyCorrectionToOutputBuffer(vel_correction, pos_correction);
	}
}

/*
* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
* down position state at the fusion time horizon using an alternative algorithm to what
* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
* This provides an alternative vertical velocity output that is closer to the first derivative
* of the position but does degrade tracking relative to the EKF state.
*/
void Ekf::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
	// vert_vel states and propagate vert_vel_integ forward using the corrected vert_vel
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// correct the velocity
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// position is propagated forward using the corrected velocity and a trapezoidal integrator
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f * next_state.dt;

		// advance the index
		index = (index + 1) % size;
	}

	// update output state to corrected values
	_output_vert_new = _output_vert_buffer.get_newest();

	// reset time delta to zero for the next accumulation of full rate IMU data
	_output_vert_new.dt = 0.0f;
}

/*
* Calculate corrections to be applied to vel and pos output state history.
* The vel and pos state history are corrected individually so they track the EKF states at
* the fusion time horizon. This option provides the most accurate tracking of EKF states.
*/
void Ekf::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// loop through the output filter state history and apply the corrections to the velocity and position states
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// a constant velocity correction is applied
		_output_buffer[index].vel += vel_correction;

		// a constant position correction is applied
		_output_buffer[index].pos += pos_correction;
	}

	// update output state to corrected values
	_output_new = _output_buffer.get_newest();
}

/*
 * Predict the previous quaternion output state forward using the latest IMU delta angle data.
*/
Quatf Ekf::calculate_quaternion() const
{
	// Correct delta angle data for bias errors using bias state estimates from the EKF and also apply
	// corrections required to track the EKF quaternion states
	const Vector3f delta_angle{_newest_high_rate_imu_sample.delta_ang - _state.delta_ang_bias * (_dt_imu_avg / _dt_ekf_avg) + _delta_angle_corr};

	// increment the quaternions using the corrected delta angle vector
	// the quaternions must always be normalised after modification
	return Quatf{_output_new.quat_nominal * AxisAnglef{delta_angle}}.unit();
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file ekf_helper.cpp
 * Definition of ekf helper functions.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::resetVelocity()
{
	if (_control_status.flags.gps && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
		// this reset is only called if we have new gps data at the fusion time horizon
		resetVelocityToGps();

	} else if (_control_status.flags.opt_flow) {
		resetHorizontalVelocityToOpticalFlow();

	} else if (_control_status.flags.ev_vel) {
		resetVelocityToVision();

	} else {
		resetHorizontalVelocityToZero();
	}
}

void Ekf::resetVelocityToGps()
{
	_information_events.flags.reset_vel_to_gps = true;
	ECL_INFO("reset velocity to GPS");
	resetVelocityTo(_gps_sample_delayed.vel);
	P.uncorrelateCovarianceSetVariance<3>(4, sq(_gps_sample_delayed.sacc));
}

void Ekf::resetHorizontalVelocityToOpticalFlow()
{
	_information_events.flags.reset_vel_to_flow = true;
	ECL_INFO("reset velocity to flow");
	// constrain height above ground to be above minimum possible
	const float heightAboveGndEst = fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance);

	// calculate absolute distance from focal point to centre of frame assuming a flat earth
	const float range = heightAboveGndEst / _range_sensor.getCosTilt();

	if ((range - _params.rng_gnd_clearance) > 0.3f) {
		// we should have reliable OF measurements so
		// calculate X and Y body relative velocities from OF measurements
		Vector3f vel_optflow_body;
		vel_optflow_body(0) = - range * _flow_compensated_XY_rad(1) / _flow_sample_delayed.dt;
		vel_optflow_body(1) =   range * _flow_compensated_XY_rad(0) / _flow_sample_delayed.dt;
		vel_optflow_body(2) = 0.0f;

		// rotate from body to earth frame
		const Vector3f vel_optflow_earth = _R_to_earth * vel_optflow_body;

		resetHorizontalVelocityTo(Vector2f(vel_optflow_earth));

	} else {
		resetHorizontalVelocityTo(Vector2f{0.f, 0.f});
	}

	// reset the horizontal velocity variance using the optical flow noise variance
	P.uncorrelateCovarianceSetVariance<2>(4, sq(range) * calcOptFlowMeasVar());
}

void Ekf::resetVelocityToVision()
{
	_information_events.flags.reset_vel_to_vision = true;
	ECL_INFO("reset to vision velocity");
	resetVelocityTo(getVisionVelocityInEkfFrame());
	P.uncorrelateCovarianceSetVariance<3>(4, getVisionVelocityVarianceInEkfFrame());
}

void Ekf::resetHorizontalVelocityToZero()
{
	_information_events.flags.reset_vel_to_zero = true;
	ECL_INFO("reset velocity to zero");
	// Used when falling back to non-aiding mode of operation
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f});
	P.uncorrelateCovarianceSetVariance<2>(4, 25.0f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel)
{
	resetHorizontalVelocityTo(Vector2f(new_vel));
	resetVerticalVelocityTo(new_vel(2));
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel.xy() += delta_horz_vel;
	}

	_output_new.vel.xy() += delta_horz_vel;

	_state_reset_status.velNE_change = delta_horz_vel;
	_state_reset_status.velNE_counter++;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel(2) += delta_vert_vel;
		_output_vert_buffer[index].vert_vel += delta_vert_vel;
	}

	_output_new.vel(2) += delta_vert_vel;
	_output_vert_new.vert_vel += delta_vert_vel;

	_state_reset_status.velD_change = delta_vert_vel;
	_state_reset_status.velD_counter++;
}

void Ekf::resetHorizontalPosition()
{
	// let the next odometry update know that the previous value of states cannot be used to calculate the change in position
	_hpos_prev_available = false;

	if (_control_status.flags.gps) {
		// this reset is only called if we have new gps data at the fusion time horizon
		resetHorizontalPositionToGps();

	} else if (_control_status.flags.ev_pos) {
		// this reset is only called if we have new ev data at the fusion time horizon
		resetHorizontalPositionToVision();

	} else if (_control_status.flags.opt_flow) {
		_information_events.flags.reset_pos_to_last_known = true;
		ECL_INFO("reset position to last known position");

		if (!_control_status.flags.in_air) {
			// we are likely starting OF for the first time so reset the horizontal position
			resetHorizontalPositionTo(Vector2f(0.f, 0.f));

		} else {
			resetHorizontalPositionTo(_last_known_posNE);
		}

		// estimate is relative to initial position in this mode, so we start with zero error.
		P.uncorrelateCovarianceSetVariance<2>(7, 0.0f);

	} else {
		_information_events.flags.reset_pos_to_last_known = true;
		ECL_INFO("reset position to last known position");
		// Used when falling back to non-aiding mode of operation
		resetHorizontalPositionTo(_last_known_posNE);
		P.uncorrelateCovarianceSetVariance<2>(7, sq(_params.pos_noaid_noise));
	}
}

void Ekf::resetHorizontalPositionToGps()
{
	_information_events.flags.reset_pos_to_gps = true;
	ECL_INFO("reset position to GPS");
	resetHorizontalPositionTo(_gps_sample_delayed.pos);
	P.uncorrelateCovarianceSetVariance<2>(7, sq(_gps_sample_delayed.hacc));
}

void Ekf::resetHorizontalPositionToVision()
{
	_information_events.flags.reset_pos_to_vision = true;
	ECL_INFO("reset position to ev position");
	Vector3f _ev_pos = _ev_sample_delayed.pos;

	if (_params.fusion_mode & MASK_ROTATE_EV) {
		_ev_pos = _R_ev_to_ekf * _ev_sample_delayed.pos;
	}

	resetHorizontalPositionTo(Vector2f(_ev_pos));
	P.uncorrelateCovarianceSetVariance<2>(7, _ev_sample_delayed.posVar.slice<2, 1>(0, 0));
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_horz_pos)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos.xy() += delta_horz_pos;
	}

	_output_new.pos.xy() += delta_horz_pos;

	_state_reset_status.posNE_change = delta_horz_pos;
	_state_reset_status.posNE_counter++;
}

void Ekf::resetVerticalPositionTo(const float &new_vert_pos)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	// store the reset amount and time to be published
	_state_reset_status.posD_change = new_vert_pos - old_vert_pos;
	_state_reset_status.posD_counter++;

	// apply the change in height / height rate to our newest height / height rate estimate
	// which have already been taken out from the output buffer
	_output_new.pos(2) += _state_reset_status.posD_change;

	// add the reset amount to the output observer buffered data
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos(2) += _state_reset_status.posD_change;
		_output_vert_buffer[i].vert_vel_integ += _state_reset_status.posD_change;
	}

	// add the reset amount to the output observer vertical position state
	_output_vert_new.vert_vel_integ = _state.pos(2);
}

// Reset height state using the last height measurement
void Ekf::resetHeight()
{
	// Get the most recent GPS data
	const gpsSample &gps_newest = _gps_buffer.get_newest();

	// reset the vertical position
	if (_control_status.flags.rng_hgt) {

		// a fallback from any other height source to rangefinder happened
		if (!_control_status_prev.flags.rng_hgt) {

			if (_control_status.flags.in_air && isTerrainEstimateValid()) {
				_hgt_sensor_offset = _terrain_vpos;

			} else if (_control_status.flags.in_air) {
				_hgt_sensor_offset = _range_sensor.getDistBottom() + _state.pos(2);

			} else {
				_hgt_sensor_offset = _params.rng_gnd_clearance;
			}

		}

		// update the state and associated variance
		resetVerticalPositionTo(_hgt_sensor_offset - _range_sensor.getDistBottom());

		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.range_noise));

		// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
		const baroSample &baro_newest = _baro_buffer.get_newest();
		_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

	} else if (_control_status.flags.baro_hgt) {
		// initialize vertical position with newest baro measurement
		const baroSample &baro_newest = _baro_buffer.get_newest();

		if (!_baro_hgt_faulty) {
			resetVerticalPositionTo(-baro_newest.hgt + _baro_hgt_offset);

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.baro_noise));

		} else {
			// TODO: reset to last known baro based estimate
		}

	} else if (_control_status.flags.gps_hgt) {
		// initialize vertical position and velocity with newest gps measurement
		if (!_gps_hgt_intermittent) {
			resetVerticalPositionTo(_hgt_sensor_offset - gps_newest.hgt + _gps_alt_ref);

			// the state variance is the same as the observation
			P.uncorrelateCovarianceSetVariance<1>(9, sq(gps_newest.vacc));

			// reset the baro offset which is subtracted from the baro reading if we need to use it as a backup
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: reset to last known gps based estimate
		}

	} else if (_control_status.flags.ev_hgt) {
		// initialize vertical position with newest measurement
		const extVisionSample &ev_newest = _ext_vision_buffer.get_newest();

		// use the most recent data if it's time offset from the fusion time horizon is smaller
		if (ev_newest.time_us >= _ev_sample_delayed.time_us) {
			resetVerticalPositionTo(ev_newest.pos(2));

		} else {
			resetVerticalPositionTo(_ev_sample_delayed.pos(2));
		}
	}

	// reset the vertical velocity state
	if (_control_status.flags.gps && !_gps_hgt_intermittent) {
		// If we are using GPS, then use it to reset the vertical velocity
		resetVerticalVelocityTo(gps_newest.vel(2));

		// the state variance is the same as the observation
		P.uncorrelateCovarianceSetVariance<1>(6, sq(1.5f * gps_newest.sacc));

	} else {
		// we don't know what the vertical velocity is, so set it to zero
		resetVerticalVelocityTo(0.0f);

		// Set the variance to a value large enough to allow the state to converge quickly
		// that does not destabilise the filter
		P.uncorrelateCovarianceSetVariance<1>(6, 10.0f);
	}
}

// align output filter states to match EKF states at the fusion time horizon
void Ekf::alignOutputFilter()
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// calculate the quaternion rotation delta from the EKF to output observer states at the EKF fusion time horizon
	Quatf q_delta{_state.quat_nominal * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// calculate the velocity and position deltas between the output and EKF at the EKF fusion time horizon
	const Vector3f vel_delta = _state.vel - output_delayed.vel;
	const Vector3f pos_delta = _state.pos - output_delayed.pos;

	// loop through the output filter state history and add the deltas
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

// Do a forced re-alignment of the yaw angle to align with the horizontal velocity vector from the GPS.
// It is used to align the yaw angle after launch or takeoff for fixed wing vehicle only.
bool Ekf::realignYawGPS()
{
	const float gpsSpeed = sqrtf(sq(_gps_sample_delayed.vel(0)) + sq(_gps_sample_delayed.vel(1)));

	// Need at least 5 m/s of GPS horizontal speed and
	// ratio of velocity error to velocity < 0.15  for a reliable alignment
	const bool gps_yaw_alignment_possible = (gpsSpeed > 5.0f) && (_gps_sample_delayed.sacc < (0.15f * gpsSpeed));

	if (!gps_yaw_alignment_possible) {
		// attempt a normal alignment using the magnetometer
		return resetMagHeading(_mag_lpf.getState());
	}

	// check for excessive horizontal GPS velocity innovations
	const bool badVelInnov = (_gps_vel_test_ratio(0) > 1.0f) && _control_status.flags.gps;

	// calculate GPS course over ground angle
	const float gpsCOG = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

	// calculate course yaw angle
	const float ekfCOG = atan2f(_state.vel(1), _state.vel(0));

	// Check the EKF and GPS course over ground for consistency
	const float courseYawError = wrap_pi(gpsCOG - ekfCOG);

	// If the angles disagree and horizontal GPS velocity innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
	const bool badYawErr = fabsf(courseYawError) > 0.5f;
	const bool badMagYaw = (badYawErr && badVelInnov);

	if (badMagYaw) {
		_num_bad_flight_yaw_events ++;
	}

	// correct yaw angle using GPS ground course if compass yaw bad or yaw is previously not aligned
	if (badMagYaw || !_control_status.flags.yaw_align) {
		_warning_events.flags.bad_yaw_using_gps_course = true;
		ECL_WARN("bad yaw, using GPS course");

		// declare the magnetometer as failed if a bad yaw has occurred more than once
		if (_control_status.flags.mag_aligned_in_flight && (_num_bad_flight_yaw_events >= 2)
		    && !_control_status.flags.mag_fault) {
			_warning_events.flags.stopping_mag_use = true;
			ECL_WARN("stopping mag use");
			_control_status.flags.mag_fault = true;
		}

		// calculate new yaw estimate
		float yaw_new;

		if (!_control_status.flags.mag_aligned_in_flight) {
			// This is our first flight alignment so we can assume that the recent change in velocity has occurred due to a
			// forward direction takeoff or launch and therefore the inertial and GPS ground course discrepancy is due to yaw error
			const float current_yaw = getEuler321Yaw(_state.quat_nominal);
			yaw_new = current_yaw + courseYawError;
			_control_status.flags.mag_aligned_in_flight = true;

		} else if (_control_status.flags.wind) {
			// we have previously aligned yaw in-flight and have wind estimates so set the yaw such that the vehicle nose is
			// aligned with the wind relative GPS velocity vector
			yaw_new = atan2f((_gps_sample_delayed.vel(1) - _state.wind_vel(1)),
					 (_gps_sample_delayed.vel(0) - _state.wind_vel(0)));

		} else {
			// we don't have wind estimates, so align yaw to the GPS velocity vector
			yaw_new = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

		}

		// use the combined EKF and GPS speed variance to calculate a rough estimate of the yaw error after alignment
		const float SpdErrorVariance = sq(_gps_sample_delayed.sacc) + P(4, 4) + P(5, 5);
		const float sineYawError = math::constrain(sqrtf(SpdErrorVariance) / gpsSpeed, 0.0f, 1.0f);
		const float yaw_variance_new = sq(asinf(sineYawError));

		// Apply updated yaw and yaw variance to states and covariances
		resetQuatStateYaw(yaw_new, yaw_variance_new, true);

		// Use the last magnetometer measurements to reset the field states
		_state.mag_B.zero();
		_R_to_earth = Dcmf(_state.quat_nominal);
		_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

		resetMagCov();

		// record the start time for the magnetic field alignment
		_flt_mag_align_start_time = _imu_sample_delayed.time_us;

		// If heading was bad, then we also need to reset the velocity and position states
		_velpos_reset_request = badMagYaw;

		return true;

	} else {
		// align mag states only

		// calculate initial earth magnetic field states
		_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

		resetMagCov();

		// record the start time for the magnetic field alignment
		_flt_mag_align_start_time = _imu_sample_delayed.time_us;

		return true;
	}
}

// Reset heading and magnetic field states
bool Ekf::resetMagHeading(const Vector3f &mag_init, bool increase_yaw_var, bool update_buffer)
{
	// prevent a reset being performed more than once on the same frame
	if (_imu_sample_delayed.time_us == _flt_mag_align_start_time) {
		return true;
	}

	// calculate the observed yaw angle and yaw variance
	float yaw_new;
	float yaw_new_variance = 0.0f;

	if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		// rotate the magnetometer measurements into earth frame using a zero yaw angle
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		// the angle of the projection onto the horizontal gives the yaw angle
		const Vector3f mag_earth_pred = R_to_earth * mag_init;
		yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		if (increase_yaw_var) {
			yaw_new_variance = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));
		}

	} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR) {
		// we are operating temporarily without knowing the earth frame yaw angle
		return true;

	} else {
		// there is no magnetic yaw observation
		return false;
	}

	// update quaternion states and corresponding covarainces
	resetQuatStateYaw(yaw_new, yaw_new_variance, update_buffer);

	// set the earth magnetic field states using the updated rotation
	_state.mag_I = _R_to_earth * mag_init;

	resetMagCov();

	// record the time for the magnetic field alignment event
	_flt_mag_align_start_time = _imu_sample_delayed.time_us;

	return true;
}

bool Ekf::resetYawToEv()
{
	const float yaw_new = getEuler312Yaw(_ev_sample_delayed.quat);
	const float yaw_new_variance = fmaxf(_ev_sample_delayed.angVar, sq(1.0e-2f));

	resetQuatStateYaw(yaw_new, yaw_new_variance, true);
	_R_ev_to_ekf.setIdentity();

	return true;
}

// Return the magnetic declination in radians to be used by the alignment and fusion processing
float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_control_status.flags.mag_aligned_in_flight) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			return _mag_declination_gps;

		} else {
			return math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		return math::radians(_params.mag_declination_deg);
	}
}

void Ekf::constrainStates()
{
	_state.quat_nominal = matrix::constrain(_state.quat_nominal, -1.0f, 1.0f);
	_state.vel = matrix::constrain(_state.vel, -1000.0f, 1000.0f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	const float delta_ang_bias_limit = math::radians(20.f) * _dt_ekf_avg;
	_state.delta_ang_bias = matrix::constrain(_state.delta_ang_bias, -delta_ang_bias_limit, delta_ang_bias_limit);

	const float delta_vel_bias_limit = _params.acc_bias_lim * _dt_ekf_avg;
	_state.delta_vel_bias = matrix::constrain(_state.delta_vel_bias, -delta_vel_bias_limit, delta_vel_bias_limit);

	_state.mag_I = matrix::constrain(_state.mag_I, -1.0f, 1.0f);
	_state.mag_B = matrix::constrain(_state.mag_B, -0.5f, 0.5f);
	_state.wind_vel = matrix::constrain(_state.wind_vel, -100.0f, 100.0f);
}

float Ekf::compensateBaroForDynamicPressure(const float baro_alt_uncompensated) const
{
	// calculate static pressure error = Pmeas - Ptruth
	// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
	// negative X and Y directions. Used to correct baro data for positional errors
	const matrix::Dcmf R_to_body(_output_new.quat_nominal.inversed());

	// Calculate airspeed in body frame
	const Vector3f velocity_earth = _output_new.vel - _vel_imu_rel_body_ned;

	const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

	const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

	const Vector3f airspeed_body = R_to_body * airspeed_earth;

	const Vector3f K_pstatic_coef(airspeed_body(0) >= 0.0f ? _params.static_pressure_coef_xp :
				      _params.static_pressure_coef_xn,
				      airspeed_body(1) >= 0.0f ? _params.static_pressure_coef_yp : _params.static_pressure_coef_yn,
				      _params.static_pressure_coef_z);

	const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.max_correction_airspeed));

	const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

	// correct baro measurement using pressure error estimate and assuming sea level gravity
	return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
}

// calculate the earth rotation vector
Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2],  float &vpos) const
{
	hvel[0] = _gps_vel_innov(0);
	hvel[1] = _gps_vel_innov(1);
	vvel    = _gps_vel_innov(2);
	hpos[0] = _gps_pos_innov(0);
	hpos[1] = _gps_pos_innov(1);
	vpos    = _gps_pos_innov(2);
}

void Ekf::getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos)  const
{
	hvel[0] = _gps_vel_innov_var(0);
	hvel[1] = _gps_vel_innov_var(1);
	vvel    = _gps_vel_innov_var(2);
	hpos[0] = _gps_pos_innov_var(0);
	hpos[1] = _gps_pos_innov_var(1);
	vpos    = _gps_pos_innov_var(2);
}

void Ekf::getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = _gps_vel_test_ratio(0);
	vvel = _gps_vel_test_ratio(1);
	hpos = _gps_pos_test_ratio(0);
	vpos = _gps_pos_test_ratio(1);
}

void Ekf::getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _ev_vel_innov(0);
	hvel[1] = _ev_vel_innov(1);
	vvel    = _ev_vel_innov(2);
	hpos[0] = _ev_pos_innov(0);
	hpos[1] = _ev_pos_innov(1);
	vpos    = _ev_pos_innov(2);
}

void Ekf::getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _ev_vel_innov_var(0);
	hvel[1] = _ev_vel_innov_var(1);
	vvel    = _ev_vel_innov_var(2);
	hpos[0] = _ev_pos_innov_var(0);
	hpos[1] = _ev_pos_innov_var(1);
	vpos    = _ev_pos_innov_var(2);
}

void Ekf::getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = _ev_vel_test_ratio(0);
	vvel = _ev_vel_test_ratio(1);
	hpos = _ev_pos_test_ratio(0);
	vpos = _ev_pos_test_ratio(1);
}

void Ekf::getAuxVelInnov(float aux_vel_innov[2]) const
{
	aux_vel_innov[0] = _aux_vel_innov(0);
	aux_vel_innov[1] = _aux_vel_innov(1);
}

void Ekf::getAuxVelInnovVar(float aux_vel_innov_var[2]) const
{
	aux_vel_innov_var[0] = _aux_vel_innov_var(0);
	aux_vel_innov_var[1] = _aux_vel_innov_var(1);
}

// get the state vector at the delayed time horizon
matrix::Vector<float, 24> Ekf::getStateAtFusionHorizonAsVector() const
{
	matrix::Vector<float, 24> state;
	state.slice<4, 1>(0, 0) = _state.quat_nominal;
	state.slice<3, 1>(4, 0) = _state.vel;
	state.slice<3, 1>(7, 0) = _state.pos;
	state.slice<3, 1>(10, 0) = _state.delta_ang_bias;
	state.slice<3, 1>(13, 0) = _state.delta_vel_bias;
	state.slice<3, 1>(16, 0) = _state.mag_I;
	state.slice<3, 1>(19, 0) = _state.mag_B;
	state.slice<2, 1>(22, 0) = _state.wind_vel;
	return state;
}

bool Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _last_gps_origin_time_us;
	latitude = _pos_ref.getProjectionReferenceLat();
	longitude = _pos_ref.getProjectionReferenceLon();
	origin_alt  = _gps_alt_ref;
	return _NED_origin_initialised;
}

void Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude)
{
	bool current_pos_available = false;
	double current_lat = static_cast<double>(NAN);
	double current_lon = static_cast<double>(NAN);
	float current_alt  = 0.f;

	// if we are already doing aiding, correct for the change in position since the EKF started navigating
	if (_pos_ref.isInitialized() && isHorizontalAidingActive()) {
		_pos_ref.reproject(_state.pos(0), _state.pos(1), current_lat, current_lon);
		current_alt = -_state.pos(2) + _gps_alt_ref;
		current_pos_available = true;
	}

	// reinitialize map projection to latitude, longitude, altitude, and reset position
	_pos_ref.initReference(latitude, longitude, _time_last_imu);
	if (current_pos_available) {
		// reset horizontal position
		Vector2f position = _pos_ref.project(current_lat, current_lon);
		resetHorizontalPositionTo(position);

		// reset altitude
		_gps_alt_ref = altitude;
		resetVerticalPositionTo(_gps_alt_ref - current_alt);

	} else {
		// reset altitude
		_gps_alt_ref = altitude;
	}
}

/*
	First argument returns GPS drift  metrics in the following array locations
	0 : Horizontal position drift rate (m/s)
	1 : Vertical position drift rate (m/s)
	2 : Filtered horizontal velocity (m/s)
	Second argument returns true when IMU movement is blocking the drift calculation
	Function returns true if the metrics have been updated and not returned previously by this function
*/
bool Ekf::get_gps_drift_metrics(float drift[3], bool *blocked)
{
	memcpy(drift, _gps_drift_metrics, 3 * sizeof(float));
	*blocked = !_control_status.flags.vehicle_at_rest;

	if (_gps_drift_updated) {
		_gps_drift_updated = false;
		return true;
	}

	return false;
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// report absolute accuracy taking into account the uncertainty in location of the origin
	// If not aiding, return 0 for horizontal position estimate as no estimate is available
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P(7, 7) + P(8, 8) + sq(_gps_origin_eph));

	// If we are dead-reckoning, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_is_dead_reckoning && (_control_status.flags.gps)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));

	} else if (_is_dead_reckoning && (_control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_ev_pos_innov(0)) + sq(_ev_pos_innov(1))));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9, 9) + sq(_gps_origin_epv));
}

// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - allow for baro drift in vertical position error
	float hpos_err = sqrtf(P(7, 7) + P(8, 8));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal position error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_deadreckon_time_exceeded && _control_status.flags.gps) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9, 9));
}

// get the 1-sigma horizontal and vertical velocity uncertainty
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	float hvel_err = sqrtf(P(4, 4) + P(5, 5));

	// If we are dead-reckoning for too long, use the innovations as a conservative alternate measure of the horizontal velocity error
	// The reason is that complete rejection of measurements is often caused by heading misalignment or inertial sensing errors
	// and using state variances for accuracy reporting is overly optimistic in these situations
	if (_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * _flow_innov.norm();
		}

		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));

		} else if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_ev_pos_innov(0)) + sq(_ev_pos_innov(1))));
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_ev_vel_innov(0)) + sq(_ev_vel_innov(1))));
		}

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(6, 6));
}

/*
Returns the following vehicle control limits required by the estimator to keep within sensor limitations.
vxy_max : Maximum ground relative horizontal speed (meters/sec). NaN when limiting is not needed.
vz_max : Maximum ground relative vertical speed (meters/sec). NaN when limiting is not needed.
hagl_min : Minimum height above ground (meters). NaN when limiting is not needed.
hagl_max : Maximum height above ground (meters). NaN when limiting is not needed.
*/
void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const
{
	// Calculate range finder limits
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();
	// Allow use of 75% of rangefinder maximum range to allow for angular motion
	const float rangefinder_hagl_max = 0.75f * _range_sensor.getValidMaxVal();

	// Calculate optical flow limits
	// Allow ground relative velocity to use 50% of available flow sensor range to allow for angular motion
	const float flow_vxy_max = fmaxf(0.5f * _flow_max_rate * (_terrain_vpos - _state.pos(2)), 0.0f);
	const float flow_hagl_min = _flow_min_distance;
	const float flow_hagl_max = _flow_max_distance;

	// TODO : calculate visual odometry limits

	const bool relying_on_rangefinder = _control_status.flags.rng_hgt && !_params.range_aid;

	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	// Do not require limiting by default
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max = NAN;

	// Keep within range sensor limit when using rangefinder as primary height source
	if (relying_on_rangefinder) {
		*vxy_max = NAN;
		*vz_max = NAN;
		*hagl_min = rangefinder_hagl_min;
		*hagl_max = rangefinder_hagl_max;
	}

	// Keep within flow AND range sensor limits when exclusively using optical flow
	if (relying_on_optical_flow) {
		*vxy_max = flow_vxy_max;
		*vz_max = NAN;
		*hagl_min = fmaxf(rangefinder_hagl_min, flow_hagl_min);
		*hagl_max = fminf(rangefinder_hagl_max, flow_hagl_max);
	}
}

void Ekf::resetImuBias()
{
	resetGyroBias();
	resetAccelBias();
}

void Ekf::resetGyroBias()
{
	// Zero the delta angle and delta velocity bias states
	_state.delta_ang_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<3>(10, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
}

void Ekf::resetAccelBias()
{
	// Zero the delta angle and delta velocity bias states
	_state.delta_vel_bias.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<3>(13, sq(_params.switch_on_accel_bias * FILTER_UPDATE_PERIOD_S));

	// Set previous frame values
	_prev_dvel_bias_var = P.slice<3, 3>(13, 13).diag();
}

void Ekf::resetMagBias()
{
	// Zero the magnetometer bias states
	_state.mag_B.zero();

	// Zero the corresponding covariances and set
	// variances to the values use for initial alignment
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

	// reset any saved covariance data for re-use when auto-switching between heading and 3-axis fusion
	// _saved_mag_bf_variance[0] is the the D earth axis
	_saved_mag_bf_variance[1] = 0;
	_saved_mag_bf_variance[2] = 0;
	_saved_mag_bf_variance[3] = 0;
}

// get EKF innovation consistency check status information comprising of:
// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
void Ekf::get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
				     float &hagl, float &beta) const
{
	// return the integer bitmask containing the consistency check pass/fail status
	status = _innov_check_fail_status.value;

	// return the largest magnetometer innovation test ratio
	mag = sqrtf(math::max(_yaw_test_ratio, _mag_test_ratio.max()));

	// return the largest velocity and position innovation test ratio
	vel = NAN;
	pos = NAN;

	if (_control_status.flags.gps) {
		float gps_vel = sqrtf(math::max(_gps_vel_test_ratio(0), _gps_vel_test_ratio(1)));
		vel = math::max(gps_vel, FLT_MIN);

		float gps_pos = sqrtf(_gps_pos_test_ratio(0));
		pos = math::max(gps_pos, FLT_MIN);
	}

	if (_control_status.flags.ev_vel) {
		float ev_vel = sqrtf(math::max(_ev_vel_test_ratio(0), _ev_vel_test_ratio(1)));
		vel = math::max(math::max(vel, ev_vel), FLT_MIN);
	}

	if (_control_status.flags.ev_pos) {
		float ev_pos = sqrtf(_ev_pos_test_ratio(0));
		pos = math::max(math::max(pos, ev_pos), FLT_MIN);
	}

	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		float of_vel = sqrtf(_optflow_test_ratio);
		vel = math::max(of_vel, FLT_MIN);
	}

	// return the vertical position innovation test ratio
	if (_control_status.flags.baro_hgt) {
		hgt = math::max(sqrtf(_baro_hgt_test_ratio(1)), FLT_MIN);

	} else if (_control_status.flags.gps_hgt) {
		hgt = math::max(sqrtf(_gps_pos_test_ratio(1)), FLT_MIN);

	} else if (_control_status.flags.rng_hgt) {
		hgt = math::max(sqrtf(_rng_hgt_test_ratio(1)), FLT_MIN);

	} else if (_control_status.flags.ev_hgt) {
		hgt = math::max(sqrtf(_ev_pos_test_ratio(1)), FLT_MIN);

	} else {
		hgt = NAN;
	}

	// return the airspeed fusion innovation test ratio
	tas = sqrtf(_tas_test_ratio);

	// return the terrain height innovation test ratio
	hagl = sqrtf(_hagl_test_ratio);

	// return the synthetic sideslip innovation test ratio
	beta = sqrtf(_beta_test_ratio);
}

// return a bitmask integer that describes which state estimates are valid
void Ekf::get_ekf_soln_status(uint16_t *status) const
{
	ekf_solution_status soln_status;
	// TODO: Is this accurate enough?
	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (isHorizontalAidingActive() || (_control_status.flags.fuse_beta && _control_status.flags.fuse_aspd)) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_horiz_abs;
	const bool gps_vel_innov_bad = (_gps_vel_test_ratio(0) > 1.0f) || (_gps_vel_test_ratio(1) > 1.0f);
	const bool gps_pos_innov_bad = (_gps_pos_test_ratio(0) > 1.0f);
	const bool mag_innov_good = (_mag_test_ratio.max() < 1.0f) && (_yaw_test_ratio < 1.0f);
	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad) && mag_innov_good;
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical;
	*status = soln_status.value;
}

void Ekf::fuse(const Vector24f &K, float innovation)
{
	_state.quat_nominal -= K.slice<4, 1>(0, 0) * innovation;
	_state.quat_nominal.normalize();
	_state.vel -= K.slice<3, 1>(4, 0) * innovation;
	_state.pos -= K.slice<3, 1>(7, 0) * innovation;
	_state.delta_ang_bias -= K.slice<3, 1>(10, 0) * innovation;
	_state.delta_vel_bias -= K.slice<3, 1>(13, 0) * innovation;
	_state.mag_I -= K.slice<3, 1>(16, 0) * innovation;
	_state.mag_B -= K.slice<3, 1>(19, 0) * innovation;
	_state.wind_vel -= K.slice<2, 1>(22, 0) * innovation;
}

void Ekf::uncorrelateQuatFromOtherStates()
{
	P.slice<_k_num_states - 4, 4>(4, 0) = 0.f;
	P.slice<4, _k_num_states - 4>(0, 4) = 0.f;
}

// return true if we are totally reliant on inertial dead-reckoning for position
void Ekf::update_deadreckoning_status()
{
	const bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.ev_vel)
				  && (isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
				      || isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
				      || isRecent(_time_last_delpos_fuse, _params.no_aid_timeout_max));
	const bool optFlowAiding = _control_status.flags.opt_flow && isRecent(_time_last_of_fuse, _params.no_aid_timeout_max);
	const bool airDataAiding = _control_status.flags.wind &&
				   isRecent(_time_last_arsp_fuse, _params.no_aid_timeout_max) &&
				   isRecent(_time_last_beta_fuse, _params.no_aid_timeout_max);

	_is_wind_dead_reckoning = !velPosAiding && !optFlowAiding && airDataAiding;
	_is_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding;

	if (!_is_dead_reckoning) {
		_time_last_aiding = _time_last_imu - _params.no_aid_timeout_max;
	}

	// report if we have been deadreckoning for too long, initial state is deadreckoning until aiding is present
	_deadreckon_time_exceeded = (_time_last_aiding == 0)
				    || isTimedOut(_time_last_aiding, (uint64_t)_params.valid_timeout_max);
}

// calculate the variances for the rotation vector equivalent
Vector3f Ekf::calcRotVecVariances()
{
	Vector3f rot_var_vec;
	float q0, q1, q2, q3;

	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float t2 = q0*q0;
	float t3 = acosf(q0);
	float t4 = -t2+1.0f;
	float t5 = t2-1.0f;
	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
		float t6 = 1.0f/t5;
		float t7 = q1*t6*2.0f;
		float t8 = 1.0f/powf(t4,1.5f);
		float t9 = q0*q1*t3*t8*2.0f;
		float t10 = t7+t9;
		float t11 = 1.0f/sqrtf(t4);
		float t12 = q2*t6*2.0f;
		float t13 = q0*q2*t3*t8*2.0f;
		float t14 = t12+t13;
		float t15 = q3*t6*2.0f;
		float t16 = q0*q3*t3*t8*2.0f;
		float t17 = t15+t16;
		rot_var_vec(0) = t10*(P(0,0)*t10+P(1,0)*t3*t11*2.0f)+t3*t11*(P(0,1)*t10+P(1,1)*t3*t11*2.0f)*2.0f;
		rot_var_vec(1) = t14*(P(0,0)*t14+P(2,0)*t3*t11*2.0f)+t3*t11*(P(0,2)*t14+P(2,2)*t3*t11*2.0f)*2.0f;
		rot_var_vec(2) = t17*(P(0,0)*t17+P(3,0)*t3*t11*2.0f)+t3*t11*(P(0,3)*t17+P(3,3)*t3*t11*2.0f)*2.0f;
	} else {
		rot_var_vec = 4.0f * P.slice<3,3>(1,1).diag();
	}

	return rot_var_vec;
}

// initialise the quaternion covariances using rotation vector variances
// do not call before quaternion states are initialised
void Ekf::initialiseQuatCovariances(Vector3f &rot_vec_var)
{
	// calculate an equivalent rotation vector from the quaternion
	float q0,q1,q2,q3;
	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float delta = 2.0f*acosf(q0);
	float scaler = (delta/sinf(delta*0.5f));
	float rotX = scaler*q1;
	float rotY = scaler*q2;
	float rotZ = scaler*q3;

	// autocode generated using matlab symbolic toolbox
	float t2 = rotX*rotX;
	float t4 = rotY*rotY;
	float t5 = rotZ*rotZ;
	float t6 = t2+t4+t5;
	if (t6 > 1e-9f) {
		float t7 = sqrtf(t6);
		float t8 = t7*0.5f;
		float t3 = sinf(t8);
		float t9 = t3*t3;
		float t10 = 1.0f/t6;
		float t11 = 1.0f/sqrtf(t6);
		float t12 = cosf(t8);
		float t13 = 1.0f/powf(t6,1.5f);
		float t14 = t3*t11;
		float t15 = rotX*rotY*t3*t13;
		float t16 = rotX*rotZ*t3*t13;
		float t17 = rotY*rotZ*t3*t13;
		float t18 = t2*t10*t12*0.5f;
		float t27 = t2*t3*t13;
		float t19 = t14+t18-t27;
		float t23 = rotX*rotY*t10*t12*0.5f;
		float t28 = t15-t23;
		float t20 = rotY*rot_vec_var(1)*t3*t11*t28*0.5f;
		float t25 = rotX*rotZ*t10*t12*0.5f;
		float t31 = t16-t25;
		float t21 = rotZ*rot_vec_var(2)*t3*t11*t31*0.5f;
		float t22 = t20+t21-rotX*rot_vec_var(0)*t3*t11*t19*0.5f;
		float t24 = t15-t23;
		float t26 = t16-t25;
		float t29 = t4*t10*t12*0.5f;
		float t34 = t3*t4*t13;
		float t30 = t14+t29-t34;
		float t32 = t5*t10*t12*0.5f;
		float t40 = t3*t5*t13;
		float t33 = t14+t32-t40;
		float t36 = rotY*rotZ*t10*t12*0.5f;
		float t39 = t17-t36;
		float t35 = rotZ*rot_vec_var(2)*t3*t11*t39*0.5f;
		float t37 = t15-t23;
		float t38 = t17-t36;
		float t41 = rot_vec_var(0)*(t15-t23)*(t16-t25);
		float t42 = t41-rot_vec_var(1)*t30*t39-rot_vec_var(2)*t33*t39;
		float t43 = t16-t25;
		float t44 = t17-t36;

		// zero all the quaternion covariances
		P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);


		// Update the quaternion internal covariances using auto-code generated using matlab symbolic toolbox
		P(0,0) = rot_vec_var(0)*t2*t9*t10*0.25f+rot_vec_var(1)*t4*t9*t10*0.25f+rot_vec_var(2)*t5*t9*t10*0.25f;
		P(0,1) = t22;
		P(0,2) = t35+rotX*rot_vec_var(0)*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rot_vec_var(1)*t3*t11*t30*0.5f;
		P(0,3) = rotX*rot_vec_var(0)*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rot_vec_var(2)*t3*t11*t33*0.5f;
		P(1,0) = t22;
		P(1,1) = rot_vec_var(0)*(t19*t19)+rot_vec_var(1)*(t24*t24)+rot_vec_var(2)*(t26*t26);
		P(1,2) = rot_vec_var(2)*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(1,3) = rot_vec_var(1)*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(2,0) = t35-rotY*rot_vec_var(1)*t3*t11*t30*0.5f+rotX*rot_vec_var(0)*t3*t11*(t15-t23)*0.5f;
		P(2,1) = rot_vec_var(2)*(t16-t25)*(t17-t36)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(2,2) = rot_vec_var(1)*(t30*t30)+rot_vec_var(0)*(t37*t37)+rot_vec_var(2)*(t38*t38);
		P(2,3) = t42;
		P(3,0) = rotZ*rot_vec_var(2)*t3*t11*t33*(-0.5f)+rotX*rot_vec_var(0)*t3*t11*(t16-t25)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-t36)*0.5f;
		P(3,1) = rot_vec_var(1)*(t15-t23)*(t17-t36)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(3,2) = t42;
		P(3,3) = rot_vec_var(2)*(t33*t33)+rot_vec_var(0)*(t43*t43)+rot_vec_var(1)*(t44*t44);

	} else {
		// the equations are badly conditioned so use a small angle approximation
		P.uncorrelateCovarianceSetVariance<1>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<3>(1, 0.25f * rot_vec_var);
	}
}

void Ekf::setControlBaroHeight()
{
	_control_status.flags.baro_hgt = true;

	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlRangeHeight()
{
	_control_status.flags.rng_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlGPSHeight()
{
	_control_status.flags.gps_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlEVHeight()
{
	_control_status.flags.ev_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
}

void Ekf::stopMagFusion()
{
	stopMag3DFusion();
	stopMagHdgFusion();
	clearMagCov();
}

void Ekf::stopMag3DFusion()
{
	// save covariance data for re-use if currently doing 3-axis fusion
	if (_control_status.flags.mag_3D) {
		saveMagCovData();
		_control_status.flags.mag_3D = false;
	}
}

void Ekf::stopMagHdgFusion()
{
	_control_status.flags.mag_hdg = false;
}

void Ekf::startMagHdgFusion()
{
	stopMag3DFusion();
	_control_status.flags.mag_hdg = true;
}

void Ekf::startMag3DFusion()
{
	if (!_control_status.flags.mag_3D) {
		stopMagHdgFusion();
		zeroMagCov();
		loadMagCovData();
		_control_status.flags.mag_3D = true;
	}
}

void Ekf::startBaroHgtFusion()
{
	setControlBaroHeight();

	// We don't need to set a height sensor offset
	// since we track a separate _baro_hgt_offset
	_hgt_sensor_offset = 0.0f;

	// Turn off ground effect compensation if it times out
	if (_control_status.flags.gnd_effect) {
		if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {

			_control_status.flags.gnd_effect = false;
		}
	}
}

void Ekf::startGpsHgtFusion()
{
	setControlGPSHeight();

	// we have just switched to using gps height, calculate height sensor offset such that current
	// measurement matches our current height estimate
	if (_control_status_prev.flags.gps_hgt != _control_status.flags.gps_hgt) {
		_hgt_sensor_offset = _gps_sample_delayed.hgt - _gps_alt_ref + _state.pos(2);
	}
}

void Ekf::updateBaroHgtOffset()
{
	// calculate a filtered offset between the baro origin and local NED origin if we are not
	// using the baro as a height reference
	if (!_control_status.flags.baro_hgt && _baro_data_ready) {
		const float local_time_step = math::constrain(1e-6f * _delta_time_baro_us, 0.0f, 1.0f);

		// apply a 10 second first order low pass filter to baro offset
		const float unbiased_baro = _baro_sample_delayed.hgt - _baro_b_est.getBias();

		const float offset_rate_correction = 0.1f * (unbiased_baro + _state.pos(2) - _baro_hgt_offset);
		_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}
}

float Ekf::getGpsHeightVariance()
{
	// observation variance - receiver defined and parameter limited
	// use 1.5 as a typical ratio of vacc/hacc
	const float lower_limit = fmaxf(1.5f * _params.gps_pos_noise, 0.01f);
	const float upper_limit = fmaxf(1.5f * _params.pos_noaid_noise, lower_limit);
	const float gps_alt_var = sq(math::constrain(_gps_sample_delayed.vacc, lower_limit, upper_limit));
	return gps_alt_var;
}

void Ekf::updateBaroHgtBias()
{
	// Baro bias estimation using GPS altitude
	if (_baro_data_ready) {
		const float dt = math::constrain(1e-6f * _delta_time_baro_us, 0.0f, 1.0f);
		_baro_b_est.setMaxStateNoise(_params.baro_noise);
		_baro_b_est.setProcessNoiseStdDev(_params.baro_drift_rate);
		_baro_b_est.predict(dt);
	}

	if (_gps_data_ready && !_gps_hgt_intermittent
	    && _gps_checks_passed && _NED_origin_initialised
	    && !_baro_hgt_faulty) {
		// Use GPS altitude as a reference to compute the baro bias measurement
		const float baro_bias = (_baro_sample_delayed.hgt - _baro_hgt_offset)
					- (_gps_sample_delayed.hgt - _gps_alt_ref);
		const float baro_bias_var = getGpsHeightVariance() + sq(_params.baro_noise);
		_baro_b_est.fuseBias(baro_bias, baro_bias_var);
	}
}

Vector3f Ekf::getVisionVelocityInEkfFrame() const
{
	Vector3f vel;
	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;

	// rotate measurement into correct earth frame if required
	switch (_ev_sample_delayed.vel_frame) {
	case velocity_frame_t::BODY_FRAME_FRD:
		vel = _R_to_earth * (_ev_sample_delayed.vel - vel_offset_body);
		break;

	case velocity_frame_t::LOCAL_FRAME_FRD:
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		if (_params.fusion_mode & MASK_ROTATE_EV) {
			vel = _R_ev_to_ekf * _ev_sample_delayed.vel - vel_offset_earth;

		} else {
			vel = _ev_sample_delayed.vel - vel_offset_earth;
		}

		break;
	}

	return vel;
}

Vector3f Ekf::getVisionVelocityVarianceInEkfFrame() const
{
	Matrix3f ev_vel_cov = _ev_sample_delayed.velCov;

	// rotate measurement into correct earth frame if required
	switch (_ev_sample_delayed.vel_frame) {
	case velocity_frame_t::BODY_FRAME_FRD:
		ev_vel_cov = _R_to_earth * ev_vel_cov * _R_to_earth.transpose();
		break;

	case velocity_frame_t::LOCAL_FRAME_FRD:
		if (_params.fusion_mode & MASK_ROTATE_EV) {
			ev_vel_cov = _R_ev_to_ekf * ev_vel_cov * _R_ev_to_ekf.transpose();
		}

		break;
	}

	return ev_vel_cov.diag();
}

// update the rotation matrix which rotates EV measurements into the EKF's navigation frame
void Ekf::calcExtVisRotMat()
{
	// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
	const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
	_R_ev_to_ekf = Dcmf(q_error);
}

// Increase the yaw error variance of the quaternions
// Argument is additional yaw variance in rad**2
void Ekf::increaseQuatYawErrVariance(float yaw_variance)
{
	// See DeriveYawResetEquations.m for derivation which produces code fragments in C_code4.txt file
	// The auto-code was cleaned up and had terms multiplied by zero removed to give the following:

	// Intermediate variables
	float SG[3];
	SG[0] = sq(_state.quat_nominal(0)) - sq(_state.quat_nominal(1)) - sq(_state.quat_nominal(2)) + sq(_state.quat_nominal(3));
	SG[1] = 2*_state.quat_nominal(0)*_state.quat_nominal(2) - 2*_state.quat_nominal(1)*_state.quat_nominal(3);
	SG[2] = 2*_state.quat_nominal(0)*_state.quat_nominal(1) + 2*_state.quat_nominal(2)*_state.quat_nominal(3);

	float SQ[4];
	SQ[0] = 0.5f * ((_state.quat_nominal(1)*SG[0]) - (_state.quat_nominal(0)*SG[2]) + (_state.quat_nominal(3)*SG[1]));
	SQ[1] = 0.5f * ((_state.quat_nominal(0)*SG[1]) - (_state.quat_nominal(2)*SG[0]) + (_state.quat_nominal(3)*SG[2]));
	SQ[2] = 0.5f * ((_state.quat_nominal(3)*SG[0]) - (_state.quat_nominal(1)*SG[1]) + (_state.quat_nominal(2)*SG[2]));
	SQ[3] = 0.5f * ((_state.quat_nominal(0)*SG[0]) + (_state.quat_nominal(1)*SG[2]) + (_state.quat_nominal(2)*SG[1]));

	// Limit yaw variance increase to prevent a badly conditioned covariance matrix
	yaw_variance = fminf(yaw_variance, 1.0e-2f);

	// Add covariances for additonal yaw uncertainty to existing covariances.
	// This assumes that the additional yaw error is uncorrrelated to existing errors
	P(0,0) += yaw_variance*sq(SQ[2]);
	P(0,1) += yaw_variance*SQ[1]*SQ[2];
	P(1,1) += yaw_variance*sq(SQ[1]);
	P(0,2) += yaw_variance*SQ[0]*SQ[2];
	P(1,2) += yaw_variance*SQ[0]*SQ[1];
	P(2,2) += yaw_variance*sq(SQ[0]);
	P(0,3) -= yaw_variance*SQ[2]*SQ[3];
	P(1,3) -= yaw_variance*SQ[1]*SQ[3];
	P(2,3) -= yaw_variance*SQ[0]*SQ[3];
	P(3,3) += yaw_variance*sq(SQ[3]);
	P(1,0) += yaw_variance*SQ[1]*SQ[2];
	P(2,0) += yaw_variance*SQ[0]*SQ[2];
	P(2,1) += yaw_variance*SQ[0]*SQ[1];
	P(3,0) -= yaw_variance*SQ[2]*SQ[3];
	P(3,1) -= yaw_variance*SQ[1]*SQ[3];
	P(3,2) -= yaw_variance*SQ[0]*SQ[3];
}

// save covariance data for re-use when auto-switching between heading and 3-axis fusion
void Ekf::saveMagCovData()
{
	// save variances for the D earth axis and XYZ body axis field
	for (uint8_t index = 0; index <= 3; index ++) {
		_saved_mag_bf_variance[index] = P(index + 18, index + 18);
	}

	// save the NE axis covariance sub-matrix
	_saved_mag_ef_covmat = P.slice<2, 2>(16, 16);
}

void Ekf::loadMagCovData()
{
	// re-instate variances for the D earth axis and XYZ body axis field
	for (uint8_t index = 0; index <= 3; index ++) {
		P(index + 18, index + 18) = _saved_mag_bf_variance[index];
	}

	// re-instate the NE axis covariance sub-matrix
	P.slice<2, 2>(16, 16) = _saved_mag_ef_covmat;
}

void Ekf::startAirspeedFusion()
{
	// If starting wind state estimation, reset the wind states and covariances before fusing any data
	if (!_control_status.flags.wind) {
		// activate the wind states
		_control_status.flags.wind = true;
		// reset the wind speed states and corresponding covariances
		resetWindUsingAirspeed();
	}

	_control_status.flags.fuse_aspd = true;
}

void Ekf::stopAirspeedFusion()
{
	_control_status.flags.fuse_aspd = false;
}

void Ekf::startGpsFusion()
{
	resetHorizontalPositionToGps();

	// when using optical flow,
	// velocity reset is not necessary
	if (!_control_status.flags.opt_flow) {
		resetVelocityToGps();
	}

	_information_events.flags.starting_gps_fusion = true;
	ECL_INFO("starting GPS fusion");
	_control_status.flags.gps = true;
}

void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		stopGpsPosFusion();
		stopGpsVelFusion();
	}

	if (_control_status.flags.gps_yaw) {
		stopGpsYawFusion();
	}

	// We do not need to know the true North anymore
	// EV yaw can start again
	_inhibit_ev_yaw_use = false;;
}

void Ekf::stopGpsPosFusion()
{
	_control_status.flags.gps = false;

	if (_control_status.flags.gps_hgt) {
		startBaroHgtFusion();
	}

	_gps_pos_innov.setZero();
	_gps_pos_innov_var.setZero();
	_gps_pos_test_ratio.setZero();
}

void Ekf::stopGpsVelFusion()
{
	_gps_vel_innov.setZero();
	_gps_vel_innov_var.setZero();
	_gps_vel_test_ratio.setZero();
}

void Ekf::startGpsYawFusion()
{
	if (resetYawToGps()) {
		_control_status.flags.yaw_align = true;
		_control_status.flags.mag_dec = false;
		stopEvYawFusion();
		stopMagHdgFusion();
		stopMag3DFusion();
		_control_status.flags.gps_yaw = true;
	}

}

void Ekf::stopGpsYawFusion()
{
	_control_status.flags.gps_yaw = false;
}

void Ekf::startEvPosFusion()
{
	_control_status.flags.ev_pos = true;
	resetHorizontalPosition();
	_information_events.flags.starting_vision_pos_fusion = true;
	ECL_INFO("starting vision pos fusion");
}

void Ekf::startEvVelFusion()
{
	_control_status.flags.ev_vel = true;
	resetVelocity();
	_information_events.flags.starting_vision_vel_fusion = true;
	ECL_INFO("starting vision vel fusion");
}

void Ekf::startEvYawFusion()
{
	// turn on fusion of external vision yaw measurements and disable all magnetometer fusion
	_control_status.flags.ev_yaw = true;
	_control_status.flags.mag_dec = false;

	stopMagHdgFusion();
	stopMag3DFusion();

	_information_events.flags.starting_vision_yaw_fusion = true;
	ECL_INFO("starting vision yaw fusion");
}

void Ekf::stopEvFusion()
{
	stopEvPosFusion();
	stopEvVelFusion();
	stopEvYawFusion();
}

void Ekf::stopEvPosFusion()
{
	_control_status.flags.ev_pos = false;
	_ev_pos_innov.setZero();
	_ev_pos_innov_var.setZero();
	_ev_pos_test_ratio.setZero();
}

void Ekf::stopEvVelFusion()
{
	_control_status.flags.ev_vel = false;
	_ev_vel_innov.setZero();
	_ev_vel_innov_var.setZero();
	_ev_vel_test_ratio.setZero();
}

void Ekf::stopEvYawFusion()
{
	_control_status.flags.ev_yaw = false;
}

void Ekf::stopAuxVelFusion()
{
	_aux_vel_innov.setZero();
	_aux_vel_innov_var.setZero();
	_aux_vel_test_ratio.setZero();
}

void Ekf::stopFlowFusion()
{
	_control_status.flags.opt_flow = false;
	_flow_innov.setZero();
	_flow_innov_var.setZero();
	_optflow_test_ratio = 0.0f;
}

void Ekf::resetQuatStateYaw(float yaw, float yaw_variance, bool update_buffer)
{
	// save a copy of the quaternion state for later use in calculating the amount of reset change
	const Quatf quat_before_reset = _state.quat_nominal;

	// update transformation matrix from body to world frame using the current estimate
	_R_to_earth = Dcmf(_state.quat_nominal);

	// update the rotation matrix using the new yaw value
	_R_to_earth = updateYawInRotMat(yaw, _R_to_earth);

	// calculate the amount that the quaternion has changed by
	const Quatf quat_after_reset(_R_to_earth);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// update quaternion states
	_state.quat_nominal = quat_after_reset;
	uncorrelateQuatFromOtherStates();

	// record the state change
	_state_reset_status.quat_change = q_error;

	// update the yaw angle variance
	if (yaw_variance > FLT_EPSILON) {
		increaseQuatYawErrVariance(yaw_variance);
	}

	// add the reset amount to the output observer buffered data
	if (update_buffer) {
		for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
			_output_buffer[i].quat_nominal = _state_reset_status.quat_change * _output_buffer[i].quat_nominal;
		}

		// apply the change in attitude quaternion to our newest quaternion estimate
		// which was already taken out from the output buffer
		_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

	}

	// capture the reset event
	_state_reset_status.quat_counter++;
}

// Resets the main Nav EKf yaw to the estimator from the EKF-GSF yaw estimator
// Resets the horizontal velocity and position to the default navigation sensor
// Returns true if the reset was successful
bool Ekf::resetYawToEKFGSF()
{
	// don't allow reet using the EKF-GSF estimate until the filter has started fusing velocity
	// data and the yaw estimate has converged
	float new_yaw, new_yaw_variance;

	if (!_yawEstimator.getYawData(&new_yaw, &new_yaw_variance)) {
		return false;
	}

	const bool has_converged = new_yaw_variance < sq(_params.EKFGSF_yaw_err_max);

	if (!has_converged) {
		return false;
	}

	resetQuatStateYaw(new_yaw, new_yaw_variance, true);

	// reset velocity and position states to GPS - if yaw is fixed then the filter should start to operate correctly
	resetVelocity();
	resetHorizontalPosition();

	// record a magnetic field alignment event to prevent possibility of the EKF trying to reset the yaw to the mag later in flight
	_flt_mag_align_start_time = _imu_sample_delayed.time_us;
	_control_status.flags.yaw_align = true;

	const bool is_mag_fusion_active = _control_status.flags.mag_hdg
	                                  || _control_status.flags.mag_3D;
	const bool is_yaw_aiding_active = is_mag_fusion_active
	                                  || _control_status.flags.gps_yaw
					  || _control_status.flags.ev_yaw;

	if (!is_yaw_aiding_active) {
		_information_events.flags.yaw_aligned_to_imu_gps = true;
		ECL_INFO("Yaw aligned using IMU and GPS");

	} else {
		if (is_mag_fusion_active) {
			// stop using the magnetometer in the main EKF otherwise it's fusion could drag the yaw around
			// and cause another navigation failure
			_control_status.flags.mag_fault = true;
			_warning_events.flags.emergency_yaw_reset_mag_stopped = true;

		} else if (_control_status.flags.gps_yaw) {
			_control_status.flags.gps_yaw_fault = true;
			_warning_events.flags.emergency_yaw_reset_gps_yaw_stopped = true;

		} else if (_control_status.flags.ev_yaw) {
			_inhibit_ev_yaw_use = true;
		}

		ECL_WARN("Emergency yaw reset");
	}

	return true;
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}

void Ekf::runYawEKFGSF()
{
	float TAS = 0.f;

	if (_control_status.flags.fixed_wing) {
		if (isTimedOut(_airspeed_sample_delayed.time_us, 1000000)) {
			TAS = _params.EKFGSF_tas_default;

		} else if (_airspeed_sample_delayed.true_airspeed >= _params.arsp_thr) {
			TAS = _airspeed_sample_delayed.true_airspeed;
		}
	}

	const Vector3f imu_gyro_bias = getGyroBias();
	_yawEstimator.update(_imu_sample_delayed, _control_status.flags.in_air, TAS, imu_gyro_bias);

	// basic sanity check on GPS velocity data
	if (_gps_data_ready && _gps_sample_delayed.vacc > FLT_EPSILON &&
	    PX4_ISFINITE(_gps_sample_delayed.vel(0)) && PX4_ISFINITE(_gps_sample_delayed.vel(1))) {
		_yawEstimator.setVelocity(_gps_sample_delayed.vel.xy(), _gps_sample_delayed.vacc);
	}
}

void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero();
	_gps_pos_deriv_filt.setZero();
}
#include "EKFGSF_yaw.h"
#include <cstdlib>

EKFGSF_yaw::EKFGSF_yaw()
{
	// this flag must be false when we start
	_ahrs_ekf_gsf_tilt_aligned = false;

	// these objects are initialised in initialise() before being used internally, but can be reported for logging before then
	memset(&_ahrs_ekf_gsf, 0, sizeof(_ahrs_ekf_gsf));
	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	_gsf_yaw = 0.0f;
	_ahrs_accel.zero();
}

void EKFGSF_yaw::update(const imuSample &imu_sample,
			bool run_EKF,			// set to true when flying or movement is suitable for yaw estimation
			float airspeed,			// true airspeed used for centripetal accel compensation - set to 0 when not required.
			const Vector3f &imu_gyro_bias)  // estimated rate gyro bias (rad/sec)
{
	// copy to class variables
	_delta_ang = imu_sample.delta_ang;
	_delta_vel = imu_sample.delta_vel;
	_delta_ang_dt = imu_sample.delta_ang_dt;
	_delta_vel_dt = imu_sample.delta_vel_dt;
	_run_ekf_gsf = run_EKF;
	_true_airspeed = airspeed;

	// to reduce effect of vibration, filter using an LPF whose time constant is 1/10 of the AHRS tilt correction time constant
	const float filter_coef = fminf(10.0f * _delta_vel_dt * _tilt_gain, 1.0f);
	const Vector3f accel = _delta_vel / fmaxf(_delta_vel_dt, 0.001f);
	_ahrs_accel = _ahrs_accel * (1.0f - filter_coef) + accel * filter_coef;

	// Initialise states first time
	if (!_ahrs_ekf_gsf_tilt_aligned) {
		// check for excessive acceleration to reduce likelihood of large initial roll/pitch errors
		// due to vehicle movement
		const float accel_norm_sq = accel.norm_squared();
		const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
		const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));

		if (ok_to_align) {
			initialiseEKFGSF();
			ahrsAlignTilt();
			_ahrs_ekf_gsf_tilt_aligned = true;
		}

		return;
	}

	// calculate common values used by the AHRS complementary filter models
	_ahrs_accel_norm = _ahrs_accel.norm();

	// AHRS prediction cycle for each model - this always runs
	_ahrs_accel_fusion_gain = ahrsCalcAccelGain();

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		predictEKF(model_index);
	}

	// The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
	if (_run_ekf_gsf && _vel_data_updated) {
		if (!_ekf_gsf_vel_fuse_started) {
			initialiseEKFGSF();
			ahrsAlignYaw();

			// Initialise to gyro bias estimate from main filter because there could be a large
			// uncorrected rate gyro bias error about the gravity vector
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				_ahrs_ekf_gsf[model_index].gyro_bias = imu_gyro_bias;
			}

			_ekf_gsf_vel_fuse_started = true;

		} else {
			bool bad_update = false;

			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				// subsequent measurements are fused as direct state observations
				if (!updateEKF(model_index)) {
					bad_update = true;
				}
			}

			if (!bad_update) {
				float total_weight = 0.0f;
				// calculate weighting for each model assuming a normal distribution
				const float min_weight = 1e-5f;
				uint8_t n_weight_clips = 0;

				for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
					_model_weights(model_index) = gaussianDensity(model_index) * _model_weights(model_index);

					if (_model_weights(model_index) < min_weight) {
						n_weight_clips++;
						_model_weights(model_index) = min_weight;
					}

					total_weight += _model_weights(model_index);
				}

				// normalise the weighting function
				if (n_weight_clips < N_MODELS_EKFGSF) {
					_model_weights /= total_weight;

				} else {
					// all weights have collapsed due to excessive innovation variances so reset filters
					initialiseEKFGSF();
				}
			}
		}

	} else if (_ekf_gsf_vel_fuse_started && !_run_ekf_gsf) {
		// wait to fly again
		_ekf_gsf_vel_fuse_started = false;
	}

	// Calculate a composite yaw vector as a weighted average of the states for each model.
	// To avoid issues with angle wrapping, the yaw state is converted to a vector with length
	// equal to the weighting value before it is summed.
	Vector2f yaw_vector;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2));
		yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2));
	}

	_gsf_yaw = atan2f(yaw_vector(1), yaw_vector(0));

	// calculate a composite variance for the yaw state from a weighted average of the variance for each model
	// models with larger innovations are weighted less
	_gsf_yaw_variance = 0.0f;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw);
		_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2, 2) + yaw_delta * yaw_delta);
	}

	// prevent the same velocity data being used more than once
	_vel_data_updated = false;
}

void EKFGSF_yaw::ahrsPredict(const uint8_t model_index)
{
	// generate attitude solution using simple complementary filter for the selected model

	const Vector3f ang_rate = _delta_ang / fmaxf(_delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias;

	const Dcmf R_to_body = _ahrs_ekf_gsf[model_index].R.transpose();
	const Vector3f gravity_direction_bf = R_to_body.col(2);

	// Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
	// During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward
	Vector3f tilt_correction;

	if (_ahrs_accel_fusion_gain > 0.0f) {

		Vector3f accel = _ahrs_accel;

		if (_true_airspeed > FLT_EPSILON) {
			// Calculate body frame centripetal acceleration with assumption X axis is aligned with the airspeed vector
			// Use cross product of body rate and body frame airspeed vector
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// correct measured accel for centripetal acceleration
			accel -= centripetal_accel_bf;
		}

		tilt_correction = (gravity_direction_bf % accel) * _ahrs_accel_fusion_gain / _ahrs_accel_norm;

	}

	// Gyro bias estimation
	constexpr float gyro_bias_limit = 0.05f;
	const float spinRate = ang_rate.length();

	if (spinRate < 0.175f) {
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * _delta_ang_dt);
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias, -gyro_bias_limit,
						       gyro_bias_limit);
	}

	// delta angle from previous to current frame
	const Vector3f delta_angle_corrected = _delta_ang + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) *
					       _delta_ang_dt;

	// Apply delta angle to rotation matrix
	_ahrs_ekf_gsf[model_index].R = ahrsPredictRotMat(_ahrs_ekf_gsf[model_index].R, delta_angle_corrected);

}

void EKFGSF_yaw::ahrsAlignTilt()
{
	// Rotation matrix is constructed directly from acceleration measurement and will be the same for
	// all models so only need to calculate it once. Assumptions are:
	// 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
	// 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.

	// Calculate earth frame Down axis unit vector rotated into body frame
	const Vector3f down_in_bf = -_delta_vel.normalized();

	// Calculate earth frame North axis unit vector rotated into body frame, orthogonal to 'down_in_bf'
	const Vector3f i_vec_bf(1.0f, 0.0f, 0.0f);
	Vector3f north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf.dot(down_in_bf));
	north_in_bf.normalize();

	// Calculate earth frame East axis unit vector rotated into body frame, orthogonal to 'down_in_bf' and 'north_in_bf'
	const Vector3f east_in_bf = down_in_bf % north_in_bf;

	// Each column in a rotation matrix from earth frame to body frame represents the projection of the
	// corresponding earth frame unit vector rotated into the body frame, eg 'north_in_bf' would be the first column.
	// We need the rotation matrix from body frame to earth frame so the earth frame unit vectors rotated into body
	// frame are copied into corresponding rows instead.
	Dcmf R;
	R.setRow(0, north_in_bf);
	R.setRow(1, east_in_bf);
	R.setRow(2, down_in_bf);

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ahrs_ekf_gsf[model_index].R = R;
	}
}

void EKFGSF_yaw::ahrsAlignYaw()
{
	// Align yaw angle for each model
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		Dcmf &R = _ahrs_ekf_gsf[model_index].R;
		const float yaw = wrap_pi(_ekf_gsf[model_index].X(2));
		R = updateYawInRotMat(yaw, R);

		_ahrs_ekf_gsf[model_index].aligned = true;
	}
}

void EKFGSF_yaw::predictEKF(const uint8_t model_index)
{
	// generate an attitude reference using IMU data
	ahrsPredict(model_index);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	const Dcmf &R = _ahrs_ekf_gsf[model_index].R;
	_ekf_gsf[model_index].X(2) = shouldUse321RotationSequence(R) ?
				     getEuler321Yaw(R) :
				     getEuler312Yaw(R);

	// calculate delta velocity in a horizontal front-right frame
	const Vector3f del_vel_NED = _ahrs_ekf_gsf[model_index].R * _delta_vel;
	const float cos_yaw = cosf(_ekf_gsf[model_index].X(2));
	const float sin_yaw = sinf(_ekf_gsf[model_index].X(2));
	const float dvx =   del_vel_NED(0) * cos_yaw + del_vel_NED(1) * sin_yaw;
	const float dvy = - del_vel_NED(0) * sin_yaw + del_vel_NED(1) * cos_yaw;

	// sum delta velocities in earth frame:
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);

	// predict covariance - equations generated using EKF/python/gsf_ekf_yaw_estimator/main.py

	// Local short variable name copies required for readability
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);
	const float &psi = _ekf_gsf[model_index].X(2);

	// Use fixed values for delta velocity and delta angle process noise variances
	const float dvxVar = sq(_accel_noise * _delta_vel_dt); // variance of forward delta velocity - (m/s)^2
	const float dvyVar = dvxVar; // variance of right delta velocity - (m/s)^2
	const float dazVar = sq(_gyro_noise * _delta_ang_dt); // variance of yaw delta angle - rad^2

	// optimized auto generated code from SymPy script src/lib/ecl/EKF/python/ekf_derivation/main.py
	const float S0 = cosf(psi);
	const float S1 = ecl::powf(S0, 2);
	const float S2 = sinf(psi);
	const float S3 = ecl::powf(S2, 2);
	const float S4 = S0*dvy + S2*dvx;
	const float S5 = P02 - P22*S4;
	const float S6 = S0*dvx - S2*dvy;
	const float S7 = S0*S2;
	const float S8 = P01 + S7*dvxVar - S7*dvyVar;
	const float S9 = P12 + P22*S6;

	_ekf_gsf[model_index].P(0,0) = P00 - P02*S4 + S1*dvxVar + S3*dvyVar - S4*S5;
	_ekf_gsf[model_index].P(0,1) = -P12*S4 + S5*S6 + S8;
	_ekf_gsf[model_index].P(1,1) = P11 + P12*S6 + S1*dvyVar + S3*dvxVar + S6*S9;
	_ekf_gsf[model_index].P(0,2) = S5;
	_ekf_gsf[model_index].P(1,2) = S9;
	_ekf_gsf[model_index].P(2,2) = P22 + dazVar;

	// covariance matrix is symmetrical, so copy upper half to lower half
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// constrain variances
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}
}

// Update EKF states and covariance for specified model index using velocity measurement
bool EKFGSF_yaw::updateEKF(const uint8_t model_index)
{
	// set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	const float velObsVar = sq(fmaxf(_vel_accuracy, 0.01f));

	// calculate velocity observation innovations
	_ekf_gsf[model_index].innov(0) = _ekf_gsf[model_index].X(0) - _vel_NE(0);
	_ekf_gsf[model_index].innov(1) = _ekf_gsf[model_index].X(1) - _vel_NE(1);

	// Use temporary variables for covariance elements to reduce verbosity of auto-code expressions
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);

	// optimized auto generated code from SymPy script src/lib/ecl/EKF/python/ekf_derivation/main.py
	const float t0 = ecl::powf(P01, 2);
	const float t1 = -t0;
	const float t2 = P00*P11 + P00*velObsVar + P11*velObsVar + t1 + ecl::powf(velObsVar, 2);
	if (fabsf(t2) < 1e-6f) {
		return false;
	}
	const float t3 = 1.0F/t2;
	const float t4 = P11 + velObsVar;
	const float t5 = P01*t3;
	const float t6 = -t5;
	const float t7 = P00 + velObsVar;
	const float t8 = P00*t4 + t1;
	const float t9 = t5*velObsVar;
	const float t10 = P11*t7;
	const float t11 = t1 + t10;
	const float t12 = P01*P12;
	const float t13 = P02*t4;
	const float t14 = P01*P02;
	const float t15 = P12*t7;
	const float t16 = t0*velObsVar;
	const float t17 = ecl::powf(t2, -2);
	const float t18 = t4*velObsVar + t8;
	const float t19 = t17*t18;
	const float t20 = t17*(t16 + t7*t8);
	const float t21 = t0 - t10;
	const float t22 = t17*t21;
	const float t23 = t14 - t15;
	const float t24 = P01*t23;
	const float t25 = t12 - t13;
	const float t26 = t16 - t21*t4;
	const float t27 = t17*t26;
	const float t28 = t11 + t7*velObsVar;
	const float t30 = t17*t28;
	const float t31 = P01*t25;
	const float t32 = t23*t4 + t31;
	const float t33 = t17*t32;
	const float t35 = t24 + t25*t7;
	const float t36 = t17*t35;

	_ekf_gsf[model_index].S_det_inverse = t3;

	_ekf_gsf[model_index].S_inverse(0,0) = t3*t4;
	_ekf_gsf[model_index].S_inverse(0,1) = t6;
	_ekf_gsf[model_index].S_inverse(1,1) = t3*t7;
	_ekf_gsf[model_index].S_inverse(1,0) = _ekf_gsf[model_index].S_inverse(0,1);

	matrix::Matrix<float, 3, 2> K;
	K(0,0) = t3*t8;
	K(1,0) = t9;
	K(2,0) = t3*(-t12 + t13);
	K(0,1) = t9;
	K(1,1) = t11*t3;
	K(2,1) = t3*(-t14 + t15);

	_ekf_gsf[model_index].P(0,0) = P00 - t16*t19 - t20*t8;
	_ekf_gsf[model_index].P(0,1) = P01*(t18*t22 - t20*velObsVar + 1);
	_ekf_gsf[model_index].P(1,1) = P11 - t16*t30 + t22*t26;
	_ekf_gsf[model_index].P(0,2) = P02 + t19*t24 + t20*t25;
	_ekf_gsf[model_index].P(1,2) = P12 + t23*t27 + t30*t31;
	_ekf_gsf[model_index].P(2,2) = P22 - t23*t33 - t25*t36;
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// constrain variances
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}

	// test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
	const float test_ratio = _ekf_gsf[model_index].innov * (_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	// Perform a chi-square innovation consistency test and calculate a compression scale factor
	// that limits the magnitude of innovations to 5-sigma
	// If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
	// This protects from large measurement spikes
	const float innov_comp_scale_factor = test_ratio > 25.f ? sqrtf(25.0f / test_ratio) : 1.f;

	// Correct the state vector and capture the change in yaw angle
	const float oldYaw = _ekf_gsf[model_index].X(2);

	_ekf_gsf[model_index].X -= (K * _ekf_gsf[model_index].innov) * innov_comp_scale_factor;

	const float yawDelta = _ekf_gsf[model_index].X(2) - oldYaw;

	// apply the change in yaw angle to the AHRS
	// take advantage of sparseness in the yaw rotation matrix
	const float cosYaw = cosf(yawDelta);
	const float sinYaw = sinf(yawDelta);
	const float R_prev00 = _ahrs_ekf_gsf[model_index].R(0, 0);
	const float R_prev01 = _ahrs_ekf_gsf[model_index].R(0, 1);
	const float R_prev02 = _ahrs_ekf_gsf[model_index].R(0, 2);

	_ahrs_ekf_gsf[model_index].R(0, 0) = R_prev00 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 0) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 1) = R_prev01 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 1) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 2) = R_prev02 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 2) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(1, 0) = R_prev00 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 0) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 1) = R_prev01 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 1) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 2) = R_prev02 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 2) * cosYaw;

	return true;
}

void EKFGSF_yaw::initialiseEKFGSF()
{
	_gsf_yaw = 0.0f;
	_ekf_gsf_vel_fuse_started = false;
	_gsf_yaw_variance = _m_pi2 * _m_pi2;
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);  // All filter models start with the same weight

	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	const float yaw_increment = 2.0f * _m_pi / (float)N_MODELS_EKFGSF;

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		// evenly space initial yaw estimates in the region between +-Pi
		_ekf_gsf[model_index].X(2) = -_m_pi + (0.5f * yaw_increment) + ((float)model_index * yaw_increment);

		// take velocity states and corresponding variance from last measurement
		_ekf_gsf[model_index].X(0) = _vel_NE(0);
		_ekf_gsf[model_index].X(1) = _vel_NE(1);
		_ekf_gsf[model_index].P(0, 0) = sq(_vel_accuracy);
		_ekf_gsf[model_index].P(1, 1) = _ekf_gsf[model_index].P(0, 0);

		// use half yaw interval for yaw uncertainty
		_ekf_gsf[model_index].P(2, 2) = sq(0.5f * yaw_increment);
	}
}

float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const
{
	// calculate transpose(innovation) * inv(S) * innovation
	const float normDist = _ekf_gsf[model_index].innov.dot(_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	return _m_2pi_inv * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * normDist);
}

bool EKFGSF_yaw::getLogData(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			    float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]) const
{
	if (_ekf_gsf_vel_fuse_started) {
		*yaw_composite = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
			yaw[model_index] = _ekf_gsf[model_index].X(2);
			innov_VN[model_index] = _ekf_gsf[model_index].innov(0);
			innov_VE[model_index] = _ekf_gsf[model_index].innov(1);
			weight[model_index] = _model_weights(model_index);
		}

		return true;
	}

	return false;
}

float EKFGSF_yaw::ahrsCalcAccelGain() const
{
	// Calculate the acceleration fusion gain using a continuous function that is unity at 1g and zero
	// at the min and max g value. Allow for more acceleration when flying as a fixed wing vehicle using centripetal
	// acceleration correction as higher and more sustained g will be experienced.
	// Use a quadratic instead of linear function to prevent vibration around 1g reducing the tilt correction effectiveness.
	// see https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f;
	const bool centripetal_accel_compensation_enabled = (_true_airspeed > FLT_EPSILON);

	if (centripetal_accel_compensation_enabled
	    && _ahrs_accel_norm > CONSTANTS_ONE_G) {
		attenuation = 1.f;
	}

	const float delta_accel_g = (_ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f));
}

Matrix3f EKFGSF_yaw::ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g)
{
	Matrix3f ret = R;
	ret(0,0) += R(0,1) * g(2) - R(0,2) * g(1);
	ret(0,1) += R(0,2) * g(0) - R(0,0) * g(2);
	ret(0,2) += R(0,0) * g(1) - R(0,1) * g(0);
	ret(1,0) += R(1,1) * g(2) - R(1,2) * g(1);
	ret(1,1) += R(1,2) * g(0) - R(1,0) * g(2);
	ret(1,2) += R(1,0) * g(1) - R(1,1) * g(0);
	ret(2,0) += R(2,1) * g(2) - R(2,2) * g(1);
	ret(2,1) += R(2,2) * g(0) - R(2,0) * g(2);
	ret(2,2) += R(2,0) * g(1) - R(2,1) * g(0);

	// Renormalise rows
	for (uint8_t r = 0; r < 3; r++) {
		const float rowLengthSq = ret.row(r).norm_squared();

		if (rowLengthSq > FLT_EPSILON) {
			// Use linear approximation for inverse sqrt taking advantage of the row length being close to 1.0
			const float rowLengthInv = 1.5f - 0.5f * rowLengthSq;
			ret.row(r) *=  rowLengthInv;
		}
	}

	return ret;
}

bool EKFGSF_yaw::getYawData(float *yaw, float *yaw_variance) const
{
	if (_ekf_gsf_vel_fuse_started) {
		*yaw = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;
		return true;
	}

	return false;
}

void EKFGSF_yaw::setVelocity(const Vector2f &velocity, float accuracy)
{
	_vel_NE = velocity;
	_vel_accuracy = accuracy;
	_vel_data_updated = true;
}
/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file estimator_interface.cpp
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#include "estimator_interface.h"

#include <mathlib/mathlib.h>

// Accumulate imu data and store to buffer at desired rate
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// TODO: resolve misplaced responsibility
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	const float dt = math::constrain((imu_sample.time_us - _time_last_imu) / 1e6f, 1.0e-4f, 0.02f);

	_time_last_imu = imu_sample.time_us;

	if (_time_last_imu > 0) {
		_dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;
	}

	_newest_high_rate_imu_sample = imu_sample;

	// Do not change order of computeVibrationMetric and checkIfVehicleAtRest
	_control_status.flags.vehicle_at_rest = checkIfVehicleAtRest(dt, imu_sample);

	_imu_updated = _imu_down_sampler.update(imu_sample);

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (_imu_updated) {

		_imu_buffer.push(_imu_down_sampler.getDownSampledImuAndTriggerReset());

		// get the oldest data from the buffer
		_imu_sample_delayed = _imu_buffer.get_oldest();

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		_min_obs_interval_us = (imu_sample.time_us - _imu_sample_delayed.time_us) / (_obs_buffer_length - 1);

		setDragData(imu_sample);
	}
}

bool EstimatorInterface::checkIfVehicleAtRest(float dt, const imuSample &imu)
{
	// detect if the vehicle is not moving when on ground
	if (!_control_status.flags.in_air) {
		if (((_vibe_metrics(1) * 4.0e4f > _params.is_moving_scaler) || (_vibe_metrics(2) * 2.1e2f > _params.is_moving_scaler))
		    && ((imu.delta_ang.norm() / dt) > 0.05f * _params.is_moving_scaler)) {

			_time_last_move_detect_us = imu.time_us;
		}

		return ((imu.time_us - _time_last_move_detect_us) > (uint64_t)1E6);

	} else {
		_time_last_move_detect_us = imu.time_us;
		return false;
	}
}


void EstimatorInterface::setMagData(const magSample &mag_sample)
{
	if (!_initialised || _mag_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_mag_buffer.get_length() < _obs_buffer_length) {
		_mag_buffer_fail = !_mag_buffer.allocate(_obs_buffer_length);

		if (_mag_buffer_fail) {
			printBufferAllocationFailed("mag");
			return;
		}
	}

	// downsample to highest possible sensor rate
	// by taking the average of incoming sample
	_mag_sample_count++;
	_mag_data_sum += mag_sample.mag;
	_mag_timestamp_sum += mag_sample.time_us / 1000; // Dividing by 1000 to avoid overflow

	// limit data rate to prevent data being lost
	if ((mag_sample.time_us - _time_last_mag) > _min_obs_interval_us) {
		_time_last_mag = mag_sample.time_us;

		magSample mag_sample_new;

		// Use the time in the middle of the downsampling interval for the sample
		mag_sample_new.time_us = 1000 * (_mag_timestamp_sum / _mag_sample_count);
		mag_sample_new.time_us -= static_cast<uint64_t>(_params.mag_delay_ms * 1000);
		mag_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		mag_sample_new.mag = _mag_data_sum / _mag_sample_count;

		_mag_buffer.push(mag_sample_new);

		_mag_sample_count = 0;
		_mag_data_sum.setZero();
		_mag_timestamp_sum = 0;
	}
}

void EstimatorInterface::setGpsData(const gps_message &gps)
{
	if (!_initialised || _gps_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_gps_buffer.get_length() < _obs_buffer_length) {
		_gps_buffer_fail = !_gps_buffer.allocate(_obs_buffer_length);

		if (_gps_buffer_fail) {
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	// limit data rate to prevent data being lost
	const bool need_gps = (_params.fusion_mode & MASK_USE_GPS) || (_params.vdist_sensor_type == VDIST_SENSOR_GPS);

	// TODO: remove checks that are not timing related
	if (((gps.time_usec - _time_last_gps) > _min_obs_interval_us) && need_gps && gps.fix_type > 2) {
		_time_last_gps = gps.time_usec;

		gpsSample gps_sample_new;

		gps_sample_new.time_us = gps.time_usec - static_cast<uint64_t>(_params.gps_delay_ms * 1000);
		gps_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		gps_sample_new.vel = gps.vel_ned;

		_gps_speed_valid = gps.vel_ned_valid;
		gps_sample_new.sacc = gps.sacc;
		gps_sample_new.hacc = gps.eph;
		gps_sample_new.vacc = gps.epv;

		gps_sample_new.hgt = (float)gps.alt * 1e-3f;

		gps_sample_new.yaw = gps.yaw;

		if (PX4_ISFINITE(gps.yaw_offset)) {
			_gps_yaw_offset = gps.yaw_offset;

		} else {
			_gps_yaw_offset = 0.0f;
		}

		// Only calculate the relative position if the WGS-84 location of the origin is set
		if (collect_gps(gps)) {
			gps_sample_new.pos = _pos_ref.project((gps.lat / 1.0e7), (gps.lon / 1.0e7));

		} else {
			gps_sample_new.pos(0) = 0.0f;
			gps_sample_new.pos(1) = 0.0f;
		}

		_gps_buffer.push(gps_sample_new);
	}
}

void EstimatorInterface::setBaroData(const baroSample &baro_sample)
{
	if (!_initialised || _baro_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_baro_buffer.get_length() < _obs_buffer_length) {
		_baro_buffer_fail = !_baro_buffer.allocate(_obs_buffer_length);

		if (_baro_buffer_fail) {
			printBufferAllocationFailed("baro");
			return;
		}
	}

	// downsample to highest possible sensor rate
	// by baro data by taking the average of incoming sample
	_baro_sample_count++;
	_baro_alt_sum += baro_sample.hgt;
	_baro_timestamp_sum += baro_sample.time_us / 1000; // Dividing by 1000 to avoid overflow

	// limit data rate to prevent data being lost
	if ((baro_sample.time_us - _time_last_baro) > _min_obs_interval_us) {
		_time_last_baro = baro_sample.time_us;

		const float baro_alt_avg = _baro_alt_sum / (float)_baro_sample_count;

		baroSample baro_sample_new;
		baro_sample_new.hgt = compensateBaroForDynamicPressure(baro_alt_avg);

		// Use the time in the middle of the downsampling interval for the sample
		baro_sample_new.time_us = 1000 * (_baro_timestamp_sum / _baro_sample_count);
		baro_sample_new.time_us -= static_cast<uint64_t>(_params.baro_delay_ms * 1000);
		baro_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_baro_buffer.push(baro_sample_new);

		_baro_sample_count = 0;
		_baro_alt_sum = 0.0f;
		_baro_timestamp_sum = 0;
	}
}

void EstimatorInterface::setAirspeedData(const airspeedSample &airspeed_sample)
{
	if (!_initialised || _airspeed_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_airspeed_buffer.get_length() < _obs_buffer_length) {
		_airspeed_buffer_fail = !_airspeed_buffer.allocate(_obs_buffer_length);

		if (_airspeed_buffer_fail) {
			printBufferAllocationFailed("airspeed");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((airspeed_sample.time_us - _time_last_airspeed) > _min_obs_interval_us) {
		_time_last_airspeed = airspeed_sample.time_us;

		airspeedSample airspeed_sample_new = airspeed_sample;

		airspeed_sample_new.time_us -= static_cast<uint64_t>(_params.airspeed_delay_ms * 1000);
		airspeed_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_airspeed_buffer.push(airspeed_sample_new);
	}
}

void EstimatorInterface::setRangeData(const rangeSample &range_sample)
{
	if (!_initialised || _range_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_range_buffer.get_length() < _obs_buffer_length) {
		_range_buffer_fail = !_range_buffer.allocate(_obs_buffer_length);

		if (_range_buffer_fail) {
			printBufferAllocationFailed("range");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((range_sample.time_us - _time_last_range) > _min_obs_interval_us) {
		_time_last_range = range_sample.time_us;

		rangeSample range_sample_new = range_sample;
		range_sample_new.time_us -= static_cast<uint64_t>(_params.range_delay_ms * 1000);
		range_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_range_buffer.push(range_sample_new);
	}
}

void EstimatorInterface::setOpticalFlowData(const flowSample &flow)
{
	if (!_initialised || _flow_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_flow_buffer.get_length() < _imu_buffer_length) {
		_flow_buffer_fail = !_flow_buffer.allocate(_imu_buffer_length);

		if (_flow_buffer_fail) {
			printBufferAllocationFailed("flow");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((flow.time_us - _time_last_optflow) > _min_obs_interval_us) {
		_time_last_optflow = flow.time_us;

		flowSample optflow_sample_new = flow;

		optflow_sample_new.time_us -= static_cast<uint64_t>(_params.flow_delay_ms * 1000);
		optflow_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_flow_buffer.push(optflow_sample_new);
	}
}

// set attitude and position data derived from an external vision system
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
	if (!_initialised || _ev_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_ext_vision_buffer.get_length() < _obs_buffer_length) {
		_ev_buffer_fail = !_ext_vision_buffer.allocate(_obs_buffer_length);

		if (_ev_buffer_fail) {
			printBufferAllocationFailed("vision");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((evdata.time_us - _time_last_ext_vision) > _min_obs_interval_us) {
		_time_last_ext_vision = evdata.time_us;

		extVisionSample ev_sample_new = evdata;
		// calculate the system time-stamp for the mid point of the integration period
		ev_sample_new.time_us -= static_cast<uint64_t>(_params.ev_delay_ms * 1000);
		ev_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_ext_vision_buffer.push(ev_sample_new);
	}
}

void EstimatorInterface::setAuxVelData(const auxVelSample &auxvel_sample)
{
	if (!_initialised || _auxvel_buffer_fail) {
		return;
	}

	// Allocate the required buffer size if not previously done
	// Do not retry if allocation has failed previously
	if (_auxvel_buffer.get_length() < _obs_buffer_length) {
		_auxvel_buffer_fail = !_auxvel_buffer.allocate(_obs_buffer_length);

		if (_auxvel_buffer_fail) {
			printBufferAllocationFailed("aux vel");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((auxvel_sample.time_us - _time_last_auxvel) > _min_obs_interval_us) {
		_time_last_auxvel = auxvel_sample.time_us;

		auxVelSample auxvel_sample_new = auxvel_sample;

		auxvel_sample_new.time_us -= static_cast<uint64_t>(_params.auxvel_delay_ms * 1000);
		auxvel_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_auxvel_buffer.push(auxvel_sample_new);
	}
}

void EstimatorInterface::setDragData(const imuSample &imu)
{
	// down-sample the drag specific force data by accumulating and calculating the mean when
	// sufficient samples have been collected
	if ((_params.fusion_mode & MASK_USE_DRAG) && !_drag_buffer_fail) {

		// Allocate the required buffer size if not previously done
		// Do not retry if allocation has failed previously
		if (_drag_buffer.get_length() < _obs_buffer_length) {
			_drag_buffer_fail = !_drag_buffer.allocate(_obs_buffer_length);

			if (_drag_buffer_fail) {
				printBufferAllocationFailed("drag");
				return;
			}
		}

		_drag_sample_count ++;
		// note acceleration is accumulated as a delta velocity
		_drag_down_sampled.accelXY(0) += imu.delta_vel(0);
		_drag_down_sampled.accelXY(1) += imu.delta_vel(1);
		_drag_down_sampled.time_us += imu.time_us;
		_drag_sample_time_dt += imu.delta_vel_dt;

		// calculate the downsample ratio for drag specific force data
		uint8_t min_sample_ratio = (uint8_t) ceilf((float)_imu_buffer_length / _obs_buffer_length);

		if (min_sample_ratio < 5) {
			min_sample_ratio = 5;
		}

		// calculate and store means from accumulated values
		if (_drag_sample_count >= min_sample_ratio) {
			// note conversion from accumulated delta velocity to acceleration
			_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;
			_drag_down_sampled.accelXY(1) /= _drag_sample_time_dt;
			_drag_down_sampled.time_us /= _drag_sample_count;

			// write to buffer
			_drag_buffer.push(_drag_down_sampled);

			// reset accumulators
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;
		}
	}
}

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	// find the maximum time delay the buffers are required to handle
	// it's reasonable to assume that barometer is always used, and its delay is low
	// it's reasonable to assume that aux velocity device has low delay. TODO: check the delay only if the aux device is used
	float max_time_delay_ms = math::max(_params.baro_delay_ms, _params.auxvel_delay_ms);

	// using airspeed
	if (_params.arsp_thr > FLT_EPSILON) {
		max_time_delay_ms = math::max(_params.airspeed_delay_ms, max_time_delay_ms);
	}

	// mag mode
	if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
		max_time_delay_ms = math::max(_params.mag_delay_ms, max_time_delay_ms);
	}

	// range aid or range height
	if (_params.range_aid || (_params.vdist_sensor_type == VDIST_SENSOR_RANGE)) {
		max_time_delay_ms = math::max(_params.range_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & MASK_USE_GPS) {
		max_time_delay_ms = math::max(_params.gps_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & MASK_USE_OF) {
		max_time_delay_ms = math::max(_params.flow_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & (MASK_USE_EVPOS | MASK_USE_EVYAW | MASK_USE_EVVEL)) {
		max_time_delay_ms = math::max(_params.ev_delay_ms, max_time_delay_ms);
	}

	// calculate the IMU buffer length required to accomodate the maximum delay with some allowance for jitter
	_imu_buffer_length = ceilf(max_time_delay_ms / FILTER_UPDATE_PERIOD_MS) + 1;

	// set the observation buffer length to handle the minimum time of arrival between observations in combination
	// with the worst case delay from current time to ekf fusion time
	// allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	const float ekf_delay_ms = max_time_delay_ms * 1.5f;
	_obs_buffer_length = ceilf(ekf_delay_ms / _params.sensor_interval_min_ms);

	// limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_buffer.allocate(_imu_buffer_length)
	    || !_output_vert_buffer.allocate(_imu_buffer_length)) {

		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_imu_sample_delayed.time_us = timestamp;
	_imu_sample_delayed.delta_vel_clipping[0] = false;
	_imu_sample_delayed.delta_vel_clipping[1] = false;
	_imu_sample_delayed.delta_vel_clipping[2] = false;

	_fault_status.value = 0;

	return true;
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.ev_vel)
	       // Combined airspeed and sideslip fusion allows sustained wind relative dead reckoning
	       // and so is treated as a single aiding source.
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}

void EstimatorInterface::print_status()
{
	ECL_INFO("imu buffer: %d (%d Bytes)", _imu_buffer.get_length(), _imu_buffer.get_total_size());
	ECL_INFO("gps buffer: %d (%d Bytes)", _gps_buffer.get_length(), _gps_buffer.get_total_size());
	ECL_INFO("mag buffer: %d (%d Bytes)", _mag_buffer.get_length(), _mag_buffer.get_total_size());
	ECL_INFO("baro buffer: %d (%d Bytes)", _baro_buffer.get_length(), _baro_buffer.get_total_size());
	ECL_INFO("range buffer: %d (%d Bytes)", _range_buffer.get_length(), _range_buffer.get_total_size());
	ECL_INFO("airspeed buffer: %d (%d Bytes)", _airspeed_buffer.get_length(), _airspeed_buffer.get_total_size());
	ECL_INFO("flow buffer: %d (%d Bytes)", _flow_buffer.get_length(), _flow_buffer.get_total_size());
	ECL_INFO("vision buffer: %d (%d Bytes)", _ext_vision_buffer.get_length(), _ext_vision_buffer.get_total_size());
	ECL_INFO("output buffer: %d (%d Bytes)", _output_buffer.get_length(), _output_buffer.get_total_size());
	ECL_INFO("output vert buffer: %d (%d Bytes)", _output_vert_buffer.get_length(), _output_vert_buffer.get_total_size());
	ECL_INFO("drag buffer: %d (%d Bytes)", _drag_buffer.get_length(), _drag_buffer.get_total_size());
}
/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file gps_checks.cpp
 * Perform pre-flight and in-flight GPS quality checks
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <mathlib/mathlib.h>

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS  (1<<0)
#define MASK_GPS_PDOP   (1<<1)
#define MASK_GPS_HACC   (1<<2)
#define MASK_GPS_VACC   (1<<3)
#define MASK_GPS_SACC   (1<<4)
#define MASK_GPS_HDRIFT (1<<5)
#define MASK_GPS_VDRIFT (1<<6)
#define MASK_GPS_HSPD   (1<<7)
#define MASK_GPS_VSPD   (1<<8)

bool Ekf::collect_gps(const gps_message &gps)
{
	// Run GPS checks always
	_gps_checks_passed = gps_is_good(gps);

	if (_filter_initialised && !_NED_origin_initialised && _gps_checks_passed) {
		// If we have good GPS data set the origin's WGS-84 position to the last gps fix
		const double lat = gps.lat * 1.0e-7;
		const double lon = gps.lon * 1.0e-7;

		if (!_pos_ref.isInitialized()) {
			_pos_ref.initReference(lat, lon, _time_last_imu);

			// if we are already doing aiding, correct for the change in position since the EKF started navigating
			if (isHorizontalAidingActive()) {
				double est_lat;
				double est_lon;
				_pos_ref.reproject(-_state.pos(0), -_state.pos(1), est_lat, est_lon);
				_pos_ref.initReference(est_lat, est_lon, _time_last_imu);
			}
		}

		// Take the current GPS height and subtract the filter height above origin to estimate the GPS height of the origin
		_gps_alt_ref = 1e-3f * (float)gps.alt + _state.pos(2);
		_NED_origin_initialised = true;

		_earth_rate_NED = calcEarthRateNED((float)math::radians(_pos_ref.getProjectionReferenceLat()));
		_last_gps_origin_time_us = _time_last_imu;

		const bool declination_was_valid = PX4_ISFINITE(_mag_declination_gps);

		// set the magnetic field data returned by the geo library using the current GPS position
		_mag_declination_gps = get_mag_declination_radians(lat, lon);
		_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
		_mag_strength_gps = get_mag_strength_gauss(lat, lon);

		// request a reset of the yaw using the new declination
		if ((_params.mag_fusion_type != MAG_FUSE_TYPE_NONE)
		     && !declination_was_valid) {
			_mag_yaw_reset_req = true;
		}

		// save the horizontal and vertical position uncertainty of the origin
		_gps_origin_eph = gps.eph;
		_gps_origin_epv = gps.epv;

		// if the user has selected GPS as the primary height source, switch across to using it
		if (_params.vdist_sensor_type == VDIST_SENSOR_GPS) {
			startGpsHgtFusion();
		}

		_information_events.flags.gps_checks_passed = true;
		ECL_INFO("GPS checks passed");

	} else if (!_NED_origin_initialised) {
		// a rough 2D fix is still sufficient to lookup declination
		if ((gps.fix_type >= 2) && (gps.eph < 1000)) {

			const bool declination_was_valid = PX4_ISFINITE(_mag_declination_gps);

			// If we have good GPS data set the origin's WGS-84 position to the last gps fix
			const double lat = gps.lat * 1.0e-7;
			const double lon = gps.lon * 1.0e-7;

			// set the magnetic field data returned by the geo library using the current GPS position
			_mag_declination_gps = get_mag_declination_radians(lat, lon);
			_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
			_mag_strength_gps = get_mag_strength_gauss(lat, lon);

			// request mag yaw reset if there's a mag declination for the first time
			if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
				if (!declination_was_valid && PX4_ISFINITE(_mag_declination_gps)) {
					_mag_yaw_reset_req = true;
				}
			}

			_earth_rate_NED = calcEarthRateNED((float)math::radians(lat));
		}
	}

	// start collecting GPS if there is a 3D fix and the NED origin has been set
	return _NED_origin_initialised && (gps.fix_type >= 3);
}

/*
 * Return true if the GPS solution quality is adequate to set an origin for the EKF
 * and start GPS aiding.
 * All activated checks must pass for 10 seconds.
 * Checks are activated using the EKF2_GPS_CHECK bitmask parameter
 * Checks are adjusted using the EKF2_REQ_* parameters
*/
bool Ekf::gps_is_good(const gps_message &gps)
{
	// Check the fix type
	_gps_check_fail_status.flags.fix = (gps.fix_type < 3);

	// Check the number of satellites
	_gps_check_fail_status.flags.nsats = (gps.nsats < _params.req_nsats);

	// Check the position dilution of precision
	_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop);

	// Check the reported horizontal and vertical position accuracy
	_gps_check_fail_status.flags.hacc = (gps.eph > _params.req_hacc);
	_gps_check_fail_status.flags.vacc = (gps.epv > _params.req_vacc);

	// Check the reported speed accuracy
	_gps_check_fail_status.flags.sacc = (gps.sacc > _params.req_sacc);

	// check if GPS quality is degraded
	_gps_error_norm = fmaxf((gps.eph / _params.req_hacc), (gps.epv / _params.req_vacc));
	_gps_error_norm = fmaxf(_gps_error_norm, (gps.sacc / _params.req_sacc));

	// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
	constexpr float filt_time_const = 10.0f;
	const float dt = math::constrain(float(int64_t(_time_last_imu) - int64_t(_gps_pos_prev.getProjectionReferenceTimestamp())) * 1e-6f, 0.001f, filt_time_const);
	const float filter_coef = dt / filt_time_const;

	// The following checks are only valid when the vehicle is at rest
	const double lat = gps.lat * 1.0e-7;
	const double lon = gps.lon * 1.0e-7;

	if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// Calculate position movement since last measurement
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// calculate position movement since last GPS fix
		if (_gps_pos_prev.getProjectionReferenceTimestamp() > 0) {
			_gps_pos_prev.project(lat, lon, delta_pos_n, delta_pos_e);

		} else {
			// no previous position has been set
			_gps_pos_prev.initReference(lat, lon, _time_last_imu);
			_gps_alt_prev = 1e-3f * (float)gps.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		Vector3f pos_derived(delta_pos_n, delta_pos_e, (_gps_alt_prev - 1e-3f * (float)gps.alt));
		pos_derived = matrix::constrain(pos_derived / dt, -10.0f * vel_limit, 10.0f * vel_limit);

		// Apply a low pass filter
		_gps_pos_deriv_filt = pos_derived * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef);

		// Calculate the horizontal drift speed and fail if too high
		_gps_drift_metrics[0] = Vector2f(_gps_pos_deriv_filt.xy()).norm();
		_gps_check_fail_status.flags.hdrift = (_gps_drift_metrics[0] > _params.req_hdrift);

		// Fail if the vertical drift speed is too high
		_gps_drift_metrics[1] = fabsf(_gps_pos_deriv_filt(2));
		_gps_check_fail_status.flags.vdrift = (_gps_drift_metrics[1] > _params.req_vdrift);

		// Check the magnitude of the filtered horizontal GPS velocity
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gps.vel_ned.xy()),
					   -10.0f * _params.req_hdrift,
					   10.0f * _params.req_hdrift);
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef);
		_gps_drift_metrics[2] = _gps_velNE_filt.norm();
		_gps_check_fail_status.flags.hspeed = (_gps_drift_metrics[2] > _params.req_hdrift);

		_gps_drift_updated = true;

	} else if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_gps_check_fail_status.flags.hdrift = false;
		_gps_check_fail_status.flags.vdrift = false;
		_gps_check_fail_status.flags.hspeed = false;
		_gps_drift_updated = false;

		resetGpsDriftCheckFilters();

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		_gps_drift_updated = true;

		resetGpsDriftCheckFilters();
	}

	// save GPS fix for next time
	_gps_pos_prev.initReference(lat, lon, _time_last_imu);
	_gps_alt_prev = 1e-3f * (float)gps.alt;

	// Check  the filtered difference between GPS and EKF vertical velocity
	const float vz_diff_limit = 10.0f * _params.req_vdrift;
	const float vertVel = math::constrain(gps.vel_ned(2) - _state.vel(2), -vz_diff_limit, vz_diff_limit);
	_gps_velD_diff_filt = vertVel * filter_coef + _gps_velD_diff_filt * (1.0f - filter_coef);
	_gps_check_fail_status.flags.vspeed = (fabsf(_gps_velD_diff_filt) > _params.req_vdrift);

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_last_imu;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
		(_gps_check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) ||
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD))
	) {
		_last_gps_fail_us = _time_last_imu;

	} else {
		_last_gps_pass_us = _time_last_imu;
	}

	// continuous period without fail of x seconds required to return a healthy status
	return isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us);
}
/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion()
{
	if (!(_params.fusion_mode & MASK_USE_GPS)) {
		stopGpsFusion();
		return;
	}

	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {
		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		controlGpsYawFusion(gps_checks_passing, gps_checks_failing);

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = _control_status.flags.tilt_align
		                                          && _control_status.flags.yaw_align
							  && _NED_origin_initialised;
		const bool continuing_conditions_passing = mandatory_conditions_passing
		                                           && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing
							 && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					fuseGpsVelPos();

					if (shouldResetGpsFusion()){
						const bool is_yaw_failure = !isVelStateAlignedWithObs();
						const bool was_gps_signal_lost = isTimedOut(_time_prev_gps_us, 1000000);

						/* A reset is not performed when getting GPS back after a significant period of no data
						 * because the timeout could have been caused by bad GPS.
						 * The total number of resets allowed per boot cycle is limited.
						 */
						if (is_yaw_failure
						    && _control_status.flags.in_air
						    && !was_gps_signal_lost
						    && _ekfgsf_yaw_reset_count < _params.EKFGSF_reset_count_limit) {

							_do_ekfgsf_yaw_reset = true;

						} else {
							// use GPS velocity data to check and correct yaw angle if a FW vehicle
							if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
								// if flying a fixed wing aircraft, do a complete reset that includes yaw
								_control_status.flags.mag_aligned_in_flight = realignYawGPS();
							}

							_warning_events.flags.gps_fusion_timout = true;
							ECL_WARN("GPS fusion timeout - resetting");
							_velpos_reset_request = true;
						}
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");

					// TODO: move this to EV control logic
					// Reset position state to external vision if we are going to use absolute values
					if (_control_status.flags.ev_pos && !(_params.fusion_mode & MASK_ROTATE_EV)) {
						resetHorizontalPosition();
					}
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (_control_status.flags.ev_yaw
				    || _mag_inhibit_yaw_reset_req
				    || _mag_yaw_reset_req) {

					_mag_yaw_reset_req = true;

					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();
					_inhibit_ev_yaw_use = true;

				} else {
					startGpsFusion();
				}

			} else if(!_control_status.flags.yaw_align
		                  && (_params.mag_fusion_type == MAG_FUSE_TYPE_NONE)) {
				// If no mag is used, align using the yaw estimator
				_do_ekfgsf_yaw_reset = true;
			}
		}

		processYawEstimatorResetRequest();
		processVelPosResetRequest();

	} else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && (_imu_sample_delayed.time_us - _gps_sample_delayed.time_us > (uint64_t)1e6)
		    && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
	}
}

bool Ekf::shouldResetGpsFusion() const
{
	/* We are relying on aiding to constrain drift so after a specified time
	 * with no aiding we need to do something
	 */
	const bool is_reset_required = hasHorizontalAidingTimedOut()
				     || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	/* Logic controlling the reset of navigation filter yaw to the EKF-GSF estimate to recover from loss of
	 * navigation casued by a bad yaw estimate.

	 * A rapid reset to the EKF-GSF estimate is performed after a recent takeoff if horizontal velocity
	 * innovation checks fail. This enables recovery from a bad yaw estimate. After 30 seconds from takeoff,
	 * different test criteria are used that take longer to trigger and reduce false positives. A reset is
	 * not performed if the fault condition was present before flight to prevent triggering due to GPS glitches
	 * or other sensor errors.
	 */
	const bool is_recent_takeoff_nav_failure = _control_status.flags.in_air
						   && isRecent(_time_last_on_ground_us, 30000000)
						   && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
						   && (_time_last_hor_vel_fuse > _time_last_on_ground_us);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_recent_takeoff_nav_failure || is_inflight_nav_failure);
}

void Ekf::processYawEstimatorResetRequest()
{
	/* The yaw reset to the EKF-GSF estimate can be requested externally at any time during flight.
	 * The minimum time interval between resets to the EKF-GSF estimate is limited to allow the EKF-GSF time
	 * to improve its estimate if the previous reset was not successful.
	 */
	if (_do_ekfgsf_yaw_reset
	    && isTimedOut(_ekfgsf_yaw_reset_time, 5000000)){
		if (resetYawToEKFGSF()) {
			_ekfgsf_yaw_reset_time = _time_last_imu;
			_time_last_hor_pos_fuse = _time_last_imu;
			_time_last_hor_vel_fuse = _time_last_imu;

			_do_ekfgsf_yaw_reset = false;
			_velpos_reset_request = false; // included in yaw reset
			_ekfgsf_yaw_reset_count++;
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2021 PX4. All rights reserved.
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

/**
 * @file gps_fusion.cpp
 * Function for fusing gps measurements
 */

/* #include <mathlib/mathlib.h> */
#include "ekf.h"

void Ekf::fuseGpsVelPos()
{
	Vector2f gps_vel_innov_gates; // [horizontal vertical]
	Vector2f gps_pos_innov_gates; // [horizontal vertical]
	Vector3f gps_pos_obs_var;

	// correct velocity for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
	_gps_sample_delayed.vel -= vel_offset_earth;

	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
	_gps_sample_delayed.pos -= pos_offset_earth.xy();
	_gps_sample_delayed.hgt += pos_offset_earth(2);

	const float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);

	if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// if we are using other sources of aiding, then relax the upper observation
		// noise limit which prevents bad GPS perturbing the position estimate
		gps_pos_obs_var(0) = gps_pos_obs_var(1) = sq(fmaxf(_gps_sample_delayed.hacc, lower_limit));

	} else {
		// if we are not using another source of aiding, then we are reliant on the GPS
		// observations to constrain attitude errors and must limit the observation noise value.
		float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
		gps_pos_obs_var(0) = gps_pos_obs_var(1) = sq(math::constrain(_gps_sample_delayed.hacc, lower_limit, upper_limit));
	}

	_gps_sample_delayed.sacc = fmaxf(_gps_sample_delayed.sacc, _params.gps_vel_noise);

	_last_vel_obs_var.setAll(sq(_gps_sample_delayed.sacc));
	_last_vel_obs_var(2) *= sq(1.5f);

	// calculate innovations
	_last_vel_obs = _gps_sample_delayed.vel;
	_gps_vel_innov = _state.vel - _last_vel_obs;
	_gps_pos_innov.xy() = Vector2f(_state.pos) - _gps_sample_delayed.pos;

	// set innovation gate size
	gps_pos_innov_gates(0) = fmaxf(_params.gps_pos_innov_gate, 1.0f);
	gps_vel_innov_gates(0) = gps_vel_innov_gates(1) = fmaxf(_params.gps_vel_innov_gate, 1.0f);

	// fuse GPS measurement
	fuseHorizontalVelocity(_gps_vel_innov, gps_vel_innov_gates, _last_vel_obs_var, _gps_vel_innov_var, _gps_vel_test_ratio);
	fuseVerticalVelocity(_gps_vel_innov, gps_vel_innov_gates, _last_vel_obs_var, _gps_vel_innov_var, _gps_vel_test_ratio);
	fuseHorizontalPosition(_gps_pos_innov, gps_pos_innov_gates, gps_pos_obs_var, _gps_pos_innov_var, _gps_pos_test_ratio);
}
/****************************************************************************
 *
 *   Copyright (c) 2018 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file gps_yaw_fusion.cpp
 * Definition of functions required to use yaw obtained from GPS dual antenna measurements.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::fuseGpsYaw()
{
	// assign intermediate state variables
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
	const float measured_hdg = wrap_pi(_gps_sample_delayed.yaw + _gps_yaw_offset);

	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return;
	}

	// calculate predicted antenna yaw angle
	const float predicted_hdg = atan2f(ant_vec_ef(1), ant_vec_ef(0));

	// using magnetic heading process noise
	// TODO extend interface to use yaw uncertainty provided by GPS if available
	const float R_YAW = sq(fmaxf(_params.gps_heading_noise, 1.0e-2f));

	// calculate intermediate variables
	const float HK0 = sinf(_gps_yaw_offset);
	const float HK1 = q0*q3;
	const float HK2 = q1*q2;
	const float HK3 = 2*HK0*(HK1 - HK2);
	const float HK4 = cosf(_gps_yaw_offset);
	const float HK5 = ecl::powf(q1, 2);
	const float HK6 = ecl::powf(q2, 2);
	const float HK7 = ecl::powf(q0, 2) - ecl::powf(q3, 2);
	const float HK8 = HK4*(HK5 - HK6 + HK7);
	const float HK9 = HK3 - HK8;

	if (fabsf(HK9) < 1e-3f) {
		return;
	}
	const float HK10 = 1.0F/HK9;
	const float HK11 = HK4*q0;
	const float HK12 = HK0*q3;
	const float HK13 = HK0*(-HK5 + HK6 + HK7) + 2*HK4*(HK1 + HK2);
	const float HK14 = HK10*HK13;
	const float HK15 = HK0*q0 + HK4*q3;
	const float HK16 = HK10*(HK14*(HK11 - HK12) + HK15);
	const float HK17 = ecl::powf(HK13, 2)/ecl::powf(HK9, 2) + 1;
	if (fabsf(HK17) < 1e-3f) {
		return;
	}
	const float HK18 = 2/HK17;
	// const float HK19 = 1.0F/(-HK3 + HK8);
	const float HK19_inverse = -HK3 + HK8;

	if (fabsf(HK19_inverse) < 1e-6f) {
		return;
	}
	const float HK19 = 1.0F/HK19_inverse;
	const float HK20 = HK4*q1;
	const float HK21 = HK0*q2;
	const float HK22 = HK13*HK19;
	const float HK23 = HK0*q1 - HK4*q2;
	const float HK24 = HK19*(HK22*(HK20 + HK21) + HK23);
	const float HK25 = HK19*(-HK20 - HK21 + HK22*HK23);
	const float HK26 = HK10*(-HK11 + HK12 + HK14*HK15);
	const float HK27 = -HK16*P(0,0) - HK24*P(0,1) - HK25*P(0,2) + HK26*P(0,3);
	const float HK28 = -HK16*P(0,1) - HK24*P(1,1) - HK25*P(1,2) + HK26*P(1,3);
	const float HK29 = 4/ecl::powf(HK17, 2);
	const float HK30 = -HK16*P(0,2) - HK24*P(1,2) - HK25*P(2,2) + HK26*P(2,3);
	const float HK31 = -HK16*P(0,3) - HK24*P(1,3) - HK25*P(2,3) + HK26*P(3,3);
	// const float HK32 = HK18/(-HK16*HK27*HK29 - HK24*HK28*HK29 - HK25*HK29*HK30 + HK26*HK29*HK31 + R_YAW);

	// check if the innovation variance calculation is badly conditioned
	_heading_innov_var = (-HK16*HK27*HK29 - HK24*HK28*HK29 - HK25*HK29*HK30 + HK26*HK29*HK31 + R_YAW);

	if (_heading_innov_var < R_YAW) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GPS yaw numerical error - covariance reset");
		return;
	}

	_fault_status.flags.bad_hdg = false;
	const float HK32 = HK18 / _heading_innov_var;

	// calculate the innovation and define the innovation gate
	const float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
	_heading_innov = predicted_hdg - measured_hdg;

	// wrap the innovation to the interval between +-pi
	_heading_innov = wrap_pi(_heading_innov);

	// innovation test ratio
	_yaw_test_ratio = sq(_heading_innov) / (sq(innov_gate) * _heading_innov_var);

	// we are no longer using 3-axis fusion so set the reported test levels to zero
	_mag_test_ratio.setZero();

	if (_yaw_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_yaw = true;
		return;

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
	}

	_yaw_signed_test_ratio_lpf.update(matrix::sign(_heading_innov) * _yaw_test_ratio);

	if (!_control_status.flags.in_air
	    && fabsf(_yaw_signed_test_ratio_lpf.getState()) > 0.2f) {

		// A constant large signed test ratio is a sign of wrong gyro bias
		// Reset the yaw gyro variance to converge faster and avoid
		// being stuck on a previous bad estimate
		resetZDeltaAngBiasCov();
	}

	// calculate observation jacobian
	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3> Hfusion;
	Hfusion.at<0>() = -HK16*HK18;
	Hfusion.at<1>() = -HK18*HK24;
	Hfusion.at<2>() = -HK18*HK25;
	Hfusion.at<3>() = HK18*HK26;

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;
	Kfusion(0) = HK27*HK32;
	Kfusion(1) = HK28*HK32;
	Kfusion(2) = HK30*HK32;
	Kfusion(3) = HK31*HK32;
	for (unsigned row = 4; row <= 23; row++) {
		Kfusion(row) = HK32*(-HK16*P(0,row) - HK24*P(1,row) - HK25*P(2,row) + HK26*P(3,row));
	}

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, _heading_innov);
	_fault_status.flags.bad_hdg = !is_fused;

	if (is_fused) {
		_time_last_gps_yaw_fuse = _time_last_imu;
	}
}

bool Ekf::resetYawToGps()
{
	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(_gps_yaw_offset), sinf(_gps_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false;
	}

	// GPS yaw measurement is alreday compensated for antenna offset in the driver
	const float measured_yaw = _gps_sample_delayed.yaw;

	const float yaw_variance = sq(fmaxf(_params.gps_heading_noise, 1.0e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance, true);

	_time_last_gps_yaw_fuse = _time_last_imu;
	_yaw_signed_test_ratio_lpf.reset(0.f);

	return true;
}
#include "imu_down_sampler.hpp"

ImuDownSampler::ImuDownSampler(float target_dt_sec) : _target_dt{target_dt_sec} { reset(); }

// integrate imu samples until target dt reached
// assumes that dt of the gyroscope is close to the dt of the accelerometer
// returns true if target dt is reached
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	if (_do_reset) {
		reset();
	}

	// accumulate time deltas
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
	_imu_down_sampled.time_us = imu_sample_new.time_us;
	_imu_down_sampled.delta_vel_clipping[0] += imu_sample_new.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] += imu_sample_new.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] += imu_sample_new.delta_vel_clipping[2];

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang));
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q;
	_delta_angle_accumulated.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	const Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f;

	// check if the target time delta between filter prediction steps has been exceeded
	if (_imu_down_sampled.delta_ang_dt >= _target_dt - _imu_collection_time_adj) {
		// accumulate the amount of time to advance the IMU collection time so that we meet the
		// average EKF update rate requirement
		_imu_collection_time_adj += 0.01f * (_imu_down_sampled.delta_ang_dt - _target_dt);
		_imu_collection_time_adj = math::constrain(_imu_collection_time_adj, -0.5f * _target_dt,
					   0.5f * _target_dt);

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated);

		return true;

	} else {

		return false;
	}
}

void ImuDownSampler::reset()
{
	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.delta_vel_clipping[0] = false;
	_imu_down_sampled.delta_vel_clipping[1] = false;
	_imu_down_sampled.delta_vel_clipping[2] = false;
	_delta_angle_accumulated.setIdentity();
	_do_reset = false;
}
/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file mag_control.cpp
 * Control functions for ekf magnetic field fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlMagFusion()
{
	checkMagFieldStrength();

	// If we are on ground, reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
		_num_bad_flight_yaw_events = 0;
	}

	// When operating without a magnetometer and no other source of yaw aiding is active,
	// yaw fusion is run selectively to enable yaw gyro bias learning when stationary on
	// ground and to prevent uncontrolled yaw variance growth
	// Also fuse zero heading innovation during the leveling fine alignment step to keep the yaw variance low
	if (_params.mag_fusion_type >= MAG_FUSE_TYPE_NONE
	    || _control_status.flags.mag_fault
	    || !_control_status.flags.tilt_align) {

		stopMagFusion();

		if (noOtherYawAidingThanMag()) {
			// TODO: setting _is_yaw_fusion_inhibited to true is required to tell
			// fuseHeading to perform a "zero innovation heading fusion"
			// We should refactor it to avoid using this flag here
			_is_yaw_fusion_inhibited = true;
			fuseHeading();
			_is_yaw_fusion_inhibited = false;
		}

		return;
	}

	_mag_yaw_reset_req |= otherHeadingSourcesHaveStopped();
	_mag_yaw_reset_req |= !_control_status.flags.yaw_align;
	_mag_yaw_reset_req |= _mag_inhibit_yaw_reset_req;

	if (noOtherYawAidingThanMag() && _mag_data_ready) {
		// Determine if we should use simple magnetic heading fusion which works better when
		// there are large external disturbances or the more accurate 3-axis fusion
		switch (_params.mag_fusion_type) {
		default:

		/* fallthrough */
		case MAG_FUSE_TYPE_AUTO:
			selectMagAuto();
			break;

		case MAG_FUSE_TYPE_INDOOR:

		/* fallthrough */
		case MAG_FUSE_TYPE_HEADING:
			startMagHdgFusion();
			break;

		case MAG_FUSE_TYPE_3D:
			startMag3DFusion();
			break;
		}

		if (_control_status.flags.in_air) {
			checkHaglYawResetReq();
			runInAirYawReset();
			runVelPosReset(); // TODO: review this; a vel/pos reset can be requested from COG reset (for fixedwing) only

		} else {
			runOnGroundYawReset();
		}

		if (!_control_status.flags.yaw_align) {
			// Having the yaw aligned is mandatory to continue
			return;
		}

		checkMagDeclRequired();
		checkMagInhibition();

		runMagAndMagDeclFusions();
	}
}

bool Ekf::noOtherYawAidingThanMag() const
{
	// If we are using external vision data or GPS-heading for heading then no magnetometer fusion is used
	return !_control_status.flags.ev_yaw && !_control_status.flags.gps_yaw;
}

void Ekf::checkHaglYawResetReq()
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (!_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		_mag_yaw_reset_req = _mag_yaw_reset_req || above_mag_anomalies;
	}
}

void Ekf::runOnGroundYawReset()
{
	if (_mag_yaw_reset_req && isYawResetAuthorized()) {
		const bool has_realigned_yaw = canResetMagHeading()
					       ? resetMagHeading(_mag_lpf.getState())
					       : false;

		if (has_realigned_yaw) {
			_mag_yaw_reset_req = false;
			_control_status.flags.yaw_align = true;

			// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
			// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
			if (_mag_inhibit_yaw_reset_req) {
				_mag_inhibit_yaw_reset_req = false;
				// Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
				P.uncorrelateCovarianceSetVariance<1>(12, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
			}
		}
	}
}

bool Ekf::canResetMagHeading() const
{
	return !isStrongMagneticDisturbance() && (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE);
}

void Ekf::runInAirYawReset()
{
	if (_mag_yaw_reset_req && isYawResetAuthorized()) {
		bool has_realigned_yaw = false;

		if (canRealignYawUsingGps()) {
			has_realigned_yaw = realignYawGPS();

		} else if (canResetMagHeading()) {
			has_realigned_yaw = resetMagHeading(_mag_lpf.getState());
		}

		if (has_realigned_yaw) {
			_mag_yaw_reset_req = false;
			_control_status.flags.yaw_align = true;
			_control_status.flags.mag_aligned_in_flight = true;

			// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
			// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
			if (_mag_inhibit_yaw_reset_req) {
				_mag_inhibit_yaw_reset_req = false;
				// Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
				P.uncorrelateCovarianceSetVariance<1>(12, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
			}
		}

	}
}

void Ekf::runVelPosReset()
{
	if (_velpos_reset_request) {
		resetVelocity();
		resetHorizontalPosition();
		_velpos_reset_request = false;
	}
}

void Ekf::selectMagAuto()
{
	check3DMagFusionSuitability();
	canUse3DMagFusion() ? startMag3DFusion() : startMagHdgFusion();
}

void Ekf::check3DMagFusionSuitability()
{
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (isMagBiasObservable() || isYawAngleObservable()) {
		_time_last_mov_3d_mag_suitable = _imu_sample_delayed.time_us;
	}
}

void Ekf::checkYawAngleObservability()
{
	// Check if there has been enough change in horizontal velocity to make yaw observable
	// Apply hysteresis to check to avoid rapid toggling
	_yaw_angle_observable = _yaw_angle_observable
				? _accel_lpf_NE.norm() > _params.mag_acc_gate
				: _accel_lpf_NE.norm() > 2.0f * _params.mag_acc_gate;

	_yaw_angle_observable = _yaw_angle_observable
				&& (_control_status.flags.gps || _control_status.flags.ev_pos); // Do we have to add ev_vel here?
}

void Ekf::checkMagBiasObservability()
{
	// check if there is enough yaw rotation to make the mag bias states observable
	if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
		// initial yaw motion is detected
		_mag_bias_observable = true;

	} else if (_mag_bias_observable) {
		// require sustained yaw motion of 50% the initial yaw rate threshold
		const float yaw_dt = 1e-6f * (float)(_imu_sample_delayed.time_us - _time_yaw_started);
		const float min_yaw_change_req =  0.5f * _params.mag_yaw_rate_gate * yaw_dt;
		_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;
	}

	_yaw_delta_ef = 0.0f;
	_time_yaw_started = _imu_sample_delayed.time_us;
}

bool Ekf::canUse3DMagFusion() const
{
	// Use of 3D fusion requires an in-air heading alignment but it should not
	// be used when the heading and mag biases are not observable for more than 2 seconds
	return _control_status.flags.mag_aligned_in_flight
	       && ((_imu_sample_delayed.time_us - _time_last_mov_3d_mag_suitable) < (uint64_t)2e6);
}

void Ekf::checkMagDeclRequired()
{
	// if we are using 3-axis magnetometer fusion, but without external NE aiding,
	// then the declination must be fused as an observation to prevent long term heading drift
	// fusing declination when gps aiding is available is optional, but recommended to prevent
	// problem if the vehicle is static for extended periods of time
	const bool user_selected = (_params.mag_declination_source & MASK_FUSE_DECL);
	const bool not_using_ne_aiding = !_control_status.flags.gps;
	_control_status.flags.mag_dec = (_control_status.flags.mag_3D && (not_using_ne_aiding || user_selected));
}

void Ekf::checkMagInhibition()
{
	_is_yaw_fusion_inhibited = shouldInhibitMag();

	if (!_is_yaw_fusion_inhibited) {
		_mag_use_not_inhibit_us = _imu_sample_delayed.time_us;
	}

	// If magnetometer use has been inhibited continuously then a yaw reset is required for a valid heading
	if (uint32_t(_imu_sample_delayed.time_us - _mag_use_not_inhibit_us) > (uint32_t)5e6) {
		_mag_inhibit_yaw_reset_req = true;
	}
}

bool Ekf::shouldInhibitMag() const
{
	// If the user has selected auto protection against indoor magnetic field errors, only use the magnetometer
	// if a yaw angle relative to true North is required for navigation. If no GPS or other earth frame aiding
	// is available, assume that we are operating indoors and the magnetometer should not be used.
	// Also inhibit mag fusion when a strong magnetic field interference is detected or the user
	// has explicitly stopped magnetometer use.
	const bool user_selected = (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR);

	const bool heading_not_required_for_navigation = !_control_status.flags.gps
			&& !_control_status.flags.ev_pos
			&& !_control_status.flags.ev_vel;

	return (user_selected && heading_not_required_for_navigation)
	       || isStrongMagneticDisturbance();
}

void Ekf::checkMagFieldStrength()
{
	if (_params.check_mag_strength) {
		_control_status.flags.mag_field_disturbed = _NED_origin_initialised
				? !isMeasuredMatchingGpsMagStrength()
				: !isMeasuredMatchingAverageMagStrength();

	} else {
		_control_status.flags.mag_field_disturbed = false;
	}
}

bool Ekf::isMeasuredMatchingGpsMagStrength() const
{
	constexpr float wmm_gate_size = 0.2f; // +/- Gauss
	return isMeasuredMatchingExpected(_mag_sample_delayed.mag.length(), _mag_strength_gps, wmm_gate_size);
}

bool Ekf::isMeasuredMatchingAverageMagStrength() const
{
	constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
	constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss
	return isMeasuredMatchingExpected(_mag_sample_delayed.mag.length(),
					  average_earth_mag_field_strength,
					  average_earth_mag_gate_size);
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
	       && (measured <= expected + gate);
}

void Ekf::runMagAndMagDeclFusions()
{
	if (_control_status.flags.mag_3D) {
		run3DMagAndDeclFusions();

	} else if (_control_status.flags.mag_hdg) {
		fuseHeading();
	}
}

void Ekf::run3DMagAndDeclFusions()
{
	if (!_mag_decl_cov_reset) {
		// After any magnetic field covariance reset event the earth field state
		// covariances need to be corrected to incorporate knowledge of the declination
		// before fusing magnetomer data to prevent rapid rotation of the earth field
		// states for the first few observations.
		fuseDeclination(0.02f);
		_mag_decl_cov_reset = true;
		fuseMag();

	} else {
		// The normal sequence is to fuse the magnetometer data first before fusing
		// declination angle at a higher uncertainty to allow some learning of
		// declination angle over time.
		fuseMag();

		if (_control_status.flags.mag_dec) {
			fuseDeclination(0.5f);
		}
	}
}

bool Ekf::otherHeadingSourcesHaveStopped()
{
	// detect rising edge of noOtherYawAidingThanMag()
	bool result = noOtherYawAidingThanMag() && _non_mag_yaw_aiding_running_prev;

	_non_mag_yaw_aiding_running_prev = !noOtherYawAidingThanMag();

	return  result;
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file heading_fusion.cpp
 * Magnetometer fusion methods.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::fuseMag()
{
	// assign intermediate variables
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	const float &magN = _state.mag_I(0);
	const float &magE = _state.mag_I(1);
	const float &magD = _state.mag_I(2);

	// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	const float R_MAG = sq(fmaxf(_params.mag_noise, 0.0f));

	// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
	const char* numerical_error_covariance_reset_string = "numerical error - covariance reset";
	const float HKX0 = -magD*q2 + magE*q3 + magN*q0;
	const float HKX1 = magD*q3 + magE*q2 + magN*q1;
	const float HKX2 = magE*q1;
	const float HKX3 = magD*q0;
	const float HKX4 = magN*q2;
	const float HKX5 = magD*q1 + magE*q0 - magN*q3;
	const float HKX6 = ecl::powf(q0, 2) + ecl::powf(q1, 2) - ecl::powf(q2, 2) - ecl::powf(q3, 2);
	const float HKX7 = q0*q3 + q1*q2;
	const float HKX8 = q1*q3;
	const float HKX9 = q0*q2;
	const float HKX10 = 2*HKX7;
	const float HKX11 = -2*HKX8 + 2*HKX9;
	const float HKX12 = 2*HKX1;
	const float HKX13 = 2*HKX0;
	const float HKX14 = -2*HKX2 + 2*HKX3 + 2*HKX4;
	const float HKX15 = 2*HKX5;
	const float HKX16 = HKX10*P(0,17) - HKX11*P(0,18) + HKX12*P(0,1) + HKX13*P(0,0) - HKX14*P(0,2) + HKX15*P(0,3) + HKX6*P(0,16) + P(0,19);
	const float HKX17 = HKX10*P(16,17) - HKX11*P(16,18) + HKX12*P(1,16) + HKX13*P(0,16) - HKX14*P(2,16) + HKX15*P(3,16) + HKX6*P(16,16) + P(16,19);
	const float HKX18 = HKX10*P(17,18) - HKX11*P(18,18) + HKX12*P(1,18) + HKX13*P(0,18) - HKX14*P(2,18) + HKX15*P(3,18) + HKX6*P(16,18) + P(18,19);
	const float HKX19 = HKX10*P(2,17) - HKX11*P(2,18) + HKX12*P(1,2) + HKX13*P(0,2) - HKX14*P(2,2) + HKX15*P(2,3) + HKX6*P(2,16) + P(2,19);
	const float HKX20 = HKX10*P(17,17) - HKX11*P(17,18) + HKX12*P(1,17) + HKX13*P(0,17) - HKX14*P(2,17) + HKX15*P(3,17) + HKX6*P(16,17) + P(17,19);
	const float HKX21 = HKX10*P(3,17) - HKX11*P(3,18) + HKX12*P(1,3) + HKX13*P(0,3) - HKX14*P(2,3) + HKX15*P(3,3) + HKX6*P(3,16) + P(3,19);
	const float HKX22 = HKX10*P(1,17) - HKX11*P(1,18) + HKX12*P(1,1) + HKX13*P(0,1) - HKX14*P(1,2) + HKX15*P(1,3) + HKX6*P(1,16) + P(1,19);
	const float HKX23 = HKX10*P(17,19) - HKX11*P(18,19) + HKX12*P(1,19) + HKX13*P(0,19) - HKX14*P(2,19) + HKX15*P(3,19) + HKX6*P(16,19) + P(19,19);

	_mag_innov_var(0) = HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG;

	if (_mag_innov_var(0) < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_x = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magX %s", numerical_error_covariance_reset_string);
		return;
	}

	_fault_status.flags.bad_mag_x = false;

	const float HKX24 = 1.0F/_mag_innov_var(0);

	// intermediate variables for calculation of innovations variances for Y and Z axes
	// don't calculate all terms needed for observation jacobians and Kalman gains because
	// these will have to be recalculated when the X and Y axes are fused
	const float IV0 = q0*q1;
	const float IV1 = q2*q3;
	const float IV2 = 2*IV0 + 2*IV1;
	const float IV3 = 2*q0*q3 - 2*q1*q2;
	const float IV4 = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
	const float IV5 = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
	const float IV6 = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
	const float IV7 = -2*magD*q2 + 2*magE*q3 + 2*magN*q0;
	const float IV8 = ecl::powf(q2, 2);
	const float IV9 = ecl::powf(q3, 2);
	const float IV10 = ecl::powf(q0, 2) - ecl::powf(q1, 2);
	const float IV11 = IV10 + IV8 - IV9;
	const float IV12 = IV7*P(2,3);
	const float IV13 = IV5*P(0,1);
	const float IV14 = IV6*P(0,1);
	const float IV15 = IV4*P(2,3);
	const float IV16 = 2*q0*q2 + 2*q1*q3;
	const float IV17 = 2*IV0 - 2*IV1;
	const float IV18 = IV10 - IV8 + IV9;

	_mag_innov_var(1) = IV11*P(17,20) + IV11*(IV11*P(17,17) + IV2*P(17,18) - IV3*P(16,17) + IV4*P(2,17) + IV5*P(0,17) + IV6*P(1,17) - IV7*P(3,17) + P(17,20)) + IV2*P(18,20) + IV2*(IV11*P(17,18) + IV2*P(18,18) - IV3*P(16,18) + IV4*P(2,18) + IV5*P(0,18) + IV6*P(1,18) - IV7*P(3,18) + P(18,20)) - IV3*P(16,20) - IV3*(IV11*P(16,17) + IV2*P(16,18) - IV3*P(16,16) + IV4*P(2,16) + IV5*P(0,16) + IV6*P(1,16) - IV7*P(3,16) + P(16,20)) + IV4*P(2,20) + IV4*(IV11*P(2,17) - IV12 + IV2*P(2,18) - IV3*P(2,16) + IV4*P(2,2) + IV5*P(0,2) + IV6*P(1,2) + P(2,20)) + IV5*P(0,20) + IV5*(IV11*P(0,17) + IV14 + IV2*P(0,18) - IV3*P(0,16) + IV4*P(0,2) + IV5*P(0,0) - IV7*P(0,3) + P(0,20)) + IV6*P(1,20) + IV6*(IV11*P(1,17) + IV13 + IV2*P(1,18) - IV3*P(1,16) + IV4*P(1,2) + IV6*P(1,1) - IV7*P(1,3) + P(1,20)) - IV7*P(3,20) - IV7*(IV11*P(3,17) + IV15 + IV2*P(3,18) - IV3*P(3,16) + IV5*P(0,3) + IV6*P(1,3) - IV7*P(3,3) + P(3,20)) + P(20,20) + R_MAG;
	_mag_innov_var(2) = IV16*P(16,21) + IV16*(IV16*P(16,16) - IV17*P(16,17) + IV18*P(16,18) + IV4*P(3,16) - IV5*P(1,16) + IV6*P(0,16) + IV7*P(2,16) + P(16,21)) - IV17*P(17,21) - IV17*(IV16*P(16,17) - IV17*P(17,17) + IV18*P(17,18) + IV4*P(3,17) - IV5*P(1,17) + IV6*P(0,17) + IV7*P(2,17) + P(17,21)) + IV18*P(18,21) + IV18*(IV16*P(16,18) - IV17*P(17,18) + IV18*P(18,18) + IV4*P(3,18) - IV5*P(1,18) + IV6*P(0,18) + IV7*P(2,18) + P(18,21)) + IV4*P(3,21) + IV4*(IV12 + IV16*P(3,16) - IV17*P(3,17) + IV18*P(3,18) + IV4*P(3,3) - IV5*P(1,3) + IV6*P(0,3) + P(3,21)) - IV5*P(1,21) - IV5*(IV14 + IV16*P(1,16) - IV17*P(1,17) + IV18*P(1,18) + IV4*P(1,3) - IV5*P(1,1) + IV7*P(1,2) + P(1,21)) + IV6*P(0,21) + IV6*(-IV13 + IV16*P(0,16) - IV17*P(0,17) + IV18*P(0,18) + IV4*P(0,3) + IV6*P(0,0) + IV7*P(0,2) + P(0,21)) + IV7*P(2,21) + IV7*(IV15 + IV16*P(2,16) - IV17*P(2,17) + IV18*P(2,18) - IV5*P(1,2) + IV6*P(0,2) + IV7*P(2,2) + P(2,21)) + P(21,21) + R_MAG;

	// chedk innovation variances for being badly conditioned

	if (_mag_innov_var(1) < R_MAG) {
		// the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_y = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magY %s", numerical_error_covariance_reset_string);
		return;
	}

	_fault_status.flags.bad_mag_y = false;

	if (_mag_innov_var(2) < R_MAG) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_mag_z = true;

		// we need to re-initialise covariances and abort this fusion step
		resetMagRelatedCovariances();
		ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
		return;
	}

	_fault_status.flags.bad_mag_z = false;

	// rotate magnetometer earth field state into body frame
	const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);

	const Vector3f mag_I_rot = R_to_body * _state.mag_I;

	// compute magnetometer innovations
	_mag_innov = mag_I_rot + _state.mag_B - _mag_sample_delayed.mag;

	// do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (_control_status.flags.synthetic_mag_z) {
		_mag_innov(2) = 0.0f;
	}

	// Perform an innovation consistency check and report the result
	bool all_innovation_checks_passed = true;

	for (uint8_t index = 0; index <= 2; index++) {
		_mag_test_ratio(index) = sq(_mag_innov(index)) / (sq(math::max(_params.mag_innov_gate, 1.0f)) * _mag_innov_var(index));

		if (_mag_test_ratio(index) > 1.0f) {
			all_innovation_checks_passed = false;
			_innov_check_fail_status.value |= (1 << (index + 3));

		} else {
			_innov_check_fail_status.value &= ~(1 << (index + 3));
		}
	}

	// we are no longer using heading fusion so set the reported test level to zero
	_yaw_test_ratio = 0.0f;

	// if any axis fails, abort the mag fusion
	if (!all_innovation_checks_passed) {
		return;
	}

	// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
	// before they are used to constrain heading drift
	const bool update_all_states = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) > (uint64_t)5e6);

	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3,16,17,18,19,20,21> Hfusion;
	Vector24f Kfusion;

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {

		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// Calculate X axis observation jacobians
			Hfusion.at<0>() = 2*HKX0;
			Hfusion.at<1>() = 2*HKX1;
			Hfusion.at<2>() = 2*HKX2 - 2*HKX3 - 2*HKX4;
			Hfusion.at<3>() = 2*HKX5;
			Hfusion.at<16>() = HKX6;
			Hfusion.at<17>() = 2*HKX7;
			Hfusion.at<18>() = 2*HKX8 - 2*HKX9;
			Hfusion.at<19>() = 1;

			// Calculate X axis Kalman gains
			if (update_all_states) {
				Kfusion(0) = HKX16*HKX24;
				Kfusion(1) = HKX22*HKX24;
				Kfusion(2) = HKX19*HKX24;
				Kfusion(3) = HKX21*HKX24;

				for (unsigned row = 4; row <= 15; row++) {
					Kfusion(row) = HKX24*(HKX10*P(row,17) - HKX11*P(row,18) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(row,16) + P(row,19));
				}

				for (unsigned row = 22; row <= 23; row++) {
					Kfusion(row) = HKX24*(HKX10*P(17,row) - HKX11*P(18,row) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(16,row) + P(19,row));
				}
			}

			Kfusion(16) = HKX17*HKX24;
			Kfusion(17) = HKX20*HKX24;
			Kfusion(18) = HKX18*HKX24;
			Kfusion(19) = HKX23*HKX24;

			for (unsigned row = 20; row <= 21; row++) {
				Kfusion(row) = HKX24*(HKX10*P(17,row) - HKX11*P(18,row) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(16,row) + P(19,row));
			}

		} else if (index == 1) {

			// recalculate innovation variance becasue states and covariances have changed due to previous fusion
			const float HKY0 = magD*q1 + magE*q0 - magN*q3;
			const float HKY1 = magD*q0 - magE*q1 + magN*q2;
			const float HKY2 = magD*q3 + magE*q2 + magN*q1;
			const float HKY3 = magD*q2;
			const float HKY4 = magE*q3;
			const float HKY5 = magN*q0;
			const float HKY6 = q1*q2;
			const float HKY7 = q0*q3;
			const float HKY8 = ecl::powf(q0, 2) - ecl::powf(q1, 2) + ecl::powf(q2, 2) - ecl::powf(q3, 2);
			const float HKY9 = q0*q1 + q2*q3;
			const float HKY10 = 2*HKY9;
			const float HKY11 = -2*HKY6 + 2*HKY7;
			const float HKY12 = 2*HKY2;
			const float HKY13 = 2*HKY0;
			const float HKY14 = 2*HKY1;
			const float HKY15 = -2*HKY3 + 2*HKY4 + 2*HKY5;
			const float HKY16 = HKY10*P(0,18) - HKY11*P(0,16) + HKY12*P(0,2) + HKY13*P(0,0) + HKY14*P(0,1) - HKY15*P(0,3) + HKY8*P(0,17) + P(0,20);
			const float HKY17 = HKY10*P(17,18) - HKY11*P(16,17) + HKY12*P(2,17) + HKY13*P(0,17) + HKY14*P(1,17) - HKY15*P(3,17) + HKY8*P(17,17) + P(17,20);
			const float HKY18 = HKY10*P(16,18) - HKY11*P(16,16) + HKY12*P(2,16) + HKY13*P(0,16) + HKY14*P(1,16) - HKY15*P(3,16) + HKY8*P(16,17) + P(16,20);
			const float HKY19 = HKY10*P(3,18) - HKY11*P(3,16) + HKY12*P(2,3) + HKY13*P(0,3) + HKY14*P(1,3) - HKY15*P(3,3) + HKY8*P(3,17) + P(3,20);
			const float HKY20 = HKY10*P(18,18) - HKY11*P(16,18) + HKY12*P(2,18) + HKY13*P(0,18) + HKY14*P(1,18) - HKY15*P(3,18) + HKY8*P(17,18) + P(18,20);
			const float HKY21 = HKY10*P(1,18) - HKY11*P(1,16) + HKY12*P(1,2) + HKY13*P(0,1) + HKY14*P(1,1) - HKY15*P(1,3) + HKY8*P(1,17) + P(1,20);
			const float HKY22 = HKY10*P(2,18) - HKY11*P(2,16) + HKY12*P(2,2) + HKY13*P(0,2) + HKY14*P(1,2) - HKY15*P(2,3) + HKY8*P(2,17) + P(2,20);
			const float HKY23 = HKY10*P(18,20) - HKY11*P(16,20) + HKY12*P(2,20) + HKY13*P(0,20) + HKY14*P(1,20) - HKY15*P(3,20) + HKY8*P(17,20) + P(20,20);

			_mag_innov_var(1) = (HKY10*HKY20 - HKY11*HKY18 + HKY12*HKY22 + HKY13*HKY16 + HKY14*HKY21 - HKY15*HKY19 + HKY17*HKY8 + HKY23 + R_MAG);

			if (_mag_innov_var(1) < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_y = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magY %s", numerical_error_covariance_reset_string);
				return;
			}
			const float HKY24 = 1.0F/_mag_innov_var(1);

			// Calculate Y axis observation jacobians
			Hfusion.setZero();
			Hfusion.at<0>() = 2*HKY0;
			Hfusion.at<1>() = 2*HKY1;
			Hfusion.at<2>() = 2*HKY2;
			Hfusion.at<3>() = 2*HKY3 - 2*HKY4 - 2*HKY5;
			Hfusion.at<16>() = 2*HKY6 - 2*HKY7;
			Hfusion.at<17>() = HKY8;
			Hfusion.at<18>() = 2*HKY9;
			Hfusion.at<20>() = 1;

			// Calculate Y axis Kalman gains
			if (update_all_states) {
				Kfusion(0) = HKY16*HKY24;
				Kfusion(1) = HKY21*HKY24;
				Kfusion(2) = HKY22*HKY24;
				Kfusion(3) = HKY19*HKY24;

				for (unsigned row = 4; row <= 15; row++) {
					Kfusion(row) = HKY24*(HKY10*P(row,18) - HKY11*P(row,16) + HKY12*P(2,row) + HKY13*P(0,row) + HKY14*P(1,row) - HKY15*P(3,row) + HKY8*P(row,17) + P(row,20));
				}

				for (unsigned row = 22; row <= 23; row++) {
					Kfusion(row) = HKY24*(HKY10*P(18,row) - HKY11*P(16,row) + HKY12*P(2,row) + HKY13*P(0,row) + HKY14*P(1,row) - HKY15*P(3,row) + HKY8*P(17,row) + P(20,row));
				}
			}

			Kfusion(16) = HKY18*HKY24;
			Kfusion(17) = HKY17*HKY24;
			Kfusion(18) = HKY20*HKY24;
			Kfusion(19) = HKY24*(HKY10*P(18,19) - HKY11*P(16,19) + HKY12*P(2,19) + HKY13*P(0,19) + HKY14*P(1,19) - HKY15*P(3,19) + HKY8*P(17,19) + P(19,20));
			Kfusion(20) = HKY23*HKY24;
			Kfusion(21) = HKY24*(HKY10*P(18,21) - HKY11*P(16,21) + HKY12*P(2,21) + HKY13*P(0,21) + HKY14*P(1,21) - HKY15*P(3,21) + HKY8*P(17,21) + P(20,21));

		} else if (index == 2) {

			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				continue;
			}

			// recalculate innovation variance becasue states and covariances have changed due to previous fusion
			const float HKZ0 = magD*q0 - magE*q1 + magN*q2;
			const float HKZ1 = magN*q3;
			const float HKZ2 = magD*q1;
			const float HKZ3 = magE*q0;
			const float HKZ4 = -magD*q2 + magE*q3 + magN*q0;
			const float HKZ5 = magD*q3 + magE*q2 + magN*q1;
			const float HKZ6 = q0*q2 + q1*q3;
			const float HKZ7 = q2*q3;
			const float HKZ8 = q0*q1;
			const float HKZ9 = ecl::powf(q0, 2) - ecl::powf(q1, 2) - ecl::powf(q2, 2) + ecl::powf(q3, 2);
			const float HKZ10 = 2*HKZ6;
			const float HKZ11 = -2*HKZ7 + 2*HKZ8;
			const float HKZ12 = 2*HKZ5;
			const float HKZ13 = 2*HKZ0;
			const float HKZ14 = -2*HKZ1 + 2*HKZ2 + 2*HKZ3;
			const float HKZ15 = 2*HKZ4;
			const float HKZ16 = HKZ10*P(0,16) - HKZ11*P(0,17) + HKZ12*P(0,3) + HKZ13*P(0,0) - HKZ14*P(0,1) + HKZ15*P(0,2) + HKZ9*P(0,18) + P(0,21);
			const float HKZ17 = HKZ10*P(16,18) - HKZ11*P(17,18) + HKZ12*P(3,18) + HKZ13*P(0,18) - HKZ14*P(1,18) + HKZ15*P(2,18) + HKZ9*P(18,18) + P(18,21);
			const float HKZ18 = HKZ10*P(16,17) - HKZ11*P(17,17) + HKZ12*P(3,17) + HKZ13*P(0,17) - HKZ14*P(1,17) + HKZ15*P(2,17) + HKZ9*P(17,18) + P(17,21);
			const float HKZ19 = HKZ10*P(1,16) - HKZ11*P(1,17) + HKZ12*P(1,3) + HKZ13*P(0,1) - HKZ14*P(1,1) + HKZ15*P(1,2) + HKZ9*P(1,18) + P(1,21);
			const float HKZ20 = HKZ10*P(16,16) - HKZ11*P(16,17) + HKZ12*P(3,16) + HKZ13*P(0,16) - HKZ14*P(1,16) + HKZ15*P(2,16) + HKZ9*P(16,18) + P(16,21);
			const float HKZ21 = HKZ10*P(3,16) - HKZ11*P(3,17) + HKZ12*P(3,3) + HKZ13*P(0,3) - HKZ14*P(1,3) + HKZ15*P(2,3) + HKZ9*P(3,18) + P(3,21);
			const float HKZ22 = HKZ10*P(2,16) - HKZ11*P(2,17) + HKZ12*P(2,3) + HKZ13*P(0,2) - HKZ14*P(1,2) + HKZ15*P(2,2) + HKZ9*P(2,18) + P(2,21);
			const float HKZ23 = HKZ10*P(16,21) - HKZ11*P(17,21) + HKZ12*P(3,21) + HKZ13*P(0,21) - HKZ14*P(1,21) + HKZ15*P(2,21) + HKZ9*P(18,21) + P(21,21);

			_mag_innov_var(2) = (HKZ10*HKZ20 - HKZ11*HKZ18 + HKZ12*HKZ21 + HKZ13*HKZ16 - HKZ14*HKZ19 + HKZ15*HKZ22 + HKZ17*HKZ9 + HKZ23 + R_MAG);

			if (_mag_innov_var(2) < R_MAG) {
				// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				_fault_status.flags.bad_mag_z = true;

				// we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
				ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
				return;
			}

			const float HKZ24 = 1.0F/_mag_innov_var(2);

			// calculate Z axis observation jacobians
			Hfusion.setZero();
			Hfusion.at<0>() = 2*HKZ0;
			Hfusion.at<1>() = 2*HKZ1 - 2*HKZ2 - 2*HKZ3;
			Hfusion.at<2>() = 2*HKZ4;
			Hfusion.at<3>() = 2*HKZ5;
			Hfusion.at<16>() = 2*HKZ6;
			Hfusion.at<17>() = 2*HKZ7 - 2*HKZ8;
			Hfusion.at<18>() = HKZ9;
			Hfusion.at<21>() = 1;

			// Calculate Z axis Kalman gains
			if (update_all_states) {
				Kfusion(0) = HKZ16*HKZ24;
				Kfusion(1) = HKZ19*HKZ24;
				Kfusion(2) = HKZ22*HKZ24;
				Kfusion(3) = HKZ21*HKZ24;

				for (unsigned row = 4; row <= 15; row++) {
					Kfusion(row) = HKZ24*(HKZ10*P(row,16) - HKZ11*P(row,17) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(row,18) + P(row,21));
				}

				for (unsigned row = 22; row <= 23; row++) {
					Kfusion(row) = HKZ24*(HKZ10*P(16,row) - HKZ11*P(17,row) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(18,row) + P(21,row));
				}
			}

			Kfusion(16) = HKZ20*HKZ24;
			Kfusion(17) = HKZ18*HKZ24;
			Kfusion(18) = HKZ17*HKZ24;

			for (unsigned row = 19; row <= 20; row++) {
				Kfusion(row) = HKZ24*(HKZ10*P(16,row) - HKZ11*P(17,row) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(18,row) + P(row,21));
			}

			Kfusion(21) = HKZ23*HKZ24;
		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _mag_innov(index));

		if (index == 0) {
			_fault_status.flags.bad_mag_x = !is_fused;

		} else if (index == 1) {
			_fault_status.flags.bad_mag_y = !is_fused;

		} else if (index == 2) {
			_fault_status.flags.bad_mag_z = !is_fused;
		}

		if (is_fused) {
			limitDeclination();
		}
	}
}

void Ekf::fuseYaw321(float yaw, float yaw_variance, bool zero_innovation)
{
	// assign intermediate state variables
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	const float R_YAW = fmaxf(yaw_variance, 1.0e-4f);
	const float measurement = wrap_pi(yaw);

	// calculate 321 yaw observation matrix
	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	const float SA0 = 2*q3;
	const float SA1 = 2*q2;
	const float SA2 = SA0*q0 + SA1*q1;
	const float SA3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	float SA4, SA5_inv;

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.0F/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	bool canUseB = false;
	const float SB0 = 2*q0;
	const float SB1 = 2*q1;
	const float SB2 = SB0*q3 + SB1*q2;
	const float SB4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
	float SB3, SB5_inv;

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.0F/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.0F/SA5_inv;
		const float SA6 = 1.0F/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
		H_YAW(1) = SA5*(SA1*SA6 - SA8*q1);
		H_YAW(2) = SA5*(SA1*SA7 + SA9*q1);
		H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.0F/SB5_inv;
		const float SB6 = 1.0F/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		H_YAW(0) = -SB5*(SB0*SB6 - SB8*q3);
		H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
		H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
		H_YAW(3) = -SB5*(-SB0*SB7 - SB9*q3);
	} else {
		return;
	}

	// calculate the yaw innovation and wrap to the interval between +-pi
	float innovation;

	if (zero_innovation) {
		innovation = 0.0f;

	} else {
		innovation = wrap_pi(atan2f(_R_to_earth(1, 0), _R_to_earth(0, 0)) - measurement);
	}

	// define the innovation gate size
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);

	// Update the quaternion states and covariance matrix
	updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

void Ekf::fuseYaw312(float yaw, float yaw_variance, bool zero_innovation)
{
	// assign intermediate state variables
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	const float R_YAW = fmaxf(yaw_variance, 1.0e-4f);
	const float measurement = wrap_pi(yaw);

	// calculate 312 yaw observation matrix
	// choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	bool canUseA = false;
	const float SA0 = 2*q3;
	const float SA1 = 2*q2;
	const float SA2 = SA0*q0 - SA1*q1;
	const float SA3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
	float SA4, SA5_inv;

	if (sq(SA3) > 1e-6f) {
		SA4 = 1.0F/sq(SA3);
		SA5_inv = sq(SA2)*SA4 + 1;
		canUseA = fabsf(SA5_inv) > 1e-6f;
	}

	bool canUseB = false;
	const float SB0 = 2*q0;
	const float SB1 = 2*q1;
	const float SB2 = -SB0*q3 + SB1*q2;
	const float SB4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
	float SB3, SB5_inv;

	if (sq(SB2) > 1e-6f) {
		SB3 = 1.0F/sq(SB2);
		SB5_inv = SB3*sq(SB4) + 1;
		canUseB = fabsf(SB5_inv) > 1e-6f;
	}

	Vector4f H_YAW;

	if (canUseA && (!canUseB || fabsf(SA5_inv) >= fabsf(SB5_inv))) {
		const float SA5 = 1.0F/SA5_inv;
		const float SA6 = 1.0F/SA3;
		const float SA7 = SA2*SA4;
		const float SA8 = 2*SA7;
		const float SA9 = 2*SA6;

		H_YAW(0) = SA5*(SA0*SA6 - SA8*q0);
		H_YAW(1) = SA5*(-SA1*SA6 + SA8*q1);
		H_YAW(2) = SA5*(-SA1*SA7 - SA9*q1);
		H_YAW(3) = SA5*(SA0*SA7 + SA9*q0);
	} else if (canUseB && (!canUseA || fabsf(SB5_inv) > fabsf(SA5_inv))) {
		const float SB5 = 1.0F/SB5_inv;
		const float SB6 = 1.0F/SB2;
		const float SB7 = SB3*SB4;
		const float SB8 = 2*SB7;
		const float SB9 = 2*SB6;

		H_YAW(0) = -SB5*(-SB0*SB6 + SB8*q3);
		H_YAW(1) = -SB5*(SB1*SB6 - SB8*q2);
		H_YAW(2) = -SB5*(-SB1*SB7 - SB9*q2);
		H_YAW(3) = -SB5*(SB0*SB7 + SB9*q3);
	} else {
		return;
	}

	float innovation;

	if (zero_innovation) {
		innovation = 0.0f;

	} else {
		// calculate the the innovation and wrap to the interval between +-pi
		innovation = wrap_pi(atan2f(-_R_to_earth(0, 1), _R_to_earth(1, 1)) - measurement);
	}

	// define the innovation gate size
	float innov_gate = math::max(_params.heading_innov_gate, 1.0f);

	// Update the quaternion states and covariance matrix
	updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);
}

// update quaternion states and covariances using the yaw innovation, yaw observation variance and yaw Jacobian
void Ekf::updateQuaternion(const float innovation, const float variance, const float gate_sigma,
			   const Vector4f &yaw_jacobian)
{
	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
	// calculate the innovation variance
	_heading_innov_var = variance;

	for (unsigned row = 0; row <= 3; row++) {
		float tmp = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			tmp += P(row, col) * yaw_jacobian(col);
		}

		_heading_innov_var += yaw_jacobian(row) * tmp;
	}

	float heading_innov_var_inv;

	// check if the innovation variance calculation is badly conditioned
	if (_heading_innov_var >= variance) {
		// the innovation variance contribution from the state covariances is not negative, no fault
		_fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.0f / _heading_innov_var;

	} else {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("mag yaw fusion numerical error - covariance reset");
		return;
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Vector24f Kfusion;

	for (uint8_t row = 0; row <= 15; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * yaw_jacobian(col);
		}

		Kfusion(row) *= heading_innov_var_inv;
	}

	if (_control_status.flags.wind) {
		for (uint8_t row = 22; row <= 23; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row, col) * yaw_jacobian(col);
			}

			Kfusion(row) *= heading_innov_var_inv;
		}
	}

	// innovation test ratio
	_yaw_test_ratio = sq(innovation) / (sq(gate_sigma) * _heading_innov_var);

	// we are no longer using 3-axis fusion so set the reported test levels to zero
	_mag_test_ratio.setZero();

	// set the magnetometer unhealthy if the test fails
	if (_yaw_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// if we are in air we don't want to fuse the measurement
		// we allow to use it when on the ground because the large innovation could be caused
		// by interference or a large initial gyro bias
		if (!_control_status.flags.in_air && isTimedOut(_time_last_in_air, (uint64_t)5e6)) {
			// constrain the innovation to the maximum set by the gate
			// we need to delay this forced fusion to avoid starting it
			// immediately after touchdown, when the drone is still armed
			float gate_limit = sqrtf((sq(gate_sigma) * _heading_innov_var));
			_heading_innov = math::constrain(innovation, -gate_limit, gate_limit);

			// also reset the yaw gyro variance to converge faster and avoid
			// being stuck on a previous bad estimate
			resetZDeltaAngBiasCov();

		} else {
			return;
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;
		_heading_innov = innovation;
	}

	// apply covariance correction via P_new = (I -K*H)*P
	// first calculate expression for KHP
	// then calculate P - KHP
	SquareMatrix24f KHP;
	float KH[4];

	for (unsigned row = 0; row < _k_num_states; row++) {

		KH[0] = Kfusion(row) * yaw_jacobian(0);
		KH[1] = Kfusion(row) * yaw_jacobian(1);
		KH[2] = Kfusion(row) * yaw_jacobian(2);
		KH[3] = Kfusion(row) * yaw_jacobian(3);

		for (unsigned column = 0; column < _k_num_states; column++) {
			float tmp = KH[0] * P(0, column);
			tmp += KH[1] * P(1, column);
			tmp += KH[2] * P(2, column);
			tmp += KH[3] * P(3, column);
			KHP(row, column) = tmp;
		}
	}

	const bool healthy = checkAndFixCovarianceUpdate(KHP);

	_fault_status.flags.bad_hdg = !healthy;

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, _heading_innov);

	}
}

void Ekf::fuseHeading()
{
	Vector3f mag_earth_pred;
	float measured_hdg;

	// Calculate the observation variance
	float R_YAW;

	if (_control_status.flags.mag_hdg) {
		// using magnetic heading tuning parameter
		R_YAW = sq(_params.mag_heading_noise);

	} else if (_control_status.flags.ev_yaw) {
		// using error estimate from external vision data
		R_YAW = _ev_sample_delayed.angVar;

	} else {
		// default value
		R_YAW = 0.01f;
	}

	// update transformation matrix from body to world frame using the current state estimate
	_R_to_earth = Dcmf(_state.quat_nominal);

	if (shouldUse321RotationSequence(_R_to_earth)) {
		const float predicted_hdg = getEuler321Yaw(_R_to_earth);

		if (_control_status.flags.mag_hdg) {
			// Rotate the measurements into earth frame using the zero yaw angle
			const Dcmf R_to_earth = updateEuler321YawInRotMat(0.f, _R_to_earth);
			mag_earth_pred = R_to_earth * (_mag_sample_delayed.mag - _state.mag_B);

			// the angle of the projection onto the horizontal gives the yaw angle
			measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		} else if (_control_status.flags.ev_yaw) {
			measured_hdg = getEuler321Yaw(_ev_sample_delayed.quat);

		} else {
			measured_hdg = predicted_hdg;
		}

		// handle special case where yaw measurement is unavailable
		bool fuse_zero_innov = false;

		if (_is_yaw_fusion_inhibited) {
			// The yaw measurement cannot be trusted but we need to fuse something to prevent a badly
			// conditioned covariance matrix developing over time.
			if (!_control_status.flags.vehicle_at_rest) {
				// Vehicle is not at rest so fuse a zero innovation if necessary to prevent
				// unconstrained quaternion variance growth and record the predicted heading
				// to use as an observation when movement ceases.
				// TODO a better way of determining when this is necessary
				const float sumQuatVar = P(0, 0) + P(1, 1) + P(2, 2) + P(3, 3);

				if (sumQuatVar > _params.quat_max_variance) {
					fuse_zero_innov = true;
					R_YAW = 0.25f;

				}

				_last_static_yaw = predicted_hdg;

			} else {
				// Vehicle is at rest so use the last moving prediction as an observation
				// to prevent the heading from drifting and to enable yaw gyro bias learning
				// before takeoff.

				if (!PX4_ISFINITE(_last_static_yaw)) {
					_last_static_yaw = predicted_hdg;
				}

				measured_hdg = _last_static_yaw;

			}

		} else {
			_last_static_yaw = predicted_hdg;

		}

		fuseYaw321(measured_hdg, R_YAW, fuse_zero_innov);

	} else {
		const float predicted_hdg = getEuler312Yaw(_R_to_earth);

		if (_control_status.flags.mag_hdg) {

			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			const Dcmf R_to_earth = updateEuler312YawInRotMat(0.f, _R_to_earth);
			mag_earth_pred = R_to_earth * (_mag_sample_delayed.mag - _state.mag_B);

			// the angle of the projection onto the horizontal gives the yaw angle
			measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		} else if (_control_status.flags.ev_yaw) {
			measured_hdg = getEuler312Yaw(_ev_sample_delayed.quat);

		} else {
			measured_hdg = predicted_hdg;
		}

		// handle special case where yaw measurement is unavailable
		bool fuse_zero_innov = false;

		if (_is_yaw_fusion_inhibited) {
			// The yaw measurement cannot be trusted but we need to fuse something to prevent a badly
			// conditioned covariance matrix developing over time.
			if (!_control_status.flags.vehicle_at_rest) {
				// Vehicle is not at rest so fuse a zero innovation if necessary to prevent
				// unconstrained quaterniion variance growth and record the predicted heading
				// to use as an observation when movement ceases.
				// TODO a better way of determining when this is necessary
				const float sumQuatVar = P(0, 0) + P(1, 1) + P(2, 2) + P(3, 3);

				if (sumQuatVar > _params.quat_max_variance) {
					fuse_zero_innov = true;
					R_YAW = 0.25f;

				}

				_last_static_yaw = predicted_hdg;

			} else {
				// Vehicle is at rest so use the last moving prediction as an observation
				// to prevent the heading from drifting and to enable yaw gyro bias learning
				// before takeoff.

				if (!PX4_ISFINITE(_last_static_yaw)) {
					_last_static_yaw = predicted_hdg;
				}

				measured_hdg = _last_static_yaw;

			}

		} else {
			_last_static_yaw = predicted_hdg;

		}

		fuseYaw312(measured_hdg, R_YAW, fuse_zero_innov);

	}
}

void Ekf::fuseDeclination(float decl_sigma)
{
	// assign intermediate state variables
	const float &magN = _state.mag_I(0);
	const float &magE = _state.mag_I(1);

	// minimum North field strength before calculation becomes badly conditioned (T)
	constexpr float N_field_min = 0.001f;

	// observation variance (rad**2)
	const float R_DECL = sq(decl_sigma);

	// Calculate intermediate variables
	if (fabsf(magN) < sq(N_field_min)) {
		// calculation is badly conditioned close to +-90 deg declination
		return;
	}

	const float HK0 = ecl::powf(magN, -2);
	const float HK1 = HK0*ecl::powf(magE, 2) + 1.0F;
	const float HK2 = 1.0F/HK1;
	const float HK3 = 1.0F/magN;
	const float HK4 = HK2*HK3;
	const float HK5 = HK3*magE;
	const float HK6 = HK5*P(16,17) - P(17,17);
	const float HK7 = ecl::powf(HK1, -2);
	const float HK8 = HK5*P(16,16) - P(16,17);
	const float innovation_variance = -HK0*HK6*HK7 + HK7*HK8*magE/ecl::powf(magN, 3) + R_DECL;
	float HK9;

	if (innovation_variance > R_DECL) {
		HK9 = HK4/innovation_variance;
	} else {
		// variance calculation is badly conditioned
		return;
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	// Note Hfusion indices do not match state indices
	SparseVector24f<16,17> Hfusion;
	Hfusion.at<16>() = -HK0*HK2*magE;
	Hfusion.at<17>() = HK4;

	// Calculate the Kalman gains
	Vector24f Kfusion;

	for (unsigned row = 0; row <= 15; row++) {
		Kfusion(row) = -HK9*(HK5*P(row,16) - P(row,17));
	}

	Kfusion(16) = -HK8*HK9;
	Kfusion(17) = -HK6*HK9;

	for (unsigned row = 18; row <= 23; row++) {
		Kfusion(row) = -HK9*(HK5*P(16,row) - P(17,row));
	}

	const float innovation = math::constrain(atan2f(magE, magN) - getMagDeclination(), -0.5f, 0.5f);

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, innovation);

	_fault_status.flags.bad_mag_decl = !is_fused;

	if (is_fused) {
		limitDeclination();
	}
}

void Ekf::limitDeclination()
{
	// get a reference value for the earth field declinaton and minimum plausible horizontal field strength
	// set to 50% of the horizontal strength from geo tables if location is known
	float decl_reference;
	float h_field_min = 0.001f;

	if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			decl_reference = _mag_declination_gps;
			h_field_min = fmaxf(h_field_min, 0.5f * _mag_strength_gps * cosf(_mag_inclination_gps));

		} else {
			decl_reference = math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		decl_reference = math::radians(_params.mag_declination_deg);
	}

	// do not allow the horizontal field length to collapse - this will make the declination fusion badly conditioned
	// and can result in a reversal of the NE field states which the filter cannot recover from
	// apply a circular limit
	float h_field = sqrtf(_state.mag_I(0) * _state.mag_I(0) + _state.mag_I(1) * _state.mag_I(1));

	if (h_field < h_field_min) {
		if (h_field > 0.001f * h_field_min) {
			const float h_scaler = h_field_min / h_field;
			_state.mag_I(0) *= h_scaler;
			_state.mag_I(1) *= h_scaler;

		} else {
			// too small to scale radially so set to expected value
			const float mag_declination = getMagDeclination();
			_state.mag_I(0) = 2.0f * h_field_min * cosf(mag_declination);
			_state.mag_I(1) = 2.0f * h_field_min * sinf(mag_declination);
		}

		h_field = h_field_min;
	}

	// do not allow the declination estimate to vary too much relative to the reference value
	constexpr float decl_tolerance = 0.5f;
	const float decl_max = decl_reference + decl_tolerance;
	const float decl_min = decl_reference - decl_tolerance;
	const float decl_estimate = atan2f(_state.mag_I(1), _state.mag_I(0));

	if (decl_estimate > decl_max)  {
		_state.mag_I(0) = h_field * cosf(decl_max);
		_state.mag_I(1) = h_field * sinf(decl_max);

	} else if (decl_estimate < decl_min)  {
		_state.mag_I(0) = h_field * cosf(decl_min);
		_state.mag_I(1) = h_field * sinf(decl_min);
	}
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetomer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file vel_pos_fusion.cpp
 * Function for fusing gps and baro measurements/
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include "utils.hpp"

void Ekf::fuseOptFlow()
{
	float gndclearance = fmaxf(_params.rng_gnd_clearance, 0.1f);

	// get latest estimated orientation
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float vn = _state.vel(0);
	const float ve = _state.vel(1);
	const float vd = _state.vel(2);

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar();

	// get rotation matrix from earth to body
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	const Vector3f vel_body = earth_to_body * vel_rel_earth;

	// height above ground of the IMU
	float heightAboveGndEst = _terrain_vpos - _state.pos(2);

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	heightAboveGndEst -= pos_offset_earth(2);

	// constrain minimum height above ground
	heightAboveGndEst = math::max(heightAboveGndEst, gndclearance);

	// calculate range from focal point to centre of image
	const float range = heightAboveGndEst / earth_to_body(2, 2); // absolute distance to the frame region in view

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

	// compute the velocities in body and local frames from corrected optical flow measurement
	// for logging only
	_flow_vel_body(0) = -opt_flow_rate(1) * range;
	_flow_vel_body(1) = opt_flow_rate(0) * range;
	_flow_vel_ne = Vector2f(_R_to_earth * Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));

	_flow_innov(0) =  vel_body(1) / range - opt_flow_rate(0); // flow around the X axis
	_flow_innov(1) = -vel_body(0) / range - opt_flow_rate(1); // flow around the Y axis

	// The derivation allows for an arbitrary body to flow sensor frame rotation which is
	// currently not supported by the EKF, so assume sensor frame is aligned with the
	// body frame
	const Dcmf Tbs = matrix::eye<float, 3>();

	// Sub Expressions
	const float HK0 = -Tbs(1,0)*q2 + Tbs(1,1)*q1 + Tbs(1,2)*q0;
	const float HK1 = Tbs(1,0)*q3 + Tbs(1,1)*q0 - Tbs(1,2)*q1;
	const float HK2 = Tbs(1,0)*q0 - Tbs(1,1)*q3 + Tbs(1,2)*q2;
	const float HK3 = HK0*vd + HK1*ve + HK2*vn;
	const float HK4 = 1.0F/range;
	const float HK5 = 2*HK4;
	const float HK6 = Tbs(1,0)*q1 + Tbs(1,1)*q2 + Tbs(1,2)*q3;
	const float HK7 = -HK0*ve + HK1*vd + HK6*vn;
	const float HK8 = HK0*vn - HK2*vd + HK6*ve;
	const float HK9 = -HK1*vn + HK2*ve + HK6*vd;
	const float HK10 = q0*q2;
	const float HK11 = q1*q3;
	const float HK12 = HK10 + HK11;
	const float HK13 = 2*Tbs(1,2);
	const float HK14 = q0*q3;
	const float HK15 = q1*q2;
	const float HK16 = HK14 - HK15;
	const float HK17 = 2*Tbs(1,1);
	const float HK18 = ecl::powf(q1, 2);
	const float HK19 = ecl::powf(q2, 2);
	const float HK20 = -HK19;
	const float HK21 = ecl::powf(q0, 2);
	const float HK22 = ecl::powf(q3, 2);
	const float HK23 = HK21 - HK22;
	const float HK24 = HK18 + HK20 + HK23;
	const float HK25 = HK12*HK13 - HK16*HK17 + HK24*Tbs(1,0);
	const float HK26 = HK14 + HK15;
	const float HK27 = 2*Tbs(1,0);
	const float HK28 = q0*q1;
	const float HK29 = q2*q3;
	const float HK30 = HK28 - HK29;
	const float HK31 = -HK18;
	const float HK32 = HK19 + HK23 + HK31;
	const float HK33 = -HK13*HK30 + HK26*HK27 + HK32*Tbs(1,1);
	const float HK34 = HK28 + HK29;
	const float HK35 = HK10 - HK11;
	const float HK36 = HK20 + HK21 + HK22 + HK31;
	const float HK37 = HK17*HK34 - HK27*HK35 + HK36*Tbs(1,2);
	const float HK38 = 2*HK3;
	const float HK39 = 2*HK7;
	const float HK40 = 2*HK8;
	const float HK41 = 2*HK9;
	const float HK42 = HK25*P(0,4) + HK33*P(0,5) + HK37*P(0,6) + HK38*P(0,0) + HK39*P(0,1) + HK40*P(0,2) + HK41*P(0,3);
	const float HK43 = ecl::powf(range, -2);
	const float HK44 = HK25*P(4,6) + HK33*P(5,6) + HK37*P(6,6) + HK38*P(0,6) + HK39*P(1,6) + HK40*P(2,6) + HK41*P(3,6);
	const float HK45 = HK25*P(4,5) + HK33*P(5,5) + HK37*P(5,6) + HK38*P(0,5) + HK39*P(1,5) + HK40*P(2,5) + HK41*P(3,5);
	const float HK46 = HK25*P(4,4) + HK33*P(4,5) + HK37*P(4,6) + HK38*P(0,4) + HK39*P(1,4) + HK40*P(2,4) + HK41*P(3,4);
	const float HK47 = HK25*P(2,4) + HK33*P(2,5) + HK37*P(2,6) + HK38*P(0,2) + HK39*P(1,2) + HK40*P(2,2) + HK41*P(2,3);
	const float HK48 = HK25*P(3,4) + HK33*P(3,5) + HK37*P(3,6) + HK38*P(0,3) + HK39*P(1,3) + HK40*P(2,3) + HK41*P(3,3);
	const float HK49 = HK25*P(1,4) + HK33*P(1,5) + HK37*P(1,6) + HK38*P(0,1) + HK39*P(1,1) + HK40*P(1,2) + HK41*P(1,3);
	// const float HK50 = HK4/(HK25*HK43*HK46 + HK33*HK43*HK45 + HK37*HK43*HK44 + HK38*HK42*HK43 + HK39*HK43*HK49 + HK40*HK43*HK47 + HK41*HK43*HK48 + R_LOS);

	// calculate innovation variance for X axis observation and protect against a badly conditioned calculation
	_flow_innov_var(0) = (HK25*HK43*HK46 + HK33*HK43*HK45 + HK37*HK43*HK44 + HK38*HK42*HK43 + HK39*HK43*HK49 + HK40*HK43*HK47 + HK41*HK43*HK48 + R_LOS);

	if (_flow_innov_var(0) < R_LOS) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}
	const float HK50 = HK4/_flow_innov_var(0);

	const float HK51 = Tbs(0,1)*q1;
	const float HK52 = Tbs(0,2)*q0;
	const float HK53 = Tbs(0,0)*q2;
	const float HK54 = HK51 + HK52 - HK53;
	const float HK55 = Tbs(0,0)*q3;
	const float HK56 = Tbs(0,1)*q0;
	const float HK57 = Tbs(0,2)*q1;
	const float HK58 = HK55 + HK56 - HK57;
	const float HK59 = Tbs(0,0)*q0;
	const float HK60 = Tbs(0,2)*q2;
	const float HK61 = Tbs(0,1)*q3;
	const float HK62 = HK59 + HK60 - HK61;
	const float HK63 = HK54*vd + HK58*ve + HK62*vn;
	const float HK64 = Tbs(0,0)*q1 + Tbs(0,1)*q2 + Tbs(0,2)*q3;
	const float HK65 = HK58*vd + HK64*vn;
	const float HK66 = -HK54*ve + HK65;
	const float HK67 = HK54*vn + HK64*ve;
	const float HK68 = -HK62*vd + HK67;
	const float HK69 = HK62*ve + HK64*vd;
	const float HK70 = -HK58*vn + HK69;
	const float HK71 = 2*Tbs(0,1);
	const float HK72 = 2*Tbs(0,2);
	const float HK73 = HK12*HK72 + HK24*Tbs(0,0);
	const float HK74 = -HK16*HK71 + HK73;
	const float HK75 = 2*Tbs(0,0);
	const float HK76 = HK26*HK75 + HK32*Tbs(0,1);
	const float HK77 = -HK30*HK72 + HK76;
	const float HK78 = HK34*HK71 + HK36*Tbs(0,2);
	const float HK79 = -HK35*HK75 + HK78;
	const float HK80 = 2*HK63;
	const float HK81 = 2*HK65 + 2*ve*(-HK51 - HK52 + HK53);
	const float HK82 = 2*HK67 + 2*vd*(-HK59 - HK60 + HK61);
	const float HK83 = 2*HK69 + 2*vn*(-HK55 - HK56 + HK57);
	const float HK84 = HK71*(-HK14 + HK15) + HK73;
	const float HK85 = HK72*(-HK28 + HK29) + HK76;
	const float HK86 = HK75*(-HK10 + HK11) + HK78;
	const float HK87 = HK80*P(0,0) + HK81*P(0,1) + HK82*P(0,2) + HK83*P(0,3) + HK84*P(0,4) + HK85*P(0,5) + HK86*P(0,6);
	const float HK88 = HK80*P(0,6) + HK81*P(1,6) + HK82*P(2,6) + HK83*P(3,6) + HK84*P(4,6) + HK85*P(5,6) + HK86*P(6,6);
	const float HK89 = HK80*P(0,5) + HK81*P(1,5) + HK82*P(2,5) + HK83*P(3,5) + HK84*P(4,5) + HK85*P(5,5) + HK86*P(5,6);
	const float HK90 = HK80*P(0,4) + HK81*P(1,4) + HK82*P(2,4) + HK83*P(3,4) + HK84*P(4,4) + HK85*P(4,5) + HK86*P(4,6);
	const float HK91 = HK80*P(0,2) + HK81*P(1,2) + HK82*P(2,2) + HK83*P(2,3) + HK84*P(2,4) + HK85*P(2,5) + HK86*P(2,6);
	const float HK92 = 2*HK43;
	const float HK93 = HK80*P(0,3) + HK81*P(1,3) + HK82*P(2,3) + HK83*P(3,3) + HK84*P(3,4) + HK85*P(3,5) + HK86*P(3,6);
	const float HK94 = HK80*P(0,1) + HK81*P(1,1) + HK82*P(1,2) + HK83*P(1,3) + HK84*P(1,4) + HK85*P(1,5) + HK86*P(1,6);
	// const float HK95 = HK4/(HK43*HK74*HK90 + HK43*HK77*HK89 + HK43*HK79*HK88 + HK43*HK80*HK87 + HK66*HK92*HK94 + HK68*HK91*HK92 + HK70*HK92*HK93 + R_LOS);

	// calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
	_flow_innov_var(1) = (HK43*HK74*HK90 + HK43*HK77*HK89 + HK43*HK79*HK88 + HK43*HK80*HK87 + HK66*HK92*HK94 + HK68*HK91*HK92 + HK70*HK92*HK93 + R_LOS);
	if (_flow_innov_var(1) < R_LOS) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		return;
	}
	const float HK95 = HK4/_flow_innov_var(1);


	// run the innovation consistency check and record result
	bool flow_fail = false;
	float test_ratio[2];
	test_ratio[0] = sq(_flow_innov(0)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(0));
	test_ratio[1] = sq(_flow_innov(1)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(1));
	_optflow_test_ratio = math::max(test_ratio[0], test_ratio[1]);

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {
		if (test_ratio[obs_index] > 1.0f) {
			flow_fail = true;
			_innov_check_fail_status.value |= (1 << (obs_index + 10));

		} else {
			_innov_check_fail_status.value &= ~(1 << (obs_index + 10));

		}
	}

	// if either axis fails we abort the fusion
	if (flow_fail) {
		return;

	}

	// fuse observation axes sequentially
	SparseVector24f<0,1,2,3,4,5,6> Hfusion; // Optical flow observation Jacobians
	Vector24f Kfusion; // Optical flow Kalman gains

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {

		// calculate observation Jocobians and Kalman gains
		if (obs_index == 0) {
			// Observation Jacobians - axis 0
			Hfusion.at<0>() = HK3*HK5;
			Hfusion.at<1>() = HK5*HK7;
			Hfusion.at<2>() = HK5*HK8;
			Hfusion.at<3>() = HK5*HK9;
			Hfusion.at<4>() = HK25*HK4;
			Hfusion.at<5>() = HK33*HK4;
			Hfusion.at<6>() = HK37*HK4;

			// Kalman gains - axis 0
			Kfusion(0) = HK42*HK50;
			Kfusion(1) = HK49*HK50;
			Kfusion(2) = HK47*HK50;
			Kfusion(3) = HK48*HK50;
			Kfusion(4) = HK46*HK50;
			Kfusion(5) = HK45*HK50;
			Kfusion(6) = HK44*HK50;

			for (unsigned row = 7; row <= 23; row++) {
				Kfusion(row) = HK50*(HK25*P(4,row) + HK33*P(5,row) + HK37*P(6,row) + HK38*P(0,row) + HK39*P(1,row) + HK40*P(2,row) + HK41*P(3,row));
			}

		} else {
			// Observation Jacobians - axis 1
			Hfusion.at<0>() = -HK5*HK63;
			Hfusion.at<1>() = -HK5*HK66;
			Hfusion.at<2>() = -HK5*HK68;
			Hfusion.at<3>() = -HK5*HK70;
			Hfusion.at<4>() = -HK4*HK74;
			Hfusion.at<5>() = -HK4*HK77;
			Hfusion.at<6>() = -HK4*HK79;

			// Kalman gains - axis 1
			Kfusion(0) = -HK87*HK95;
			Kfusion(1) = -HK94*HK95;
			Kfusion(2) = -HK91*HK95;
			Kfusion(3) = -HK93*HK95;
			Kfusion(4) = -HK90*HK95;
			Kfusion(5) = -HK89*HK95;
			Kfusion(6) = -HK88*HK95;

			for (unsigned row = 7; row <= 23; row++) {
				Kfusion(row) = -HK95*(HK80*P(0,row) + HK81*P(1,row) + HK82*P(2,row) + HK83*P(3,row) + HK84*P(4,row) + HK85*P(5,row) + HK86*P(6,row));
			}

		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _flow_innov(obs_index));

		if (obs_index == 0) {
			_fault_status.flags.bad_optflow_X = !is_fused;

		} else if (obs_index == 1) {
			_fault_status.flags.bad_optflow_Y = !is_fused;
		}

		if (is_fused) {
			_time_last_of_fuse = _time_last_imu;
		}
	}
}

// calculate optical flow body angular rate compensation
// returns false if bias corrected body rate data is unavailable
bool Ekf::calcOptFlowBodyRateComp()
{
	// reset the accumulators if the time interval is too large
	if (_delta_time_of > 1.0f) {
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
		return false;
	}

	bool is_body_rate_comp_available = false;
	const bool use_flow_sensor_gyro = PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(0)) && PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(1)) && PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(2));

	if (use_flow_sensor_gyro) {

		// if accumulation time differences are not excessive and accumulation time is adequate
		// compare the optical flow and and navigation rate data and calculate a bias error
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)
		    && (fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.1f)) {

			const Vector3f reference_body_rate(_imu_del_ang_of * (1.0f / _delta_time_of));

			const Vector3f measured_body_rate(_flow_sample_delayed.gyro_xyz * (1.0f / _flow_sample_delayed.dt));

			// calculate the bias estimate using  a combined LPF and spike filter
			_flow_gyro_bias = _flow_gyro_bias * 0.99f + matrix::constrain(measured_body_rate - reference_body_rate, -0.1f, 0.1f) * 0.01f;

			is_body_rate_comp_available = true;
		}

	} else {
		// Use the EKF gyro data if optical flow sensor gyro data is not available
		// for clarification of the sign see definition of flowSample and imuSample in common.h
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)) {
			_flow_sample_delayed.gyro_xyz = -_imu_del_ang_of / _delta_time_of * _flow_sample_delayed.dt;
			_flow_gyro_bias.zero();

			is_body_rate_comp_available = true;
		}
	}

	// reset the accumulators
	_imu_del_ang_of.setZero();
	_delta_time_of = 0.0f;
	return is_body_rate_comp_available;
}

// calculate the measurement variance for the optical flow sensor (rad/sec)^2
float Ekf::calcOptFlowMeasVar()
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.0f - (float)_params.flow_qual_min);

	if (weighting >= 1.0f) {
		weighting = math::constrain(((float)_flow_sample_delayed.quality - (float)_params.flow_qual_min) / weighting, 0.0f,
					    1.0f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.0f - weighting));

	return R_LOS;
}
/****************************************************************************
 *
 *   Copyright (c) 2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file sensor_range_finder.cpp
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#include "sensor_range_finder.hpp"

namespace estimator
{
namespace sensor
{

void SensorRangeFinder::runChecks(const uint64_t current_time_us, const Dcmf &R_to_earth)
{
	updateSensorToEarthRotation(R_to_earth);
	updateValidity(current_time_us);
}

void SensorRangeFinder::updateSensorToEarthRotation(const Dcmf &R_to_earth)
{
	// calculate 2,2 element of rotation matrix from sensor frame to earth frame
	// this is required for use of range finder and flow data
	_cos_tilt_rng_to_earth = R_to_earth(2, 0) * _sin_pitch_offset + R_to_earth(2, 2) * _cos_pitch_offset;
}

void SensorRangeFinder::updateValidity(uint64_t current_time_us)
{
	updateDtDataLpf(current_time_us);

	if (isSampleOutOfDate(current_time_us) || !isDataContinuous()) {
		_is_sample_valid = false;
		_is_regularly_sending_data = false;
		return;
	}

	_is_regularly_sending_data = true;

	// Don't run the checks unless we have retrieved new data from the buffer
	if (_is_sample_ready) {
		_is_sample_valid = false;

		if (_sample.quality == 0) {
			_time_bad_quality_us = current_time_us;

		} else if (current_time_us - _time_bad_quality_us > _quality_hyst_us) {
			// We did not receive bad quality data for some time

			if (isTiltOk() && isDataInRange()) {
				updateStuckCheck();

				if (!_is_stuck) {
					_is_sample_valid = true;
					_time_last_valid_us = _sample.time_us;
				}
			}
		}
	}
}

void SensorRangeFinder::updateDtDataLpf(uint64_t current_time_us)
{
	// Calculate a first order IIR low-pass filtered time of arrival between samples using a 2 second time constant.
	float alpha = 0.5f * _dt_update;
	_dt_data_lpf = _dt_data_lpf * (1.0f - alpha) + alpha * (current_time_us - _sample.time_us);

	// Apply spike protection to the filter state.
	_dt_data_lpf = fminf(_dt_data_lpf, 4e6f);
}

inline bool SensorRangeFinder::isSampleOutOfDate(uint64_t current_time_us) const
{
	return (current_time_us - _sample.time_us) > 2 * RNG_MAX_INTERVAL;
}

inline bool SensorRangeFinder::isDataInRange() const
{
	return (_sample.rng >= _rng_valid_min_val) && (_sample.rng <= _rng_valid_max_val);
}

void SensorRangeFinder::updateStuckCheck()
{
	// Check for "stuck" range finder measurements when range was not valid for certain period
	// This handles a failure mode observed with some lidar sensors
	if (((_sample.time_us - _time_last_valid_us) > (uint64_t)10e6)) {

		// require a variance of rangefinder values to check for "stuck" measurements
		if (_stuck_max_val - _stuck_min_val > _stuck_threshold) {
			_stuck_min_val = 0.0f;
			_stuck_max_val = 0.0f;
			_is_stuck = false;

		} else {
			if (_sample.rng > _stuck_max_val) {
				_stuck_max_val = _sample.rng;
			}

			if (_stuck_min_val < 0.1f || _sample.rng < _stuck_min_val) {
				_stuck_min_val = _sample.rng;
			}

			_is_stuck = true;
		}
	}
}

} // namespace sensor
} // namespace estimator
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file sideslip_fusion.cpp
 * sideslip fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::fuseSideslip()
{
	// get latest estimated orientation
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float &vn = _state.vel(0);
	const float &ve = _state.vel(1);
	const float &vd = _state.vel(2);

	// get latest wind velocity in earth frame
	const float &vwn = _state.wind_vel(0);
	const float &vwe = _state.wind_vel(1);

	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(vn - vwn, ve - vwe, vd);
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);
	const Vector3f rel_wind_body = earth_to_body * rel_wind_earth;

	// perform fusion of assumed sideslip  = 0
	if (rel_wind_body.norm() > 7.0f) {
		const float R_BETA = sq(_params.beta_noise); // observation noise variance

		// determine if we need the sideslip fusion to correct states other than wind
		bool update_wind_only = !_is_wind_dead_reckoning;

		// Intermediate Values
		const float HK0 = vn - vwn;
		const float HK1 = ve - vwe;
		const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
		const float HK3 = q0*q2 - q1*q3;
		const float HK4 = 2*vd;
		const float HK5 = q0*q3;
		const float HK6 = q1*q2;
		const float HK7 = 2*HK5 + 2*HK6;
		const float HK8 = ecl::powf(q0, 2);
		const float HK9 = ecl::powf(q3, 2);
		const float HK10 = HK8 - HK9;
		const float HK11 = ecl::powf(q1, 2);
		const float HK12 = ecl::powf(q2, 2);
		const float HK13 = HK11 - HK12;
		const float HK14 = HK10 + HK13;
		const float HK15 = HK0*HK14 + HK1*HK7 - HK3*HK4;
		const float HK16 = 1.0F/HK15;
		const float HK17 = q0*q1 + q2*q3;
		const float HK18 = HK10 - HK11 + HK12;
		const float HK19 = HK16*(-2*HK0*(HK5 - HK6) + HK1*HK18 + HK17*HK4);
		const float HK20 = -HK0*q3 + HK1*q0 + q1*vd;
		const float HK21 = -HK19*HK2 + HK20;
		const float HK22 = 2*HK16;
		const float HK23 = HK0*q1 + HK1*q2 + q3*vd;
		const float HK24 = HK0*q2 - HK1*q1 + q0*vd;
		const float HK25 = -HK19*HK23 + HK24;
		const float HK26 = HK19*HK24 + HK23;
		const float HK27 = HK19*HK20 + HK2;
		const float HK28 = HK14*HK19 + 2*HK5 - 2*HK6;
		const float HK29 = HK16*HK28;
		const float HK30 = HK19*HK7;
		const float HK31 = HK17 + HK19*HK3;
		const float HK32 = HK13 + HK30 - HK8 + HK9;
		const float HK33 = 2*HK31;
		const float HK34 = 2*HK26;
		const float HK35 = 2*HK25;
		const float HK36 = 2*HK27;
		const float HK37 = 2*HK21;
		const float HK38 = HK28*P(0,22) - HK28*P(0,4) + HK32*P(0,23) - HK32*P(0,5) + HK33*P(0,6) + HK34*P(0,2) + HK35*P(0,1) - HK36*P(0,3) + HK37*P(0,0);
		const float HK39 = ecl::powf(HK15, -2);
		const float HK40 = -HK28*P(4,6) + HK28*P(6,22) - HK32*P(5,6) + HK32*P(6,23) + HK33*P(6,6) + HK34*P(2,6) + HK35*P(1,6) - HK36*P(3,6) + HK37*P(0,6);
		const float HK41 = HK32*P(5,23);
		const float HK42 = HK28*P(22,23) - HK28*P(4,23) + HK32*P(23,23) + HK33*P(6,23) + HK34*P(2,23) + HK35*P(1,23) - HK36*P(3,23) + HK37*P(0,23) - HK41;
		const float HK43 = HK32*HK39;
		const float HK44 = HK28*P(4,22);
		const float HK45 = HK28*P(22,22) + HK32*P(22,23) - HK32*P(5,22) + HK33*P(6,22) + HK34*P(2,22) + HK35*P(1,22) - HK36*P(3,22) + HK37*P(0,22) - HK44;
		const float HK46 = HK28*HK39;
		const float HK47 = -HK28*P(4,5) + HK28*P(5,22) - HK32*P(5,5) + HK33*P(5,6) + HK34*P(2,5) + HK35*P(1,5) - HK36*P(3,5) + HK37*P(0,5) + HK41;
		const float HK48 = -HK28*P(4,4) + HK32*P(4,23) - HK32*P(4,5) + HK33*P(4,6) + HK34*P(2,4) + HK35*P(1,4) - HK36*P(3,4) + HK37*P(0,4) + HK44;
		const float HK49 = HK28*P(2,22) - HK28*P(2,4) + HK32*P(2,23) - HK32*P(2,5) + HK33*P(2,6) + HK34*P(2,2) + HK35*P(1,2) - HK36*P(2,3) + HK37*P(0,2);
		const float HK50 = HK28*P(1,22) - HK28*P(1,4) + HK32*P(1,23) - HK32*P(1,5) + HK33*P(1,6) + HK34*P(1,2) + HK35*P(1,1) - HK36*P(1,3) + HK37*P(0,1);
		const float HK51 = HK28*P(3,22) - HK28*P(3,4) + HK32*P(3,23) - HK32*P(3,5) + HK33*P(3,6) + HK34*P(2,3) + HK35*P(1,3) - HK36*P(3,3) + HK37*P(0,3);
		//const float HK52 = HK16/(HK33*HK39*HK40 + HK34*HK39*HK49 + HK35*HK39*HK50 - HK36*HK39*HK51 + HK37*HK38*HK39 + HK42*HK43 - HK43*HK47 + HK45*HK46 - HK46*HK48 + R_BETA);

		// innovation variance
		_beta_innov_var = (HK33*HK39*HK40 + HK34*HK39*HK49 + HK35*HK39*HK50 - HK36*HK39*HK51 + HK37*HK38*HK39 + HK42*HK43 - HK43*HK47 + HK45*HK46 - HK46*HK48 + R_BETA);

		// Reset covariance and states if the calculation is badly conditioned
		if (_beta_innov_var < R_BETA) {
			_fault_status.flags.bad_sideslip = true;

			// if we are getting aiding from other sources, warn and reset the wind states and covariances only
			const char *action_string = nullptr;

			if (update_wind_only) {
				resetWind();
				action_string = "wind";

			} else {
				initialiseCovariance();
				_state.wind_vel.setZero();
				action_string = "full";
			}

			ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

			return;
		}

		_fault_status.flags.bad_sideslip = false;
		const float HK52 = HK16 / _beta_innov_var;

		// Calculate predicted sideslip angle and innovation using small angle approximation
		_beta_innov = rel_wind_body(1) / rel_wind_body(0);

		// Compute the ratio of innovation to gate size
		_beta_test_ratio = sq(_beta_innov) / (sq(fmaxf(_params.beta_innov_gate, 1.0f)) * _beta_innov_var);

		// if the innovation consistency check fails then don't fuse the sample and indicate bad beta health
		if (_beta_test_ratio > 1.0f) {
			_innov_check_fail_status.flags.reject_sideslip = true;
			return;

		} else {
			_innov_check_fail_status.flags.reject_sideslip = false;
		}

		// Observation Jacobians
		SparseVector24f<0,1,2,3,4,5,6,22,23> Hfusion;
		Hfusion.at<0>() = HK21*HK22;
		Hfusion.at<1>() = HK22*HK25;
		Hfusion.at<2>() = HK22*HK26;
		Hfusion.at<3>() = -HK22*HK27;
		Hfusion.at<4>() = -HK29;
		Hfusion.at<5>() = HK16*(HK18 - HK30);
		Hfusion.at<6>() = HK22*HK31;
		Hfusion.at<22>() = HK29;
		Hfusion.at<23>() = HK16*HK32;

		// Calculate Kalman gains
		Vector24f Kfusion;

		if (!update_wind_only) {

			Kfusion(0) = HK38*HK52;
			Kfusion(1) = HK50*HK52;
			Kfusion(2) = HK49*HK52;
			Kfusion(3) = HK51*HK52;
			Kfusion(4) = HK48*HK52;
			Kfusion(5) = HK47*HK52;
			Kfusion(6) = HK40*HK52;

			for (unsigned row = 7; row <= 21; row++) {
				Kfusion(row) = HK52*(HK28*P(row,22) - HK28*P(4,row) + HK32*P(row,23) - HK32*P(5,row) + HK33*P(6,row) + HK34*P(2,row) + HK35*P(1,row) - HK36*P(3,row) + HK37*P(0,row));
			}

		}

		Kfusion(22) = HK45*HK52;
		Kfusion(23) = HK42*HK52;

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _beta_innov);

		_fault_status.flags.bad_sideslip = !is_fused;

		if (is_fused) {
			_time_last_beta_fuse = _time_last_imu;
		}
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder measurements to estimate terrain vertical position/
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::initHagl()
{
	bool initialized = false;

	if (!_control_status.flags.in_air) {
		// if on ground, do not trust the range sensor, but assume a ground clearance
		_terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;
		// use the ground clearance value as our uncertainty
		_terrain_var = sq(_params.rng_gnd_clearance);
		_time_last_fake_hagl_fuse = _time_last_imu;
		initialized = true;

	} else if (shouldUseRangeFinderForHagl()
		   && _range_sensor.isDataHealthy()) {
		// if we have a fresh measurement, use it to initialise the terrain estimator
		_terrain_vpos = _state.pos(2) + _range_sensor.getDistBottom();
		// initialise state variance to variance of measurement
		_terrain_var = sq(_params.range_noise);
		// success
		initialized = true;

	} else if (shouldUseOpticalFlowForHagl()
		   && _flow_for_terrain_data_ready) {
		// initialise terrain vertical position to origin as this is the best guess we have
		_terrain_vpos = fmaxf(0.0f,  _state.pos(2));
		_terrain_var = 100.0f;
		initialized = true;

	} else {
		// no information - cannot initialise
	}

	if (initialized) {
		// has initialized with valid data
		_time_last_hagl_fuse = _time_last_imu;
	}

	return initialized;
}

void Ekf::runTerrainEstimator()
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
	}

	// Perform initialisation check and
	// on ground, continuously reset the terrain estimator
	if (!_terrain_initialised || !_control_status.flags.in_air) {
		_terrain_initialised = initHagl();

	} else {

		// predict the state variance growth where the state is the vertical position of the terrain underneath the vehicle

		// process noise due to errors in vehicle height estimate
		_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_p_noise);

		// process noise due to terrain gradient
		_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_gradient)
				* (sq(_state.vel(0)) + sq(_state.vel(1)));

		// limit the variance to prevent it becoming badly conditioned
		_terrain_var = math::constrain(_terrain_var, 0.0f, 1e4f);

		// Fuse range finder data if available
		if (shouldUseRangeFinderForHagl()
		    && _range_sensor.isDataHealthy()) {
			fuseHagl();
		}

		if (shouldUseOpticalFlowForHagl()
		    && _flow_for_terrain_data_ready) {
			fuseFlowForTerrain();
			_flow_for_terrain_data_ready = false;
		}

		// constrain _terrain_vpos to be a minimum of _params.rng_gnd_clearance larger than _state.pos(2)
		if (_terrain_vpos - _state.pos(2) < _params.rng_gnd_clearance) {
			_terrain_vpos = _params.rng_gnd_clearance + _state.pos(2);
		}
	}

	updateTerrainValidity();
}

void Ekf::fuseHagl()
{
	// get a height above ground measurement from the range finder assuming a flat earth
	const float meas_hagl = _range_sensor.getDistBottom();

	// predict the hagl from the vehicle position and terrain height
	const float pred_hagl = _terrain_vpos - _state.pos(2);

	// calculate the innovation
	_hagl_innov = pred_hagl - meas_hagl;

	// calculate the observation variance adding the variance of the vehicles own height uncertainty
	const float obs_variance = fmaxf(P(9, 9) * _params.vehicle_variance_scaler, 0.0f)
				   + sq(_params.range_noise)
				   + sq(_params.range_noise_scaler * _range_sensor.getRange());

	// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
	_hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

	// perform an innovation consistency check and only fuse data if it passes
	const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
	_hagl_test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

	if (_hagl_test_ratio <= 1.0f) {
		// calculate the Kalman gain
		const float gain = _terrain_var / _hagl_innov_var;
		// correct the state
		_terrain_vpos -= gain * _hagl_innov;
		// correct the variance
		_terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);
		// record last successful fusion event
		_time_last_hagl_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_hagl = false;

	} else {
		// If we have been rejecting range data for too long, reset to measurement
		const uint64_t timeout = static_cast<uint64_t>(_params.terrain_timeout * 1e6f);

		if (isTimedOut(_time_last_hagl_fuse, timeout)) {
			_terrain_vpos = _state.pos(2) + meas_hagl;
			_terrain_var = obs_variance;
			_terrain_vpos_reset_counter++;

		} else {
			_innov_check_fail_status.flags.reject_hagl = true;
		}
	}
}

void Ekf::fuseFlowForTerrain()
{
	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

	// get latest estimated orientation
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar();

	// get rotation matrix from earth to body
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	const Vector3f vel_body = earth_to_body * vel_rel_earth;

	const float t0 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	// constrain terrain to minimum allowed value and predict height above ground
	_terrain_vpos = fmaxf(_terrain_vpos, _params.rng_gnd_clearance + _state.pos(2));
	const float pred_hagl_inv = 1.f / (_terrain_vpos - _state.pos(2));

	// Calculate observation matrix for flow around the vehicle x axis
	const float Hx = vel_body(1) * t0 * pred_hagl_inv * pred_hagl_inv;

	// Constrain terrain variance to be non-negative
	_terrain_var = fmaxf(_terrain_var, 0.0f);

	// Cacluate innovation variance
	_flow_innov_var(0) = Hx * Hx * _terrain_var + R_LOS;

	// calculate the kalman gain for the flow x measurement
	const float Kx = _terrain_var * Hx / _flow_innov_var(0);

	// calculate prediced optical flow about x axis
	const float pred_flow_x = vel_body(1) * earth_to_body(2, 2) * pred_hagl_inv;

	// calculate flow innovation (x axis)
	_flow_innov(0) = pred_flow_x - opt_flow_rate(0);

	// calculate correction term for terrain variance
	const float KxHxP =  Kx * Hx * _terrain_var;

	// innovation consistency check
	const float gate_size = fmaxf(_params.flow_innov_gate, 1.0f);
	float flow_test_ratio = sq(_flow_innov(0)) / (sq(gate_size) * _flow_innov_var(0));

	// do not perform measurement update if badly conditioned
	if (flow_test_ratio <= 1.0f) {
		_terrain_vpos += Kx * _flow_innov(0);
		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - KxHxP, 0.0f);
		_time_last_flow_terrain_fuse = _time_last_imu;
	}

	// Calculate observation matrix for flow around the vehicle y axis
	const float Hy = -vel_body(0) * t0 * pred_hagl_inv * pred_hagl_inv;

	// Calculuate innovation variance
	_flow_innov_var(1) = Hy * Hy * _terrain_var + R_LOS;

	// calculate the kalman gain for the flow y measurement
	const float Ky = _terrain_var * Hy / _flow_innov_var(1);

	// calculate prediced optical flow about y axis
	const float pred_flow_y = -vel_body(0) * earth_to_body(2, 2) * pred_hagl_inv;

	// calculate flow innovation (y axis)
	_flow_innov(1) = pred_flow_y - opt_flow_rate(1);

	// calculate correction term for terrain variance
	const float KyHyP =  Ky * Hy * _terrain_var;

	// innovation consistency check
	flow_test_ratio = sq(_flow_innov(1)) / (sq(gate_size) * _flow_innov_var(1));

	if (flow_test_ratio <= 1.0f) {
		_terrain_vpos += Ky * _flow_innov(1);
		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - KyHyP, 0.0f);
		_time_last_flow_terrain_fuse = _time_last_imu;
	}
}

void Ekf::updateTerrainValidity()
{
	// we have been fusing range finder measurements in the last 5 seconds
	const bool recent_range_fusion = isRecent(_time_last_hagl_fuse, (uint64_t)5e6);

	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	const bool recent_flow_for_terrain_fusion = isRecent(_time_last_flow_terrain_fuse, (uint64_t)5e6);

	_hagl_valid = (_terrain_initialised && (recent_range_fusion || recent_flow_for_terrain_fusion));

	_hagl_sensor_status.flags.range_finder = shouldUseRangeFinderForHagl()
			&& recent_range_fusion && (_time_last_fake_hagl_fuse != _time_last_hagl_fuse);

	_hagl_sensor_status.flags.flow = shouldUseOpticalFlowForHagl() && recent_flow_for_terrain_fusion;
}
#include "utils.hpp"

matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312)
{
	// Calculate the frame2 to frame 1 rotation matrix from a 312 Tait-Bryan rotation sequence
	const float c2 = cosf(rot312(2)); // third rotation is pitch
	const float s2 = sinf(rot312(2));
	const float s1 = sinf(rot312(1)); // second rotation is roll
	const float c1 = cosf(rot312(1));
	const float s0 = sinf(rot312(0)); // first rotation is yaw
	const float c0 = cosf(rot312(0));

	matrix::Dcmf R;
	R(0, 0) = c0 * c2 - s0 * s1 * s2;
	R(1, 1) = c0 * c1;
	R(2, 2) = c2 * c1;
	R(0, 1) = -c1 * s0;
	R(0, 2) = s2 * c0 + c2 * s1 * s0;
	R(1, 0) = c2 * s0 + s2 * s1 * c0;
	R(1, 2) = s0 * s2 - s1 * c0 * c2;
	R(2, 0) = -s2 * c1;
	R(2, 1) = s1;

	return R;
}

float kahanSummation(float sum_previous, float input, float &accumulator)
{
	const float y = input - accumulator;
	const float t = sum_previous + y;
	accumulator = (t - sum_previous) - y;
	return t;
}

matrix::Dcmf quatToInverseRotMat(const  matrix::Quatf &quat)
{
	const float q00 = quat(0) * quat(0);
	const float q11 = quat(1) * quat(1);
	const float q22 = quat(2) * quat(2);
	const float q33 = quat(3) * quat(3);
	const float q01 = quat(0) * quat(1);
	const float q02 = quat(0) * quat(2);
	const float q03 = quat(0) * quat(3);
	const float q12 = quat(1) * quat(2);
	const float q13 = quat(1) * quat(3);
	const float q23 = quat(2) * quat(3);

	matrix::Dcmf dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(1, 0) = 2.0f * (q12 - q03);
	dcm(2, 0) = 2.0f * (q13 + q02);
	dcm(0, 1) = 2.0f * (q12 + q03);
	dcm(2, 1) = 2.0f * (q23 - q01);
	dcm(0, 2) = 2.0f * (q13 - q02);
	dcm(1, 2) = 2.0f * (q23 + q01);

	return dcm;
}

bool shouldUse321RotationSequence(const matrix::Dcmf &R)
{
	return fabsf(R(2, 0)) < fabsf(R(2, 1));
}

float getEuler321Yaw(const matrix::Quatf &q)
{
	// Values from yaw_input_321.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m
	const float a = 2.f * (q(0) * q(3) + q(1) * q(2));
	const float b = sq(q(0)) + sq(q(1)) - sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

float getEuler321Yaw(const matrix::Dcmf &R)
{
	return atan2f(R(1, 0), R(0, 0));
}

float getEuler312Yaw(const matrix::Quatf &q)
{
	// Values from yaw_input_312.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
	const float a = 2.f * (q(0) * q(3) - q(1) * q(2));
	const float b = sq(q(0)) - sq(q(1)) + sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

float getEuler312Yaw(const matrix::Dcmf &R)
{
	return atan2f(-R(0, 1), R(1, 1));
}

matrix::Dcmf updateEuler321YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	matrix::Eulerf euler321(rot_in);
	euler321(2) = yaw;
	return matrix::Dcmf(euler321);
}

matrix::Dcmf updateEuler312YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	const matrix::Vector3f rotVec312(yaw,  // yaw
					 asinf(rot_in(2, 1)),  // roll
					 atan2f(-rot_in(2, 0), rot_in(2, 2)));  // pitch
	return taitBryan312ToRotMat(rotVec312);
}

matrix::Dcmf updateYawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	return shouldUse321RotationSequence(rot_in) ?
	       updateEuler321YawInRotMat(yaw, rot_in) :
	       updateEuler312YawInRotMat(yaw, rot_in);
}
/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file vel_pos_fusion.cpp
 * Function for fusing gps and baro measurements/
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include <mathlib/mathlib.h>
#include "ekf.h"

bool Ekf::fuseHorizontalVelocity(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
				 Vector3f &innov_var, Vector2f &test_ratio)
{

	innov_var(0) = P(4, 4) + obs_var(0);
	innov_var(1) = P(5, 5) + obs_var(1);
	test_ratio(0) = fmaxf(sq(innov(0)) / (sq(innov_gate(0)) * innov_var(0)),
			      sq(innov(1)) / (sq(innov_gate(0)) * innov_var(1)));

	const bool innov_check_pass = (test_ratio(0) <= 1.0f);

	if (innov_check_pass) {
		_time_last_hor_vel_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_hor_vel = false;

		fuseVelPosHeight(innov(0), innov_var(0), 0);
		fuseVelPosHeight(innov(1), innov_var(1), 1);

		return true;

	} else {
		_last_fail_hvel_innov(0) = innov(0);
		_last_fail_hvel_innov(1) = innov(1);
		_innov_check_fail_status.flags.reject_hor_vel = true;
		return false;
	}
}

bool Ekf::fuseVerticalVelocity(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
			       Vector3f &innov_var, Vector2f &test_ratio)
{

	innov_var(2) = P(6, 6) + obs_var(2);
	test_ratio(1) = sq(innov(2)) / (sq(innov_gate(1)) * innov_var(2));
	_vert_vel_innov_ratio = innov(2) / sqrtf(innov_var(2));
	_vert_vel_fuse_time_us = _time_last_imu;
	bool innov_check_pass = (test_ratio(1) <= 1.0f);

	// if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	float innovation;

	if (_bad_vert_accel_detected && !innov_check_pass) {
		const float innov_limit = innov_gate(1) * sqrtf(innov_var(2));
		innovation = math::constrain(innov(2), -innov_limit, innov_limit);
		innov_check_pass = true;

	} else {
		innovation = innov(2);
	}

	if (innov_check_pass) {
		_time_last_ver_vel_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_ver_vel = false;

		fuseVelPosHeight(innovation, innov_var(2), 2);

		return true;

	} else {
		_innov_check_fail_status.flags.reject_ver_vel = true;
		return false;
	}
}

bool Ekf::fuseHorizontalPosition(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
				 Vector3f &innov_var, Vector2f &test_ratio, bool inhibit_gate)
{

	innov_var(0) = P(7, 7) + obs_var(0);
	innov_var(1) = P(8, 8) + obs_var(1);
	test_ratio(0) = fmaxf(sq(innov(0)) / (sq(innov_gate(0)) * innov_var(0)),
			      sq(innov(1)) / (sq(innov_gate(0)) * innov_var(1)));

	const bool innov_check_pass = test_ratio(0) <= 1.0f;

	if (innov_check_pass || inhibit_gate) {
		if (inhibit_gate && test_ratio(0) > sq(100.0f / innov_gate(0))) {
			// always protect against extreme values that could result in a NaN
			return false;
		}

		if (!_fuse_hpos_as_odom) {
			_time_last_hor_pos_fuse = _time_last_imu;

		} else {
			_time_last_delpos_fuse = _time_last_imu;
		}

		_innov_check_fail_status.flags.reject_hor_pos = false;

		fuseVelPosHeight(innov(0), innov_var(0), 3);
		fuseVelPosHeight(innov(1), innov_var(1), 4);

		return true;

	} else {
		_innov_check_fail_status.flags.reject_hor_pos = true;
		return false;
	}
}

bool Ekf::fuseVerticalPosition(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
			       Vector3f &innov_var, Vector2f &test_ratio)
{

	innov_var(2) = P(9, 9) + obs_var(2);
	test_ratio(1) = sq(innov(2)) / (sq(innov_gate(1)) * innov_var(2));
	_vert_pos_innov_ratio = innov(2) / sqrtf(innov_var(2));
	_vert_pos_fuse_attempt_time_us = _time_last_imu;
	bool innov_check_pass = test_ratio(1) <= 1.0f;

	// if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	float innovation;

	if (_bad_vert_accel_detected && !innov_check_pass) {
		const float innov_limit = innov_gate(1) * sqrtf(innov_var(2));
		innovation = math::constrain(innov(2), -innov_limit, innov_limit);
		innov_check_pass = true;

	} else {
		innovation = innov(2);
	}

	if (innov_check_pass) {
		_time_last_hgt_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_ver_pos = false;
		fuseVelPosHeight(innovation, innov_var(2), 5);

		return true;

	} else {
		_innov_check_fail_status.flags.reject_ver_pos = true;
		return false;
	}
}

// Helper function that fuses a single velocity or position measurement
void Ekf::fuseVelPosHeight(const float innov, const float innov_var, const int obs_index)
{

	Vector24f Kfusion;  // Kalman gain vector for any single observation - sequential fusion is used.
	const unsigned state_index = obs_index + 4;  // we start with vx and this is the 4. state

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < _k_num_states; row++) {
		Kfusion(row) = P(row, state_index) / innov_var;
	}

	SquareMatrix24f KHP;

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			KHP(row, column) = Kfusion(row) * P(state_index, column);
		}
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P(i, i) < KHP(i, i)) {
			// zero rows and columns
			P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

			healthy = false;
		}
	}

	setVelPosFaultStatus(obs_index, !healthy);

	if (healthy) {
		// apply the covariance corrections
		P -= KHP;

		fixCovarianceErrors(true);

		// apply the state corrections
		fuse(Kfusion, innov);
	}
}

void Ekf::setVelPosFaultStatus(const int index, const bool status)
{
	if (index == 0) {
		_fault_status.flags.bad_vel_N = status;

	} else if (index == 1) {
		_fault_status.flags.bad_vel_E = status;

	} else if (index == 2) {
		_fault_status.flags.bad_vel_D = status;

	} else if (index == 3) {
		_fault_status.flags.bad_pos_N = status;

	} else if (index == 4) {
		_fault_status.flags.bad_pos_E = status;

	} else if (index == 5) {
		_fault_status.flags.bad_pos_D = status;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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

#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if !defined(CONSTRAINED_FLASH)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // !CONSTRAINED_FLASH

EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, config),
	_replay_mode(replay_mode && !multi_mode),
	_multi_mode(multi_mode),
	_instance(multi_mode ? -1 : 0),
	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)),
	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)),
	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)),
	_params(_ekf.getParamHandle()),
	_param_ekf2_min_obs_dt(_params->sensor_interval_min_ms),
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_wind_noise(_params->wind_vel_p_noise),
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate),
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_yawlim(_params->mag_yaw_rate_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_pdop(_params->req_pdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_aid_mask(_params->fusion_mode),
	_param_ekf2_hgt_mode(_params->vdist_sensor_type),
	_param_ekf2_terr_mask(_params->terrain_fusion_mode),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_aid(_params->range_aid),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_rng_qlty_t(_params->range_valid_quality_s),
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate),
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
	_param_ekf2_arsp_thr(_params->arsp_thr),
	_param_ekf2_tau_vel(_params->vel_Tau),
	_param_ekf2_tau_pos(_params->pos_Tau),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_mcoef(_params->mcoef),
	_param_ekf2_aspd_max(_params->max_correction_airspeed),
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp),
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn),
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp),
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn),
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z),
	_param_ekf2_move_test(_params->is_moving_scaler),
	_param_ekf2_mag_check(_params->check_mag_strength),
	_param_ekf2_synthetic_mag_z(_params->synthesize_mag_z),
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default)
{
}

EKF2::~EKF2()
{
	perf_free(_ecl_ekf_update_perf);
	perf_free(_ecl_ekf_update_full_perf);
	perf_free(_msg_missed_imu_perf);
	perf_free(_msg_missed_air_data_perf);
	perf_free(_msg_missed_airspeed_perf);
	perf_free(_msg_missed_distance_sensor_perf);
	perf_free(_msg_missed_gps_perf);
	perf_free(_msg_missed_landing_target_pose_perf);
	perf_free(_msg_missed_magnetometer_perf);
	perf_free(_msg_missed_odometry_perf);
	perf_free(_msg_missed_optical_flow_perf);
}

bool EKF2::multi_init(int imu, int mag)
{
	// advertise immediately to ensure consistent uORB instance numbering
	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_global_position_pub.advertise();
	_odometry_pub.advertise();
	_wind_pub.advertise();

	_ekf2_timestamps_pub.advertise();
	_ekf_gps_drift_pub.advertise();
	_estimator_baro_bias_pub.advertise();
	_estimator_event_flags_pub.advertise();
	_estimator_innovation_test_ratios_pub.advertise();
	_estimator_innovation_variances_pub.advertise();
	_estimator_innovations_pub.advertise();
	_estimator_optical_flow_vel_pub.advertise();
	_estimator_sensor_bias_pub.advertise();
	_estimator_states_pub.advertise();
	_estimator_status_pub.advertise();
	_estimator_status_flags_pub.advertise();
	_estimator_visual_odometry_aligned_pub.advertised();
	_yaw_est_pub.advertise();

	bool changed_instance = _vehicle_imu_sub.ChangeInstance(imu) && _magnetometer_sub.ChangeInstance(mag);

	const int status_instance = _estimator_states_pub.get_instance();

	if ((status_instance >= 0) && changed_instance
	    && (_attitude_pub.get_instance() == status_instance)
	    && (_local_position_pub.get_instance() == status_instance)
	    && (_global_position_pub.get_instance() == status_instance)) {

		_instance = status_instance;

		ScheduleNow();
		return true;
	}

	PX4_ERR("publication instance problem: %d att: %d lpos: %d gpos: %d", status_instance,
		_attitude_pub.get_instance(), _local_position_pub.get_instance(), _global_position_pub.get_instance());

	return false;
}

int EKF2::print_status()
{
	PX4_INFO_RAW("ekf2:%d attitude: %d, local position: %d, global position: %d\n", _instance, _ekf.attitude_valid(),
		     _ekf.local_position_is_valid(), _ekf.global_position_is_valid());
	perf_print_counter(_ecl_ekf_update_perf);
	perf_print_counter(_ecl_ekf_update_full_perf);
	perf_print_counter(_msg_missed_imu_perf);
	perf_print_counter(_msg_missed_air_data_perf);
	perf_print_counter(_msg_missed_airspeed_perf);
	perf_print_counter(_msg_missed_distance_sensor_perf);
	perf_print_counter(_msg_missed_gps_perf);
	perf_print_counter(_msg_missed_landing_target_pose_perf);
	perf_print_counter(_msg_missed_magnetometer_perf);
	perf_print_counter(_msg_missed_odometry_perf);
	perf_print_counter(_msg_missed_optical_flow_perf);

	return 0;
}

void EKF2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();

		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated() || !_callback_registered) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s);

		// The airspeed scale factor correcton is only available via parameter as used by the airspeed module
		param_t param_aspd_scale = param_find("ASPD_SCALE_1");

		if (param_aspd_scale != PARAM_INVALID) {
			param_get(param_aspd_scale, &_airspeed_scale_factor);
		}
	}

	if (!_callback_registered) {
		if (_multi_mode) {
			_callback_registered = _vehicle_imu_sub.registerCallback();

		} else {
			_callback_registered = _sensor_combined_sub.registerCallback();
		}

		if (!_callback_registered) {
			PX4_WARN("%d - failed to register callback, retrying", _instance);
			ScheduleDelayed(1_s);
			return;
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				if (!_ekf.control_status_flags().in_air) {

					uint64_t origin_time {};
					double latitude = vehicle_command.param5;
					double longitude = vehicle_command.param6;
					float altitude = vehicle_command.param7;

					_ekf.setEkfGlobalOrigin(latitude, longitude, altitude);

					// Validate the ekf origin status.
					_ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude);
					PX4_INFO("New NED origin (LLA): %3.10f, %3.10f, %4.3f\n", latitude, longitude, static_cast<double>(altitude));
				}
			}
		}
	}

	bool imu_updated = false;
	imuSample imu_sample_new {};

	hrt_abstime imu_dt = 0; // for tracking time slip later

	if (_multi_mode) {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		vehicle_imu_s imu;
		imu_updated = _vehicle_imu_sub.update(&imu);

		if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		imu_sample_new.time_us = imu.timestamp_sample;
		imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{imu.delta_angle};
		imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{imu.delta_velocity};

		if (imu.delta_velocity_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z;
		}

		imu_dt = imu.delta_angle_dt;

		if ((_device_id_accel == 0) || (_device_id_gyro == 0)) {
			_device_id_accel = imu.accel_device_id;
			_device_id_gyro = imu.gyro_device_id;
			_accel_calibration_count = imu.accel_calibration_count;
			_gyro_calibration_count = imu.gyro_calibration_count;

		} else {
			bool reset_actioned = false;

			if ((imu.accel_calibration_count != _accel_calibration_count)
			    || (imu.accel_device_id != _device_id_accel)) {

				PX4_DEBUG("%d - resetting accelerometer bias", _instance);
				_device_id_accel = imu.accel_device_id;

				_ekf.resetAccelBias();
				_accel_calibration_count = imu.accel_calibration_count;

				// reset bias learning
				_accel_cal = {};

				reset_actioned = true;
			}

			if ((imu.gyro_calibration_count != _gyro_calibration_count)
			    || (imu.gyro_device_id != _device_id_gyro)) {

				PX4_DEBUG("%d - resetting rate gyro bias", _instance);
				_device_id_gyro = imu.gyro_device_id;

				_ekf.resetGyroBias();
				_gyro_calibration_count = imu.gyro_calibration_count;

				// reset bias learning
				_gyro_cal = {};

				reset_actioned = true;
			}

			if (reset_actioned) {
				SelectImuStatus();
			}
		}

	} else {
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation();
		sensor_combined_s sensor_combined;
		imu_updated = _sensor_combined_sub.update(&sensor_combined);

		if (imu_updated && (_sensor_combined_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf);
		}

		imu_sample_new.time_us = sensor_combined.timestamp;
		imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt;
		imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

		if (sensor_combined.accelerometer_clipping > 0) {
			imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
			imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
			imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
		}

		imu_dt = sensor_combined.gyro_integral_dt;

		if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0)) {
			sensor_selection_s sensor_selection;

			if (_sensor_selection_sub.copy(&sensor_selection)) {
				if (_device_id_accel != sensor_selection.accel_device_id) {

					_device_id_accel = sensor_selection.accel_device_id;

					_ekf.resetAccelBias();

					// reset bias learning
					_accel_cal = {};

					SelectImuStatus();
				}

				if (_device_id_gyro != sensor_selection.gyro_device_id) {

					_device_id_gyro = sensor_selection.gyro_device_id;

					_ekf.resetGyroBias();

					// reset bias learning
					_gyro_cal = {};

					SelectImuStatus();
				}
			}
		}
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;

		UpdateImuStatus();

		// push imu data into estimator
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now); // publish attitude immediately (uses quaternion from output predictor)

		// integrate time to monitor time slippage
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt;
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us;

		} else {
			_start_time_us = imu_sample_new.time_us;
			_last_time_slip_us = 0;
		}

		// update all other topics if they have new data
		if (_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_status_sub.copy(&vehicle_status)) {
				const bool is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

				// only fuse synthetic sideslip measurements if conditions are met
				_ekf.set_fuse_beta_flag(is_fixed_wing && (_param_ekf2_fuse_beta.get() == 1));

				// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
				_ekf.set_is_fixed_wing(is_fixed_wing);

				_preflt_checker.setVehicleCanObserveHeadingInFlight(vehicle_status.vehicle_type !=
						vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);

				_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

				// update standby (arming state) flag
				const bool standby = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY);

				if (_standby != standby) {
					_standby = standby;

					// reset preflight checks if transitioning in or out of standby arming state
					_preflt_checker.reset();
				}
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				const bool was_in_air = _ekf.control_status_flags().in_air;
				_ekf.set_in_air_status(!vehicle_land_detected.landed);

				if (_armed && (_param_ekf2_gnd_eff_dz.get() > 0.f)) {
					if (!_had_valid_terrain) {
						// update ground effect flag based on land detector state if we've never had valid terrain data
						_ekf.set_gnd_effect_flag(vehicle_land_detected.in_ground_effect);
					}

				} else {
					_ekf.set_gnd_effect_flag(false);
				}

				// reset learned sensor calibrations on takeoff
				if (_ekf.control_status_flags().in_air && !was_in_air) {
					_accel_cal = {};
					_gyro_cal = {};
					_mag_cal = {};
				}
			}
		}

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now,
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID,
		};

		UpdateAirspeedSample(ekf2_timestamps);
		UpdateAuxVelSample(ekf2_timestamps);
		UpdateBaroSample(ekf2_timestamps);
		UpdateGpsSample(ekf2_timestamps);
		UpdateMagSample(ekf2_timestamps);
		UpdateRangeSample(ekf2_timestamps);

		vehicle_odometry_s ev_odom;
		const bool new_ev_odom = UpdateExtVisionSample(ekf2_timestamps, ev_odom);

		optical_flow_s optical_flow;
		const bool new_optical_flow = UpdateFlowSample(ekf2_timestamps, optical_flow);


		// run the EKF update and output
		const hrt_abstime ekf_update_start = hrt_absolute_time();

		if (_ekf.update()) {
			perf_set_elapsed(_ecl_ekf_update_full_perf, hrt_elapsed_time(&ekf_update_start));

			PublishLocalPosition(now);
			PublishOdometry(now, imu_sample_new);
			PublishGlobalPosition(now);
			PublishWindEstimate(now);

			// publish status/logging messages
			PublishBaroBias(now);
			PublishEkfDriftMetrics(now);
			PublishEventFlags(now);
			PublishStates(now);
			PublishStatus(now);
			PublishStatusFlags(now);
			PublishInnovations(now, imu_sample_new);
			PublishInnovationTestRatios(now);
			PublishInnovationVariances(now);
			PublishYawEstimatorStatus(now);

			UpdateAccelCalibration(now);
			UpdateGyroCalibration(now);
			UpdateMagCalibration(now);
			PublishSensorBias(now);

		} else {
			// ekf no update
			perf_set_elapsed(_ecl_ekf_update_perf, hrt_elapsed_time(&ekf_update_start));
		}

		// publish external visual odometry after fixed frame alignment if new odometry is received
		if (new_ev_odom) {
			PublishOdometryAligned(now, ev_odom);
		}

		if (new_optical_flow) {
			PublishOpticalFlowVel(now, optical_flow);
		}

		// publish ekf2_timestamps
		_ekf2_timestamps_pub.publish(ekf2_timestamps);
	}
}

void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp_sample = timestamp;
		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);
		att.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_attitude_pub.publish(att);

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att{};
		_attitude_pub.publish(att);
	}
}

void EKF2::PublishBaroBias(const hrt_abstime &timestamp)
{
	if (_device_id_baro != 0) {
		const BaroBiasEstimator::status &status = _ekf.getBaroBiasEstimatorStatus();

		if (fabsf(status.bias - _last_baro_bias_published) > 0.001f) {
			estimator_baro_bias_s baro_bias{};
			baro_bias.timestamp_sample = timestamp;
			baro_bias.baro_device_id = _device_id_baro;
			baro_bias.bias = status.bias;
			baro_bias.bias_var = status.bias_var;
			baro_bias.innov = status.innov;
			baro_bias.innov_var = status.innov_var;
			baro_bias.innov_test_ratio = status.innov_test_ratio;
			baro_bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
			_estimator_baro_bias_pub.publish(baro_bias);

			_last_baro_bias_published = status.bias;
		}
	}
}

void EKF2::PublishEkfDriftMetrics(const hrt_abstime &timestamp)
{
	// publish GPS drift data only when updated to minimise overhead
	float gps_drift[3];
	bool blocked;

	if (_ekf.get_gps_drift_metrics(gps_drift, &blocked)) {
		ekf_gps_drift_s drift_data;
		drift_data.hpos_drift_rate = gps_drift[0];
		drift_data.vpos_drift_rate = gps_drift[1];
		drift_data.hspd = gps_drift[2];
		drift_data.blocked = blocked;
		drift_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_ekf_gps_drift_pub.publish(drift_data);
	}
}

void EKF2::PublishEventFlags(const hrt_abstime &timestamp)
{
	// information events
	uint32_t information_events = _ekf.information_event_status().value;
	bool information_event_updated = false;

	if (information_events != 0) {
		information_event_updated = true;
		_filter_information_event_changes++;
	}

	// warning events
	uint32_t warning_events = _ekf.warning_event_status().value;
	bool warning_event_updated = false;

	if (warning_events != 0) {
		warning_event_updated = true;
		_filter_warning_event_changes++;
	}

	if (information_event_updated || warning_event_updated) {
		estimator_event_flags_s event_flags{};
		event_flags.timestamp_sample = timestamp;

		event_flags.information_event_changes           = _filter_information_event_changes;
		event_flags.gps_checks_passed                   = _ekf.information_event_flags().gps_checks_passed;
		event_flags.reset_vel_to_gps                    = _ekf.information_event_flags().reset_vel_to_gps;
		event_flags.reset_vel_to_flow                   = _ekf.information_event_flags().reset_vel_to_flow;
		event_flags.reset_vel_to_vision                 = _ekf.information_event_flags().reset_vel_to_vision;
		event_flags.reset_vel_to_zero                   = _ekf.information_event_flags().reset_vel_to_zero;
		event_flags.reset_pos_to_last_known             = _ekf.information_event_flags().reset_pos_to_last_known;
		event_flags.reset_pos_to_gps                    = _ekf.information_event_flags().reset_pos_to_gps;
		event_flags.reset_pos_to_vision                 = _ekf.information_event_flags().reset_pos_to_vision;
		event_flags.starting_gps_fusion                 = _ekf.information_event_flags().starting_gps_fusion;
		event_flags.starting_vision_pos_fusion          = _ekf.information_event_flags().starting_vision_pos_fusion;
		event_flags.starting_vision_vel_fusion          = _ekf.information_event_flags().starting_vision_vel_fusion;
		event_flags.starting_vision_yaw_fusion          = _ekf.information_event_flags().starting_vision_yaw_fusion;
		event_flags.yaw_aligned_to_imu_gps              = _ekf.information_event_flags().yaw_aligned_to_imu_gps;

		event_flags.warning_event_changes               = _filter_warning_event_changes;
		event_flags.gps_quality_poor                    = _ekf.warning_event_flags().gps_quality_poor;
		event_flags.gps_fusion_timout                   = _ekf.warning_event_flags().gps_fusion_timout;
		event_flags.gps_data_stopped                    = _ekf.warning_event_flags().gps_data_stopped;
		event_flags.gps_data_stopped_using_alternate    = _ekf.warning_event_flags().gps_data_stopped_using_alternate;
		event_flags.height_sensor_timeout               = _ekf.warning_event_flags().height_sensor_timeout;
		event_flags.stopping_navigation                 = _ekf.warning_event_flags().stopping_mag_use;
		event_flags.invalid_accel_bias_cov_reset        = _ekf.warning_event_flags().invalid_accel_bias_cov_reset;
		event_flags.bad_yaw_using_gps_course            = _ekf.warning_event_flags().bad_yaw_using_gps_course;
		event_flags.stopping_mag_use                    = _ekf.warning_event_flags().stopping_mag_use;
		event_flags.vision_data_stopped                 = _ekf.warning_event_flags().vision_data_stopped;
		event_flags.emergency_yaw_reset_mag_stopped     = _ekf.warning_event_flags().emergency_yaw_reset_mag_stopped;
		event_flags.emergency_yaw_reset_gps_yaw_stopped = _ekf.warning_event_flags().emergency_yaw_reset_gps_yaw_stopped;

		event_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_event_flags_pub.publish(event_flags);
	}

	_ekf.clear_information_events();
	_ekf.clear_warning_events();
}

void EKF2::PublishGlobalPosition(const hrt_abstime &timestamp)
{
	if (_ekf.global_position_is_valid() && !_preflt_checker.hasFailed()) {
		const Vector3f position{_ekf.getPosition()};

		// generate and publish global position data
		vehicle_global_position_s global_pos;
		global_pos.timestamp_sample = timestamp;

		// Position of local NED origin in GPS / WGS84 frame
		_ekf.global_origin().reproject(position(0), position(1), global_pos.lat, global_pos.lon);

		float delta_xy[2];
		_ekf.get_posNE_reset(delta_xy, &global_pos.lat_lon_reset_counter);

		global_pos.alt = -position(2) + _ekf.getEkfGlobalOriginAltitude(); // Altitude AMSL in meters
		global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

		// global altitude has opposite sign of local down position
		float delta_z;
		uint8_t z_reset_counter;
		_ekf.get_posD_reset(&delta_z, &z_reset_counter);
		global_pos.delta_alt = -delta_z;

		_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

		if (_ekf.isTerrainEstimateValid()) {
			// Terrain altitude in m, WGS84
			global_pos.terrain_alt = _ekf.getEkfGlobalOriginAltitude() - _ekf.getTerrainVertPos();
			global_pos.terrain_alt_valid = true;

		} else {
			global_pos.terrain_alt = NAN;
			global_pos.terrain_alt_valid = false;
		}

		global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning
		global_pos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_global_position_pub.publish(global_pos);
	}
}

void EKF2::PublishInnovations(const hrt_abstime &timestamp, const imuSample &imu)
{
	// publish estimator innovation data
	estimator_innovations_s innovations{};
	innovations.timestamp_sample = timestamp;
	_ekf.getGpsVelPosInnov(innovations.gps_hvel, innovations.gps_vvel, innovations.gps_hpos, innovations.gps_vpos);
	_ekf.getEvVelPosInnov(innovations.ev_hvel, innovations.ev_vvel, innovations.ev_hpos, innovations.ev_vpos);
	_ekf.getBaroHgtInnov(innovations.baro_vpos);
	_ekf.getRngHgtInnov(innovations.rng_vpos);
	_ekf.getAuxVelInnov(innovations.aux_hvel);
	_ekf.getFlowInnov(innovations.flow);
	_ekf.getHeadingInnov(innovations.heading);
	_ekf.getMagInnov(innovations.mag_field);
	_ekf.getDragInnov(innovations.drag);
	_ekf.getAirspeedInnov(innovations.airspeed);
	_ekf.getBetaInnov(innovations.beta);
	_ekf.getHaglInnov(innovations.hagl);
	// Not yet supported
	innovations.aux_vvel = NAN;

	innovations.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovations_pub.publish(innovations);

	// calculate noise filtered velocity innovations which are used for pre-flight checking
	if (_standby) {
		// TODO: move to run before publications
		_preflt_checker.setUsingGpsAiding(_ekf.control_status_flags().gps);
		_preflt_checker.setUsingFlowAiding(_ekf.control_status_flags().opt_flow);
		_preflt_checker.setUsingEvPosAiding(_ekf.control_status_flags().ev_pos);
		_preflt_checker.setUsingEvVelAiding(_ekf.control_status_flags().ev_vel);

		_preflt_checker.update(imu.delta_ang_dt, innovations);
	}
}

void EKF2::PublishInnovationTestRatios(const hrt_abstime &timestamp)
{
	// publish estimator innovation test ratio data
	estimator_innovations_s test_ratios{};
	test_ratios.timestamp_sample = timestamp;
	_ekf.getGpsVelPosInnovRatio(test_ratios.gps_hvel[0], test_ratios.gps_vvel, test_ratios.gps_hpos[0],
				    test_ratios.gps_vpos);
	_ekf.getEvVelPosInnovRatio(test_ratios.ev_hvel[0], test_ratios.ev_vvel, test_ratios.ev_hpos[0], test_ratios.ev_vpos);
	_ekf.getBaroHgtInnovRatio(test_ratios.baro_vpos);
	_ekf.getRngHgtInnovRatio(test_ratios.rng_vpos);
	_ekf.getAuxVelInnovRatio(test_ratios.aux_hvel[0]);
	_ekf.getFlowInnovRatio(test_ratios.flow[0]);
	_ekf.getHeadingInnovRatio(test_ratios.heading);
	_ekf.getMagInnovRatio(test_ratios.mag_field[0]);
	_ekf.getDragInnovRatio(&test_ratios.drag[0]);
	_ekf.getAirspeedInnovRatio(test_ratios.airspeed);
	_ekf.getBetaInnovRatio(test_ratios.beta);
	_ekf.getHaglInnovRatio(test_ratios.hagl);
	// Not yet supported
	test_ratios.aux_vvel = NAN;

	test_ratios.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_test_ratios_pub.publish(test_ratios);
}

void EKF2::PublishInnovationVariances(const hrt_abstime &timestamp)
{
	// publish estimator innovation variance data
	estimator_innovations_s variances{};
	variances.timestamp_sample = timestamp;
	_ekf.getGpsVelPosInnovVar(variances.gps_hvel, variances.gps_vvel, variances.gps_hpos, variances.gps_vpos);
	_ekf.getEvVelPosInnovVar(variances.ev_hvel, variances.ev_vvel, variances.ev_hpos, variances.ev_vpos);
	_ekf.getBaroHgtInnovVar(variances.baro_vpos);
	_ekf.getRngHgtInnovVar(variances.rng_vpos);
	_ekf.getAuxVelInnovVar(variances.aux_hvel);
	_ekf.getFlowInnovVar(variances.flow);
	_ekf.getHeadingInnovVar(variances.heading);
	_ekf.getMagInnovVar(variances.mag_field);
	_ekf.getDragInnovVar(variances.drag);
	_ekf.getAirspeedInnovVar(variances.airspeed);
	_ekf.getBetaInnovVar(variances.beta);
	_ekf.getHaglInnovVar(variances.hagl);
	// Not yet supported
	variances.aux_vvel = NAN;

	variances.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_innovation_variances_pub.publish(variances);
}

void EKF2::PublishLocalPosition(const hrt_abstime &timestamp)
{
	vehicle_local_position_s lpos;
	// generate vehicle local position data
	lpos.timestamp_sample = timestamp;

	// Position of body origin in local NED frame
	const Vector3f position{_ekf.getPosition()};
	lpos.x = position(0);
	lpos.y = position(1);
	lpos.z = position(2);

	// Velocity of body origin in local NED frame (m/s)
	const Vector3f velocity{_ekf.getVelocity()};
	lpos.vx = velocity(0);
	lpos.vy = velocity(1);
	lpos.vz = velocity(2);

	// vertical position time derivative (m/s)
	lpos.z_deriv = _ekf.getVerticalPositionDerivative();

	// Acceleration of body origin in local frame
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()};
	lpos.ax = vel_deriv(0);
	lpos.ay = vel_deriv(1);
	lpos.az = vel_deriv(2);

	// TODO: better status reporting
	lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
	lpos.z_valid = !_preflt_checker.hasVertFailed();
	lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_checker.hasHorizFailed();
	lpos.v_z_valid = !_preflt_checker.hasVertFailed();

	// Position of local NED origin in GPS / WGS84 frame
	if (_ekf.global_origin_valid()) {
		lpos.ref_timestamp = _ekf.global_origin().getProjectionReferenceTimestamp();
		lpos.ref_lat = _ekf.global_origin().getProjectionReferenceLat(); // Reference point latitude in degrees
		lpos.ref_lon = _ekf.global_origin().getProjectionReferenceLon(); // Reference point longitude in degrees
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude();           // Reference point in MSL altitude meters
		lpos.xy_global = true;
		lpos.z_global = true;

	} else {
		lpos.ref_timestamp = 0;
		lpos.ref_lat = static_cast<double>(NAN);
		lpos.ref_lon = static_cast<double>(NAN);
		lpos.ref_alt = NAN;
		lpos.xy_global = false;
		lpos.z_global = false;
	}

	Quatf delta_q_reset;
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter);

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi();
	lpos.delta_heading = Eulerf(delta_q_reset).psi();
	lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete();

	// Distance to bottom surface (ground) in meters
	// constrain the distance to ground to _rng_gnd_clearance
	lpos.dist_bottom = math::max(_ekf.getTerrainVertPos() - lpos.z, _param_ekf2_min_rng.get());
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();
	lpos.dist_bottom_sensor_bitfield = _ekf.getTerrainEstimateSensorBitfield();

	if (!_had_valid_terrain) {
		_had_valid_terrain = lpos.dist_bottom_valid;
	}

	// only consider ground effect if compensation is configured and the vehicle is armed (props spinning)
	if ((_param_ekf2_gnd_eff_dz.get() > 0.0f) && _armed && lpos.dist_bottom_valid) {
		// set ground effect flag if vehicle is closer than a specified distance to the ground
		_ekf.set_gnd_effect_flag(lpos.dist_bottom < _param_ekf2_gnd_max_hgt.get());

		// if we have no valid terrain estimate and never had one then use ground effect flag from land detector
		// _had_valid_terrain is used to make sure that we don't fall back to using this option
		// if we temporarily lose terrain data due to the distance sensor getting out of range
	}

	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

	// get state reset information of position and velocity
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

	// get control limit information
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

	// convert NaN to INFINITY
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY;
	}

	if (!PX4_ISFINITE(lpos.hagl_max)) {
		lpos.hagl_max = INFINITY;
	}

	// publish vehicle local position data
	lpos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_local_position_pub.publish(lpos);
}

void EKF2::PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu)
{
	// generate vehicle odometry data
	vehicle_odometry_s odom;
	odom.timestamp_sample = imu.time_us;

	odom.local_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// Vehicle odometry position
	const Vector3f position{_ekf.getPosition()};
	odom.x = position(0);
	odom.y = position(1);
	odom.z = position(2);

	// Vehicle odometry linear velocity
	odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_FRD;
	const Vector3f velocity{_ekf.getVelocity()};
	odom.vx = velocity(0);
	odom.vy = velocity(1);
	odom.vz = velocity(2);

	// Vehicle odometry quaternion
	_ekf.getQuaternion().copyTo(odom.q);

	// Vehicle odometry angular rates
	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f rates{imu.delta_ang / imu.delta_ang_dt};
	odom.rollspeed = rates(0) - gyro_bias(0);
	odom.pitchspeed = rates(1) - gyro_bias(1);
	odom.yawspeed = rates(2) - gyro_bias(2);

	// get the covariance matrix size
	static constexpr size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
	static constexpr size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);

	// Get covariances to vehicle odometry
	float covariances[24];
	_ekf.covariances_diagonal().copyTo(covariances);

	// initially set pose covariances to 0
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odom.pose_covariance[i] = 0.0;
	}

	// set the position variances
	odom.pose_covariance[odom.COVARIANCE_MATRIX_X_VARIANCE] = covariances[7];
	odom.pose_covariance[odom.COVARIANCE_MATRIX_Y_VARIANCE] = covariances[8];
	odom.pose_covariance[odom.COVARIANCE_MATRIX_Z_VARIANCE] = covariances[9];

	// TODO: implement propagation from quaternion covariance to Euler angle covariance
	// by employing the covariance law

	// initially set velocity covariances to 0
	for (size_t i = 0; i < VEL_URT_SIZE; i++) {
		odom.velocity_covariance[i] = 0.0;
	}

	// set the linear velocity variances
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VX_VARIANCE] = covariances[4];
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VY_VARIANCE] = covariances[5];
	odom.velocity_covariance[odom.COVARIANCE_MATRIX_VZ_VARIANCE] = covariances[6];

	// publish vehicle odometry data
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_odometry_pub.publish(odom);
}

void EKF2::PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom)
{
	const Quatf quat_ev2ekf = _ekf.getVisionAlignmentQuaternion(); // rotates from EV to EKF navigation frame
	const Dcmf ev_rot_mat(quat_ev2ekf);

	vehicle_odometry_s aligned_ev_odom{ev_odom};

	// Rotate external position and velocity into EKF navigation frame
	const Vector3f aligned_pos = ev_rot_mat * Vector3f(ev_odom.x, ev_odom.y, ev_odom.z);
	aligned_ev_odom.x = aligned_pos(0);
	aligned_ev_odom.y = aligned_pos(1);
	aligned_ev_odom.z = aligned_pos(2);

	switch (ev_odom.velocity_frame) {
	case vehicle_odometry_s::BODY_FRAME_FRD: {
			const Vector3f aligned_vel = Dcmf(_ekf.getQuaternion()) * Vector3f(ev_odom.vx, ev_odom.vy, ev_odom.vz);
			aligned_ev_odom.vx = aligned_vel(0);
			aligned_ev_odom.vy = aligned_vel(1);
			aligned_ev_odom.vz = aligned_vel(2);
			break;
		}

	case vehicle_odometry_s::LOCAL_FRAME_FRD: {
			const Vector3f aligned_vel = ev_rot_mat * Vector3f(ev_odom.vx, ev_odom.vy, ev_odom.vz);
			aligned_ev_odom.vx = aligned_vel(0);
			aligned_ev_odom.vy = aligned_vel(1);
			aligned_ev_odom.vz = aligned_vel(2);
			break;
		}
	}

	aligned_ev_odom.velocity_frame = vehicle_odometry_s::LOCAL_FRAME_NED;

	// Compute orientation in EKF navigation frame
	Quatf ev_quat_aligned = quat_ev2ekf * Quatf(ev_odom.q) ;
	ev_quat_aligned.normalize();

	ev_quat_aligned.copyTo(aligned_ev_odom.q);
	quat_ev2ekf.copyTo(aligned_ev_odom.q_offset);

	_estimator_visual_odometry_aligned_pub.publish(aligned_ev_odom);
}

void EKF2::PublishSensorBias(const hrt_abstime &timestamp)
{
	// estimator_sensor_bias
	estimator_sensor_bias_s bias{};
	bias.timestamp_sample = timestamp;

	const Vector3f gyro_bias{_ekf.getGyroBias()};
	const Vector3f accel_bias{_ekf.getAccelBias()};
	const Vector3f mag_bias{_ekf.getMagBias()};

	// only publish on change
	if ((gyro_bias - _last_gyro_bias_published).longerThan(0.001f)
	    || (accel_bias - _last_accel_bias_published).longerThan(0.001f)
	    || (mag_bias - _last_mag_bias_published).longerThan(0.001f)) {

		// take device ids from sensor_selection_s if not using specific vehicle_imu_s
		if (_device_id_gyro != 0) {
			bias.gyro_device_id = _device_id_gyro;
			gyro_bias.copyTo(bias.gyro_bias);
			bias.gyro_bias_limit = math::radians(20.f); // 20 degrees/s see Ekf::constrainStates()
			_ekf.getGyroBiasVariance().copyTo(bias.gyro_bias_variance);
			bias.gyro_bias_valid = true;  // TODO
			bias.gyro_bias_stable = _gyro_cal.cal_available;
			_last_gyro_bias_published = gyro_bias;
		}

		if ((_device_id_accel != 0) && !(_param_ekf2_aid_mask.get() & MASK_INHIBIT_ACC_BIAS)) {
			bias.accel_device_id = _device_id_accel;
			accel_bias.copyTo(bias.accel_bias);
			bias.accel_bias_limit = _params->acc_bias_lim;
			_ekf.getAccelBiasVariance().copyTo(bias.accel_bias_variance);
			bias.accel_bias_valid = true;  // TODO
			bias.accel_bias_stable = _accel_cal.cal_available;
			_last_accel_bias_published = accel_bias;
		}

		if (_device_id_mag != 0) {
			bias.mag_device_id = _device_id_mag;
			mag_bias.copyTo(bias.mag_bias);
			bias.mag_bias_limit = 0.5f; // 0.5 Gauss see Ekf::constrainStates()
			_ekf.getMagBiasVariance().copyTo(bias.mag_bias_variance);
			bias.mag_bias_valid = true; // TODO
			bias.mag_bias_stable = _mag_cal.cal_available;
			_last_mag_bias_published = mag_bias;
		}

		bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_sensor_bias_pub.publish(bias);
	}
}

void EKF2::PublishStates(const hrt_abstime &timestamp)
{
	// publish estimator states
	estimator_states_s states;
	states.timestamp_sample = timestamp;
	states.n_states = Ekf::_k_num_states;
	_ekf.getStateAtFusionHorizonAsVector().copyTo(states.states);
	_ekf.covariances_diagonal().copyTo(states.covariances);
	states.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_states_pub.publish(states);
}

void EKF2::PublishStatus(const hrt_abstime &timestamp)
{
	estimator_status_s status{};
	status.timestamp_sample = timestamp;

	_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);

	_ekf.get_gps_check_status(&status.gps_check_fail_flags);

	// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
	// the GPS Fix bit, which is always checked)
	status.gps_check_fail_flags &= ((uint16_t)_params->gps_check_mask << 1) | 1;

	status.control_mode_flags = _ekf.control_status().value;
	status.filter_fault_flags = _ekf.fault_status().value;

	uint16_t innov_check_flags_temp = 0;
	_ekf.get_innovation_test_status(innov_check_flags_temp, status.mag_test_ratio,
					status.vel_test_ratio, status.pos_test_ratio,
					status.hgt_test_ratio, status.tas_test_ratio,
					status.hagl_test_ratio, status.beta_test_ratio);

	// Bit mismatch between ecl and Firmware, combine the 2 first bits to preserve msg definition
	// TODO: legacy use only, those flags are also in estimator_status_flags
	status.innovation_check_flags = (innov_check_flags_temp >> 1) | (innov_check_flags_temp & 0x1);

	_ekf.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy, &status.pos_vert_accuracy);
	_ekf.get_ekf_soln_status(&status.solution_status_flags);
	_ekf.getImuVibrationMetrics().copyTo(status.vibe);

	// reset counters
	status.reset_count_vel_ne = _ekf.state_reset_status().velNE_counter;
	status.reset_count_vel_d = _ekf.state_reset_status().velD_counter;
	status.reset_count_pos_ne = _ekf.state_reset_status().posNE_counter;
	status.reset_count_pod_d = _ekf.state_reset_status().posD_counter;
	status.reset_count_quat = _ekf.state_reset_status().quat_counter;

	status.time_slip = _last_time_slip_us * 1e-6f;

	status.pre_flt_fail_innov_heading = _preflt_checker.hasHeadingFailed();
	status.pre_flt_fail_innov_vel_horiz = _preflt_checker.hasHorizVelFailed();
	status.pre_flt_fail_innov_vel_vert = _preflt_checker.hasVertVelFailed();
	status.pre_flt_fail_innov_height = _preflt_checker.hasHeightFailed();
	status.pre_flt_fail_mag_field_disturbed = _ekf.control_status_flags().mag_field_disturbed;

	status.accel_device_id = _device_id_accel;
	status.baro_device_id = _device_id_baro;
	status.gyro_device_id = _device_id_gyro;
	status.mag_device_id = _device_id_mag;

	status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_status_pub.publish(status);
}

void EKF2::PublishStatusFlags(const hrt_abstime &timestamp)
{
	// publish at ~ 1 Hz (or immediately if filter control status or fault status changes)
	bool update = (hrt_elapsed_time(&_last_status_flag_update) >= 1_s);

	// filter control status
	if (_ekf.control_status().value != _filter_control_status) {
		update = true;
		_filter_control_status = _ekf.control_status().value;
		_filter_control_status_changes++;
	}

	// filter fault status
	if (_ekf.fault_status().value != _filter_fault_status) {
		update = true;
		_filter_fault_status = _ekf.fault_status().value;
		_filter_fault_status_changes++;
	}

	// innovation check fail status
	if (_ekf.innov_check_fail_status().value != _innov_check_fail_status) {
		update = true;
		_innov_check_fail_status = _ekf.innov_check_fail_status().value;
		_innov_check_fail_status_changes++;
	}

	if (update) {
		estimator_status_flags_s status_flags{};
		status_flags.timestamp_sample = timestamp;

		status_flags.control_status_changes   = _filter_control_status_changes;
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align;
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align;
		status_flags.cs_gps                   = _ekf.control_status_flags().gps;
		status_flags.cs_opt_flow              = _ekf.control_status_flags().opt_flow;
		status_flags.cs_mag_hdg               = _ekf.control_status_flags().mag_hdg;
		status_flags.cs_mag_3d                = _ekf.control_status_flags().mag_3D;
		status_flags.cs_mag_dec               = _ekf.control_status_flags().mag_dec;
		status_flags.cs_in_air                = _ekf.control_status_flags().in_air;
		status_flags.cs_wind                  = _ekf.control_status_flags().wind;
		status_flags.cs_baro_hgt              = _ekf.control_status_flags().baro_hgt;
		status_flags.cs_rng_hgt               = _ekf.control_status_flags().rng_hgt;
		status_flags.cs_gps_hgt               = _ekf.control_status_flags().gps_hgt;
		status_flags.cs_ev_pos                = _ekf.control_status_flags().ev_pos;
		status_flags.cs_ev_yaw                = _ekf.control_status_flags().ev_yaw;
		status_flags.cs_ev_hgt                = _ekf.control_status_flags().ev_hgt;
		status_flags.cs_fuse_beta             = _ekf.control_status_flags().fuse_beta;
		status_flags.cs_mag_field_disturbed   = _ekf.control_status_flags().mag_field_disturbed;
		status_flags.cs_fixed_wing            = _ekf.control_status_flags().fixed_wing;
		status_flags.cs_mag_fault             = _ekf.control_status_flags().mag_fault;
		status_flags.cs_fuse_aspd             = _ekf.control_status_flags().fuse_aspd;
		status_flags.cs_gnd_effect            = _ekf.control_status_flags().gnd_effect;
		status_flags.cs_rng_stuck             = _ekf.control_status_flags().rng_stuck;
		status_flags.cs_gps_yaw               = _ekf.control_status_flags().gps_yaw;
		status_flags.cs_mag_aligned_in_flight = _ekf.control_status_flags().mag_aligned_in_flight;
		status_flags.cs_ev_vel                = _ekf.control_status_flags().ev_vel;
		status_flags.cs_synthetic_mag_z       = _ekf.control_status_flags().synthetic_mag_z;
		status_flags.cs_vehicle_at_rest       = _ekf.control_status_flags().vehicle_at_rest;
		status_flags.cs_gps_yaw_fault         = _ekf.control_status_flags().gps_yaw_fault;

		status_flags.fault_status_changes     = _filter_fault_status_changes;
		status_flags.fs_bad_mag_x             = _ekf.fault_status_flags().bad_mag_x;
		status_flags.fs_bad_mag_y             = _ekf.fault_status_flags().bad_mag_y;
		status_flags.fs_bad_mag_z             = _ekf.fault_status_flags().bad_mag_z;
		status_flags.fs_bad_hdg               = _ekf.fault_status_flags().bad_hdg;
		status_flags.fs_bad_mag_decl          = _ekf.fault_status_flags().bad_mag_decl;
		status_flags.fs_bad_airspeed          = _ekf.fault_status_flags().bad_airspeed;
		status_flags.fs_bad_sideslip          = _ekf.fault_status_flags().bad_sideslip;
		status_flags.fs_bad_optflow_x         = _ekf.fault_status_flags().bad_optflow_X;
		status_flags.fs_bad_optflow_y         = _ekf.fault_status_flags().bad_optflow_Y;
		status_flags.fs_bad_vel_n             = _ekf.fault_status_flags().bad_vel_N;
		status_flags.fs_bad_vel_e             = _ekf.fault_status_flags().bad_vel_E;
		status_flags.fs_bad_vel_d             = _ekf.fault_status_flags().bad_vel_D;
		status_flags.fs_bad_pos_n             = _ekf.fault_status_flags().bad_pos_N;
		status_flags.fs_bad_pos_e             = _ekf.fault_status_flags().bad_pos_E;
		status_flags.fs_bad_pos_d             = _ekf.fault_status_flags().bad_pos_D;
		status_flags.fs_bad_acc_bias          = _ekf.fault_status_flags().bad_acc_bias;
		status_flags.fs_bad_acc_vertical      = _ekf.fault_status_flags().bad_acc_vertical;
		status_flags.fs_bad_acc_clipping      = _ekf.fault_status_flags().bad_acc_clipping;

		status_flags.innovation_fault_status_changes = _innov_check_fail_status_changes;
		status_flags.reject_hor_vel                  = _ekf.innov_check_fail_status_flags().reject_hor_vel;
		status_flags.reject_ver_vel                  = _ekf.innov_check_fail_status_flags().reject_ver_vel;
		status_flags.reject_hor_pos                  = _ekf.innov_check_fail_status_flags().reject_hor_pos;
		status_flags.reject_ver_pos                  = _ekf.innov_check_fail_status_flags().reject_ver_pos;
		status_flags.reject_mag_x                    = _ekf.innov_check_fail_status_flags().reject_mag_x;
		status_flags.reject_mag_y                    = _ekf.innov_check_fail_status_flags().reject_mag_y;
		status_flags.reject_mag_z                    = _ekf.innov_check_fail_status_flags().reject_mag_z;
		status_flags.reject_yaw                      = _ekf.innov_check_fail_status_flags().reject_yaw;
		status_flags.reject_airspeed                 = _ekf.innov_check_fail_status_flags().reject_airspeed;
		status_flags.reject_sideslip                 = _ekf.innov_check_fail_status_flags().reject_sideslip;
		status_flags.reject_hagl                     = _ekf.innov_check_fail_status_flags().reject_hagl;
		status_flags.reject_optflow_x                = _ekf.innov_check_fail_status_flags().reject_optflow_X;
		status_flags.reject_optflow_y                = _ekf.innov_check_fail_status_flags().reject_optflow_Y;

		status_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_estimator_status_flags_pub.publish(status_flags);

		_last_status_flag_update = status_flags.timestamp;
	}
}

void EKF2::PublishYawEstimatorStatus(const hrt_abstime &timestamp)
{
	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	yaw_estimator_status_s yaw_est_test_data;

	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       yaw_est_test_data.yaw,
			       yaw_est_test_data.innov_vn, yaw_est_test_data.innov_ve,
			       yaw_est_test_data.weight)) {

		yaw_est_test_data.timestamp_sample = timestamp;
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_yaw_est_pub.publish(yaw_est_test_data);
	}
}

void EKF2::PublishWindEstimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		// Publish wind estimate only if ekf declares them valid
		wind_s wind{};
		wind.timestamp_sample = timestamp;

		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();
		_ekf.getAirspeedInnov(wind.tas_innov);
		_ekf.getAirspeedInnovVar(wind.tas_innov_var);
		_ekf.getBetaInnov(wind.beta_innov);
		_ekf.getBetaInnovVar(wind.beta_innov_var);

		wind.windspeed_north = wind_vel(0);
		wind.windspeed_east = wind_vel(1);
		wind.variance_north = wind_vel_var(0);
		wind.variance_east = wind_vel_var(1);
		wind.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		_wind_pub.publish(wind);
	}
}

void EKF2::PublishOpticalFlowVel(const hrt_abstime &timestamp, const optical_flow_s &flow_sample)
{
	estimator_optical_flow_vel_s flow_vel{};
	flow_vel.timestamp_sample = flow_sample.timestamp;

	_ekf.getFlowVelBody().copyTo(flow_vel.vel_body);
	_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne);
	_ekf.getFlowUncompensated().copyTo(flow_vel.flow_uncompensated_integral);
	_ekf.getFlowCompensated().copyTo(flow_vel.flow_compensated_integral);
	_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate_integral);
	flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

	_estimator_optical_flow_vel_pub.publish(flow_vel);
}

float EKF2::filter_altitude_ellipsoid(float amsl_hgt)
{
	float height_diff = static_cast<float>(_gps_alttitude_ellipsoid) * 1e-3f - amsl_hgt;

	if (_gps_alttitude_ellipsoid_previous_timestamp == 0) {

		_wgs84_hgt_offset = height_diff;
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;

	} else if (_gps_time_usec != _gps_alttitude_ellipsoid_previous_timestamp) {

		// apply a 10 second first order low pass filter to baro offset
		float dt = 1e-6f * (_gps_time_usec - _gps_alttitude_ellipsoid_previous_timestamp);
		_gps_alttitude_ellipsoid_previous_timestamp = _gps_time_usec;
		float offset_rate_correction = 0.1f * (height_diff - _wgs84_hgt_offset);
		_wgs84_hgt_offset += dt * constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	return amsl_hgt + _wgs84_hgt_offset;
}

void EKF2::SelectImuStatus()
{
	for (uint8_t imu_instance = 0; imu_instance < MAX_NUM_IMUS; imu_instance++) {
		uORB::Subscription imu_status_sub{ORB_ID(vehicle_imu_status), imu_instance};

		vehicle_imu_status_s imu_status{};
		imu_status_sub.copy(&imu_status);

		if (imu_status.accel_device_id == _device_id_accel) {
			_vehicle_imu_status_sub.ChangeInstance(imu_instance);
			return;
		}
	}

	PX4_WARN("%d - IMU status not found for accel %" PRId32 ", gyro %" PRId32, _instance, _device_id_accel,
		 _device_id_gyro);
}

void EKF2::UpdateImuStatus()
{
	vehicle_imu_status_s imu_status;

	if (_vehicle_imu_status_sub.update(&imu_status)) {
		if (imu_status.accel_device_id != _device_id_accel) {
			SelectImuStatus();
			return;
		}

		// accel -> delta velocity
		_ekf.setDeltaVelocityHighFrequencyVibrationMetric(imu_status.accel_vibration_metric * _ekf.get_dt_imu_avg());

		// gyro -> delta angle
		_ekf.setDeltaAngleHighFrequencyVibrationMetric(imu_status.gyro_vibration_metric * _ekf.get_dt_imu_avg());
		_ekf.setDeltaAngleConingMetric(imu_status.gyro_coning_vibration * _ekf.get_dt_imu_avg());
	}
}

void EKF2::UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF airspeed sample
	const unsigned last_generation = _airspeed_sub.get_last_generation();
	airspeed_s airspeed;

	if (_airspeed_sub.update(&airspeed)) {
		if (_msg_missed_airspeed_perf == nullptr) {
			_msg_missed_airspeed_perf = perf_alloc(PC_COUNT, MODULE_NAME": airspeed messages missed");

		} else if (_airspeed_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_airspeed_perf);
		}

		// The airspeed measurement received via the airspeed.msg topic has not been corrected
		// for scale favtor errors and requires the ASPD_SCALE correction to be applied.
		// This could be avoided if true_airspeed_m_s from the airspeed-validated.msg topic
		// was used instead, however this would introduce a potential circular dependency
		// via the wind estimator that uses EKF velocity estimates.
		const float true_airspeed_m_s = airspeed.true_airspeed_m_s * _airspeed_scale_factor;

		airspeedSample airspeed_sample {
			.time_us = airspeed.timestamp,
			.true_airspeed = true_airspeed_m_s,
			.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s,
		};
		_ekf.setAirspeedData(airspeed_sample);

		ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF auxillary velocity sample
	//  - use the landing target pose estimate as another source of velocity data
	const unsigned last_generation = _landing_target_pose_sub.get_last_generation();
	landing_target_pose_s landing_target_pose;

	if (_landing_target_pose_sub.update(&landing_target_pose)) {
		if (_msg_missed_landing_target_pose_perf == nullptr) {
			_msg_missed_landing_target_pose_perf = perf_alloc(PC_COUNT, MODULE_NAME": landing_target_pose messages missed");

		} else if (_landing_target_pose_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_landing_target_pose_perf);
		}

		// we can only use the landing target if it has a fixed position and  a valid velocity estimate
		if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
			// velocity of vehicle relative to target has opposite sign to target relative to vehicle
			auxVelSample auxvel_sample{
				.time_us = landing_target_pose.timestamp,
				.vel = Vector3f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel, 0.0f},
				.velVar = Vector3f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel, 0.0f},
			};
			_ekf.setAuxVelData(auxvel_sample);
		}
	}
}

void EKF2::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF baro sample
	const unsigned last_generation = _airdata_sub.get_last_generation();
	vehicle_air_data_s airdata;

	if (_airdata_sub.update(&airdata)) {
		if (_msg_missed_air_data_perf == nullptr) {
			_msg_missed_air_data_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_air_data messages missed");

		} else if (_airdata_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_air_data_perf);
		}

		_ekf.set_air_density(airdata.rho);

		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter});

		_device_id_baro = airdata.baro_device_id;

		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

bool EKF2::UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps, vehicle_odometry_s &ev_odom)
{
	// EKF external vision sample
	bool new_ev_odom = false;
	const unsigned last_generation = _ev_odom_sub.get_last_generation();

	if (_ev_odom_sub.update(&ev_odom)) {
		if (_msg_missed_odometry_perf == nullptr) {
			_msg_missed_odometry_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_visual_odometry messages missed");

		} else if (_ev_odom_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_odometry_perf);
		}

		extVisionSample ev_data{};

		// if error estimates are unavailable, use parameter defined defaults

		// check for valid velocity data
		if (PX4_ISFINITE(ev_odom.vx) && PX4_ISFINITE(ev_odom.vy) && PX4_ISFINITE(ev_odom.vz)) {
			ev_data.vel(0) = ev_odom.vx;
			ev_data.vel(1) = ev_odom.vy;
			ev_data.vel(2) = ev_odom.vz;

			if (ev_odom.velocity_frame == vehicle_odometry_s::BODY_FRAME_FRD) {
				ev_data.vel_frame = velocity_frame_t::BODY_FRAME_FRD;

			} else {
				ev_data.vel_frame = velocity_frame_t::LOCAL_FRAME_FRD;
			}

			// velocity measurement error from ev_data or parameters
			float param_evv_noise_var = sq(_param_ekf2_evv_noise.get());

			if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VX_VARIANCE])
			    && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VY_VARIANCE])
			    && PX4_ISFINITE(ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE])) {
				ev_data.velCov(0, 0) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VX_VARIANCE];
				ev_data.velCov(0, 1) = ev_data.velCov(1, 0) = ev_odom.velocity_covariance[1];
				ev_data.velCov(0, 2) = ev_data.velCov(2, 0) = ev_odom.velocity_covariance[2];
				ev_data.velCov(1, 1) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VY_VARIANCE];
				ev_data.velCov(1, 2) = ev_data.velCov(2, 1) = ev_odom.velocity_covariance[7];
				ev_data.velCov(2, 2) = ev_odom.velocity_covariance[ev_odom.COVARIANCE_MATRIX_VZ_VARIANCE];

			} else {
				ev_data.velCov = matrix::eye<float, 3>() * param_evv_noise_var;
			}
		}

		// check for valid position data
		if (PX4_ISFINITE(ev_odom.x) && PX4_ISFINITE(ev_odom.y) && PX4_ISFINITE(ev_odom.z)) {
			ev_data.pos(0) = ev_odom.x;
			ev_data.pos(1) = ev_odom.y;
			ev_data.pos(2) = ev_odom.z;

			float param_evp_noise_var = sq(_param_ekf2_evp_noise.get());

			// position measurement error from ev_data or parameters
			if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE])
			    && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE])
			    && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE])) {
				ev_data.posVar(0) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE]);
				ev_data.posVar(1) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE]);
				ev_data.posVar(2) = fmaxf(param_evp_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE]);

			} else {
				ev_data.posVar.setAll(param_evp_noise_var);
			}
		}

		// check for valid orientation data
		if (PX4_ISFINITE(ev_odom.q[0])) {
			ev_data.quat = Quatf(ev_odom.q);

			// orientation measurement error from ev_data or parameters
			float param_eva_noise_var = sq(_param_ekf2_eva_noise.get());

			if (!_param_ekf2_ev_noise_md.get() && PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE])) {
				ev_data.angVar = fmaxf(param_eva_noise_var, ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE]);

			} else {
				ev_data.angVar = param_eva_noise_var;
			}
		}

		// use timestamp from external computer, clocks are synchronized when using MAVROS
		ev_data.time_us = ev_odom.timestamp_sample;
		_ekf.setExtVisionData(ev_data);

		new_ev_odom = true;

		ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_ev_odom;
}

bool EKF2::UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps, optical_flow_s &optical_flow)
{
	// EKF flow sample
	bool new_optical_flow = false;
	const unsigned last_generation = _optical_flow_sub.get_last_generation();

	if (_optical_flow_sub.update(&optical_flow)) {
		if (_msg_missed_optical_flow_perf == nullptr) {
			_msg_missed_optical_flow_perf = perf_alloc(PC_COUNT, MODULE_NAME": optical_flow messages missed");

		} else if (_optical_flow_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_optical_flow_perf);
		}

		flowSample flow {
			.time_us = optical_flow.timestamp,
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
			// is produced by a RH rotation of the image about the sensor axis.
			.flow_xy_rad = Vector2f{-optical_flow.pixel_flow_x_integral, -optical_flow.pixel_flow_y_integral},
			.gyro_xyz = Vector3f{-optical_flow.gyro_x_rate_integral, -optical_flow.gyro_y_rate_integral, -optical_flow.gyro_z_rate_integral},
			.dt = 1e-6f * (float)optical_flow.integration_timespan,
			.quality = optical_flow.quality,
		};

		if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
		    PX4_ISFINITE(optical_flow.pixel_flow_x_integral) &&
		    flow.dt < 1) {

			// Save sensor limits reported by the optical flow sensor
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			_ekf.setOpticalFlowData(flow);

			new_optical_flow = true;
		}

		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_optical_flow;
}

void EKF2::UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF GPS message
	const unsigned last_generation = _vehicle_gps_position_sub.get_last_generation();
	vehicle_gps_position_s vehicle_gps_position;

	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		if (_msg_missed_gps_perf == nullptr) {
			_msg_missed_gps_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_gps_position messages missed");

		} else if (_vehicle_gps_position_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_gps_perf);
		}

		gps_message gps_msg{
			.time_usec = vehicle_gps_position.timestamp,
			.lat = vehicle_gps_position.lat,
			.lon = vehicle_gps_position.lon,
			.alt = vehicle_gps_position.alt,
			.yaw = vehicle_gps_position.heading,
			.yaw_offset = vehicle_gps_position.heading_offset,
			.fix_type = vehicle_gps_position.fix_type,
			.eph = vehicle_gps_position.eph,
			.epv = vehicle_gps_position.epv,
			.sacc = vehicle_gps_position.s_variance_m_s,
			.vel_m_s = vehicle_gps_position.vel_m_s,
			.vel_ned = Vector3f{
				vehicle_gps_position.vel_n_m_s,
				vehicle_gps_position.vel_e_m_s,
				vehicle_gps_position.vel_d_m_s
			},
			.vel_ned_valid = vehicle_gps_position.vel_ned_valid,
			.nsats = vehicle_gps_position.satellites_used,
			.pdop = sqrtf(vehicle_gps_position.hdop *vehicle_gps_position.hdop
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
		};
		_ekf.setGpsData(gps_msg);

		_gps_time_usec = gps_msg.time_usec;
		_gps_alttitude_ellipsoid = vehicle_gps_position.alt_ellipsoid;
	}
}

void EKF2::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	const unsigned last_generation = _magnetometer_sub.get_last_generation();
	vehicle_magnetometer_s magnetometer;

	if (_magnetometer_sub.update(&magnetometer)) {
		if (_msg_missed_magnetometer_perf == nullptr) {
			_msg_missed_magnetometer_perf = perf_alloc(PC_COUNT, MODULE_NAME": vehicle_magnetometer messages missed");

		} else if (_magnetometer_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_magnetometer_perf);
		}

		bool reset = false;

		// check if magnetometer has changed
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				PX4_WARN("%d - mag sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_mag, magnetometer.device_id);
			}

			reset = true;

		} else if (magnetometer.calibration_count > _mag_calibration_count) {
			// existing calibration has changed, reset saved mag bias
			PX4_DEBUG("%d - mag %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_mag);
			reset = true;
		}

		if (reset) {
			_ekf.resetMagBias();
			_device_id_mag = magnetometer.device_id;
			_mag_calibration_count = magnetometer.calibration_count;

			// reset magnetometer bias learning
			_mag_cal = {};
		}

		_ekf.setMagData(magSample{magnetometer.timestamp_sample, Vector3f{magnetometer.magnetometer_ga}});

		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void EKF2::UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps)
{
	distance_sensor_s distance_sensor;

	if (_distance_sensor_selected < 0) {

		if (_distance_sensor_subs.advertised()) {
			for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

				if (_distance_sensor_subs[i].update(&distance_sensor)) {
					// only use the first instace which has the correct orientation
					if ((hrt_elapsed_time(&distance_sensor.timestamp) < 100_ms)
					    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

						int ndist = orb_group_count(ORB_ID(distance_sensor));

						if (ndist > 1) {
							PX4_INFO("%d - selected distance_sensor:%d (%d advertised)", _instance, i, ndist);
						}

						_distance_sensor_selected = i;
						_last_range_sensor_update = distance_sensor.timestamp;
						_distance_sensor_last_generation = _distance_sensor_subs[_distance_sensor_selected].get_last_generation() - 1;
						break;
					}
				}
			}
		}
	}

	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		// EKF range sample

		if (_msg_missed_distance_sensor_perf == nullptr) {
			_msg_missed_distance_sensor_perf = perf_alloc(PC_COUNT, MODULE_NAME": distance_sensor messages missed");

		} else if (_distance_sensor_subs[_distance_sensor_selected].get_last_generation() != _distance_sensor_last_generation +
			   1) {
			perf_count(_msg_missed_distance_sensor_perf);
		}

		_distance_sensor_last_generation = _distance_sensor_subs[_distance_sensor_selected].get_last_generation();

		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			rangeSample range_sample {
				.time_us = distance_sensor.timestamp,
				.rng = distance_sensor.current_distance,
				.quality = distance_sensor.signal_quality,
			};
			_ekf.setRangeData(range_sample);

			// Save sensor limits reported by the rangefinder
			_ekf.set_rangefinder_limits(distance_sensor.min_distance, distance_sensor.max_distance);

			_last_range_sensor_update = distance_sensor.timestamp;
			return;
		}

		ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)distance_sensor.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	if (hrt_elapsed_time(&_last_range_sensor_update) > 1_s) {
		_distance_sensor_selected = -1;
	}
}

void EKF2::UpdateAccelCalibration(const hrt_abstime &timestamp)
{
	// Check if conditions are OK for learning of accelerometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	if (_ekf.control_status_flags().in_air && (_ekf.fault_status().value == 0)
	    && !(_param_ekf2_aid_mask.get() & MASK_INHIBIT_ACC_BIAS)) {

		if (_accel_cal.last_us != 0) {
			_accel_cal.total_time_us += timestamp - _accel_cal.last_us;

			// Start checking accel bias estimates when we have accumulated sufficient calibration time
			if (_accel_cal.total_time_us > 30_s) {
				_accel_cal.last_bias = _ekf.getAccelBias();
				_accel_cal.last_bias_variance = _ekf.getAccelBiasVariance();
				_accel_cal.cal_available = true;
			}
		}

		_accel_cal.last_us = timestamp;

	} else {
		// conditions are NOT OK for learning accelerometer bias, reset timestamp
		// but keep the accumulated calibration time
		_accel_cal.last_us = 0;

		if (_ekf.fault_status().value != 0) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			_accel_cal.total_time_us = 0;
		}
	}
}

void EKF2::UpdateGyroCalibration(const hrt_abstime &timestamp)
{
	// Check if conditions are OK for learning of gyro bias values
	// the EKF is operating in the correct mode and there are no filter faults
	if (_ekf.control_status_flags().in_air && (_ekf.fault_status().value == 0)) {

		if (_gyro_cal.last_us != 0) {
			_gyro_cal.total_time_us += timestamp - _gyro_cal.last_us;

			// Start checking gyro bias estimates when we have accumulated sufficient calibration time
			if (_gyro_cal.total_time_us > 30_s) {
				_gyro_cal.last_bias = _ekf.getGyroBias();
				_gyro_cal.last_bias_variance = _ekf.getGyroBiasVariance();
				_gyro_cal.cal_available = true;
			}
		}

		_gyro_cal.last_us = timestamp;

	} else {
		// conditions are NOT OK for learning gyro bias, reset timestamp
		// but keep the accumulated calibration time
		_gyro_cal.last_us = 0;

		if (_ekf.fault_status().value != 0) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			_gyro_cal.total_time_us = 0;
		}
	}
}

void EKF2::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	// Check if conditions are OK for learning of magnetometer bias values
	// the EKF is operating in the correct mode and there are no filter faults
	if (_ekf.control_status_flags().in_air && _ekf.control_status_flags().mag_3D && (_ekf.fault_status().value == 0)) {

		if (_mag_cal.last_us != 0) {
			_mag_cal.total_time_us += timestamp - _mag_cal.last_us;

			// Start checking mag bias estimates when we have accumulated sufficient calibration time
			if (_mag_cal.total_time_us > 30_s) {
				_mag_cal.last_bias = _ekf.getMagBias();
				_mag_cal.last_bias_variance = _ekf.getMagBiasVariance();
				_mag_cal.cal_available = true;
			}
		}

		_mag_cal.last_us = timestamp;

	} else {
		// conditions are NOT OK for learning magnetometer bias, reset timestamp
		// but keep the accumulated calibration time
		_mag_cal.last_us = 0;

		if (_ekf.fault_status().value != 0) {
			// if a filter fault has occurred, assume previous learning was invalid and do not
			// count it towards total learning time.
			_mag_cal.total_time_us = 0;
		}
	}

	if (!_armed) {
		// update stored declination value
		if (!_mag_decl_saved) {
			float declination_deg;

			if (_ekf.get_mag_decl_deg(&declination_deg)) {
				_param_ekf2_mag_decl.update();

				if (PX4_ISFINITE(declination_deg) && (fabsf(declination_deg - _param_ekf2_mag_decl.get()) > 0.1f)) {
					_param_ekf2_mag_decl.set(declination_deg);
					_param_ekf2_mag_decl.commit_no_notification();
				}

				_mag_decl_saved = true;
			}
		}
	}
}

int EKF2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EKF2::task_spawn(int argc, char *argv[])
{
	bool success = false;
	bool replay_mode = false;

	if (argc > 1 && !strcmp(argv[1], "-r")) {
		PX4_INFO("replay mode enabled");
		replay_mode = true;
	}

#if !defined(CONSTRAINED_FLASH)
	bool multi_mode = false;
	int32_t imu_instances = 0;
	int32_t mag_instances = 0;

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		// ekf selector requires SENS_IMU_MODE = 0
		multi_mode = true;

		// IMUs (1 - 4 supported)
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances);

		if (imu_instances < 1 || imu_instances > 4) {
			const int32_t imu_instances_limited = math::constrain(imu_instances, static_cast<int32_t>(1), static_cast<int32_t>(4));
			PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32, imu_instances, imu_instances_limited);
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited);
			imu_instances = imu_instances_limited;
		}

		int32_t sens_mag_mode = 1;
		param_get(param_find("SENS_MAG_MODE"), &sens_mag_mode);

		if (sens_mag_mode == 0) {
			param_get(param_find("EKF2_MULTI_MAG"), &mag_instances);

			// Mags (1 - 4 supported)
			if (mag_instances < 1 || mag_instances > 4) {
				const int32_t mag_instances_limited = math::constrain(mag_instances, static_cast<int32_t>(1), static_cast<int32_t>(4));
				PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32, mag_instances, mag_instances_limited);
				param_set_no_notification(param_find("EKF2_MULTI_MAG"), &mag_instances_limited);
				mag_instances = mag_instances_limited;
			}

		} else {
			mag_instances = 1;
		}
	}

	if (multi_mode) {
		// Start EKF2Selector if it's not already running
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector();

			if (inst) {
				_ekf2_selector.store(inst);

			} else {
				PX4_ERR("Failed to create EKF2 selector");
				return PX4_ERROR;
			}
		}

		const hrt_abstime time_started = hrt_absolute_time();
		const int multi_instances = math::min(imu_instances * mag_instances, static_cast<int32_t>(EKF2_MAX_INSTANCES));
		int multi_instances_allocated = 0;

		// allocate EKF2 instances until all found or arming
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};

		bool ekf2_instance_created[MAX_NUM_IMUS][MAX_NUM_MAGS] {}; // IMUs * mags

		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED)
		       && ((hrt_elapsed_time(&time_started) < 30_s)
			   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) {

			vehicle_status_sub.update();

			for (uint8_t mag = 0; mag < mag_instances; mag++) {
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag};

				for (uint8_t imu = 0; imu < imu_instances; imu++) {

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu};
					vehicle_mag_sub.update();

					// Mag & IMU data must be valid, first mag can be ignored initially
					if ((vehicle_mag_sub.get().device_id != 0 || mag == 0)
					    && (vehicle_imu_sub.get().accel_device_id != 0)
					    && (vehicle_imu_sub.get().gyro_device_id != 0)) {

						if (!ekf2_instance_created[imu][mag]) {
							EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false);

							if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) {
								int actual_instance = ekf2_inst->instance(); // match uORB instance numbering

								if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) {
									_objects[actual_instance].store(ekf2_inst);
									success = true;
									multi_instances_allocated++;
									ekf2_instance_created[imu][mag] = true;

									if (actual_instance == 0) {
										// force selector to run immediately if first instance started
										_ekf2_selector.load()->ScheduleNow();
									}

									PX4_DEBUG("starting instance %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")", actual_instance,
										  imu, vehicle_imu_sub.get().accel_device_id,
										  mag, vehicle_mag_sub.get().device_id);

									// sleep briefly before starting more instances
									px4_usleep(10000);

								} else {
									PX4_ERR("instance numbering problem instance: %d", actual_instance);
									delete ekf2_inst;
									break;
								}

							} else {
								PX4_ERR("alloc and init failed imu: %" PRIu8 " mag:%" PRIu8, imu, mag);
								px4_usleep(1000000);
								break;
							}
						}

					} else {
						px4_usleep(50000); // give the sensors extra time to start
						continue;
					}
				}
			}

			if (multi_instances_allocated < multi_instances) {
				px4_usleep(100000);
			}
		}

	}

#endif // !CONSTRAINED_FLASH

	else {
		// otherwise launch regular
		EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

		if (ekf2_inst) {
			_objects[0].store(ekf2_inst);
			ekf2_inst->ScheduleNow();
			success = true;
		}
	}

	return success ? PX4_OK : PX4_ERROR;
}

int EKF2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode, it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
#if !defined(CONSTRAINED_FLASH)
	PRINT_MODULE_USAGE_COMMAND_DESCR("select_instance", "Request switch to new estimator instance");
	PRINT_MODULE_USAGE_ARG("<instance>", "Specify desired estimator instance", false);
#endif // !CONSTRAINED_FLASH
	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return EKF2::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = 0;
		EKF2::lock_module();

		ret = EKF2::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		EKF2::unlock_module();
		return ret;

#if !defined(CONSTRAINED_FLASH)
	} else if (strcmp(argv[1], "select_instance") == 0) {

		if (EKF2::trylock_module()) {
			if (_ekf2_selector.load()) {
				if (argc > 2) {
					int instance = atoi(argv[2]);
					_ekf2_selector.load()->RequestInstance(instance);
				} else {
					EKF2::unlock_module();
					return EKF2::print_usage("instance required");
				}

			} else {
				PX4_ERR("multi-EKF not active, unable to select instance");
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;
#endif // !CONSTRAINED_FLASH
	} else if (strcmp(argv[1], "status") == 0) {
		if (EKF2::trylock_module()) {
#if !defined(CONSTRAINED_FLASH)
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus();
			}
#endif // !CONSTRAINED_FLASH

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
				}
			}

			EKF2::unlock_module();

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		EKF2::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				EKF2 *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

#if !defined(CONSTRAINED_FLASH)
			if (_ekf2_selector.load()) {
				PX4_INFO("stopping ekf2 selector");
				_ekf2_selector.load()->Stop();
				delete _ekf2_selector.load();
				_ekf2_selector.store(nullptr);
				was_running = true;
			}
#endif // !CONSTRAINED_FLASH

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				EKF2 *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping ekf2 instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		EKF2::unlock_module();
		return PX4_OK;
	}

	EKF2::lock_module(); // Lock here, as the method could access _object.
	int ret = EKF2::custom_command(argc - 1, argv + 1);
	EKF2::unlock_module();

	return ret;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file EKF2.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF/ekf.h"
#include "Utility/PreFlightChecker.hpp"

#include "EKF2Selector.hpp"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/ekf_gps_drift.h>
#include <uORB/topics/estimator_baro_bias.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_optical_flow_vel.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/yaw_estimator_status.h>


extern pthread_mutex_t ekf2_module_mutex;

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2() = delete;
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
	~EKF2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status();

	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

	bool multi_init(int imu, int mag);

	int instance() const { return _instance; }

private:

	static constexpr uint8_t MAX_NUM_IMUS = 4;
	static constexpr uint8_t MAX_NUM_MAGS = 4;

	void Run() override;

	void PublishAttitude(const hrt_abstime &timestamp);
	void PublishBaroBias(const hrt_abstime &timestamp);
	void PublishEkfDriftMetrics(const hrt_abstime &timestamp);
	void PublishEventFlags(const hrt_abstime &timestamp);
	void PublishGlobalPosition(const hrt_abstime &timestamp);
	void PublishInnovations(const hrt_abstime &timestamp, const imuSample &imu);
	void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	void PublishInnovationVariances(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu);
	void PublishOdometryAligned(const hrt_abstime &timestamp, const vehicle_odometry_s &ev_odom);
	void PublishOpticalFlowVel(const hrt_abstime &timestamp, const optical_flow_s &optical_flow);
	void PublishSensorBias(const hrt_abstime &timestamp);
	void PublishStates(const hrt_abstime &timestamp);
	void PublishStatus(const hrt_abstime &timestamp);
	void PublishStatusFlags(const hrt_abstime &timestamp);
	void PublishWindEstimate(const hrt_abstime &timestamp);
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);

	void SelectImuStatus();

	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps, vehicle_odometry_s &ev_odom);
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps, optical_flow_s &optical_flow);
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateImuStatus();

	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
	void UpdateMagCalibration(const hrt_abstime &timestamp);


	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	static constexpr float sq(float x) { return x * x; };

	const bool _replay_mode{false};			///< true when we use replay data from a log
	const bool _multi_mode;
	int _instance{0};

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _ecl_ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ECL update")};
	perf_counter_t _ecl_ekf_update_full_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": ECL full update")};
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")};
	perf_counter_t _msg_missed_air_data_perf{nullptr};
	perf_counter_t _msg_missed_airspeed_perf{nullptr};
	perf_counter_t _msg_missed_distance_sensor_perf{nullptr};
	perf_counter_t _msg_missed_gps_perf{nullptr};
	perf_counter_t _msg_missed_landing_target_pose_perf{nullptr};
	perf_counter_t _msg_missed_magnetometer_perf{nullptr};
	perf_counter_t _msg_missed_odometry_perf{nullptr};
	perf_counter_t _msg_missed_optical_flow_perf{nullptr};

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	// Used to check, save and use learned accel/gyro/mag biases
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		Vector3f last_bias{};           ///< last valid XYZ accelerometer bias estimates (Gauss)
		Vector3f last_bias_variance{};  ///< variances for the last valid accelerometer XYZ bias estimates (m/s**2)**2
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};
	InFlightCalibration _mag_cal{};

	bool _had_valid_terrain{false};			///< true if at any time there was a valid terrain estimate

	uint64_t _gps_time_usec{0};
	int32_t _gps_alttitude_ellipsoid{0};			///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp{0}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	uint8_t _accel_calibration_count{0};
	uint8_t _gyro_calibration_count{0};
	uint8_t _mag_calibration_count{0};

	uint32_t _device_id_accel{0};
	uint32_t _device_id_baro{0};
	uint32_t _device_id_gyro{0};
	uint32_t _device_id_mag{0};

	Vector3f _last_accel_bias_published{};
	Vector3f _last_gyro_bias_published{};
	Vector3f _last_mag_bias_published{};

	Vector3f _last_accel_calibration_published{};
	Vector3f _last_gyro_calibration_published{};
	Vector3f _last_mag_calibration_published{};

	float _last_baro_bias_published{};

	float _airspeed_scale_factor{1.0f}; ///< scale factor correction applied to airspeed measurements

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription _landing_target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _optical_flow_sub{ORB_ID(optical_flow)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	int _distance_sensor_selected{-1}; // because we can have several distance sensor instances with different orientations
	unsigned _distance_sensor_last_generation{0};

	bool _callback_registered{false};

	bool _armed{false};
	bool _standby{false}; // standby arming state

	hrt_abstime _last_status_flag_update{0};
	hrt_abstime _last_range_sensor_update{0};

	uint32_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _innov_check_fail_status{0};

	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uint32_t _innov_check_fail_status_changes{0};
	uint32_t _filter_warning_event_changes{0};
	uint32_t _filter_information_event_changes{0};

	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::PublicationMulti<ekf_gps_drift_s>              _ekf_gps_drift_pub{ORB_ID(ekf_gps_drift)};
	uORB::PublicationMulti<estimator_baro_bias_s>        _estimator_baro_bias_pub{ORB_ID(estimator_baro_bias)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::PublicationMulti<estimator_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)};
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_event_flags_s>      _estimator_event_flags_pub{ORB_ID(estimator_event_flags)};
	uORB::PublicationMulti<vehicle_odometry_s>           _estimator_visual_odometry_aligned_pub{ORB_ID(estimator_visual_odometry_aligned)};
	uORB::PublicationMulti<yaw_estimator_status_s>       _yaw_est_pub{ORB_ID(yaw_estimator_status)};

	// publications with topic dependent on multi-mode
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub;
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub;
	uORB::PublicationMulti<wind_s>              _wind_pub;


	PreFlightChecker _preflt_checker;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	DEFINE_PARAMETERS(
		(ParamExtInt<px4::params::EKF2_MIN_OBS_DT>)
		_param_ekf2_min_obs_dt,	///< Maximum time delay of any sensor used to increase buffer length to handle large timing jitter (mSec)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>)
		_param_ekf2_mag_delay,	///< magnetometer measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>)
		_param_ekf2_baro_delay,	///< barometer height measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>)
		_param_ekf2_gps_delay,	///< GPS measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay,	///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>)
		_param_ekf2_rng_delay,	///< range finder measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay,	///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay,	///< off-board vision measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxillary velocity measurement delay relative to the IMU (mSec)

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>)
		_param_ekf2_mag_e_noise,	///< process noise for earth magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>)
		_param_ekf2_mag_b_noise,	///< process noise for body magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_WIND_NOISE>)
		_param_ekf2_wind_noise,	///< process noise for wind velocity prediction (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise,	///< process noise for terrain offset (m/sec)
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>)
		_param_ekf2_terr_grad,	///< gradient of terrain used to estimate process noise due to changing position (m/m)

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>)
		_param_ekf2_gps_v_noise,	///< minimum allowed observation noise for gps velocity fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>)
		_param_ekf2_gps_p_noise,	///< minimum allowed observation noise for gps position fusion (m)
		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>)
		_param_ekf2_noaid_noise,	///< observation noise for non-aiding position fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>)
		_param_ekf2_baro_noise,	///< observation noise for barometric height fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>)
		_param_ekf2_baro_gate,	///< barometric height innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>)
		_param_ekf2_gnd_eff_dz,	///< barometric deadzone range for negative innovations (m)
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>)
		_param_ekf2_gnd_max_hgt,	///< maximum height above the ground level for expected negative baro innovations (m)
		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>)
		_param_ekf2_gps_p_gate,	///< GPS horizontal position innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>)
		_param_ekf2_gps_v_gate,	///< GPS velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate,	///< True Airspeed innovation consistency gate size (STD)

		// control of magnetometer fusion
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>)
		_param_ekf2_head_noise,	///< measurement noise used for simple heading fusion (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>)
		_param_ekf2_mag_noise,		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise,		///< measurement noise used for airspeed fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>)
		_param_ekf2_beta_gate, ///< synthetic sideslip innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise,	///< synthetic sideslip noise (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,///< magnetic declination (degrees)
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>)
		_param_ekf2_hdg_gate,///< heading fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>)
		_param_ekf2_mag_gate,	///< magnetometer fusion innovation consistency gate size (STD)
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>)
		_param_ekf2_decl_type,	///< bitmask used to control the handling of declination data
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>)
		_param_ekf2_mag_type,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>)
		_param_ekf2_mag_acclim,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_YAWLIM>)
		_param_ekf2_mag_yawlim,	///< yaw rate threshold used by mode select logic (rad/sec)

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>)
		_param_ekf2_gps_check,	///< bitmask used to control which GPS quality checks are used
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>) _param_ekf2_req_eph,	///< maximum acceptable horiz position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>) _param_ekf2_req_epv,	///< maximum acceptable vert position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>) _param_ekf2_req_sacc,	///< maximum acceptable speed error (m/s)
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>) _param_ekf2_req_nsats,	///< minimum acceptable satellite count
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)
		_param_ekf2_req_pdop,	///< maximum acceptable position dilution of precision
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>)
		_param_ekf2_req_hdrift,	///< maximum acceptable horizontal drift speed (m/s)
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,	///< maximum acceptable vertical drift speed (m/s)

		// measurement source control
		(ParamExtInt<px4::params::EKF2_AID_MASK>)
		_param_ekf2_aid_mask,		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
		(ParamExtInt<px4::params::EKF2_HGT_MODE>) _param_ekf2_hgt_mode,	///< selects the primary source for height data
		(ParamExtInt<px4::params::EKF2_TERR_MASK>)
		_param_ekf2_terr_mask, ///< bitmasked integer that selects which of range finder and optical flow aiding sources will be used for terrain estimation
		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

		// range finder fusion
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>)
		_param_ekf2_rng_noise,	///< observation noise for range finder measurements (m)
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe, ///< scale factor from range to range noise (m/m)
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>)
		_param_ekf2_rng_gate,	///< range finder fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng,	///< minimum valid value for range when on ground (m)
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch,	///< range sensor pitch offset (rad)
		(ParamExtInt<px4::params::EKF2_RNG_AID>)
		_param_ekf2_rng_aid,		///< enables use of a range finder even if primary height source is not range finder
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>)
		_param_ekf2_rng_a_vmax,	///< maximum allowed horizontal velocity for range aid (m/s)
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>)
		_param_ekf2_rng_a_hmax,	///< maximum allowed absolute altitude (AGL) for range aid (m)
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>)
		_param_ekf2_rng_a_igate,	///< gate size used for innovation consistency checks for range aid fusion (STD)
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>)
		_param_ekf2_rng_qlty_t, ///< Minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (s)

		// vision estimate fusion
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>)
		_param_ekf2_ev_noise_md,	///< determine source of vision observation noise
		(ParamFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise,	///< default position observation noise for exernal vision measurements (m)
		(ParamFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise,	///< default velocity observation noise for exernal vision measurements (m/s)
		(ParamFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise,	///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate,	///< external vision velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate,	///< external vision position innovation consistency gate size (STD)

		// optical flow fusion
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min,	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max,	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin,	///< minimum acceptable quality integer from  the flow sensor
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate,	///< optical flow fusion innovation consistency gate size (STD)

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,		///< X position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,		///< Y position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,		///< Z position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x,		///< X position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y,		///< Y position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z,		///< Z position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x,	///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y,	///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z,	///< Z position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x,		///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y,		///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z,		///< Z position of VI sensor focal point in body frame (m)

		// control of airspeed and sideslip fusion
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
		(ParamInt<px4::params::EKF2_FUSE_BETA>)
		_param_ekf2_fuse_beta,		///< Controls synthetic sideslip fusion, 0 disables, 1 enables

		// output predictor filter time constants
		(ParamExtFloat<px4::params::EKF2_TAU_VEL>)
		_param_ekf2_tau_vel,		///< time constant used by the output velocity complementary filter (sec)
		(ParamExtFloat<px4::params::EKF2_TAU_POS>)
		_param_ekf2_tau_pos,		///< time constant used by the output position complementary filter (sec)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< propeller momentum drag coefficient (1/s)

		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>)
		_param_ekf2_aspd_max,		///< upper limit on airspeed used for correction  (m/s**2)
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>)
		_param_ekf2_pcoef_xp,	///< static pressure position error coefficient along the positive X body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>)
		_param_ekf2_pcoef_xn,	///< static pressure position error coefficient along the negative X body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>)
		_param_ekf2_pcoef_yp,	///< static pressure position error coefficient along the positive Y body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>)
		_param_ekf2_pcoef_yn,	///< static pressure position error coefficient along the negative Y body axis
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>)
		_param_ekf2_pcoef_z,	///< static pressure position error coefficient along the Z body axis

		// Test used to determine if the vehicle is static or moving
		(ParamExtFloat<px4::params::EKF2_MOVE_TEST>)
		_param_ekf2_move_test,	///< scaling applied to IMU data thresholds used to determine if the vehicle is static or moving.

		(ParamFloat<px4::params::EKF2_REQ_GPS_H>) _param_ekf2_req_gps_h, ///< Required GPS health time
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check, ///< Mag field strength check
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>)
		_param_ekf2_synthetic_mag_z, ///< Enables the use of a synthetic value for the Z axis of the magnetometer calculated from the 3D magnetic field vector at the location of the drone.

		// Used by EKF-GSF experimental yaw estimator
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>)
		_param_ekf2_gsf_tas_default	///< default value of true airspeed assumed during fixed wing operation

	)
};
#endif // !EKF2_HPP
/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;
using math::constrain;
using math::radians;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers)
{
}

EKF2Selector::~EKF2Selector()
{
	Stop();
}

bool EKF2Selector::Start()
{
	ScheduleNow();
	return true;
}

void EKF2Selector::Stop()
{
	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		_instance[i].estimator_attitude_sub.unregisterCallback();
		_instance[i].estimator_status_sub.unregisterCallback();
	}

	ScheduleClear();
}

void EKF2Selector::PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance)
{
	const char *old_reason = nullptr;

	if (_instance[old_instance].filter_fault) {
		old_reason = " (filter fault)";

	} else if (_instance[old_instance].timeout) {
		old_reason = " (timeout)";

	} else if (_gyro_fault_detected) {
		old_reason = " (gyro fault)";

	} else if (_accel_fault_detected) {
		old_reason = " (accel fault)";

	} else if (!_instance[_selected_instance].healthy && (_instance[_selected_instance].healthy_count > 0)) {
		// skipped if previous instance was never healthy in the first place (eg initialization)
		old_reason = " (unhealthy)";
	}

	const char *new_reason = nullptr;

	if (_request_instance.load() == new_instance) {
		new_reason = " (user selected)";
	}

	if (old_reason || new_reason) {
		if (old_reason == nullptr) {
			old_reason = "";
		}

		if (new_reason == nullptr) {
			new_reason = "";
		}

		PX4_WARN("primary EKF changed %" PRIu8 "%s -> %" PRIu8 "%s", old_instance, old_reason, new_instance, new_reason);
	}
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance)
{
	if ((ekf_instance != _selected_instance) && (ekf_instance < _available_instances)) {
		// update sensor_selection immediately
		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = _instance[ekf_instance].accel_device_id;
		sensor_selection.gyro_device_id = _instance[ekf_instance].gyro_device_id;
		sensor_selection.timestamp = hrt_absolute_time();
		_sensor_selection_pub.publish(sensor_selection);

		if (_selected_instance != INVALID_INSTANCE) {
			// switch callback registration
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
			_instance[_selected_instance].estimator_status_sub.unregisterCallback();

			PrintInstanceChange(_selected_instance, ekf_instance);
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();
		_instance[ekf_instance].estimator_status_sub.registerCallback();

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = sensor_selection.timestamp;
		_instance[ekf_instance].time_last_selected = _last_instance_change;

		// reset all relative test ratios
		for (uint8_t i = 0; i < _available_instances; i++) {
			_instance[i].relative_test_ratio = 0;
		}

		return true;
	}

	return false;
}

bool EKF2Selector::UpdateErrorScores()
{
	// first check imu inconsistencies
	_gyro_fault_detected = false;
	uint32_t faulty_gyro_id = 0;
	_accel_fault_detected = false;
	uint32_t faulty_accel_id = 0;

	if (_sensors_status_imu.updated()) {
		sensors_status_imu_s sensors_status_imu;

		if (_sensors_status_imu.copy(&sensors_status_imu)) {

			const float time_step_s = constrain((sensors_status_imu.timestamp - _last_update_us) * 1e-6f, 0.f, 0.02f);
			_last_update_us = sensors_status_imu.timestamp;

			{
				const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get());
				const float angle_threshold = radians(_param_ekf2_sel_imu_angle.get());
				uint8_t n_gyros = 0;
				uint8_t n_gyro_exceedances = 0;
				float largest_accumulated_gyro_error = 0.0f;
				uint8_t largest_gyro_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for gyros with excessive difference to mean using accumulated error
					if (sensors_status_imu.gyro_device_ids[i] != 0) {
						n_gyros++;
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s;
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f);

						if (_accumulated_gyro_error[i] > angle_threshold) {
							n_gyro_exceedances++;
						}

						if (_accumulated_gyro_error[i] > largest_accumulated_gyro_error) {
							largest_accumulated_gyro_error = _accumulated_gyro_error[i];
							largest_gyro_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_gyro_error[i] = NAN;
					}
				}

				if (n_gyro_exceedances > 0) {
					if (n_gyros >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_gyro_fault_detected = true;
						faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index];

					} else if (n_gyros == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_gyro_fault_detected = true;
					}
				}
			}

			{
				const float accel_threshold = _param_ekf2_sel_imu_accel.get();
				const float velocity_threshold = _param_ekf2_sel_imu_velocity.get();
				uint8_t n_accels = 0;
				uint8_t n_accel_exceedances = 0;
				float largest_accumulated_accel_error = 0.0f;
				uint8_t largest_accel_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for accelerometers with excessive difference to mean using accumulated error
					if (sensors_status_imu.accel_device_ids[i] != 0) {
						n_accels++;
						_accumulated_accel_error[i] += (sensors_status_imu.accel_inconsistency_m_s_s[i] - accel_threshold) * time_step_s;
						_accumulated_accel_error[i] = fmaxf(_accumulated_accel_error[i], 0.f);

						if (_accumulated_accel_error[i] > velocity_threshold) {
							n_accel_exceedances++;
						}

						if (_accumulated_accel_error[i] > largest_accumulated_accel_error) {
							largest_accumulated_accel_error = _accumulated_accel_error[i];
							largest_accel_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_accel_error[i] = NAN;
					}
				}

				if (n_accel_exceedances > 0) {
					if (n_accels >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_accel_fault_detected = true;
						faulty_accel_id = sensors_status_imu.accel_device_ids[largest_accel_error_index];

					} else if (n_accels == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_accel_fault_detected = true;
					}
				}
			}
		}
	}

	bool updated = false;
	bool primary_updated = false;

	// default estimator timeout
	const hrt_abstime status_timeout = 50_ms;

	// calculate individual error scores
	for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
		const bool prev_healthy = _instance[i].healthy;

		estimator_status_s status;

		if (_instance[i].estimator_status_sub.update(&status)) {

			_instance[i].timestamp_sample_last = status.timestamp_sample;

			_instance[i].accel_device_id = status.accel_device_id;
			_instance[i].gyro_device_id = status.gyro_device_id;
			_instance[i].baro_device_id = status.baro_device_id;
			_instance[i].mag_device_id = status.mag_device_id;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
				updated = true;
			}

			if (i == _selected_instance) {
				primary_updated = true;
			}

			// test ratios are invalid when 0, >= 1 is a failure
			if (!PX4_ISFINITE(status.vel_test_ratio) || (status.vel_test_ratio <= 0.f)) {
				status.vel_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.pos_test_ratio) || (status.pos_test_ratio <= 0.f)) {
				status.pos_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.hgt_test_ratio) || (status.hgt_test_ratio <= 0.f)) {
				status.hgt_test_ratio = 1.f;
			}

			float combined_test_ratio = fmaxf(0.5f * (status.vel_test_ratio + status.pos_test_ratio), status.hgt_test_ratio);

			_instance[i].combined_test_ratio = combined_test_ratio;
			_instance[i].healthy = (status.filter_fault_flags == 0) && (combined_test_ratio > 0.f);
			_instance[i].warning = (combined_test_ratio >= 1.f);
			_instance[i].filter_fault = (status.filter_fault_flags != 0);
			_instance[i].timeout = false;

			if (!_instance[i].warning) {
				_instance[i].time_last_no_warning = status.timestamp_sample;
			}

			if (!PX4_ISFINITE(_instance[i].relative_test_ratio)) {
				_instance[i].relative_test_ratio = 0;
			}

		} else if (!_instance[i].timeout && (hrt_elapsed_time(&_instance[i].timestamp_sample_last) > status_timeout)) {
			_instance[i].healthy = false;
			_instance[i].timeout = true;
		}

		// if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
			_instance[i].healthy = false;
		}

		// if the accelerometer used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_accel_fault_detected && (faulty_accel_id != 0) && (_instance[i].accel_device_id == faulty_accel_id)) {
			_instance[i].healthy = false;
		}

		if (prev_healthy != _instance[i].healthy) {
			updated = true;
			_selector_status_publish = true;

			if (!prev_healthy) {
				_instance[i].healthy_count++;
			}
		}
	}

	// update relative test ratios if primary has updated
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) {
			if (i != _selected_instance) {

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio;

				// reduce error only if its better than the primary instance by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
				const float threshold = _gyro_fault_detected ? 0.0f : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);

				if (error_delta > 0 || error_delta < -threshold) {
					_instance[i].relative_test_ratio += error_delta;
					_instance[i].relative_test_ratio = constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim, _rel_err_score_lim);

					if ((error_delta < -threshold) && (_instance[i].relative_test_ratio < 1.f)) {
						// increase status publication rate if there's movement towards a potential instance change
						_selector_status_publish = true;
					}
				}
			}
		}
	}

	return (primary_updated || updated);
}

void EKF2Selector::PublishVehicleAttitude()
{
	// selected estimator_attitude -> vehicle_attitude
	vehicle_attitude_s attitude;

	if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_attitude_sub.get_instance() != _attitude_instance_prev) {
			_attitude_instance_prev = _instance[_selected_instance].estimator_attitude_sub.get_instance();
			instance_change = true;
		}

		if (_attitude_last.timestamp != 0) {
			if (!instance_change && (attitude.quat_reset_counter == _attitude_last.quat_reset_counter + 1)) {
				// propogate deltas from estimator data while maintaining the overall reset counts
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};

			} else if (instance_change || (attitude.quat_reset_counter != _attitude_last.quat_reset_counter)) {
				// on reset compute deltas from last published data
				++_quat_reset_counter;
				_delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized();
			}

		} else {
			_quat_reset_counter = attitude.quat_reset_counter;
			_delta_q_reset = Quatf{attitude.delta_q_reset};
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's attitude for system (vehicle_attitude) if it's stale
		if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample)
		    || (attitude.timestamp_sample < _instance[_selected_instance].timestamp_sample_last)) {

			publish = false;
		}

		// save last primary estimator_attitude as published with original resets
		_attitude_last = attitude;

		if (publish) {
			// republish with total reset count and current timestamp
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			attitude.timestamp = hrt_absolute_time();
			_vehicle_attitude_pub.publish(attitude);
		}
	}
}

void EKF2Selector::PublishVehicleLocalPosition()
{
	// selected estimator_local_position -> vehicle_local_position
	vehicle_local_position_s local_position;

	if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_local_position_sub.get_instance() != _local_position_instance_prev) {
			_local_position_instance_prev = _instance[_selected_instance].estimator_local_position_sub.get_instance();
			instance_change = true;
		}

		if (_local_position_last.timestamp != 0) {
			// XY reset
			if (!instance_change && (local_position.xy_reset_counter == _local_position_last.xy_reset_counter + 1)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.delta_xy};

			} else if (instance_change || (local_position.xy_reset_counter != _local_position_last.xy_reset_counter)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.x, local_position.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			}

			// Z reset
			if (!instance_change && (local_position.z_reset_counter == _local_position_last.z_reset_counter + 1)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.delta_z;

			} else if (instance_change || (local_position.z_reset_counter != _local_position_last.z_reset_counter)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.z - _local_position_last.z;
			}

			// VXY reset
			if (!instance_change && (local_position.vxy_reset_counter == _local_position_last.vxy_reset_counter + 1)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.delta_vxy};

			} else if (instance_change || (local_position.vxy_reset_counter != _local_position_last.vxy_reset_counter)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.vx, local_position.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			}

			// VZ reset
			if (!instance_change && (local_position.vz_reset_counter == _local_position_last.vz_reset_counter + 1)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.delta_vz;

			} else if (instance_change || (local_position.vz_reset_counter != _local_position_last.vz_reset_counter)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.vz - _local_position_last.vz;
			}

			// heading reset
			if (!instance_change && (local_position.heading_reset_counter == _local_position_last.heading_reset_counter + 1)) {
				++_heading_reset_counter;
				_delta_heading_reset = local_position.delta_heading;

			} else if (instance_change || (local_position.heading_reset_counter != _local_position_last.heading_reset_counter)) {
				++_heading_reset_counter;
				_delta_heading_reset = matrix::wrap_pi(local_position.heading - _local_position_last.heading);
			}

		} else {
			_xy_reset_counter = local_position.xy_reset_counter;
			_z_reset_counter = local_position.z_reset_counter;
			_vxy_reset_counter = local_position.vxy_reset_counter;
			_vz_reset_counter = local_position.vz_reset_counter;
			_heading_reset_counter = local_position.heading_reset_counter;

			_delta_xy_reset = Vector2f{local_position.delta_xy};
			_delta_z_reset = local_position.delta_z;
			_delta_vxy_reset = Vector2f{local_position.delta_vxy};
			_delta_vz_reset = local_position.delta_vz;
			_delta_heading_reset = local_position.delta_heading;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's local position for system (vehicle_local_position) if it's stale
		if ((local_position.timestamp_sample <= _local_position_last.timestamp_sample)
		    || (local_position.timestamp_sample < _instance[_selected_instance].timestamp_sample_last)) {

			publish = false;
		}

		// save last primary estimator_local_position as published with original resets
		_local_position_last = local_position;

		if (publish) {
			// republish with total reset count and current timestamp
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;

			_delta_xy_reset.copyTo(local_position.delta_xy);
			local_position.delta_z = _delta_z_reset;
			_delta_vxy_reset.copyTo(local_position.delta_vxy);
			local_position.delta_vz = _delta_vz_reset;
			local_position.delta_heading = _delta_heading_reset;

			local_position.timestamp = hrt_absolute_time();
			_vehicle_local_position_pub.publish(local_position);
		}
	}
}

void EKF2Selector::PublishVehicleOdometry()
{
	// selected estimator_odometry -> vehicle_odometry
	vehicle_odometry_s odometry;

	if (_instance[_selected_instance].estimator_odometry_sub.update(&odometry)) {
		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's odometry for system (vehicle_odometry) if it's stale
		if ((odometry.timestamp_sample <= _odometry_last.timestamp_sample)
		    || (odometry.timestamp_sample < _instance[_selected_instance].timestamp_sample_last)) {

			publish = false;
		}

		// save last primary estimator_odometry
		_odometry_last = odometry;

		if (publish) {
			odometry.timestamp = hrt_absolute_time();
			_vehicle_odometry_pub.publish(odometry);
		}
	}
}

void EKF2Selector::PublishVehicleGlobalPosition()
{
	// selected estimator_global_position -> vehicle_global_position
	vehicle_global_position_s global_position;

	if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_global_position_sub.get_instance() != _global_position_instance_prev) {
			_global_position_instance_prev = _instance[_selected_instance].estimator_global_position_sub.get_instance();
			instance_change = true;
		}

		if (_global_position_last.timestamp != 0) {
			// lat/lon reset
			if (!instance_change && (global_position.lat_lon_reset_counter == _global_position_last.lat_lon_reset_counter + 1)) {
				++_lat_lon_reset_counter;

				// TODO: delta latitude/longitude
				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;

			} else if (instance_change || (global_position.lat_lon_reset_counter != _global_position_last.lat_lon_reset_counter)) {
				++_lat_lon_reset_counter;

				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;
			}

			// alt reset
			if (!instance_change && (global_position.alt_reset_counter == _global_position_last.alt_reset_counter + 1)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt;

			} else if (instance_change || (global_position.alt_reset_counter != _global_position_last.alt_reset_counter)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt - _global_position_last.delta_alt;
			}

		} else {
			_lat_lon_reset_counter = global_position.lat_lon_reset_counter;
			_alt_reset_counter = global_position.alt_reset_counter;

			_delta_alt_reset = global_position.delta_alt;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's global position for system (vehicle_global_position) if it's stale
		if ((global_position.timestamp_sample <= _global_position_last.timestamp_sample)
		    || (global_position.timestamp_sample < _instance[_selected_instance].timestamp_sample_last)) {

			publish = false;
		}

		// save last primary estimator_global_position as published with original resets
		_global_position_last = global_position;

		if (publish) {
			// republish with total reset count and current timestamp
			global_position.lat_lon_reset_counter = _lat_lon_reset_counter;
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt_reset;

			global_position.timestamp = hrt_absolute_time();
			_vehicle_global_position_pub.publish(global_position);
		}
	}
}

void EKF2Selector::PublishWindEstimate()
{
	// selected estimator_wind -> wind
	wind_s wind;

	if (_instance[_selected_instance].estimator_wind_sub.update(&wind)) {
		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's wind for system (wind) if it's stale
		if ((wind.timestamp_sample <= _wind_last.timestamp_sample)
		    || (wind.timestamp_sample < _instance[_selected_instance].timestamp_sample_last)) {

			publish = false;
		}

		// save last primary wind
		_wind_last = wind;

		// publish estimator's wind for system unless it's stale
		if (publish) {
			// republish with current timestamp
			wind.timestamp = hrt_absolute_time();
			_wind_pub.publish(wind);
		}
	}
}

void EKF2Selector::Run()
{
	// re-schedule as backup timeout
	ScheduleDelayed(FILTER_UPDATE_PERIOD);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// update combined test ratio for all estimators
	const bool updated = UpdateErrorScores();

	// if no valid instance then force select first instance with valid IMU
	if (_selected_instance == INVALID_INSTANCE) {
		for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
			if ((_instance[i].accel_device_id != 0)
			    && (_instance[i].gyro_device_id != 0)) {

				if (SelectInstance(i)) {
					break;
				}
			}
		}

		// if still invalid return early and check again on next scheduled run
		if (_selected_instance == INVALID_INSTANCE) {
			return;
		}
	}

	if (updated) {
		const uint8_t available_instances_prev = _available_instances;
		const uint8_t selected_instance_prev = _selected_instance;
		const uint32_t instance_changed_count_prev = _instance_changed_count;
		const hrt_abstime last_instance_change_prev = _last_instance_change;

		bool lower_error_available = false;
		float alternative_error = 0.f; // looking for instances that have error lower than the current primary
		float best_test_ratio = FLT_MAX;

		uint8_t best_ekf = _selected_instance;
		uint8_t best_ekf_alternate = INVALID_INSTANCE;
		uint8_t best_ekf_different_imu = INVALID_INSTANCE;

		// loop through all available instances to find if an alternative is available
		for (int i = 0; i < _available_instances; i++) {
			// Use an alternative instance if  -
			// (healthy and has updated recently)
			// AND
			// (has relative error less than selected instance and has not been the selected instance for at least 10 seconds
			// OR
			// selected instance has stopped updating
			if (_instance[i].healthy && (i != _selected_instance)) {
				const float test_ratio = _instance[i].combined_test_ratio;
				const float relative_error = _instance[i].relative_test_ratio;

				if (relative_error < alternative_error) {
					best_ekf_alternate = i;
					alternative_error = relative_error;

					// relative error less than selected instance and has not been the selected instance for at least 10 seconds
					if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) {
						lower_error_available = true;
					}
				}

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
					best_ekf = i;
					best_test_ratio = test_ratio;

					// also check next best available ekf using a different IMU
					if (_instance[i].accel_device_id != _instance[_selected_instance].accel_device_id) {
						best_ekf_different_imu = i;
					}
				}
			}
		}

		if (!_instance[_selected_instance].healthy) {
			// prefer the best healthy instance using a different IMU
			if (!SelectInstance(best_ekf_different_imu)) {
				// otherwise switch to the healthy instance with best overall test ratio
				SelectInstance(best_ekf);
			}

		} else if (lower_error_available
			   && ((hrt_elapsed_time(&_last_instance_change) > 10_s)
			       || (_instance[_selected_instance].warning
				   && (hrt_elapsed_time(&_instance[_selected_instance].time_last_no_warning) > 1_s)))) {

			// if this instance has a significantly lower relative error to the active primary, we consider it as a
			// better instance and would like to switch to it even if the current primary is healthy
			SelectInstance(best_ekf_alternate);

		} else if (_request_instance.load() != INVALID_INSTANCE) {

			const uint8_t new_instance = _request_instance.load();

			// attempt to switch to user manually selected instance
			if (!SelectInstance(new_instance)) {
				PX4_ERR("unable to switch to user selected instance %d", new_instance);
			}

			// reset
			_request_instance.store(INVALID_INSTANCE);
		}

		// publish selector status at ~1 Hz or immediately on any change
		if (_selector_status_publish || (hrt_elapsed_time(&_last_status_publish) > 1_s)
		    || (available_instances_prev != _available_instances)
		    || (selected_instance_prev != _selected_instance)
		    || (instance_changed_count_prev != _instance_changed_count)
		    || (last_instance_change_prev != _last_instance_change)
		    || _accel_fault_detected || _gyro_fault_detected) {

			PublishEstimatorSelectorStatus();
			_selector_status_publish = false;
		}
	}

	// republish selected estimator data for system
	PublishVehicleAttitude();
	PublishVehicleLocalPosition();
	PublishVehicleGlobalPosition();
	PublishVehicleOdometry();
	PublishWindEstimate();
}

void EKF2Selector::PublishEstimatorSelectorStatus()
{
	estimator_selector_status_s selector_status{};
	selector_status.primary_instance = _selected_instance;
	selector_status.instances_available = _available_instances;
	selector_status.instance_changed_count = _instance_changed_count;
	selector_status.last_instance_change = _last_instance_change;
	selector_status.accel_device_id = _instance[_selected_instance].accel_device_id;
	selector_status.baro_device_id = _instance[_selected_instance].baro_device_id;
	selector_status.gyro_device_id = _instance[_selected_instance].gyro_device_id;
	selector_status.mag_device_id = _instance[_selected_instance].mag_device_id;
	selector_status.gyro_fault_detected = _gyro_fault_detected;
	selector_status.accel_fault_detected = _accel_fault_detected;

	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio;
		selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio;
		selector_status.healthy[i] = _instance[i].healthy;
	}

	for (int i = 0; i < IMU_STATUS_SIZE; i++) {
		selector_status.accumulated_gyro_error[i] = _accumulated_gyro_error[i];
		selector_status.accumulated_accel_error[i] = _accumulated_accel_error[i];
	}

	selector_status.timestamp = hrt_absolute_time();
	_estimator_selector_status_pub.publish(selector_status);
	_last_status_publish = selector_status.timestamp;
}

void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %" PRIu8, _available_instances);

	if (_selected_instance == INVALID_INSTANCE) {
		PX4_WARN("selected instance: None");
	}

	for (int i = 0; i < _available_instances; i++) {
		const EstimatorInstance &inst = _instance[i];

		PX4_INFO("%" PRIu8 ": ACC: %" PRIu32 ", GYRO: %" PRIu32 ", MAG: %" PRIu32 ", %s, test ratio: %.7f (%.5f) %s",
			 inst.instance, inst.accel_device_id, inst.gyro_device_id, inst.mag_device_id,
			 inst.healthy ? "healthy" : "unhealthy",
			 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio,
			 (_selected_instance == i) ? "*" : "");
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#ifndef EKF2SELECTOR_HPP
#define EKF2SELECTOR_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>

#if CONSTRAINED_MEMORY
# define EKF2_MAX_INSTANCES 2
#else
# define EKF2_MAX_INSTANCES 9
#endif

using namespace time_literals;

class EKF2Selector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2Selector();
	~EKF2Selector() override;

	bool Start();
	void Stop();

	void PrintStatus();

	void RequestInstance(uint8_t instance) { _request_instance.store(instance); }

private:
	static constexpr uint8_t INVALID_INSTANCE{UINT8_MAX};
	static constexpr uint64_t FILTER_UPDATE_PERIOD{10_ms};

	void Run() override;

	void PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance);

	void PublishEstimatorSelectorStatus();
	void PublishVehicleAttitude();
	void PublishVehicleLocalPosition();
	void PublishVehicleGlobalPosition();
	void PublishVehicleOdometry();
	void PublishWindEstimate();

	bool SelectInstance(uint8_t instance);

	// Update the error scores for all available instances
	bool UpdateErrorScores();

	// Subscriptions (per estimator instance)
	struct EstimatorInstance {

		EstimatorInstance(EKF2Selector *selector, uint8_t i) :
			estimator_attitude_sub{selector, ORB_ID(estimator_attitude), i},
			estimator_status_sub{selector, ORB_ID(estimator_status), i},
			estimator_local_position_sub{ORB_ID(estimator_local_position), i},
			estimator_global_position_sub{ORB_ID(estimator_global_position), i},
			estimator_odometry_sub{ORB_ID(estimator_odometry), i},
			estimator_wind_sub{ORB_ID(estimator_wind), i},
			instance(i)
		{}

		uORB::SubscriptionCallbackWorkItem estimator_attitude_sub;
		uORB::SubscriptionCallbackWorkItem estimator_status_sub;

		uORB::Subscription estimator_local_position_sub;
		uORB::Subscription estimator_global_position_sub;
		uORB::Subscription estimator_odometry_sub;
		uORB::Subscription estimator_wind_sub;

		uint64_t timestamp_sample_last{0};

		uint32_t accel_device_id{0};
		uint32_t gyro_device_id{0};
		uint32_t baro_device_id{0};
		uint32_t mag_device_id{0};

		hrt_abstime time_last_selected{0};
		hrt_abstime time_last_no_warning{0};

		float combined_test_ratio{NAN};
		float relative_test_ratio{NAN};

		bool healthy{false};
		bool warning{false};
		bool filter_fault{false};
		bool timeout{false};

		uint8_t healthy_count{0};

		const uint8_t instance;
	};

	static constexpr float _rel_err_score_lim{1.0f}; // +- limit applied to the relative error score
	static constexpr float _rel_err_thresh{0.5f};    // the relative score difference needs to be greater than this to switch from an otherwise healthy instance

	EstimatorInstance _instance[EKF2_MAX_INSTANCES] {
		{this, 0},
		{this, 1},
#if EKF2_MAX_INSTANCES > 2
		{this, 2},
		{this, 3},
#if EKF2_MAX_INSTANCES > 4
		{this, 4},
		{this, 5},
		{this, 6},
		{this, 7},
		{this, 8},
#endif
#endif
	};

	static constexpr uint8_t IMU_STATUS_SIZE = (sizeof(sensors_status_imu_s::gyro_inconsistency_rad_s) / sizeof(
				sensors_status_imu_s::gyro_inconsistency_rad_s[0]));
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_gyro_error) / sizeof(
			      estimator_selector_status_s::accumulated_gyro_error[0]),
		      "increase estimator_selector_status_s::accumulated_gyro_error size");
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_accel_error) / sizeof(
			      estimator_selector_status_s::accumulated_accel_error[0]),
		      "increase estimator_selector_status_s::accumulated_accel_error size");
	static_assert(EKF2_MAX_INSTANCES <= sizeof(estimator_selector_status_s::combined_test_ratio) / sizeof(
			      estimator_selector_status_s::combined_test_ratio[0]),
		      "increase estimator_selector_status_s::combined_test_ratio size");

	float _accumulated_gyro_error[IMU_STATUS_SIZE] {};
	float _accumulated_accel_error[IMU_STATUS_SIZE] {};
	hrt_abstime _last_update_us{0};
	bool _gyro_fault_detected{false};
	bool _accel_fault_detected{false};

	uint8_t _available_instances{0};
	uint8_t _selected_instance{INVALID_INSTANCE};
	px4::atomic<uint8_t> _request_instance{INVALID_INSTANCE};

	uint32_t _instance_changed_count{0};
	hrt_abstime _last_instance_change{0};

	hrt_abstime _last_status_publish{0};
	bool _selector_status_publish{false};

	// vehicle_attitude: reset counters
	vehicle_attitude_s _attitude_last{};
	matrix::Quatf _delta_q_reset{};
	uint8_t _quat_reset_counter{0};

	// vehicle_local_position: reset counters
	vehicle_local_position_s _local_position_last{};
	matrix::Vector2f _delta_xy_reset{};
	float _delta_z_reset{0.f};
	matrix::Vector2f _delta_vxy_reset{};
	float _delta_vz_reset{0.f};
	float _delta_heading_reset{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// vehicle_odometry
	vehicle_odometry_s _odometry_last{};

	// vehicle_global_position: reset counters
	vehicle_global_position_s _global_position_last{};
	double _delta_lat_reset{0};
	double _delta_lon_reset{0};
	float _delta_alt_reset{0.f};
	uint8_t _lat_lon_reset_counter{0};
	uint8_t _alt_reset_counter{0};

	// wind estimate
	wind_s _wind_last{};

	uint8_t _attitude_instance_prev{INVALID_INSTANCE};
	uint8_t _local_position_instance_prev{INVALID_INSTANCE};
	uint8_t _global_position_instance_prev{INVALID_INSTANCE};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensors_status_imu{ORB_ID(sensors_status_imu)};

	// Publications
	uORB::Publication<estimator_selector_status_s> _estimator_selector_status_pub{ORB_ID(estimator_selector_status)};
	uORB::Publication<sensor_selection_s>          _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_attitude_s>          _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>   _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>    _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_odometry_s>          _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<wind_s>             _wind_pub{ORB_ID(wind)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_SEL_ERR_RED>) _param_ekf2_sel_err_red,
		(ParamFloat<px4::params::EKF2_SEL_IMU_RAT>) _param_ekf2_sel_imu_angle_rate,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ANG>) _param_ekf2_sel_imu_angle,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ACC>) _param_ekf2_sel_imu_accel,
		(ParamFloat<px4::params::EKF2_SEL_IMU_VEL>) _param_ekf2_sel_imu_velocity
	)
};
#endif // !EKF2SELECTOR_HPP
