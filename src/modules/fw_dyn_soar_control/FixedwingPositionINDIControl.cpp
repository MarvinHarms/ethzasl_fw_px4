/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "FixedwingPositionINDIControl.hpp"

using namespace std;

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Matrix;
using matrix::Euler;
using matrix::Quatf;
using matrix::AxisAnglef;
using matrix::Vector3f;
using matrix::Vector;
using matrix::wrap_pi;


FixedwingPositionINDIControl::FixedwingPositionINDIControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuators_0_pub(ORB_ID(actuator_controls_0)),
	_attitude_sp_pub(ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// limit to 100 Hz
	_vehicle_angular_velocity_sub.set_interval_ms(1000.f / _sample_frequency);


	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionINDIControl::~FixedwingPositionINDIControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingPositionINDIControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle position callback registration failed!");
		return false;
	}

	PX4_INFO("Starting FW_DYN_SOAR_CONTROLLER");
	char filename[] = "trajectory0.csv";
	_read_trajectory_coeffs_csv(filename);

	// initialize transformations
	_R_ned_to_enu *= 0.f;
	_R_ned_to_enu(0, 1) = 1.f;
	_R_ned_to_enu(1, 0) = 1.f;
	_R_ned_to_enu(2, 2) = -1.f;
	_R_ned_to_enu.renormalize();
	_R_enu_to_ned = _R_ned_to_enu;

	// initialize wind shear params
	_shear_v_max = 8.f;
	_shear_alpha = 0.5f;
	_shear_h_ref = 0.f;
	_shear_heading = M_PI_2_F;
	_shear_aspd = 20.f;

	// init wind estimates
	for (int i = 0; i < 3; i++) {
		_wind_estimate(i) = 0.0f;
		_wind_estimate_EKF(i) = 0.0f;
	}

	// initialize in manual feedthrough
	_switch_manual = true;

	// fix sitl mode on startup (no switch at runtime)
	_switch_sitl = _param_switch_sitl.get();

	// initialize transform to trajec frame
	_compute_trajectory_transform();

	return true;
}

int
FixedwingPositionINDIControl::parameters_update()
{
	updateParams();

	// INDI parameters
	_K_x *= 0.f;
	_K_v *= 0.f;
	_K_a *= 0.f;
	_K_q *= 0.f;
	_K_w *= 0.f;
	_K_x(0, 0) = _param_lin_k_x.get();
	_K_x(1, 1) = _param_lin_k_y.get();
	_K_x(2, 2) = _param_lin_k_z.get();
	_K_v(0, 0) = _param_lin_c_x.get() * 2.f * sqrtf(_param_lin_k_x.get());
	_K_v(1, 1) = _param_lin_c_y.get() * 2.f * sqrtf(_param_lin_k_y.get());
	_K_v(2, 2) = _param_lin_c_z.get() * 2.f * sqrtf(_param_lin_k_z.get());
	_K_a(0, 0) = _param_lin_ff_x.get();
	_K_a(1, 1) = _param_lin_ff_y.get();
	_K_a(2, 2) = _param_lin_ff_z.get();
	_K_q(0, 0) = _param_rot_k_roll.get();
	_K_q(1, 1) = _param_rot_k_pitch.get();
	_K_q(2, 2) = _param_rot_ff_yaw.get(); // rudder is controlled via turn coordination, not INDI
	_K_w(0, 0) = _param_rot_c_roll.get() * 2.f * sqrtf(_param_rot_k_roll.get());
	_K_w(1, 1) = _param_rot_c_pitch.get() * 2.f * sqrtf(_param_rot_k_pitch.get());
	_K_w(2, 2) = _param_rot_p_yaw.get(); // rudder is controlled via turn coordination, not INDI

	// aircraft parameters
	_inertia(0, 0) = _param_fw_inertia_roll.get();
	_inertia(1, 1) = _param_fw_inertia_pitch.get();
	_inertia(2, 2) = _param_fw_inertia_yaw.get();
	_inertia(0, 2) = _param_fw_inertia_rp.get();
	_inertia(2, 0) = _param_fw_inertia_rp.get();
	_mass = _param_fw_mass.get();
	_area = _param_fw_wing_area.get();
	_rho = _param_rho.get();
	_C_L0 = _param_fw_c_l0.get();
	_C_L1 = _param_fw_c_l1.get();
	_C_D0 = _param_fw_c_d0.get();
	_C_D1 = _param_fw_c_d1.get();
	_C_D2 = _param_fw_c_d2.get();
	_C_B1 = _param_fw_c_b1.get();
	_aoa_offset = _param_aoa_offset.get();
	_stall_speed = _param_stall_speed.get();

	// actuator gains
	_k_ail = _param_k_act_roll.get();
	_k_ele = _param_k_act_pitch.get();
	_k_d_roll = _param_k_damping_roll.get();
	_k_d_pitch = _param_k_damping_pitch.get();

	// trajectory origin
	_origin_lat = _param_origin_lat.get();
	_origin_lon = _param_origin_lon.get();
	_origin_alt = _param_origin_alt.get();
	// check if local NED reference frame origin has changed:
	// || (_local_pos.vxy_reset_counter != _pos_reset_counter
	// initialize projection
	map_projection_init_timestamped(&_global_local_proj_ref, _local_pos.ref_lat, _local_pos.ref_lon,
					_local_pos.ref_timestamp);
	// project the origin of the soaring ENU frame to the current NED frame
	map_projection_project(&_global_local_proj_ref, _origin_lat, _origin_lon, &_origin_N, &_origin_E);
	_origin_D =  _local_pos.ref_alt - _origin_alt;
	PX4_INFO("local reference frame updated");

	_thrust_pos = _param_thrust.get();
	_thrust = _thrust_pos;
	_switch_saturation = _param_switch_saturation.get();
	_switch_origin_hardcoded = _param_switch_origin_hardcoded.get();
	_switch_cl_soaring = _param_switch_cloop.get();

	if (_switch_sitl) {
		// only use switch manual param in sitl mode
		_switch_manual = _param_switch_manual.get();
	}

	_loiter = _param_loiter.get();

	// only update shear heading and height with params, if desired
	if (_switch_origin_hardcoded) {
		_shear_heading = _param_shear_heading.get() / 180.f * M_PI_F + M_PI_2_F;
		_shear_h_ref = _param_shear_height.get();
		_select_loiter_trajectory();

	} else {
		_shear_v_max = _param_shear_estimated_v_max.get();
		_shear_alpha = _param_shear_estimated_alpha.get();
		_shear_h_ref = _param_shear_estimated_h_ref.get();
		_shear_heading = _param_shear_estimated_heading.get();
	}


	return PX4_OK;
}

void
FixedwingPositionINDIControl::airspeed_poll()
{
	bool airspeed_valid = _airspeed_valid;
	airspeed_validated_s airspeed_validated;

	if (_airspeed_validated_sub.update(&airspeed_validated)) {

		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && (airspeed_validated.calibrated_airspeed_m_s > 0.0f)) {

			airspeed_valid = true;

			_airspeed_last_valid = airspeed_validated.timestamp;
			_true_airspeed = airspeed_validated.true_airspeed_m_s;
			_cal_airspeed = airspeed_validated.calibrated_airspeed_m_s;
		}

	} else {
		// no airspeed updates for one second
		if (airspeed_valid && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s)) {
			airspeed_valid = false;
		}
	}

	_airspeed_valid = airspeed_valid;
}

void
FixedwingPositionINDIControl::airflow_aoa_poll()
{
	bool aoa_valid = _aoa_valid;
	airflow_aoa_s aoa_validated;

	if (_airflow_aoa_sub.update(&aoa_validated)) {

		if (PX4_ISFINITE(aoa_validated.aoa_rad)
		    && (aoa_validated.valid)) {

			aoa_valid = true;

			_aoa_last_valid = aoa_validated.timestamp;
			_aoa = aoa_validated.aoa_rad;
		}

	} else {
		// no aoa updates for one second
		if (aoa_valid && (hrt_elapsed_time(&_aoa_last_valid) > 1_s)) {
			aoa_valid = false;
		}
	}

	_aoa_valid = aoa_valid;
}

void
FixedwingPositionINDIControl::airflow_slip_poll()
{
	bool slip_valid = _slip_valid;
	airflow_slip_s slip_validated;

	if (_airflow_slip_sub.update(&slip_validated)) {

		if (PX4_ISFINITE(slip_validated.slip_rad)
		    && (slip_validated.valid)) {

			slip_valid = true;

			_slip_last_valid = slip_validated.timestamp;
			_slip = slip_validated.slip_rad;
		}

	} else {
		// no aoa updates for one second
		if (slip_valid && (hrt_elapsed_time(&_slip_last_valid) > 1_s)) {
			slip_valid = false;
		}
	}

	_slip_valid = slip_valid;
}

void
FixedwingPositionINDIControl::vehicle_status_poll()
{
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		//print_message(_vehicle_status);
	}
}

void
FixedwingPositionINDIControl::manual_control_setpoint_poll()
{
	if (_switch_manual) {
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_thrust = _manual_control_setpoint.z;

	} else {
		_thrust = _thrust_pos;
	}
}

void
FixedwingPositionINDIControl::rc_channels_poll()
{
	if (_rc_channels_sub.update(&_rc_channels)) {
		// use flaps channel to select manual feedthrough
		if (!_switch_sitl) {
			if (_rc_channels.channels[5] >= 0.f) {
				_switch_manual = true;

			} else {
				_switch_manual = false;
			}
		}
	}
}

void
FixedwingPositionINDIControl::vehicle_attitude_poll()
{
	if (_vehicle_attitude_sub.update(&_attitude)) {
		// get rotation between NED frames
		Dcmf R_ned_frd(Quatf(_attitude.q));
		// get rotation from FRD to ENU frame (change of basis)
		Dcmf R_enu_frd(_R_ned_to_enu * R_ned_frd);
		_att = Quatf(R_enu_frd);
	}

	if (hrt_absolute_time() - _attitude.timestamp > 20_ms
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_ERR("attitude sample is too old");
	}
}

void
FixedwingPositionINDIControl::vehicle_angular_velocity_poll()
{
	//
	// no need to check if it was updated as the main loop is fired based on an update...
	//
	_omega = Vector3f(_angular_vel.xyz);

	if (hrt_absolute_time() - _angular_vel.timestamp > 20_ms
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_ERR("angular velocity sample is too old");
	}
}

void
FixedwingPositionINDIControl::vehicle_angular_acceleration_poll()
{
	if (_vehicle_angular_acceleration_sub.update(&_angular_accel)) {
		_alpha = Vector3f(_angular_accel.xyz);
	}

	if (hrt_absolute_time() - _angular_accel.timestamp > 20_ms
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_ERR("angular acceleration sample is too old");
	}
}

void
FixedwingPositionINDIControl::vehicle_local_position_poll()
{
	//vehicle_local_position_s pos;
	if (_vehicle_local_position_sub.update(&_local_pos)) {
		_pos = _R_ned_to_enu * Vector3f{_local_pos.x, _local_pos.y, _local_pos.z};
		_vel = _R_ned_to_enu * Vector3f{_local_pos.vx, _local_pos.vy, _local_pos.vz};
		// take accel from faster message, since 50Hz is too slow...
		// transform to soaring frame
		_pos = _pos - _R_ned_to_enu * Vector3f{_origin_N, _origin_E, _origin_D};
	}

	if (hrt_absolute_time() - _local_pos.timestamp > 50_ms
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_ERR("local position sample is too old");
	}
}

void
FixedwingPositionINDIControl::vehicle_acceleration_poll()
{
	//vehicle_local_position_s pos;
	if (_vehicle_acceleration_sub.update(&_acceleration)) {
		Dcmf R_ib(_att);
		_acc = R_ib * Vector3f(_acceleration.xyz) - Vector3f{0.f, 0.f, 9.81f};
	}

	if (hrt_absolute_time() - _acceleration.timestamp > 20_ms
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_ERR("linear acceleration sample is too old");
	}
}

void
FixedwingPositionINDIControl::actuator_controls_poll()
{
	_actuator_controls_sub.update(&_actuators);
}

void
FixedwingPositionINDIControl::soaring_controller_status_poll()
{
	if (_soaring_controller_status_sub.update(&_soaring_controller_status)) {
		if (!_soaring_controller_status.soaring_controller_running) {
			//PX4_INFO("Soaring controller turned off");
		}

		if (_soaring_controller_status.timeout_detected) {
			//PX4_INFO("Controller timeout detected");
		}
	}
}

void
FixedwingPositionINDIControl::soaring_estimator_shear_poll()
{
	if (!_switch_origin_hardcoded) {
		if (_soaring_estimator_shear_sub.update(&_soaring_estimator_shear)) {
			// update the shear estimate, only if we are flying in manual feedthrough for safety reasons
			_shear_v_max = _soaring_estimator_shear.v_max;
			_shear_alpha = _soaring_estimator_shear.alpha;
			_shear_h_ref = _soaring_estimator_shear.h_ref;
			_shear_heading = _soaring_estimator_shear.psi - M_PI_2_F;
			_soaring_feasible =  _soaring_estimator_shear.soaring_feasible;
			// the initial speed of the target trajectory can safely be updated during soaring :)
			_shear_aspd = _soaring_estimator_shear.aspd;
			_param_shear_estimated_v_max.set(_shear_v_max);
			_param_shear_estimated_v_max.commit()
			_param_shear_estimated_alpha.set(_shear_alpha);
			_param_shear_estimated_alpha.commit()
			_param_shear_estimated_h_ref.set(_shear_h_ref);
			_param_shear_estimated_h_ref.commit()
			_param_shear_estimated_heading.set(_shear_heading);
			_param_shear_estimated_heading.commit()
		}

	}
}

void
FixedwingPositionINDIControl::_compute_trajectory_transform()
{
	Eulerf e(0.f, 0.f, _shear_heading);
	_R_enu_to_trajec = Dcmf(e);
	_R_trajec_to_enu = _R_enu_to_trajec.transpose();
	_vec_enu_to_trajec = Vector3f{0.f, 0.f, _shear_h_ref};
}

Vector3f
FixedwingPositionINDIControl::_compute_wind_estimate()
{
	Dcmf R_ib(_att);
	Dcmf R_bi(R_ib.transpose());
	// compute expected AoA from g-forces:
	Vector3f body_force = _mass * R_bi * (_acc + Vector3f{0.f, 0.f, 9.81f});

	// approximate lift force, since implicit equation cannot be solved analytically:
	// since alpha<<1, we approximate the lift force L = sin(alpha)*Fx - cos(alpha)*Fz
	// as L = alpha*Fx - Fz
	float Fx = cosf(_aoa_offset) * body_force(0) - sinf(_aoa_offset) * body_force(2);
	float Fz = -cosf(_aoa_offset) * body_force(2) - sinf(_aoa_offset) * body_force(0);
	float AoA_approx = (((2.f * Fz) / (_rho * _area * (fmaxf(_cal_airspeed * _cal_airspeed,
					   _stall_speed * _stall_speed)) + 0.001f) - _C_L0) / _C_L1) /
			   (1 - ((2.f * Fx) / (_rho * _area * (fmaxf(_cal_airspeed * _cal_airspeed,
					       _stall_speed * _stall_speed)) + 0.001f) / _C_L1));
	AoA_approx = constrain(AoA_approx, -0.2f, 0.3f);
	float speed = fmaxf(_cal_airspeed, _stall_speed);
	float u_approx = _true_airspeed;
	float v_approx = body_force(1) * _true_airspeed / (0.5f * _rho * powf(speed, 2) * _area * _C_B1);
	float w_approx = tanf(AoA_approx - _aoa_offset) * _true_airspeed;
	Vector3f vel_air = R_ib * (Vector3f{u_approx, v_approx, w_approx});

	// compute wind from wind triangle
	Vector3f wind = _vel - vel_air;
	//PX4_INFO("wind estimate: \t%.1f, \t%.1f, \t%.1f", (double)wind(0), (double)wind(1), (double)wind(2));
	return wind;
}

Vector3f
FixedwingPositionINDIControl::_compute_wind_estimate_EKF()
{
	Dcmf R_ib(_att);
	Dcmf R_bi(R_ib.transpose());
	// compute expected AoA from g-forces:
	Vector3f body_force = _mass * R_bi * (_acc + Vector3f{0.f, 0.f, 9.81f});

	// ***************** NEW COMPUTATION FROM MATLAB CALIBRATION **********************
	float speed = fmaxf(_cal_airspeed, _stall_speed);
	float u_approx = _true_airspeed;
	float v_approx = body_force(1) * _true_airspeed / (0.5f * _rho * powf(speed, 2) * _area * _C_B1);
	float w_approx = (-body_force(2) * _true_airspeed / (0.5f * _rho * powf(speed, 2) * _area) - 0.1949f) / 3.5928f;
	Vector3f vel_air = R_ib * (Vector3f{u_approx, v_approx, w_approx});

	// compute wind from wind triangle
	Vector3f wind = _vel - vel_air;
	//PX4_INFO("wind estimate: \t%.1f, \t%.1f, \t%.1f", (double)wind(0), (double)wind(1), (double)wind(2));
	return wind;
}

void
FixedwingPositionINDIControl::_set_wind_estimate(Vector3f wind)
{
	// apply some filtering
	wind(0) = _lp_filter_wind[0].apply(wind(0));
	wind(1) = _lp_filter_wind[1].apply(wind(1));
	wind(2) = _lp_filter_wind[2].apply(wind(2));
	_wind_estimate = wind;
	return;
}

void
FixedwingPositionINDIControl::_set_wind_estimate_EKF(Vector3f wind)
{
	// apply some filtering
	wind(0) = _lp_filter_wind_EKF[0].apply(wind(0));
	wind(1) = _lp_filter_wind_EKF[1].apply(wind(1));
	wind(2) = _lp_filter_wind_EKF[2].apply(wind(2));
	_wind_estimate_EKF = wind;
	return;
}


void
FixedwingPositionINDIControl::_reverse(char *str, int len)
{
	// copied from: https://www.geeksforgeeks.org/convert-floating-point-number-string/
	int i = 0, j = len - 1, temp;

	while (i < j) {
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

int
FixedwingPositionINDIControl::_int_to_str(int x, char str[], int d)
{
	// copied from: https://www.geeksforgeeks.org/convert-floating-point-number-string/
	int i = 0;

	while (x) {
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d) {
		str[i++] = '0';
	}

	_reverse(str, i);
	str[i] = '\0';
	return i;
}

void
FixedwingPositionINDIControl::_float_to_str(float n, char *res, int afterpoint)
{
	// copied from: https://www.geeksforgeeks.org/convert-floating-point-number-string/
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = _int_to_str(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0) {
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter
		// is needed to handle cases like 233.007
		fpart = fpart * (float)pow(10, afterpoint);

		_int_to_str((int)fpart, res + i + 1, afterpoint);
	}
}

void
FixedwingPositionINDIControl::_select_loiter_trajectory()
{

	// select loiter trajectory for loiter test
	char filename[16];

	switch (_loiter) {
	case 0:
		strcpy(filename, "trajectory0.csv");
		break;

	case 1:
		strcpy(filename, "trajectory1.csv");
		break;

	case 2:
		strcpy(filename, "trajectory2.csv");
		break;

	case 3:
		strcpy(filename, "trajectory3.csv");
		break;

	case 4:
		strcpy(filename, "trajectory4.csv");
		break;

	case 5:
		strcpy(filename, "trajectory5.csv");
		break;

	case 6:
		strcpy(filename, "trajectory6.csv");
		break;

	case 7:
		strcpy(filename, "trajectory7.csv");
		break;

	default:
		strcpy(filename, "trajectory0.csv");
	}

	/*
	Also, we need to place the current trajectory, centered around zero height and assuming wind from the west, into the soaring frame.
	Since the necessary computations for position, velocity and acceleration require the basis coefficients, which are not easy to transform,
	we choose to transform our position in soaring frame into the "trajectory frame", compute position vector and it's derivatives from the basis coeffs
	in the trajectory frame, and then transform these back to soaring frame for control purposes.
	Therefore we define a new transform between the soaring frame and the trajectory frame.
	*/

	_read_trajectory_coeffs_csv(filename);
}

void
FixedwingPositionINDIControl::_select_soaring_trajectory()
{
	/*
	We read a trajectory based on initial energy available at the beginning of the trajectory.
	So the trajectory is selected based on two criteria, first the correct wind shear params (alpha and V_max) and the initial energy (potential + kinetic).

	The filename structure of the trajectories is the following:
	trajec_<type>_<V_max>_<alpha>_<energy>_
	with
	<type> in {nominal, robust}
	<V_max> in [08,12]
	<alpha> in [020, 100]
	<energy> in [E_min, E_max]
	*/

	char file[40] = "robust/coeffs_robust";
	char v_str[3];
	char a_str[3];
	char e_str[3];
	// get the correct filename
	_float_to_str(_shear_v_max, v_str, 0);
	_float_to_str(_shear_alpha * 100.f, a_str, 0);
	_float_to_str(_shear_aspd, e_str, 0);

	strcat(file, "_");
	strcat(file, v_str);
	strcat(file, "_");
	strcat(file, a_str);
	strcat(file, "_");
	strcat(file, e_str);
	strcat(file, ".csv");
	PX4_INFO("filename: \t%.40s", file);
	// get the basis coeffs from file
	_read_trajectory_coeffs_csv(file);
}

void
FixedwingPositionINDIControl::_read_trajectory_coeffs_csv(char *filename)
{

	// =======================================================================
	bool error = false;

	//char home_dir[200] = "/home/marvin/Documents/master_thesis_ADS/PX4/Git/ethzasl_fw_px4/src/modules/fw_dyn_soar_control/trajectories/";
	char home_dir[200] = PX4_ROOTFSDIR"/fs/microsd/trajectories/";
	//PX4_ERR(home_dir);
	strcat(home_dir, filename);
	FILE *fp = fopen(home_dir, "r");

	if (fp == nullptr) {
		PX4_ERR("Can't open file");
		error = true;

	} else {
		// Here we have taken size of
		// array 1024 you can modify it
		const uint buffersize = _num_basis_funs * 32;
		char buffer[buffersize];

		int row = 0;
		int column = 0;

		// loop over rows
		while (fgets(buffer,
			     buffersize, fp)) {
			column = 0;

			// Splitting the data
			char *value = strtok(buffer, ",");

			// loop over columns
			while (value) {
				if (*value == '\0' || *value == ' ') {
					// simply skip extra characters
					continue;
				}

				switch (row) {
				case 0:
					_basis_coeffs_x(column) = (float)atof(value);
					break;

				case 1:
					_basis_coeffs_y(column) = (float)atof(value);
					break;

				case 2:
					_basis_coeffs_z(column) = (float)atof(value);
					break;

				default:
					break;
				}

				//PX4_INFO("row: %d, col: %d, read value: %.3f", row, column, (double)atof(value));
				value = strtok(NULL, ",");
				column++;

			}

			row++;
		}

		int failure = fclose(fp);

		if (failure == -1) {
			PX4_ERR("Can't close file");
		}
	}

	// =======================================================================


	// go back to safety mode loiter circle in 30m height
	if (error) {
		// 100m radius circle trajec
		_basis_coeffs_x(0) = 0.000038f;
		_basis_coeffs_x(1) = 1812.140143f;
		_basis_coeffs_x(2) = -6365.976106f;
		_basis_coeffs_x(3) = 10773.875378f;
		_basis_coeffs_x(4) = -9441.287977f;
		_basis_coeffs_x(5) = 1439.744061f;
		_basis_coeffs_x(6) = 6853.112823f;
		_basis_coeffs_x(7) = -7433.361925f;
		_basis_coeffs_x(8) = -72.566660f;
		_basis_coeffs_x(9) = 7518.521784f;
		_basis_coeffs_x(10) = -6807.858677f;
		_basis_coeffs_x(11) = -1586.021605f;
		_basis_coeffs_x(12) = 9599.405711f;
		_basis_coeffs_x(13) = -10876.256865f;
		_basis_coeffs_x(14) = 6406.017620f;
		_basis_coeffs_x(15) = -1819.600542f;

		_basis_coeffs_y(0) = -59.999852f;
		_basis_coeffs_y(1) = -2811.660383f;
		_basis_coeffs_y(2) = 13178.399227f;
		_basis_coeffs_y(3) = -30339.925641f;
		_basis_coeffs_y(4) = 43145.286828f;
		_basis_coeffs_y(5) = -37009.839292f;
		_basis_coeffs_y(6) = 9438.328009f;
		_basis_coeffs_y(7) = 23631.637452f;
		_basis_coeffs_y(8) = -38371.559953f;
		_basis_coeffs_y(9) = 23715.306334f;
		_basis_coeffs_y(10) = 9316.038368f;
		_basis_coeffs_y(11) = -36903.451639f;
		_basis_coeffs_y(12) = 43082.749551f;
		_basis_coeffs_y(13) = -30315.482005f;
		_basis_coeffs_y(14) = 13172.915247f;
		_basis_coeffs_y(15) = -2811.186858f;

		_basis_coeffs_z(0) = 30.0f;
		_basis_coeffs_z(1) = 0.0f;
		_basis_coeffs_z(2) = 0.0f;
		_basis_coeffs_z(3) = 0.0f;
		_basis_coeffs_z(4) = 0.0f;
		_basis_coeffs_z(5) = 0.0f;
		_basis_coeffs_z(6) = 0.0f;
		_basis_coeffs_z(7) = 0.0f;
		_basis_coeffs_z(8) = 0.0f;
		_basis_coeffs_z(9) = 0.0f;
		_basis_coeffs_z(10) = 0.0f;
		_basis_coeffs_z(11) = 0.0f;
		_basis_coeffs_z(12) = 0.0f;
		_basis_coeffs_z(13) = 0.0f;
		_basis_coeffs_z(14) = 0.0f;
		_basis_coeffs_z(15) = 0.0f;
	}

}

void
FixedwingPositionINDIControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if pos, vel, acc changed
	if (_vehicle_angular_velocity_sub.update(&_angular_vel)) {
		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		//const float dt = math::constrain((pos.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		//_last_run = _local_pos.timestamp;

		// check if local NED reference frame origin has changed:
		// || (_local_pos.vxy_reset_counter != _pos_reset_counter
		if (!map_projection_initialized(&_global_local_proj_ref)
		    || (_global_local_proj_ref.timestamp != _local_pos.ref_timestamp)
		    || (_local_pos.xy_reset_counter != _pos_reset_counter)
		    || (_local_pos.z_reset_counter != _alt_reset_counter)) {
			// initialize projection
			map_projection_init_timestamped(&_global_local_proj_ref, _local_pos.ref_lat, _local_pos.ref_lon,
							_local_pos.ref_timestamp);
			// project the origin of the soaring ENU frame to the current NED frame
			map_projection_project(&_global_local_proj_ref, _origin_lat, _origin_lon, &_origin_N, &_origin_E);
			_origin_D =  _local_pos.ref_alt - _origin_alt;
			PX4_INFO("local reference frame updated");
		}

		// update reset counters
		_pos_reset_counter = _local_pos.xy_reset_counter;
		_alt_reset_counter = _local_pos.z_reset_counter;

		// run polls
		vehicle_status_poll();
		airspeed_poll();
		airflow_aoa_poll();
		airflow_slip_poll();
		rc_channels_poll();
		manual_control_setpoint_poll();
		vehicle_local_position_poll();
		vehicle_attitude_poll();
		vehicle_acceleration_poll();
		vehicle_angular_velocity_poll();
		vehicle_angular_acceleration_poll();
		soaring_controller_status_poll();

		// update the shear estimate, only target airspeed is updated in soaring mode
		soaring_estimator_shear_poll();

		// update transform from trajectory frame to ENU frame (soaring frame)
		_compute_trajectory_transform();

		// ===============================
		// compute wind pseudo-measurement
		// ===============================
		Vector3f wind = _compute_wind_estimate();
		_set_wind_estimate(wind);
		Vector3f wind_EKF = _compute_wind_estimate_EKF();
		_set_wind_estimate_EKF(wind_EKF);


		// only run actuators poll, when our module is not publishing:
		if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			actuator_controls_poll();
		}

		// =================================
		// get reference point on trajectory
		// =================================
		float t_ref = _get_closest_t(_pos);

		// =================================================================
		// possibly select a new trajectory, if we are finishing the old one
		// =================================================================
		if (!_switch_origin_hardcoded && t_ref >= 0.97f && (hrt_absolute_time() - _last_time_trajec) > 1000000) {
			_select_soaring_trajectory();
			_last_time_trajec = hrt_absolute_time();
		}

		// ============================
		// compute reference kinematics
		// ============================
		// downscale velocity to match current one,
		// terminal time is determined such that current velocity is met
		Vector3f v_ref_ = _get_velocity_ref(t_ref, 1.0f);
		float T = sqrtf((v_ref_ * v_ref_) / (_vel * _vel + 0.001f));
		Vector3f pos_ref = _get_position_ref(t_ref);                    // in inertial ENU
		Vector3f vel_ref = _get_velocity_ref(t_ref, T);                 // in inertial ENU
		Vector3f acc_ref = _get_acceleration_ref(t_ref, T);             // gravity-corrected acceleration (ENU)
		Quatf q = _get_attitude_ref(t_ref, T);
		Vector3f omega_ref = _get_angular_velocity_ref(t_ref, T);       // body angular velocity
		Vector3f alpha_ref = _get_angular_acceleration_ref(t_ref, T);   // body angular acceleration

		// =====================
		// compute control input
		// =====================
		Vector3f ctrl = _compute_INDI_stage_1(pos_ref, vel_ref, acc_ref, omega_ref, alpha_ref);
		Vector3f ctrl1 = _compute_INDI_stage_2(ctrl);

		// ============================
		// compute actuator deflections
		// ============================
		Vector3f ctrl2 = _compute_actuator_deflections(ctrl1);

		// =================================
		// publish offboard control commands
		// =================================
		offboard_control_mode_s ocm{};
		ocm.actuator = true;
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);

		// Publish actuator controls only once in OFFBOARD
		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {

			// ========================================
			// publish controller position in ENU frame
			// ========================================
			_soaring_controller_position.timestamp = hrt_absolute_time();

			for (int i = 0; i < 3; i++) {
				_soaring_controller_position.pos[i] = _pos(i);
				_soaring_controller_position.vel[i] = _vel(i);
				_soaring_controller_position.acc[i] = _acc(i);
			}

			_soaring_controller_position_pub.publish(_soaring_controller_position);

			// ====================================
			// publish controller position setpoint
			// ====================================
			_soaring_controller_position_setpoint.timestamp = hrt_absolute_time();

			for (int i = 0; i < 3; i++) {
				_soaring_controller_position_setpoint.pos[i] = pos_ref(i);
				_soaring_controller_position_setpoint.vel[i] = vel_ref(i);
				_soaring_controller_position_setpoint.acc[i] = acc_ref(i);
				_soaring_controller_position_setpoint.f_command[i] = _f_command(i);
				_soaring_controller_position_setpoint.m_command[i] = _m_command(i);
				_soaring_controller_position_setpoint.w_err[i] = _w_err(i);
			}

			_soaring_controller_position_setpoint_pub.publish(_soaring_controller_position_setpoint);

			// =====================
			// publish control input
			// =====================
			//_angular_accel_sp = {};
			_angular_accel_sp.timestamp = hrt_absolute_time();
			//_angular_accel_sp.timestamp_sample = hrt_absolute_time();
			_angular_accel_sp.xyz[0] = ctrl(0);
			_angular_accel_sp.xyz[1] = ctrl(1);
			_angular_accel_sp.xyz[2] = ctrl(2);
			_angular_accel_sp_pub.publish(_angular_accel_sp);

			// =========================
			// publish attitude setpoint
			// =========================
			//_attitude_sp = {};
			Quatf q_sp(_R_enu_to_ned * Dcmf(q));
			_attitude_sp.timestamp = hrt_absolute_time();
			_attitude_sp.q_d[0] = q_sp(0);
			_attitude_sp.q_d[1] = q_sp(1);
			_attitude_sp.q_d[2] = q_sp(2);
			_attitude_sp.q_d[3] = q_sp(3);
			_attitude_sp_pub.publish(_attitude_sp);

			// ======================
			// publish rates setpoint
			// ======================
			//_angular_vel_sp = {};
			_angular_vel_sp.timestamp = hrt_absolute_time();
			_angular_vel_sp.roll = omega_ref(0);
			_angular_vel_sp.pitch = omega_ref(1);
			_angular_vel_sp.yaw = omega_ref(2);
			_angular_vel_sp_pub.publish(_angular_vel_sp);

			// =========================
			// publish acutator controls
			// =========================
			//_actuators = {};
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = hrt_absolute_time();
			_actuators.control[actuator_controls_s::INDEX_ROLL] = ctrl2(0);
			_actuators.control[actuator_controls_s::INDEX_PITCH] = ctrl2(1);
			_actuators.control[actuator_controls_s::INDEX_YAW] = ctrl2(2);
			_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _thrust;
			_actuators_0_pub.publish(_actuators);
			//print_message(_actuators);

			// =====================
			// publish wind estimate
			// =====================
			//_soaring_controller_wind = {};
			_soaring_controller_wind.timestamp = hrt_absolute_time();
			_soaring_controller_wind.wind_estimate[0] = wind(0);
			_soaring_controller_wind.wind_estimate[1] = wind(1);
			_soaring_controller_wind.wind_estimate[2] = wind(2);
			_soaring_controller_wind.wind_estimate_filtered[0] = _wind_estimate_EKF(0);
			_soaring_controller_wind.wind_estimate_filtered[1] = _wind_estimate_EKF(1);
			_soaring_controller_wind.wind_estimate_filtered[2] = _wind_estimate_EKF(2);
			_soaring_controller_wind.position[0] = _pos(0);
			_soaring_controller_wind.position[1] = _pos(1);
			_soaring_controller_wind.position[2] = _pos(2);
			_soaring_controller_wind.airspeed = _true_airspeed;

			if (_switch_cl_soaring) {
				// always update shear params in closed loop soaring mode
				_soaring_controller_wind.lock_params = false;

			} else {
				// only update in manual feedthrough in open loop soaring
				_soaring_controller_wind.lock_params = !_switch_manual;
			}

			//Eulerf e(Quatf(_attitude.q));
			//float bank = e(0);
			// only declare wind estimate valid for shear estimator, if we are close to the soaring center
			if ((float)sqrtf(powf(_pos(0), 2) + powf(_pos(1), 2)) < 100.f) {
				_soaring_controller_wind.valid = true;

			} else {
				_soaring_controller_wind.valid = false;
			}

			_soaring_controller_wind_pub.publish(_soaring_controller_wind);



			if (_counter == 100) {
				_counter = 0;
				//PX4_INFO("Feedthrough switch: \t%.2f", (double)(_rc_channels.channels[5]));
				//PX4_INFO("frequency: \t%.3f", (double)(1000000*100)/(hrt_absolute_time()-_last_time));
				_last_time = hrt_absolute_time();

			} else {
				_counter += 1;
			}
		}

		// ===========================
		// publish rate control status
		// ===========================
		rate_ctrl_status_s rate_ctrl_status{};
		rate_ctrl_status.timestamp = hrt_absolute_time();
		rate_ctrl_status.rollspeed_integ = 0.0f;
		rate_ctrl_status.pitchspeed_integ = 0.0f;
		rate_ctrl_status.yawspeed_integ = 0.0f;
		_rate_ctrl_status_pub.publish(rate_ctrl_status);

		// ==============================
		// publish soaring control status
		// ==============================
		//_soaring_controller_heartbeat_s _soaring_controller_heartbeat{};
		_soaring_controller_heartbeat.timestamp = hrt_absolute_time();
		_soaring_controller_heartbeat.heartbeat = hrt_absolute_time();
		_soaring_controller_heartbeat_pub.publish(_soaring_controller_heartbeat);

		// ====================
		// publish debug values
		// ====================
		Dcmf R_ib(_att);
		Dcmf R_bi(R_ib.transpose());
		Vector3f vel_body = R_bi * (_vel - _wind_estimate);
		_slip = atan2f(vel_body(1), vel_body(0)) * 180.f / M_PI_2_F;
		_debug_value.timestamp = hrt_absolute_time();
		_debug_value.value = _slip;
		_debug_value_pub.publish(_debug_value);

		perf_end(_loop_perf);
	}

}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_basis_funs(float t)
{
	Vector<float, _num_basis_funs> vec;
	vec(0) = 1.0f;
	float sigma = 1.0f / _num_basis_funs;

	for (uint i = 1; i < _num_basis_funs; i++) {
		float fun1 = sinf(M_PI_F * t);
		float fun2 = exp(-powf((t - float(i) / float(_num_basis_funs)), 2) / sigma);
		vec(i) = fun1 * fun2;
	}

	return vec;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_d_dt_basis_funs(float t)
{
	Vector<float, _num_basis_funs> vec;
	vec(0) = 0.0f;
	float sigma = 1.0f / _num_basis_funs;

	for (uint i = 1; i < _num_basis_funs; i++) {
		float fun1 = sinf(M_PI_F * t);
		float fun2 = exp(-powf((t - float(i) / _num_basis_funs), 2) / sigma);
		vec(i) = fun2 * (M_PI_F * sigma * cosf(M_PI_F * t) - 2 * (t - float(i) / _num_basis_funs) * fun1) / sigma;
	}

	return vec;
}

Vector<float, FixedwingPositionINDIControl::_num_basis_funs>
FixedwingPositionINDIControl::_get_d2_dt2_basis_funs(float t)
{
	Vector<float, _num_basis_funs> vec;
	vec(0) = 0.0f;
	float sigma = 1.0f / _num_basis_funs;

	for (uint i = 1; i < _num_basis_funs; i++) {
		float fun1 = sinf(M_PI_F * t);
		float fun2 = exp(-powf((t - float(i) / _num_basis_funs), 2) / sigma);
		vec(i) = fun2 * (fun1 * (4 * powf((float(i) / _num_basis_funs - t), 2) - \
					 sigma * (powf(M_PI_F, 2) * sigma + 2)) + 4 * M_PI_F * sigma * (float(i) / _num_basis_funs - t) * cosf(M_PI_F * t)) /
			 (powf(sigma, 2));

	}

	return vec;
}

Vector3f
FixedwingPositionINDIControl::_get_position_ref(float t)
{
	Vector<float, _num_basis_funs> basis = _get_basis_funs(t);
	float x = _basis_coeffs_x * basis;
	float y = _basis_coeffs_y * basis;
	float z = _basis_coeffs_z * basis;
	return _R_trajec_to_enu * Vector3f{x, y, z} + _vec_enu_to_trajec;
}

Vector3f
FixedwingPositionINDIControl::_get_velocity_ref(float t, float T)
{
	Vector<float, _num_basis_funs> basis = _get_d_dt_basis_funs(t);
	float x = _basis_coeffs_x * basis;
	float y = _basis_coeffs_y * basis;
	float z = _basis_coeffs_z * basis;
	return _R_trajec_to_enu * (Vector3f{x, y, z} / T);
}

Vector3f
FixedwingPositionINDIControl::_get_acceleration_ref(float t, float T)
{
	Vector<float, _num_basis_funs> basis = _get_d2_dt2_basis_funs(t);
	float x = _basis_coeffs_x * basis;
	float y = _basis_coeffs_y * basis;
	float z = _basis_coeffs_z * basis;
	return _R_trajec_to_enu * (Vector3f{x, y, z} / powf(T, 2));
}

Quatf
FixedwingPositionINDIControl::_get_attitude_ref(float t, float T)
{
	Vector3f vel = _get_velocity_ref(t, T);
	Vector3f vel_air = vel - _wind_estimate;
	Vector3f acc = _get_acceleration_ref(t, T);
	// add gravity
	acc(2) += 9.81f;
	// compute required force
	Vector3f f = _mass * acc;
	// compute force component projected onto lift axis
	Vector3f vel_normalized = vel_air.normalized();
	Vector3f f_lift = f - (f * vel_normalized) * vel_normalized;
	Vector3f lift_normalized = f_lift.normalized();
	Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
	// compute rotation matrix between ENU and FRD frame
	Dcmf R_bi;
	R_bi(0, 0) = vel_normalized(0);
	R_bi(0, 1) = vel_normalized(1);
	R_bi(0, 2) = vel_normalized(2);
	R_bi(1, 0) = wing_normalized(0);
	R_bi(1, 1) = wing_normalized(1);
	R_bi(1, 2) = wing_normalized(2);
	R_bi(2, 0) = lift_normalized(0);
	R_bi(2, 1) = lift_normalized(1);
	R_bi(2, 2) = lift_normalized(2);
	R_bi.renormalize();
	// compute actual air density to be used with true airspeed
	float rho_corrected;

	if (_cal_airspeed >= _stall_speed) {
		rho_corrected = _rho * powf(_cal_airspeed / _true_airspeed, 2);

	} else {
		rho_corrected = _rho;
	}

	// compute required AoA
	Vector3f f_phi = R_bi * f_lift;
	float AoA = ((2.f * f_phi(2)) / (rho_corrected * _area * (vel_air * vel_air) + 0.001f) - _C_L0) / _C_L1 - _aoa_offset;
	// compute final rotation matrix
	Eulerf e(0.f, AoA, 0.f);
	Dcmf R_pitch(e);
	Dcmf Rotation(R_pitch * R_bi);
	// switch from FRD to ENU frame
	Rotation(1, 0) *= -1.f;
	Rotation(1, 1) *= -1.f;
	Rotation(1, 2) *= -1.f;
	Rotation(2, 0) *= -1.f;
	Rotation(2, 1) *= -1.f;
	Rotation(2, 2) *= -1.f;
	// plausibility check
	/*
	float determinant = Rotation(0,0)*(Rotation(1,1)*Rotation(2,2)-Rotation(2,1)*Rotation(1,2)) -
	                    Rotation(1,0)*(Rotation(0,1)*Rotation(2,2)-Rotation(2,1)*Rotation(0,2)) +
	                    Rotation(2,0)*(Rotation(0,1)*Rotation(1,2)-Rotation(1,1)*Rotation(0,2));
	PX4_INFO("determinant: %.2f", (double)determinant);
	PX4_INFO("length: %.2f", (double)(wing_normalized*wing_normalized));
	*/



	Quatf q(Rotation.transpose());
	return q;
}

Vector3f
FixedwingPositionINDIControl::_get_angular_velocity_ref(float t, float T)
{
	float dt = 0.001;
	float t_lower = fmaxf(0.f, t - dt);
	float t_upper = fminf(t + dt, 1.f);
	Dcmf R_i0(_get_attitude_ref(t_lower, T));
	Dcmf R_i1(_get_attitude_ref(t_upper, T));
	Dcmf R_10 = R_i1.transpose() * R_i0;
	AxisAnglef w_01(R_10);
	return -w_01.axis() * w_01.angle() / (T * (t_upper - t_lower));
}

Vector3f
FixedwingPositionINDIControl::_get_angular_acceleration_ref(float t, float T)
{
	float dt = 0.001;
	float t_lower = fmaxf(0.f, t - dt);
	float t_upper = fminf(t + dt, 1.f);
	// compute roational velocity in inertial frame
	Dcmf R_i0(_get_attitude_ref(t_lower, T));
	AxisAnglef w_0(R_i0 * _get_angular_velocity_ref(t_lower, T));
	// compute roational velocity in inertial frame
	Dcmf R_i1(_get_attitude_ref(t_upper, T));
	AxisAnglef w_1(R_i1 * _get_angular_velocity_ref(t_upper, T));
	// compute gradient via finite differences
	Vector3f dw_dt = (w_1.axis() * w_1.angle() - w_0.axis() * w_0.angle()) / (T * (t_upper - t_lower));
	// transform back to body frame
	return R_i0.transpose() * dw_dt;
}

float
FixedwingPositionINDIControl::_get_closest_t(Vector3f pos)
{
	// multi-stage computation of the closest point on the reference path

	const uint n_1 = 20;
	Vector<float, n_1> distances;
	float t_ref;

	// =======
	// STAGE 1
	// =======
	// STAGE 1: compute all distances
	for (uint i = 0; i < n_1; i++) {
		t_ref = float(i) / float(n_1);
		Vector3f pos_ref = _get_position_ref(t_ref);
		distances(i) = (pos_ref - pos) * (pos_ref - pos);
	}

	// STAGE 1: get index of smallest distance
	float t_1 = 0.f;
	float min_dist = distances(0);

	for (uint i = 1; i < n_1; i++) {
		if (distances(i) < min_dist) {
			min_dist = distances(i);
			t_1 = float(i);
		}
	}

	t_1 = t_1 / float(n_1);

	// =======
	// STAGE 2
	// =======
	const uint n_2 = 2 * n_1 - 1;
	Vector < float, n_2 + 1 > distances_2;
	float t_lower = fmod(t_1 - 1.0f / n_1, 1.0f);

	// STAGE 2: compute all distances
	for (uint i = 0; i <= n_2; i++) {
		t_ref = fmod(t_lower + float(i) * 2.f / float(n_1 * n_2), 1.0f);
		Vector3f pos_ref = _get_position_ref(t_ref);
		distances_2(i) = (pos_ref - pos) * (pos_ref - pos);
	}

	// STAGE 2: get index of smallest distance
	float t_2 = 0.f;
	min_dist = distances_2(0);

	for (uint i = 1; i <= n_2; i++) {
		if (distances_2(i) < min_dist) {
			min_dist = distances_2(i);
			t_2 = float(i);
		}
	}

	t_2 = fmod(t_lower + float(t_2) * 2.f / (float(n_2) * float(n_1)), 1.0f);

	return t_2;
}

Quatf
FixedwingPositionINDIControl::_get_attitude(Vector3f vel, Vector3f f)
{
	Vector3f vel_air = vel - _wind_estimate;
	// compute force component projected onto lift axis
	Vector3f vel_normalized = vel_air.normalized();
	Vector3f f_lift = f - (f * vel_normalized) * vel_normalized;
	Vector3f lift_normalized = f_lift.normalized();
	Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
	// compute rotation matrix between ENU and FRD frame
	Dcmf R_bi;
	R_bi(0, 0) = vel_normalized(0);
	R_bi(0, 1) = vel_normalized(1);
	R_bi(0, 2) = vel_normalized(2);
	R_bi(1, 0) = wing_normalized(0);
	R_bi(1, 1) = wing_normalized(1);
	R_bi(1, 2) = wing_normalized(2);
	R_bi(2, 0) = lift_normalized(0);
	R_bi(2, 1) = lift_normalized(1);
	R_bi(2, 2) = lift_normalized(2);
	R_bi.renormalize();
	float rho_corrected;

	if (_cal_airspeed >= _stall_speed) {
		rho_corrected = _rho * powf(_cal_airspeed / _true_airspeed, 2);

	} else {
		rho_corrected = _rho;
	}

	// compute required AoA
	Vector3f f_phi = R_bi * f_lift;
	float AoA = ((2.f * f_phi(2)) / (rho_corrected * _area * (vel_air * vel_air) + 0.001f) - _C_L0) / _C_L1 - _aoa_offset;
	// compute final rotation matrix
	Eulerf e(0.f, AoA, 0.f);
	Dcmf R_pitch(e);
	Dcmf Rotation(R_pitch * R_bi);
	// switch from FRD to ENU frame
	Rotation(1, 0) *= -1.f;
	Rotation(1, 1) *= -1.f;
	Rotation(1, 2) *= -1.f;
	Rotation(2, 0) *= -1.f;
	Rotation(2, 1) *= -1.f;
	Rotation(2, 2) *= -1.f;

	Quatf q(Rotation.transpose());
	return q;
}

Vector3f
FixedwingPositionINDIControl::_compute_INDI_stage_1(Vector3f pos_ref, Vector3f vel_ref, Vector3f acc_ref,
		Vector3f omega_ref, Vector3f alpha_ref)
{
	Dcmf R_ib(_att);
	Dcmf R_bi(R_ib.transpose());
	// apply LP filter to acceleration & velocity
	Vector3f acc_filtered;
	acc_filtered(0) = _lp_filter_accel[0].apply(_acc(0));
	acc_filtered(1) = _lp_filter_accel[1].apply(_acc(1));
	acc_filtered(2) = _lp_filter_accel[2].apply(_acc(2));
	Vector3f omega_filtered;
	omega_filtered(0) = _lp_filter_omega[0].apply(_omega(0));
	omega_filtered(1) = _lp_filter_omega[1].apply(_omega(1));
	omega_filtered(2) = _lp_filter_omega[2].apply(_omega(2));

	// =========================================
	// apply PD control law on the body position
	// =========================================
	Vector3f acc_command = R_ib * (_K_x * R_bi * (pos_ref - _pos) + _K_v * R_bi * (vel_ref - _vel) + _K_a * R_bi *
				       (acc_ref - _acc)) + acc_ref;

	// ==================================
	// compute expected aerodynamic force
	// ==================================
	Vector3f f_current;
	Vector3f vel_body = R_bi * (_vel - _wind_estimate);
	float AoA = atan2f(vel_body(2), vel_body(0)) + _aoa_offset;
	float C_l = _C_L0 + _C_L1 * AoA;
	float C_d = _C_D0 + _C_D1 * AoA + _C_D2 * powf(AoA, 2);
	// compute actual air density
	float rho_corrected;

	if (_cal_airspeed >= _stall_speed) {
		rho_corrected = _rho * powf(_cal_airspeed / _true_airspeed, 2);

	} else {
		rho_corrected = _rho;
	}

	float factor = -0.5f * rho_corrected * _area * sqrtf(vel_body * vel_body);
	Vector3f w_x = vel_body;
	Vector3f w_z = w_x.cross(Vector3f{0.f, 1.f, 0.f});
	f_current = R_ib * (factor * (C_l * w_z + C_d * w_x));
	// apply LP filter to force
	Vector3f f_current_filtered;
	f_current_filtered(0) = _lp_filter_force[0].apply(f_current(0));
	f_current_filtered(1) = _lp_filter_force[1].apply(f_current(1));
	f_current_filtered(2) = _lp_filter_force[2].apply(f_current(2));

	// ================================
	// get force command in world frame
	// ================================
	Vector3f f_command = _mass * (acc_command - acc_filtered) + f_current_filtered;

	// ============================================================================================================
	// apply some filtering to the force command. This introduces some time delay,
	// which is not desired for stability reasons, but it rejects some of the noise fed to the low-level controller
	// ============================================================================================================
	f_command(0) = _lp_filter_ctrl0[0].apply(f_command(0));
	f_command(1) = _lp_filter_ctrl0[1].apply(f_command(1));
	f_command(2) = _lp_filter_ctrl0[2].apply(f_command(2));
	_f_command = f_command;
	// limit maximum lift force by the maximum lift force, the aircraft can produce (assume max force at 15° aoa)

	// ====================================================================
	// saturate force command to avoid overly agressive maneuvers and stall
	// ====================================================================
	if (_switch_saturation) {
		float speed = vel_body * vel_body;
		// compute maximum achievable force
		float f_max;

		if (speed > _stall_speed) {
			f_max = -factor * sqrtf(vel_body * vel_body) * (_C_L0 + _C_L1 * 0.25f); // assume stall at 15° AoA

		} else {
			f_max = -factor * _stall_speed * (_C_L0 + _C_L1 * 0.25f); // assume stall at 15° AoA
		}

		// compute current command
		float f_now = sqrtf(f_command * f_command);

		// saturate current command
		if (f_now > f_max) {
			f_command = f_max / f_now * f_command;
		}
	}

	// ==========================================================================
	// get required attitude (assuming we can fly the target velocity), and error
	// ==========================================================================
	Dcmf R_ref(_get_attitude(vel_ref, f_command));
	// get attitude error
	Dcmf R_ref_true(R_ref.transpose()*R_ib);
	// get required rotation vector (in body frame)
	AxisAnglef q_err(R_ref_true);
	Vector3f w_err;

	// project rotation angle to [-pi,pi]
	if (q_err.angle()*q_err.angle() < M_PI_F * M_PI_F) {
		w_err = -q_err.angle() * q_err.axis();

	} else {
		if (q_err.angle() > 0.f) {
			w_err = (2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();

		} else {
			w_err = (-2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();
		}
	}

	// =========================================
	// apply PD control law on the body attitude
	// =========================================
	Vector3f rot_acc_command = _K_q * w_err + _K_w * (omega_ref - _omega) + alpha_ref;

	if (sqrtf(w_err * w_err) > M_PI_F) {
		PX4_ERR("rotation angle larger than pi: \t%.2f, \t%.2f, \t%.2f", (double)sqrtf(w_err * w_err), (double)q_err.angle(),
			(double)(q_err.axis()*q_err.axis()));
	}

	// ====================================
	// manual attitude setpoint feedthrough
	// ====================================
	if (_switch_manual) {
		// get an attitude setpoint from the current manual inputs
		float roll_ref = 1.f * _manual_control_setpoint.y * 1.0f;
		float pitch_ref = -1.f * _manual_control_setpoint.x * M_PI_4_F;
		Eulerf E_current(Quatf(_attitude.q));
		float yaw_ref = E_current.psi();
		Dcmf R_ned_frd_ref(Eulerf(roll_ref, pitch_ref, yaw_ref));
		Dcmf R_enu_frd_ref(_R_ned_to_enu * R_ned_frd_ref);
		Quatf att_ref(R_enu_frd_ref);
		R_ref = Dcmf(att_ref);

		// get attitude error
		R_ref_true = Dcmf(R_ref.transpose() * R_ib);
		// get required rotation vector (in body frame)
		q_err = AxisAnglef(R_ref_true);

		// project rotation angle to [-pi,pi]
		if (q_err.angle()*q_err.angle() < M_PI_F * M_PI_F) {
			w_err = -q_err.angle() * q_err.axis();

		} else {
			if (q_err.angle() > 0.f) {
				w_err = (2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();

			} else {
				w_err = (-2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();
			}
		}

		_w_err = w_err;

		// compute rot acc command
		rot_acc_command = _K_q * w_err + _K_w * (Vector3f{0.f, 0.f, 0.f} -_omega);

	}

	// ==============================================================
	// overwrite rudder rot_acc_command with turn coordination values
	// ==============================================================
	Vector3f vel_air = _vel - _wind_estimate;
	Vector3f vel_normalized = vel_air.normalized();
	Vector3f acc_normalized = acc_filtered.normalized();
	// compute ideal angular velocity
	Vector3f omega_turn_ref_normalized = vel_normalized.cross(acc_normalized);
	Vector3f omega_turn_ref;
	// constuct acc perpendicular to flight path
	Vector3f acc_perp = acc_filtered - (acc_filtered * vel_normalized) * vel_normalized;

	if (_airspeed_valid && _cal_airspeed > _stall_speed) {
		omega_turn_ref = sqrtf(acc_perp * acc_perp) / (_true_airspeed) * R_bi * omega_turn_ref_normalized.normalized();
		//PX4_INFO("yaw rate ref, yaw rate: \t%.2f\t%.2f", (double)(omega_turn_ref(2)), (double)(omega_filtered(2)));

	} else {
		omega_turn_ref = sqrtf(acc_perp * acc_perp) / (_stall_speed) * R_bi * omega_turn_ref_normalized.normalized();
		//PX4_ERR("No valid airspeed message detected or airspeed too low");
	}

	// apply some smoothing since we don't want HF components in our rudder output
	omega_turn_ref(2) = _lp_filter_rud.apply(omega_turn_ref(2));

	// transform rate vector to body frame
	float scaler = (_stall_speed * _stall_speed) / fmaxf(sqrtf(vel_body * vel_body) * vel_body(0),
			_stall_speed * _stall_speed);
	// not really an accel command, rather a FF-P command
	rot_acc_command(2) = _K_q(2, 2) * omega_turn_ref(2) * scaler + _K_w(2,
			     2) * (omega_turn_ref(2) - omega_filtered(2)) * scaler * scaler;


	return rot_acc_command;
}

Vector3f
FixedwingPositionINDIControl::_compute_INDI_stage_2(Vector3f ctrl)
{
	// compute velocity in body frame
	Dcmf R_ib(_att);
	Vector3f vel_body = R_ib.transpose() * (_vel - _wind_estimate);
	float q = fmaxf(0.5f * sqrtf(vel_body * vel_body) * vel_body(0),
			0.5f * _stall_speed * _stall_speed); // dynamic pressure, saturates at stall speed

	// compute moments
	Vector3f moment;
	moment(0) = _k_ail * q * _actuators.control[actuator_controls_s::INDEX_ROLL] - _k_d_roll * q * _omega(0);
	moment(1) = _k_ele * q * _actuators.control[actuator_controls_s::INDEX_PITCH] - _k_d_pitch * q * _omega(1);
	moment(2) = 0.f;

	// introduce artificial time delay that is also present in acceleration
	Vector3f moment_filtered;
	moment_filtered(0) = _lp_filter_delay[0].apply(moment(0));
	moment_filtered(1) = _lp_filter_delay[1].apply(moment(1));
	moment_filtered(2) = _lp_filter_delay[2].apply(moment(2));

	// No filter for alpha, since it is already filtered...
	Vector3f alpha_filtered = _alpha;
	Vector3f moment_command = _inertia * (ctrl - alpha_filtered) + moment_filtered;
	_m_command = R_ib.transpose() * moment_command;

	// perform dynamic inversion
	Vector3f deflection;
	deflection(0) = (moment_command(0) + _k_d_roll * q * _omega(0)) / fmaxf((_k_ail * q), 0.0001f);
	deflection(1) = (moment_command(1) + _k_d_pitch * q * _omega(1)) / fmaxf((_k_ele * q), 0.0001f);

	// overwrite rudder deflection with NDI turn coordination (no INDI)
	deflection(2) = ctrl(2);

	return deflection;
}


Vector3f
FixedwingPositionINDIControl::_compute_actuator_deflections(Vector3f ctrl)
{
	// compute the normalized actuator deflection, including airspeed scaling
	Vector3f deflection = ctrl;

	// limit actuator deflection
	for (int i = 0; i < 3; i++) {
		deflection(i) = constrain(deflection(i), -1.f, 1.f);
	}

	/*
	// add servo slew
	float current_ail = _actuators.control[actuator_controls_s::INDEX_ROLL];
	float current_ele = _actuators.control[actuator_controls_s::INDEX_PITCH];
	float current_rud = _actuators.control[actuator_controls_s::INDEX_YAW];
	//
	float max_rate = 0.5f/0.18f;    //
	float dt = 1.f/_sample_frequency;
	//
	deflection(0) = constrain(deflection(0),current_ail-dt*max_rate,current_ail+dt*max_rate);
	deflection(1) = constrain(deflection(1),current_ele-dt*max_rate,current_ele+dt*max_rate);
	deflection(2) = constrain(deflection(2),current_rud-dt*max_rate,current_rud+dt*max_rate);
	*/
	return deflection;
}



int FixedwingPositionINDIControl::task_spawn(int argc, char *argv[])
{
	FixedwingPositionINDIControl *instance = new FixedwingPositionINDIControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int FixedwingPositionINDIControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingPositionINDIControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_dyn_soar_control is the fixed wing controller for dynamic soaring tasks.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_dyn_soar_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_dyn_soar_control_main(int argc, char *argv[])
{
	return FixedwingPositionINDIControl::main(argc, argv);
}
