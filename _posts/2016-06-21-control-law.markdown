---
layout: post
title:  "Multi Copter Control Law of Pixhawk"
date:   2016-06-21 22:43:16 +0200
categories: essay
---

{% highlight c++ %}
/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
{% endhighlight %}

{% highlight c++ %}
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;
	R_sp.set(_v_att_sp.R_body);

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();
{% endhighlight %}



QG = R QP

R is the rotation matrix;

{% highlight c++ %}	
	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

{% endhighlight %}

R_z    is the Z axis vector of current attitude in ground coordinate;
R_sp_z is the Z axis vector of target  attitude in ground coordinate;

(R_z % R_sp_z) is cross product(向量积); the direction is Perpendicular(垂直) to the two inputs, which means the rotation axis (in ground coordinate) to align the airframe Z axis of the current attitude to target attitude;

so `e_R = R.transposed() * (R_z % R_sp_z)` means the same rotation axis as above, but in body coordinate;

{% highlight c++ %}

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f: -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if (_v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));
		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* weather-vane mode, dampen yaw rate */
	if (_v_att_sp.disable_mc_yaw_control == true && _v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
	}

	/* feed forward yaw setpoint rate */
	_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

	/* weather-vane mode, scale down yaw rate */
	if (_v_att_sp.disable_mc_yaw_control == true && _v_control_mode.flag_control_velocity_enabled && !_v_control_mode.flag_control_manual_enabled) {
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
	}

}
{% endhighlight %}
