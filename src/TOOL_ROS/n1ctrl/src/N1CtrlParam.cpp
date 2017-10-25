#include "N1CtrlParam.h"

Parameter_t::Parameter_t()
{

}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle& nh)
{
	read_essential_param(nh, "gain/hover/Kp0", hover_gain.Kp0);
	read_essential_param(nh, "gain/hover/Kp1", hover_gain.Kp1);
	read_essential_param(nh, "gain/hover/Kp2", hover_gain.Kp2);
	read_essential_param(nh, "gain/hover/Kv0", hover_gain.Kv0);
	read_essential_param(nh, "gain/hover/Kv1", hover_gain.Kv1);
	read_essential_param(nh, "gain/hover/Kv2", hover_gain.Kv2);

	read_essential_param(nh, "gain/hover/Kvi0", hover_gain.Kvi0);
	read_essential_param(nh, "gain/hover/Kvi1", hover_gain.Kvi1);
	read_essential_param(nh, "gain/hover/Kvi2", hover_gain.Kvi2);

	read_essential_param(nh, "gain/hover/Kr0", hover_gain.Kr0);
	read_essential_param(nh, "gain/hover/Kr1", hover_gain.Kr1);
	read_essential_param(nh, "gain/hover/Kr2", hover_gain.Kr2);
	read_essential_param(nh, "gain/hover/Kw0", hover_gain.Kw0);
	read_essential_param(nh, "gain/hover/Kw1", hover_gain.Kw1);
	read_essential_param(nh, "gain/hover/Kw2", hover_gain.Kw2);
	read_essential_param(nh, "gain/hover/Ka0", hover_gain.Ka0);
	read_essential_param(nh, "gain/hover/Ka1", hover_gain.Ka1);
	read_essential_param(nh, "gain/hover/Ka2", hover_gain.Ka2);
	read_essential_param(nh, "gain/track/Kp0", track_gain.Kp0);
	read_essential_param(nh, "gain/track/Kp1", track_gain.Kp1);
	read_essential_param(nh, "gain/track/Kp2", track_gain.Kp2);
	read_essential_param(nh, "gain/track/Kv0", track_gain.Kv0);
	read_essential_param(nh, "gain/track/Kv1", track_gain.Kv1);
	read_essential_param(nh, "gain/track/Kv2", track_gain.Kv2);
	read_essential_param(nh, "gain/track/Kr0", track_gain.Kr0);
	read_essential_param(nh, "gain/track/Kr1", track_gain.Kr1);
	read_essential_param(nh, "gain/track/Kr2", track_gain.Kr2);
	read_essential_param(nh, "gain/track/Kw0", track_gain.Kw0);
	read_essential_param(nh, "gain/track/Kw1", track_gain.Kw1);
	read_essential_param(nh, "gain/track/Kw2", track_gain.Kw2);
	read_essential_param(nh, "gain/track/Ka0", track_gain.Ka0);
	read_essential_param(nh, "gain/track/Ka1", track_gain.Ka1);
	read_essential_param(nh, "gain/track/Ka2", track_gain.Ka2);

	read_essential_param(nh, "idling/desired_height_limit", idling.desired_height_limit);
	read_essential_param(nh, "idling/desired_velo_limit", idling.desired_velo_limit); // 0.01
	read_essential_param(nh, "idling/feedback_velo_limit", idling.feedback_velo_limit); // 0.1
	read_essential_param(nh, "idling/js_thrust_limit", idling.js_thrust_limit);
	read_essential_param(nh, "idling/landing_timeout", idling.landing_timeout);
	read_essential_param(nh, "idling/landing_thrust_percent", idling.landing_thrust_percent);
	read_essential_param(nh, "idling/lowest_thrust", idling.lowest_thrust);

	read_essential_param(nh, "rc/hori_velo_scale", rc.hori_velo_scale);
	read_essential_param(nh, "rc/vert_velo_scale", rc.vert_velo_scale);
	read_essential_param(nh, "rc/yaw_scale", rc.yaw_scale);
	read_essential_param(nh, "rc/attitude_scale", rc.attitude_scale);
	read_essential_param(nh, "rc/deadzone", rc.deadzone);
	read_essential_param(nh, "rc/exit_cmd_deadzone", rc.exit_cmd_deadzone);

	read_essential_param(nh, "hover/set_hov_percent_to_zero", hover.set_hov_percent_to_zero);
	read_essential_param(nh, "hover/use_hov_percent_kf", hover.use_hov_percent_kf);
	read_essential_param(nh, "hover/vert_velo_limit_for_update", hover.vert_velo_limit_for_update);
	read_essential_param(nh, "hover/vert_height_limit_for_update", hover.vert_height_limit_for_update);
	read_essential_param(nh, "hover/percent_lower_limit", hover.percent_lower_limit);
	read_essential_param(nh, "hover/percent_higher_limit", hover.percent_higher_limit);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);

	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "hov_percent", hov_percent);
	read_essential_param(nh, "full_thrust", full_thrust);
	read_essential_param(nh, "ctrl_rate", ctrl_rate);
	read_essential_param(nh, "js_ctrl_lock_velo", js_ctrl_lock_velo);
	
	// read_essential_param(nh, "paramname", paramval);

	nh.param("work_mode", work_mode, std::string("realtime"));
	nh.param("js_ctrl_mode", js_ctrl_mode, std::string("feedback"));
};

void Parameter_t::init()
{
	full_thrust = mass * gra / hov_percent;
};

void Parameter_t::config_full_thrust(double hov)
{
	ROS_ASSERT(full_thrust>0.1);
	full_thrust = hover.use_hov_percent_kf?(mass * gra / hov):full_thrust;
};
