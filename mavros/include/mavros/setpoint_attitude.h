/**
 * @brief SetpointAttitude plugin
 * @file setpoint_attitude.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

namespace mavplugin {

/**
 * @brief Setpoint attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointAttitudePlugin : public MavRosPlugin,
	private TFListenerMixin<SetpointAttitudePlugin> {
public:
	SetpointAttitudePlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	const std::string get_name() const;

	const message_map get_rx_handlers();

private:
	friend class TFListenerMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber att_sub;
	ros::Subscriber throttle_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;
	bool reverse_throttle;

	/* -*- low-level send -*- */

	void set_attitude_target(uint32_t time_boot_ms,
			uint8_t type_mask,
			float q[4],
			float roll_rate, float pitch_rate, float yaw_rate,
			float thrust);

	/* -*- mid-level helpers -*- */

	/**
	 * Send attitude setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_transform(const tf::Transform &transform, const ros::Time &stamp);

	/**
	 * Send angular velocity setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_ang_velocity(const ros::Time &stamp, const float vx, const float vy, const float vz);

	/**
	 * Send throttle to FCU attitude controller
	 */
	void send_attitude_throttle(const float throttle);

	/* -*- callbacks -*- */

	void pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req);

	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &req);

	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req);

	inline bool is_normalized(float throttle, const float min, const float max);

	void throttle_cb(const std_msgs::Float64::ConstPtr &req);

};

}; // namespace mavplugin

