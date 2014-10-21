/**
 * @brief SetpointPosition plugin
 * @file setpoint_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
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

namespace mavplugin {

/**
 * @brief Setpoint position plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointPositionPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
	private TFListenerMixin<SetpointPositionPlugin> {
public:
	SetpointPositionPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	const std::string get_name() const;

	const message_map get_rx_handlers();

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class TFListenerMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber setpoint_sub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;

	/* -*- mid-level helpers -*- */

	/**
	 * Send transform to FCU position controller
	 *
	 * Note: send only XYZ, Yaw
	 */
	void send_setpoint_transform(const tf::Transform &transform, const ros::Time &stamp);

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr &req);

};

}; // namespace mavplugin

