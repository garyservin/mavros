/**
 * @brief SetpointAcceleration plugin
 * @file setpoint_accel.cpp
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

#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/Vector3Stamped.h>

namespace mavplugin {

/**
 * @brief Setpoint acceleration/force plugin
 *
 * Send setpoint accelerations/forces to FCU controller.
 */
class SetpointAccelerationPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin> {
public:
	SetpointAccelerationPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	const std::string get_name() const;

	const message_map get_rx_handlers();

private:
	friend class SetPositionTargetLocalNEDMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber accel_sub;

	bool send_force;

	/* -*- mid-level helpers -*- */

	/**
	 * Send acceleration/force to FCU acceleration controller
	 *
	 * Note: send only AFX AFY AFZ. ENU frame.
	 */
	void send_setpoint_acceleration(const ros::Time &stamp, float afx, float afy, float afz);

	/* -*- callbacks -*- */

	void accel_cb(const geometry_msgs::Vector3Stamped::ConstPtr &req);

};

}; // namespace mavplugin

