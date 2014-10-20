/**
 * @brief LocalPosition plugin
 * @file local_position.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
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
#ifndef MAVPLUGIN_LOCAL_POSITION_H__
#define MAVPLUGIN_LOCAL_POSITION_H__

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavplugin {
/**
 * @brief Local position plugin.
 * Publish local position to TF and PositionStamped,
 * send local position and visual position estimates to FCU.
 */
class LocalPositionPlugin : public MavRosPlugin {
public:
	LocalPositionPlugin();
	void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater);
	std::string const get_name() const;
	const message_map get_rx_handlers();

private:
	UAS *uas;

	ros::NodeHandle pos_nh;
	ros::Publisher local_position;
	tf::TransformBroadcaster tf_broadcaster;

	std::string frame_id;		//!< origin for TF
	std::string child_frame_id;	//!< frame for TF and Pose
	bool send_tf;

	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

};

};

#endif
