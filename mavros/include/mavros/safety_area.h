/**
 * @brief SafetyArea plugin
 * @file safety_area.cpp
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

#include <geometry_msgs/PolygonStamped.h>

namespace mavplugin {

/**
 * @brief Safety allopwed area plugin
 *
 * Send safety area to FCU controller.
 */
class SafetyAreaPlugin : public MavRosPlugin {
public:
	SafetyAreaPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	const std::string get_name() const;

	const message_map get_rx_handlers();

private:
	UAS *uas;

	ros::NodeHandle safety_nh;
	ros::Subscriber safetyarea_sub;

	/* -*- low-level send -*- */

	void safety_set_allowed_area(
			uint8_t coordinate_frame,
			float p1x, float p1y, float p1z,
			float p2x, float p2y, float p2z);

	/* -*- mid-level helpers -*- */

	/**
	 * Send a safety zone (volume), which is defined by two corners of a cube,
	 * to the FCU.
	 *
	 * @note ENU frame.
	 */
	void send_safety_set_allowed_area(float p1x, float p1y, float p1z,
			float p2x, float p2y, float p2z);

	/* -*- callbacks -*- */

	void safetyarea_cb(const geometry_msgs::PolygonStamped::ConstPtr &req);

};

}; // namespace mavplugin

