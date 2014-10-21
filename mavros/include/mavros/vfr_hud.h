/**
 * @brief VFR HUD plugin
 * @file vfr_hud.cpp
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

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>

#include <mavros/VFR_HUD.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {

/**
 * @brief VFR HUD plugin.
 */
class VfrHudPlugin : public MavRosPlugin {
public:
	VfrHudPlugin();

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	std::string const get_name() const;

	const message_map get_rx_handlers();

private:
	ros::Publisher vfr_pub;
	ros::Publisher wind_pub;

	void handle_vfr_hud(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

#ifdef MAVLINK_MSG_ID_WIND
	/**
	 * Handle APM specific wind direction estimation message
	 */
	void handle_wind(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
#endif
};

}; // namespace mavplugin

