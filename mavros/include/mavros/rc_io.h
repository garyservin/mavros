/**
 * @brief RC IO plugin
 * @file rc_io.cpp
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

#include <mavros/RCIn.h>
#include <mavros/RCOut.h>
#include <mavros/OverrideRCIn.h>

namespace mavplugin {

/**
 * @brief RC IO plugin
 */
class RCIOPlugin : public MavRosPlugin {
public:
	RCIOPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	std::string const get_name() const;

	const message_map get_rx_handlers();

private:
	std::recursive_mutex mutex;
	UAS *uas;

	std::vector<uint16_t> raw_rc_in;
	std::vector<uint16_t> raw_rc_out;
	bool has_rc_channels_msg;

	ros::NodeHandle rc_nh;
	ros::Publisher rc_in_pub;
	ros::Publisher rc_out_pub;
	ros::Subscriber override_sub;

	/* -*- rx handlers -*- */

	void handle_rc_channels_raw(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_rc_channels(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_servo_output_raw(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	/* -*- low-level send functions -*- */

	void rc_channels_override(const boost::array<uint16_t, 8> &channels);

	/* -*- callbacks -*- */

	void connection_cb(bool connected);

	void override_cb(const mavros::OverrideRCIn::ConstPtr req);

};

}; // namespace mavplugin

