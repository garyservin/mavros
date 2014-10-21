/**
 * @brief GPS publish plugin
 * @file gps.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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
#ifndef MAVPLUGIN_GPS_H__
#define MAVPLUGIN_GPS_H__

#include <angles/angles.h>
#include <mavros/mavros_plugin.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>

#ifdef __ANDROID__
  #define UINT16_MAX 65535
#endif

namespace mavplugin {

class GPSInfo : public diagnostic_updater::DiagnosticTask
{
public:
	explicit GPSInfo(const std::string name);
	void set_gps_raw(mavlink_gps_raw_int_t &gps);
	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
	std::atomic<int> satellites_visible;
	std::atomic<int> fix_type;
	std::atomic<uint16_t> eph;
	std::atomic<uint16_t> epv;
};


/**
 * @brief GPS plugin
 *
 * This plugin implements same ROS topics as nmea_navsat_driver package.
 */
class GPSPlugin : public MavRosPlugin {
public:
	GPSPlugin();
	void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater);
	std::string const get_name() const;
	const message_map get_rx_handlers();

private:
	UAS *uas;
	std::string frame_id;
	std::string time_ref_source;

	GPSInfo gps_diag;

	ros::Publisher fix_pub;
	ros::Publisher time_ref_pub;
	ros::Publisher vel_pub;

	void handle_gps_raw_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void handle_gps_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
};

}; // namespace navplugin

#endif
