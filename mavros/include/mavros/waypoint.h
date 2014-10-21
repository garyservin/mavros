/**
 * @brief Waypoint plugin
 * @file waypoint.cpp
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

#ifndef MAVPLUGIN_WAYPOINT_H__
#define MAVPLUGIN_WAYPOINT_H__

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>

#include <mavros/WaypointList.h>
#include <mavros/WaypointSetCurrent.h>
#include <mavros/WaypointClear.h>
#include <mavros/WaypointPull.h>
#include <mavros/WaypointPush.h>
#include <mavros/WaypointGOTO.h>

namespace mavplugin {

class WaypointItem {
public:
	uint16_t seq;
	enum MAV_FRAME frame;
	enum MAV_CMD command;
	uint8_t current; /* APM use some magical numbers */
	bool autocontinue;
	float param1;
	float param2;
	float param3;
	float param4;
	double x_lat;
	double y_long;
	double z_alt;

	static mavros::Waypoint to_msg(WaypointItem &wp);
	static WaypointItem from_msg(mavros::Waypoint &wp, uint16_t seq);
	static WaypointItem from_mission_item(mavlink_mission_item_t &mit);
	static std::string to_string_frame(WaypointItem &wpi);
	static std::string to_string_command(WaypointItem &wpi);
};

/**
 * @brief Mission manupulation plugin
 */
class WaypointPlugin : public MavRosPlugin {
public:
	WaypointPlugin();
	void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater);
	std::string const get_name() const;
	const message_map get_rx_handlers();

private:
	std::recursive_mutex mutex;
	UAS *uas;

	ros::NodeHandle wp_nh;
	ros::Publisher wp_list_pub;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer clear_srv;
	ros::ServiceServer set_cur_srv;
	ros::ServiceServer goto_srv;

	std::vector<WaypointItem> waypoints;
	std::vector<WaypointItem> send_waypoints;
	enum {
		WP_IDLE,
		WP_RXLIST,
		WP_RXWP,
		WP_TXLIST,
		WP_TXWP,
		WP_CLEAR,
		WP_SET_CUR
	} wp_state;

	size_t wp_count;
	size_t wp_cur_id;
	size_t wp_cur_active;
	size_t wp_set_active;
	size_t wp_retries;
	bool is_timedout;
	std::mutex recv_cond_mutex;
	std::mutex send_cond_mutex;
	std::condition_variable list_receiving;
	std::condition_variable list_sending;

	ros::Timer wp_timer;
	ros::Timer shedule_timer;
	bool do_pull_after_gcs;
	bool reshedule_pull;

	static constexpr int BOOTUP_TIME_MS = 15000;	//! system startup delay before start pull
	static constexpr int LIST_TIMEOUT_MS = 30000;	//! Timeout for pull/push operations
	static constexpr int WP_TIMEOUT_MS = 1000;
	static constexpr int RESHEDULE_MS = 5000;
	static constexpr int RETRIES_COUNT = 3;

	const ros::Duration BOOTUP_TIME_DT;
	const ros::Duration LIST_TIMEOUT_DT;
	const ros::Duration WP_TIMEOUT_DT;
	const ros::Duration RESHEDULE_DT;

	void handle_mission_item(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void handle_mission_request(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void handle_mission_current(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void handle_mission_count(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void handle_mission_item_reached(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void handle_mission_ack(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void timeout_cb(const ros::TimerEvent &event);
	void connection_cb(bool connected);
	void sheduled_pull_cb(const ros::TimerEvent &event);
	void request_mission_done(void);
	void go_idle(void);
	void restart_timeout_timer(void);
  void restart_timeout_timer_int(void);
	void shedule_pull(const ros::Duration &dt);
	void send_waypoint(size_t seq);
	bool wait_fetch_all();
	bool wait_push_all();
	void set_current_waypoint(size_t seq);
	void publish_waypoints();
	void mission_item(WaypointItem &wp);
	void mission_request(uint16_t seq);
	void mission_set_current(uint16_t seq);
	void mission_request_list();
	void mission_count(uint16_t cnt);
	void mission_clear_all();
	void mission_ack(enum MAV_MISSION_RESULT type);
	bool pull_cb(mavros::WaypointPull::Request &req, mavros::WaypointPull::Response &res);
	bool push_cb(mavros::WaypointPush::Request &req, mavros::WaypointPush::Response &res);
	bool clear_cb(mavros::WaypointClear::Request &req, mavros::WaypointClear::Response &res);
	bool set_cur_cb(mavros::WaypointSetCurrent::Request &req, mavros::WaypointSetCurrent::Response &res);
	bool goto_cb(mavros::WaypointGOTO::Request &req, mavros::WaypointGOTO::Response &res);

};

}; //namespace
#endif
