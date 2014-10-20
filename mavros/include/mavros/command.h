/**
 * @brief Command plugin
 * @file command.h
 * @author Gary Servin <gary@creativa77.com>
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
#ifndef MAVPLUGIN_COMMAND_H__
#define MAVPLUGIN_COMMAND_H__

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros/CommandLong.h>
#include <mavros/CommandInt.h>
#include <mavros/CommandBool.h>
#include <mavros/CommandHome.h>
#include <mavros/CommandTOL.h>

namespace mavplugin {

class CommandTransaction {
public:
	std::mutex cond_mutex;
	std::condition_variable ack;
	uint16_t expected_command;
	uint8_t result;

	explicit CommandTransaction(uint16_t command) :
		ack(),
		expected_command(command),
		result(MAV_RESULT_FAILED)
	{ }
};

/**
 * @brief Command plugin.
 *
 * Send any command via COMMAND_LONG
 */
class CommandPlugin : public MavRosPlugin {
public:
	CommandPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	std::string const get_name() const;

	const message_map get_rx_handlers();

private:
	std::recursive_mutex mutex;
	UAS *uas;

	ros::NodeHandle cmd_nh;
	ros::ServiceServer command_long_srv;
	ros::ServiceServer command_int_srv;
	ros::ServiceServer arming_srv;
	ros::ServiceServer set_home_srv;
	ros::ServiceServer takeoff_srv;
	ros::ServiceServer land_srv;
	ros::ServiceServer guided_srv;

	std::list<CommandTransaction *> ack_waiting_list;
	static constexpr int ACK_TIMEOUT_MS = 5000;

	const ros::Duration ACK_TIMEOUT_DT;

	/* -*- message handlers -*- */

	void handle_command_ack(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	/* -*- mid-level functions -*- */

	bool wait_ack_for(CommandTransaction *tr);

	/**
	 * Common function for command service callbacks.
	 *
	 * NOTE: success is bool in messages, but has unsigned char type in C++
	 */
	bool send_command_long_and_wait(uint16_t command, uint8_t confirmation,
			float param1, float param2,
			float param3, float param4,
			float param5, float param6,
			float param7,
			unsigned char &success, uint8_t &result);

	/**
	 * Common function for COMMAND_INT service callbacks.
	 */
	bool send_command_int(uint8_t frame, uint16_t command,
			uint8_t current, uint8_t autocontinue,
			float param1, float param2,
			float param3, float param4,
			int32_t x, int32_t y,
			float z,
			unsigned char &success);

	/* -*- low-level send -*- */

	void command_long(uint16_t command, uint8_t confirmation,
			float param1, float param2,
			float param3, float param4,
			float param5, float param6,
			float param7);

	void command_int(uint8_t frame, uint16_t command,
			uint8_t current, uint8_t autocontinue,
			float param1, float param2,
			float param3, float param4,
			int32_t x, int32_t y,
			float z);

	/* -*- callbacks -*- */

	bool command_long_cb(mavros::CommandLong::Request &req,
			mavros::CommandLong::Response &res);

	bool command_int_cb(mavros::CommandInt::Request &req,
			mavros::CommandInt::Response &res);

	bool arming_cb(mavros::CommandBool::Request &req,
			mavros::CommandBool::Response &res);

	bool set_home_cb(mavros::CommandHome::Request &req,
			mavros::CommandHome::Response &res);

	bool takeoff_cb(mavros::CommandTOL::Request &req,
			mavros::CommandTOL::Response &res);

	bool land_cb(mavros::CommandTOL::Request &req,
			mavros::CommandTOL::Response &res);

	bool guided_cb(mavros::CommandBool::Request &req,
			mavros::CommandBool::Response &res);

};

}; // namespace mavplugin

#endif
