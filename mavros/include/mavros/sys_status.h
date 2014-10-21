/**
 * @brief System Status plugin
 * @file sys_status.cpp
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

#include <mavros/mavros_plugin.h>

#include <mavros/State.h>
#include <mavros/BatteryStatus.h>
#include <mavros/StreamRate.h>
#include <mavros/SetMode.h>

namespace mavplugin {

/**
 * Heartbeat status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class HeartbeatStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HeartbeatStatus(const std::string name, size_t win_size);

	void clear();

	void tick(mavlink_heartbeat_t &hb_struct);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	std::recursive_mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;
	mavlink_heartbeat_t last_hb;
};


class SystemStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	SystemStatusDiag(const std::string name);

	void set(mavlink_sys_status_t &st);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
	std::recursive_mutex mutex;
	mavlink_sys_status_t last_st;
};


class BatteryStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	BatteryStatusDiag(const std::string name);

	void set_min_voltage(float volt);

	void set(float volt, float curr, float rem);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
	std::recursive_mutex mutex;
	float voltage;
	float current;
	float remaining;
	float min_voltage;
};


class MemInfo : public diagnostic_updater::DiagnosticTask
{
public:
	MemInfo(const std::string name);

	void set(uint16_t f, uint16_t b);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
	std::recursive_mutex mutex;
	ssize_t freemem;
	uint16_t brkval;
};


class HwStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HwStatus(const std::string name);

	void set(uint16_t v, uint8_t e);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
	std::recursive_mutex mutex;
	float vcc;
	size_t i2cerr;
	size_t i2cerr_last;
};


/**
 * @brief System status plugin.
 * Required for most applications.
 */
class SystemStatusPlugin : public MavRosPlugin
{
public:
	SystemStatusPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	const std::string get_name() const;

	const message_map get_rx_handlers();

private:
	UAS *uas;
	HeartbeatStatus hb_diag;
	MemInfo mem_diag;
	HwStatus hwst_diag;
	SystemStatusDiag sys_diag;
	BatteryStatusDiag batt_diag;
	ros::Timer timeout_timer;
	ros::Timer heartbeat_timer;

	ros::Publisher state_pub;
	ros::Publisher batt_pub;
	ros::ServiceServer rate_srv;
	ros::ServiceServer mode_srv;

	/* -*- mid-level helpers -*- */

	/**
	 * Sent STATUSTEXT message to rosout
	 *
	 * @param[in] severity  Levels defined in common.xml
	 */
	void process_statustext_normal(uint8_t severity, std::string &text);

	/**
	 * Send STATUSTEXT messate to rosout with APM severity levels
	 *
	 * @param[in] severity  APM levels.
	 */
	void process_statustext_apm_quirk(uint8_t severity, std::string &text);

	/* -*- message handlers -*- */

	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_sys_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_statustext(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

#ifdef MAVLINK_MSG_ID_MEMINFO
	void handle_meminfo(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
#endif

#ifdef MAVLINK_MSG_ID_HWSTATUS
	void handle_hwstatus(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
#endif

	/* -*- timer callbacks -*- */

	void timeout_cb(const ros::TimerEvent &event);

	void heartbeat_cb(const ros::TimerEvent &event);

	/* -*- ros callbacks -*- */

	bool set_rate_cb(mavros::StreamRate::Request &req,
			mavros::StreamRate::Response &res);

	bool set_mode_cb(mavros::SetMode::Request &req,
			mavros::SetMode::Response &res);

};

}; // namespace mavplugin

