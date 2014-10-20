/**
 * @brief Parameter plugin
 * @file param_plugin.h
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

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <boost/any.hpp>

#include <mavros/ParamSet.h>
#include <mavros/ParamGet.h>
#include <mavros/ParamPull.h>
#include <mavros/ParamPush.h>

namespace mavplugin {
/**
 * @brief Parameter storage
 *
 * Stores parameter value.
 */
class Parameter {
public:
	typedef boost::any param_t;

	std::string param_id;
	param_t param_value;
	uint16_t param_index;
	uint16_t param_count;

	/**
	 * Convert mavlink_param_value_t to internal format
	 */
	static param_t from_param_value(mavlink_param_value_t &pmsg);
	static param_t from_param_value_apm_quirk(mavlink_param_value_t &pmsg);
	static std::string to_string_vt(param_t p);
	static mavlink_param_union_t to_param_union(param_t p);
	static mavlink_param_union_t to_param_union_apm_quirk(param_t p);
	static int64_t to_integer(param_t &p);
	static double to_real(param_t &p);
  static XmlRpc::XmlRpcValue to_xmlrpc_value(param_t &p);
	static param_t from_xmlrpc_value(XmlRpc::XmlRpcValue &xml);
	static bool check_exclude_param_id(std::string param_id);
};

/**
 * @brief Parameter set transaction data
 */
class ParamSetOpt {
public:
	Parameter param;
	size_t retries_remaining;
	bool is_timedout;
	std::mutex cond_mutex;
	std::condition_variable ack;

	ParamSetOpt(Parameter &_p, size_t _rem);
};

/**
 * @brief Parameter manipulation plugin
 */
class ParamPlugin : public MavRosPlugin {
public:
	ParamPlugin();

	void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater);
	std::string const get_name() const;
	const message_map get_rx_handlers();

private:
	std::recursive_mutex mutex;
	UAS *uas;

	ros::NodeHandle param_nh;
	ros::ServiceServer pull_srv;
	ros::ServiceServer push_srv;
	ros::ServiceServer set_srv;
	ros::ServiceServer get_srv;

	ros::Timer shedule_timer;			//!< for startup shedule fetch
	ros::Timer timeout_timer;			//!< for timeout resend

	static constexpr int BOOTUP_TIME_MS = 10000;	//!< APM boot time
	static constexpr int PARAM_TIMEOUT_MS = 1000;	//!< Param wait time
	static constexpr int LIST_TIMEOUT_MS = 30000;	//!< Receive all time
	static constexpr int RETRIES_COUNT = 3;

	const ros::Duration BOOTUP_TIME_DT;
	const ros::Duration LIST_TIMEOUT_DT;
	const ros::Duration PARAM_TIMEOUT_DT;

	std::map<std::string, Parameter> parameters;
	std::list<uint16_t> parameters_missing_idx;
	std::map<std::string, ParamSetOpt*> set_parameters;
	ssize_t param_count;
	enum {
		PR_IDLE,
		PR_RXLIST,
		PR_RXPARAM,
		PR_TXPARAM
	} param_state;

	size_t param_rx_retries;
	bool is_timedout;
	std::mutex list_cond_mutex;
	std::condition_variable list_receiving;

	inline Parameter::param_t from_param_value(mavlink_param_value_t &msg) {
		if (uas->is_ardupilotmega())
			return Parameter::from_param_value_apm_quirk(msg);
		else
			return Parameter::from_param_value(msg);
	}

	inline mavlink_param_union_t to_param_union(Parameter::param_t p) {
		if (uas->is_ardupilotmega())
			return Parameter::to_param_union_apm_quirk(p);
		else
			return Parameter::to_param_union(p);
	}

	void handle_param_value(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);
	void param_request_list();
	void param_request_read(std::string id, int16_t index=-1);
	void param_set(Parameter &param);
	void connection_cb(bool connected);
	void shedule_pull(const ros::Duration &dt);
	void shedule_cb(const ros::TimerEvent &event);
	void timeout_cb(const ros::TimerEvent &event);
	void restart_timeout_timer();
	void go_idle();
	bool wait_fetch_all();
	bool wait_param_set_ack_for(ParamSetOpt *opt);
	bool send_param_set_and_wait(Parameter &param);
	bool pull_cb(mavros::ParamPull::Request &req, mavros::ParamPull::Response &res);
	bool push_cb(mavros::ParamPush::Request &req, mavros::ParamPush::Response &res);
	bool set_cb(mavros::ParamSet::Request &req, mavros::ParamSet::Response &res);
	bool get_cb(mavros::ParamGet::Request &req, mavros::ParamGet::Response &res);
};

}; // namespace
