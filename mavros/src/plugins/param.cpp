/**
 * @brief Parameter plugin
 * @file param.cpp
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

#include <mavros/param_plugin.h>

namespace mavplugin {

Parameter::param_t Parameter::from_param_value(mavlink_param_value_t &pmsg) {
	mavlink_param_union_t uv;
	uv.param_float = pmsg.param_value;

	switch (pmsg.param_type) {
	case MAV_PARAM_TYPE_UINT8:
		return uv.param_uint8;
	case MAV_PARAM_TYPE_INT8:
		return uv.param_int8;
	case MAV_PARAM_TYPE_UINT16:
		return uv.param_uint16;
	case MAV_PARAM_TYPE_INT16:
		return uv.param_int16;
	case MAV_PARAM_TYPE_UINT32:
		return uv.param_uint32;
	case MAV_PARAM_TYPE_INT32:
		return uv.param_int32;
	case MAV_PARAM_TYPE_REAL32:
		return uv.param_float;

	default:
	case MAV_PARAM_TYPE_UINT64:
	case MAV_PARAM_TYPE_INT64:
	case MAV_PARAM_TYPE_REAL64:
		ROS_WARN_NAMED("param", "Unsupported param '%.16s' type: %d, index: %d of %d",
				pmsg.param_id, pmsg.param_type,
				pmsg.param_index, pmsg.param_count);
		return Parameter::param_t();
	};
}

/**
 * Variation of @a Parameter::from_param_value with quirks for ArduPilotMega
 */
Parameter::param_t Parameter::from_param_value_apm_quirk(mavlink_param_value_t &pmsg) {
	switch (pmsg.param_type) {
	case MAV_PARAM_TYPE_UINT8:
		return (uint8_t) pmsg.param_value;
	case MAV_PARAM_TYPE_INT8:
		return (int8_t) pmsg.param_value;
	case MAV_PARAM_TYPE_UINT16:
		return (uint16_t) pmsg.param_value;
	case MAV_PARAM_TYPE_INT16:
		return (int16_t) pmsg.param_value;
	case MAV_PARAM_TYPE_UINT32:
		return (uint32_t) pmsg.param_value;
	case MAV_PARAM_TYPE_INT32:
		return (int32_t) pmsg.param_value;
	case MAV_PARAM_TYPE_REAL32:
		return pmsg.param_value;

	default:
	case MAV_PARAM_TYPE_UINT64:
	case MAV_PARAM_TYPE_INT64:
	case MAV_PARAM_TYPE_REAL64:
		ROS_WARN_NAMED("param", "Unsupported param '%.16s' type: %d, index: %d of %d",
				pmsg.param_id, pmsg.param_type,
				pmsg.param_index, pmsg.param_count);
		return Parameter::param_t();
	};
}

/**
 * Convert internal type to std::string (for debugging)
 */
std::string Parameter::to_string_vt(Parameter::param_t p) {
	std::ostringstream sout;

	if (p.type() == typeid(uint8_t))
		sout << (unsigned) boost::any_cast<uint8_t>(p) << " ubyte";
	else if (p.type() == typeid(int8_t))
		sout << (int) boost::any_cast<int8_t>(p) << " byte";
	else if (p.type() == typeid(uint16_t))
		sout << boost::any_cast<uint16_t>(p) << " ushort";
	else if (p.type() == typeid(int16_t))
		sout << boost::any_cast<int16_t>(p) << " short";
	else if (p.type() == typeid(uint32_t))
		sout << boost::any_cast<uint32_t>(p) << " uint";
	else if (p.type() == typeid(int32_t))
		sout << boost::any_cast<int32_t>(p) << " int";
	else if (p.type() == typeid(float))
		sout << boost::any_cast<float>(p) << " float";
	else {
		ROS_FATAL_STREAM_NAMED("param", "Wrong param_t type: " << p.type().name());
		sout << "UNK " << p.type().name();
	}

	return sout.str();
};

/**
 * Convert internal type to mavlink_param_union_t
 */
mavlink_param_union_t Parameter::to_param_union(Parameter::param_t p) {
	mavlink_param_union_t ret;

	if (p.type() == typeid(uint8_t)) {
		ret.param_uint8 = boost::any_cast<uint8_t>(p);
		ret.type = MAV_PARAM_TYPE_UINT8;
	}
	else if (p.type() == typeid(int8_t)) {
		ret.param_int8 = boost::any_cast<int8_t>(p);
		ret.type = MAV_PARAM_TYPE_INT8;
	}
	else if (p.type() == typeid(uint16_t)) {
		ret.param_uint16 = boost::any_cast<uint16_t>(p);
		ret.type = MAV_PARAM_TYPE_UINT16;
	}
	else if (p.type() == typeid(int16_t)){
		ret.param_int16 = boost::any_cast<int16_t>(p);
		ret.type = MAV_PARAM_TYPE_INT16;
	}
	else if (p.type() == typeid(uint32_t)) {
		ret.param_uint32 = boost::any_cast<uint32_t>(p);
		ret.type = MAV_PARAM_TYPE_UINT32;
	}
	else if (p.type() == typeid(int32_t)) {
		ret.param_int32 = boost::any_cast<int32_t>(p);
		ret.type = MAV_PARAM_TYPE_INT32;
	}
	else if (p.type() == typeid(float)) {
		ret.param_float = boost::any_cast<float>(p);
		ret.type = MAV_PARAM_TYPE_REAL32;
	}
	else {
		ROS_FATAL_STREAM_NAMED("param", "Wrong param_t type: " << p.type().name());
		ret.param_float = 0.0;
		ret.type = 255;
	}

	return ret;
};

/**
 * Variation of @a Parameter::to_param_union with quirks for ArduPilotMega
 */
mavlink_param_union_t Parameter::to_param_union_apm_quirk(Parameter::param_t p) {
	mavlink_param_union_t ret;

	if (p.type() == typeid(uint8_t)) {
		ret.param_float = boost::any_cast<uint8_t>(p);
		ret.type = MAV_PARAM_TYPE_UINT8;
	}
	else if (p.type() == typeid(int8_t)) {
		ret.param_float = boost::any_cast<int8_t>(p);
		ret.type = MAV_PARAM_TYPE_INT8;
	}
	else if (p.type() == typeid(uint16_t)) {
		ret.param_float = boost::any_cast<uint16_t>(p);
		ret.type = MAV_PARAM_TYPE_UINT16;
	}
	else if (p.type() == typeid(int16_t)){
		ret.param_float = boost::any_cast<int16_t>(p);
		ret.type = MAV_PARAM_TYPE_INT16;
	}
	else if (p.type() == typeid(uint32_t)) {
		ret.param_float = boost::any_cast<uint32_t>(p);
		ret.type = MAV_PARAM_TYPE_UINT32;
	}
	else if (p.type() == typeid(int32_t)) {
		ret.param_float = boost::any_cast<int32_t>(p);
		ret.type = MAV_PARAM_TYPE_INT32;
	}
	else if (p.type() == typeid(float)) {
		ret.param_float = boost::any_cast<float>(p);
		ret.type = MAV_PARAM_TYPE_REAL32;
	}
	else {
		ROS_FATAL_STREAM_NAMED("param", "Wrong param_t type: " << p.type().name());
		ret.param_float = 0.0;
		ret.type = 255;
	}

	return ret;
};

/**
 * For get/set services
 */
int64_t Parameter::to_integer(Parameter::param_t &p) {
	if (p.type() == typeid(uint8_t))
		return boost::any_cast<uint8_t>(p);
	else if (p.type() == typeid(int8_t))
		return boost::any_cast<int8_t>(p);
	else if (p.type() == typeid(uint16_t))
		return boost::any_cast<uint16_t>(p);
	else if (p.type() == typeid(int16_t))
		return boost::any_cast<int16_t>(p);
	else if (p.type() == typeid(uint32_t))
		return boost::any_cast<uint32_t>(p);
	else if (p.type() == typeid(int32_t))
		return boost::any_cast<int32_t>(p);
	else
		return 0;
};

double Parameter::to_real(Parameter::param_t &p) {
	if (p.type() == typeid(float))
		return boost::any_cast<float>(p);
	else
		return 0.0;
};

/**
 * Convert internal value to rosparam XmlRpcValue
 */
XmlRpc::XmlRpcValue Parameter::to_xmlrpc_value(Parameter::param_t &p) {
	if (p.type() == typeid(uint8_t))
		return (int) boost::any_cast<uint8_t>(p);
	else if (p.type() == typeid(int8_t))
		return (int) boost::any_cast<int8_t>(p);
	else if (p.type() == typeid(uint16_t))
		return (int) boost::any_cast<uint16_t>(p);
	else if (p.type() == typeid(int16_t))
		return (int) boost::any_cast<int16_t>(p);
	else if (p.type() == typeid(uint32_t))
		return (int) boost::any_cast<uint32_t>(p);
	else if (p.type() == typeid(int32_t))
		return (int) boost::any_cast<int32_t>(p);
	else if (p.type() == typeid(float))
		return (double) boost::any_cast<float>(p);
	else {
		ROS_FATAL_STREAM_NAMED("param", "Wrong param_t type: " << p.type().name());
		return XmlRpc::XmlRpcValue();
	}
}

/**
 * Convert rosparam to internal value
 */
Parameter::param_t Parameter::from_xmlrpc_value(XmlRpc::XmlRpcValue &xml) {
	switch (xml.getType()) {
	case XmlRpc::XmlRpcValue::TypeBoolean:
		return (uint8_t) static_cast<bool>(xml);
	case XmlRpc::XmlRpcValue::TypeInt:
		return static_cast<int32_t>(xml);
	case XmlRpc::XmlRpcValue::TypeDouble:
		return (float) static_cast<double>(xml);

	default:
		ROS_FATAL_NAMED("param", "Unsupported XmlRpcValye type: %d", xml.getType());
		return Parameter::param_t();
	};
}

/**
 * Exclude this parameters from ~param/push
 */
bool Parameter::check_exclude_param_id(std::string param_id) {
	return	param_id == "SYSID_SW_MREV"	||
		param_id == "SYS_NUM_RESETS"	||
		param_id == "ARSPD_OFFSET"	||
		param_id == "GND_ABS_PRESS"	||
		param_id == "GND_TEMP"		||
		param_id == "CMD_TOTAL"		||
		param_id == "CMD_INDEX"		||
		param_id == "LOG_LASTFILE"	||
		param_id == "FENCE_TOTAL"	||
		param_id == "FORMAT_VERSION";
}

/**
 * @brief Parameter set transaction data
 */
ParamSetOpt::ParamSetOpt(Parameter &_p, size_t _rem) :
		param(_p),
		retries_remaining(_rem),
		is_timedout(false)
	{ };

/**
 * @brief Parameter manipulation plugin
 */

ParamPlugin::ParamPlugin() :
	uas(nullptr),
	param_count(-1),
	param_state(PR_IDLE),
	is_timedout(false),
	param_rx_retries(RETRIES_COUNT),
	BOOTUP_TIME_DT(BOOTUP_TIME_MS / 1000.0),
	LIST_TIMEOUT_DT(LIST_TIMEOUT_MS / 1000.0),
	PARAM_TIMEOUT_DT(PARAM_TIMEOUT_MS / 1000.0)
{ };

void ParamPlugin::initialize(UAS &uas_,
		ros::NodeHandle &nh,
		diagnostic_updater::Updater &diag_updater)
{
	uas = &uas_;
	param_nh = ros::NodeHandle(nh, "param");

	pull_srv = param_nh.advertiseService("pull", &ParamPlugin::pull_cb, this);
	push_srv = param_nh.advertiseService("push", &ParamPlugin::push_cb, this);
	set_srv = param_nh.advertiseService("set", &ParamPlugin::set_cb, this);
	get_srv = param_nh.advertiseService("get", &ParamPlugin::get_cb, this);

	shedule_timer = param_nh.createTimer(BOOTUP_TIME_DT, &ParamPlugin::shedule_cb, this, true);
	shedule_timer.stop();
	timeout_timer = param_nh.createTimer(PARAM_TIMEOUT_DT, &ParamPlugin::timeout_cb, this, true);
	timeout_timer.stop();
	uas->sig_connection_changed.connect(boost::bind(&ParamPlugin::connection_cb, this, _1));
}

std::string const ParamPlugin::get_name() const {
	return "Param";
}

const MavRosPlugin::message_map ParamPlugin::get_rx_handlers() {
	return {
		MESSAGE_HANDLER(MAVLINK_MSG_ID_PARAM_VALUE, &ParamPlugin::handle_param_value)
	};
}

/* -*- message handlers -*- */

void ParamPlugin::handle_param_value(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	mavlink_param_value_t pmsg;
	mavlink_msg_param_value_decode(msg, &pmsg);

	std::string param_id(pmsg.param_id,
			strnlen(pmsg.param_id, sizeof(pmsg.param_id)));

	lock_guard lock(mutex);
	// search
	auto param_it = parameters.find(param_id);
	if (param_it != parameters.end()) {
		// parameter exists
		Parameter *p = &param_it->second;
		p->param_value = from_param_value(pmsg);

		// check that ack required
		auto set_it = set_parameters.find(param_id);
		if (set_it != set_parameters.end()) {
			set_it->second->ack.notify_all();
		}

		ROS_WARN_STREAM_COND_NAMED(((p->param_index != pmsg.param_index &&
					    pmsg.param_index != UINT16_MAX) ||
					p->param_count != pmsg.param_count),
				"param",
				"PR: Param " << param_id << " index(" << p->param_index <<
				"->" << pmsg.param_index << ")/count(" << p->param_count <<
				"->" << pmsg.param_count << ") changed! FCU changed?");
		ROS_DEBUG_STREAM_NAMED("param", "PR: Update param " << param_id <<
				" (" << p->param_index << "/" << p->param_count <<
				") value: " << Parameter::to_string_vt(p->param_value));
	}
	else {
		// insert new element
		Parameter p;
		p.param_id = param_id;
		p.param_index = pmsg.param_index;
		p.param_count = pmsg.param_count;
		p.param_value = from_param_value(pmsg);

		parameters[param_id] = p;

		ROS_DEBUG_STREAM_NAMED("param", "PR: New param " << param_id <<
				" (" << p.param_index << "/" << p.param_count <<
				") value: " << Parameter::to_string_vt(p.param_value));
	}

	if (param_state == PR_RXLIST || param_state == PR_RXPARAM) {
		// we received first param. setup list timeout
		if (param_state == PR_RXLIST) {
			param_count = pmsg.param_count;
			param_state = PR_RXPARAM;

			parameters_missing_idx.clear();
			if (param_count != UINT16_MAX) {
				ROS_DEBUG_NAMED("param", "PR: waiting %zu parameters", param_count);
				// declare that all parameters are missing
				for (uint16_t idx = 0; idx < param_count; idx++)
					parameters_missing_idx.push_back(idx);
			}
			else
				ROS_WARN_NAMED("param", "PR: FCU does not know index for first element! "
						"Param list may be truncated.");
		}

		// remove idx for that message
		parameters_missing_idx.remove(pmsg.param_index);

		// in receiving mode we use param_rx_retries for LIST and PARAM
		param_rx_retries = RETRIES_COUNT;
		restart_timeout_timer();

		/* index starting from 0, receivig done */
		if (parameters_missing_idx.empty()) {
			ssize_t missed = param_count - parameters.size();
			ROS_INFO_COND_NAMED(missed == 0, "param", "PR: parameters list received");
			ROS_WARN_COND_NAMED(missed > 0, "param",
					"PR: parameters list received, but %zd parametars are missed",
					missed);
			go_idle();
			list_receiving.notify_all();
		}
	}
}

/* -*- low-level send function -*- */

void ParamPlugin::param_request_list() {
	mavlink_message_t msg;

	ROS_DEBUG_NAMED("param", "PR:m: request list");
	mavlink_msg_param_request_list_pack_chan(UAS_PACK_CHAN(uas), &msg,
			UAS_PACK_TGT(uas)
			);
	UAS_FCU(uas)->send_message(&msg);
}

void ParamPlugin::param_request_read(std::string id, int16_t index) {
	ROS_ASSERT(index >= -1);

	mavlink_message_t msg;
	char param_id[sizeof(mavlink_param_request_read_t::param_id)];

	ROS_DEBUG_NAMED("param", "PR:m: request '%s', idx %d", id.c_str(), index);
	if (index != -1) {
		// by specs if len < 16: place null termination
		// else if len == 16: don't
		strncpy(param_id, id.c_str(), sizeof(param_id));
	}
	else
		param_id[0] = '\0'; // force NULL termination

	mavlink_msg_param_request_read_pack_chan(UAS_PACK_CHAN(uas), &msg,
			UAS_PACK_TGT(uas),
			param_id,
			index
			);
	UAS_FCU(uas)->send_message(&msg);
}

void ParamPlugin::param_set(Parameter &param) {
	mavlink_param_union_t pu = to_param_union(param.param_value);

	mavlink_message_t msg;
	char param_id[sizeof(mavlink_param_set_t::param_id)];
	strncpy(param_id, param.param_id.c_str(), sizeof(param_id));

	ROS_DEBUG_STREAM_NAMED("param", "PR:m: set param " << param.param_id <<
			" (" << param.param_index << "/" << param.param_count <<
			") value: " << Parameter::to_string_vt(param.param_value));
	mavlink_msg_param_set_pack_chan(UAS_PACK_CHAN(uas), &msg,
			UAS_PACK_TGT(uas),
			param_id,
			pu.param_float,
			pu.type
			);
	UAS_FCU(uas)->send_message(&msg);
}

/* -*- mid-level functions -*- */

void ParamPlugin::connection_cb(bool connected) {
	lock_guard lock(mutex);
	if (connected) {
		shedule_pull(BOOTUP_TIME_DT);
	}
	else {
		shedule_timer.stop();
	}
}

void ParamPlugin::shedule_pull(const ros::Duration &dt) {
	shedule_timer.stop();
	shedule_timer.setPeriod(dt);
	shedule_timer.start();
}

void ParamPlugin::shedule_cb(const ros::TimerEvent &event) {
	lock_guard lock(mutex);
	if (param_state != PR_IDLE) {
		// try later
		ROS_DEBUG_NAMED("param", "PR: busy, reshedule pull");
		shedule_pull(BOOTUP_TIME_DT);
	}

	ROS_DEBUG_NAMED("param", "PR: start sheduled pull");
	param_state = PR_RXLIST;
	param_rx_retries = RETRIES_COUNT;
	parameters.clear();

	restart_timeout_timer();
	param_request_list();
}

void ParamPlugin::timeout_cb(const ros::TimerEvent &event) {
	lock_guard lock(mutex);
	if (param_state == PR_RXLIST && param_rx_retries > 0) {
		param_rx_retries--;
		ROS_WARN_NAMED("param", "PR: request list timeout, retries left %zu", param_rx_retries);

		restart_timeout_timer();
		param_request_list();
	}
	else if (param_state == PR_RXPARAM) {
		if (parameters_missing_idx.empty()) {
			ROS_WARN_NAMED("param", "PR: missing list is clear, but we in RXPARAM state, "
					"maybe last rerequest fails. Params missed: %zd",
					param_count - parameters.size());
			go_idle();
			list_receiving.notify_all();
			return;
		}

		uint16_t first_miss_idx = parameters_missing_idx.front();
		if (param_rx_retries > 0) {
			param_rx_retries--;
			ROS_WARN_NAMED("param", "PR: request param #%u timeout, retries left %zu, and %zu params still missing",
					first_miss_idx, param_rx_retries, parameters_missing_idx.size());
			restart_timeout_timer();
			param_request_read("", first_miss_idx);
		}
		else {
			ROS_ERROR_NAMED("param", "PR: request param #%u completely missing.", first_miss_idx);
			parameters_missing_idx.pop_front();
			restart_timeout_timer();
			if (!parameters_missing_idx.empty()) {
				param_rx_retries = RETRIES_COUNT;
				first_miss_idx = parameters_missing_idx.front();

				ROS_WARN_NAMED("param", "PR: %zu params still missing, trying to request next: #%u",
						parameters_missing_idx.size(), first_miss_idx);
				param_request_read("", first_miss_idx);
			}
		}
	}
	else if (param_state == PR_TXPARAM) {
		auto it = set_parameters.begin();
		if (it == set_parameters.end()) {
			ROS_DEBUG_NAMED("param", "PR: send list empty, but state TXPARAM");
			go_idle();
			return;
		}

		if (it->second->retries_remaining > 0) {
			it->second->retries_remaining--;
			ROS_WARN_NAMED("param", "PR: Resend param set for %s, retries left %zu",
					it->second->param.param_id.c_str(),
					it->second->retries_remaining);
			restart_timeout_timer();
			param_set(it->second->param);
		}
		else {
			ROS_ERROR_NAMED("param", "PR: Param set for %s timed out.",
					it->second->param.param_id.c_str());
			it->second->is_timedout = true;
			it->second->ack.notify_all();
		}
	}
	else {
		ROS_DEBUG_NAMED("param", "PR: timeout in IDLE!");
	}
}

void ParamPlugin::restart_timeout_timer() {
	is_timedout = false;
	timeout_timer.stop();
	timeout_timer.start();
}

void ParamPlugin::go_idle() {
	param_state = PR_IDLE;
	timeout_timer.stop();
}

bool ParamPlugin::wait_fetch_all() {
	std::unique_lock<std::mutex> lock(list_cond_mutex);

	return list_receiving.wait_for(lock, std::chrono::nanoseconds(LIST_TIMEOUT_DT.toNSec()))
		== std::cv_status::no_timeout
		&& !is_timedout;
}

bool ParamPlugin::wait_param_set_ack_for(ParamSetOpt *opt) {
	std::unique_lock<std::mutex> lock(opt->cond_mutex);

	return opt->ack.wait_for(lock, std::chrono::nanoseconds(PARAM_TIMEOUT_DT.toNSec()) * (RETRIES_COUNT + 2))
		== std::cv_status::no_timeout
		&& !opt->is_timedout;
}

bool ParamPlugin::send_param_set_and_wait(Parameter &param) {
	unique_lock lock(mutex);

	// add to waiting list
	set_parameters[param.param_id] = new ParamSetOpt(param, RETRIES_COUNT);

	auto it = set_parameters.find(param.param_id);
	if (it == set_parameters.end()) {
		ROS_ERROR_STREAM_NAMED("param", "ParamSetOpt not found for " << param.param_id);
		return false;
	}

	param_state = PR_TXPARAM;
	restart_timeout_timer();
	param_set(param);

	lock.unlock();
	bool is_not_timeout = wait_param_set_ack_for(it->second);
	lock.lock();

	// free opt data
	delete it->second;
	set_parameters.erase(it);

	go_idle();
	return is_not_timeout;
}

/* -*- ROS callbacks -*- */

/**
 * @brief fetches all parameters from device
 * @service ~param/pull
 */
bool ParamPlugin::pull_cb(mavros::ParamPull::Request &req,
		mavros::ParamPull::Response &res) {
	unique_lock lock(mutex);

	if ((param_state == PR_IDLE && parameters.empty())
			|| req.force_pull) {
		if (!req.force_pull)
			ROS_DEBUG_NAMED("param", "PR: start pull");
		else
			ROS_INFO_NAMED("param", "PR: start force pull");

		param_state = PR_RXLIST;
		param_rx_retries = RETRIES_COUNT;
		parameters.clear();

		shedule_timer.stop();
		restart_timeout_timer();
		param_request_list();

		lock.unlock();
		res.success = wait_fetch_all();
	}
	else if (param_state == PR_RXLIST || param_state == PR_RXPARAM) {
		lock.unlock();
		res.success = wait_fetch_all();
	}
	else {
		lock.unlock();
		res.success = true;
	}

	lock.lock();
	res.param_received = parameters.size();

	for (auto param_it = parameters.begin();
			param_it != parameters.end();
			param_it++) {
		Parameter *p = &param_it->second;
		auto pv = Parameter::to_xmlrpc_value(p->param_value);

		lock.unlock();
		param_nh.setParam(p->param_id, pv);
		lock.lock();
	}

	return true;
}

/**
 * @brief push all parameter value to device
 * @service ~param/push
 */
bool ParamPlugin::push_cb(mavros::ParamPush::Request &req,
		mavros::ParamPush::Response &res) {

	XmlRpc::XmlRpcValue param_dict;
	if (!param_nh.getParam("", param_dict))
		return true;

	ROS_ASSERT(param_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	int tx_count = 0;
	for (auto param = param_dict.begin();
			param != param_dict.end();
			param++) {
		if (Parameter::check_exclude_param_id(param->first)) {
			ROS_DEBUG_STREAM_NAMED("param", "PR: Exclude param: " << param->first);
			continue;
		}

		unique_lock lock(mutex);
		auto param_it = parameters.find(param->first);
		if (param_it != parameters.end()) {
			Parameter *p = &param_it->second;
			Parameter to_send = *p;

			to_send.param_value = Parameter::from_xmlrpc_value(param->second);

			lock.unlock();
			bool set_res = send_param_set_and_wait(to_send);
			lock.lock();

			if (set_res)
				tx_count++;
		}
		else {
			ROS_WARN_STREAM_NAMED("param", "PR: Unknown rosparam: " << param->first);
		}
	}

	res.success = true;
	res.param_transfered = tx_count;

	return true;
}

/**
 * @brief sets parameter value
 * @service ~param/set
 */
bool ParamPlugin::set_cb(mavros::ParamSet::Request &req,
		mavros::ParamSet::Response &res) {
	unique_lock lock(mutex);

	if (param_state == PR_RXLIST || param_state == PR_RXPARAM) {
		ROS_ERROR_NAMED("param", "PR: receiving not complete");
		return false;
	}

	auto param_it = parameters.find(req.param_id);
	if (param_it != parameters.end()) {
		Parameter *p = &param_it->second;
		Parameter to_send = *p;

		// according to ParamSet/Get description
		if (req.integer > 0)
			to_send.param_value = (uint32_t) req.integer;
		else if (req.integer < 0)
			to_send.param_value = (int32_t) req.integer;
		else if (req.real != 0.0)
			to_send.param_value = (float) req.real;
		else
			to_send.param_value = (uint32_t) 0;

		lock.unlock();
		res.success = send_param_set_and_wait(to_send);
		lock.lock();

		res.integer = Parameter::to_integer(p->param_value);
		res.real = Parameter::to_real(p->param_value);

		auto pv = Parameter::to_xmlrpc_value(p->param_value);
		lock.unlock();

		param_nh.setParam(p->param_id, pv);
	}
	else {
		ROS_ERROR_STREAM_NAMED("param", "PR: Unknown parameter to set: " << req.param_id);
		res.success = false;
	}

	return true;
}

/**
 * @brief get parameter
 * @service ~param/get
 */
bool ParamPlugin::get_cb(mavros::ParamGet::Request &req,
		mavros::ParamGet::Response &res) {
	lock_guard lock(mutex);

	auto param_it = parameters.find(req.param_id);
	if (param_it != parameters.end()) {
		Parameter *p = &param_it->second;

		res.success = true;
		res.integer = Parameter::to_integer(p->param_value);
		res.real = Parameter::to_real(p->param_value);
	}
	else {
		ROS_ERROR_STREAM_NAMED("param", "PR: Unknown parameter to get: " << req.param_id);
		res.success = false;
	}

	return true;
}

}; // namespace mavplugin


