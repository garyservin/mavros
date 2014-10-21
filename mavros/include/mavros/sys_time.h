/**
 * @brief System Time plugin
 * @file sys_time.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
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

#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Duration.h>

namespace mavplugin {

/**
 * Time syncronization status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class TimeSyncStatus : public diagnostic_updater::DiagnosticTask
{
public:
	TimeSyncStatus(const std::string name, size_t win_size);

	void clear();

	void tick(int64_t dt, uint64_t timestamp_us);

	void set_timestamp(uint64_t timestamp_us);

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
	int64_t last_dt;
	int64_t dt_sum;
	uint64_t last_ts;
};



class SystemTimePlugin : public MavRosPlugin {
public:
	SystemTimePlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);


	std::string const get_name() const;

	const message_map get_rx_handlers();

private:
	UAS *uas;
	ros::Publisher time_ref_pub;
	ros::Publisher time_offset_pub;
	ros::Timer sys_time_timer;
	TimeSyncStatus dt_diag;

	std::string frame_id;
	std::string time_ref_source;
	uint64_t time_offset_us;

	void handle_system_time(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void sys_time_cb(const ros::TimerEvent &event);

};

}; // namespace mavplugin

