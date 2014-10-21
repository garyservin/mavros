/**
 * @brief IMU publish plugin
 * @file imu_pub.h
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
#ifndef MAVPLUGIN_IMU_PUB_H__
#define MAVPLUGIN_IMU_PUB_H__

#include <cmath>
#include <mavros/mavros_plugin.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3.h>

namespace mavplugin {

/**
 * @brief IMU data publication plugin
 */
class IMUPubPlugin : public MavRosPlugin {
public:
	IMUPubPlugin();
	void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater);
	std::string const get_name() const;
	const message_map get_rx_handlers();

private:
	std::string frame_id;
	UAS *uas;

	ros::Publisher imu_pub;
	ros::Publisher imu_raw_pub;
	ros::Publisher magn_pub;
	ros::Publisher temp_pub;
	ros::Publisher press_pub;

	bool has_hr_imu;
	bool has_scaled_imu;
	bool has_att_quat;
	geometry_msgs::Vector3 linear_accel_vec;
	boost::array<double, 9> linear_acceleration_cov;
	boost::array<double, 9> angular_velocity_cov;
	boost::array<double, 9> orientation_cov;
	boost::array<double, 9> unk_orientation_cov;
	boost::array<double, 9> magnetic_cov;

	static constexpr double GAUSS_TO_TESLA = 1.0e-4;
	static constexpr double MILLIT_TO_TESLA = 1000.0;
	static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
	static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
	static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;

	/* -*- helpers -*- */
	void setup_covariance(boost::array<double, 9> &cov, double stdev);

	void uas_store_attitude(tf::Quaternion &orientation, geometry_msgs::Vector3 &gyro_vec,
			geometry_msgs::Vector3 &acc_vec);

	void fill_imu_msg_attitude(sensor_msgs::ImuPtr &imu_msg, tf::Quaternion &orientation,
			double xg, double yg, double zg);

	void fill_imu_msg_raw(sensor_msgs::ImuPtr &imu_msg, double xg, double yg, double zg,
			double xa, double ya, double za);

	void handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_attitude_quaternion(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_highres_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_raw_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_scaled_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_scaled_pressure(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

};

};

#endif
