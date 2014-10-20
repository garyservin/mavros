#include <mavros/imu_pub.h>

namespace mavplugin {

IMUPubPlugin::IMUPubPlugin() :
		uas(nullptr),
		linear_accel_vec(),
		has_hr_imu(false),
		has_scaled_imu(false),
		has_att_quat(false)
	{ };

void IMUPubPlugin::initialize(UAS &uas_,
		ros::NodeHandle &nh,
		diagnostic_updater::Updater &diag_updater)
{
	double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

	uas = &uas_;

	nh.param<std::string>("imu/frame_id", frame_id, "fcu");
	nh.param("imu/linear_acceleration_stdev", linear_stdev, 0.0003); // check default by MPU6000 spec
	nh.param("imu/angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0)); // check default by MPU6000 spec
	nh.param("imu/orientation_stdev", orientation_stdev, 1.0);
	nh.param("imu/magnetic_stdev", mag_stdev, 0.0);

	setup_covariance(linear_acceleration_cov, linear_stdev);
	setup_covariance(angular_velocity_cov, angular_stdev);
	setup_covariance(orientation_cov, orientation_stdev);
	setup_covariance(magnetic_cov, mag_stdev);
	setup_covariance(unk_orientation_cov, 0.0);

	imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
	magn_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
	temp_pub = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
	press_pub = nh.advertise<sensor_msgs::FluidPressure>("imu/atm_pressure", 10);
	imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
}

std::string const IMUPubPlugin::get_name() const {
	return "IMUPub";
}

const MavRosPlugin::message_map IMUPubPlugin::get_rx_handlers() {
	return {
		MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE, &IMUPubPlugin::handle_attitude),
		MESSAGE_HANDLER(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, &IMUPubPlugin::handle_attitude_quaternion),
		MESSAGE_HANDLER(MAVLINK_MSG_ID_HIGHRES_IMU, &IMUPubPlugin::handle_highres_imu),
		MESSAGE_HANDLER(MAVLINK_MSG_ID_RAW_IMU, &IMUPubPlugin::handle_raw_imu),
		MESSAGE_HANDLER(MAVLINK_MSG_ID_SCALED_IMU, &IMUPubPlugin::handle_scaled_imu),
		MESSAGE_HANDLER(MAVLINK_MSG_ID_SCALED_PRESSURE, &IMUPubPlugin::handle_scaled_pressure),
	};
}

/* -*- helpers -*- */

void IMUPubPlugin::setup_covariance(boost::array<double, 9> &cov, double stdev) {
	std::fill(cov.begin(), cov.end(), 0.0);
	if (stdev == 0.0)
		cov[0] = -1.0;
	else {
		cov[0+0] = cov[3+1] = cov[6+2] = std::pow(stdev, 2);
	}
}

void IMUPubPlugin::uas_store_attitude(tf::Quaternion &orientation,
		geometry_msgs::Vector3 &gyro_vec,
		geometry_msgs::Vector3 &acc_vec)
{
	tf::Vector3 angular_velocity;
	tf::Vector3 linear_acceleration;
	tf::vector3MsgToTF(gyro_vec, angular_velocity);
	tf::vector3MsgToTF(acc_vec, linear_acceleration);

	uas->set_attitude_orientation(orientation);
	uas->set_attitude_angular_velocity(angular_velocity);
	uas->set_attitude_linear_acceleration(linear_acceleration);
}

//! fill imu/data message
void IMUPubPlugin::fill_imu_msg_attitude(sensor_msgs::ImuPtr &imu_msg,
		tf::Quaternion &orientation,
		double xg, double yg, double zg)
{
	tf::quaternionTFToMsg(orientation, imu_msg->orientation);

	imu_msg->angular_velocity.x = xg;
	imu_msg->angular_velocity.y = yg;
	imu_msg->angular_velocity.z = zg;

	// vector from HIGHRES_IMU or RAW_IMU
	imu_msg->linear_acceleration = linear_accel_vec;

	imu_msg->orientation_covariance = orientation_cov;
	imu_msg->angular_velocity_covariance = angular_velocity_cov;
	imu_msg->linear_acceleration_covariance = linear_acceleration_cov;
}

//! fill imu/data_raw message, store linear acceleration for imu/data
void IMUPubPlugin::fill_imu_msg_raw(sensor_msgs::ImuPtr &imu_msg,
		double xg, double yg, double zg,
		double xa, double ya, double za)
{
	imu_msg->angular_velocity.x = xg;
	imu_msg->angular_velocity.y = yg;
	imu_msg->angular_velocity.z = zg;

	imu_msg->linear_acceleration.x = xa;
	imu_msg->linear_acceleration.y = ya;
	imu_msg->linear_acceleration.z = za;
	linear_accel_vec = imu_msg->linear_acceleration;

	imu_msg->orientation_covariance = unk_orientation_cov;
	imu_msg->angular_velocity_covariance = angular_velocity_cov;
	imu_msg->linear_acceleration_covariance = linear_acceleration_cov;
}

/* -*- message handlers -*- */

void IMUPubPlugin::handle_attitude(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	if (has_att_quat)
		return;

	mavlink_attitude_t att;
	mavlink_msg_attitude_decode(msg, &att);

	sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();

	// NED -> ENU (body-fixed)
	tf::Quaternion orientation = tf::createQuaternionFromRPY(
			att.roll, -att.pitch, -att.yaw);

	fill_imu_msg_attitude(imu_msg, orientation,
			att.rollspeed,
			-att.pitchspeed,
			-att.yawspeed);

	uas_store_attitude(orientation,
			imu_msg->angular_velocity,
			imu_msg->linear_acceleration);

	// publish data
	imu_msg->header.frame_id = frame_id;
	imu_msg->header.stamp = ros::Time::now();
	imu_pub.publish(imu_msg);
}

// almost the same as handle_attitude(), but for ATTITUDE_QUATERNION
void IMUPubPlugin::handle_attitude_quaternion(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	mavlink_attitude_quaternion_t att_q;
	mavlink_msg_attitude_quaternion_decode(msg, &att_q);

	ROS_INFO_COND_NAMED(!has_att_quat, "imu", "Attitude quaternion IMU detected!");
	has_att_quat = true;

	sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();

	// PX4 NED (w x y z) -> ROS ENU (x -y -z w) (body-fixed)
	tf::Quaternion orientation(att_q.q2, -att_q.q3, -att_q.q4, att_q.q1);

	fill_imu_msg_attitude(imu_msg, orientation,
			att_q.rollspeed,
			-att_q.pitchspeed,
			-att_q.yawspeed);

	uas_store_attitude(orientation,
			imu_msg->angular_velocity,
			imu_msg->linear_acceleration);

	// publish data
	imu_msg->header.frame_id = frame_id;
	imu_msg->header.stamp = ros::Time::now();
	imu_pub.publish(imu_msg);
}

void IMUPubPlugin::handle_highres_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	mavlink_highres_imu_t imu_hr;
	mavlink_msg_highres_imu_decode(msg, &imu_hr);

	ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "High resolution IMU detected!");
	has_hr_imu = true;

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;

	/* imu/data_raw filled by HR IMU */
	if (imu_hr.fields_updated & ((7<<3)|(7<<0))) {
		sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();

		fill_imu_msg_raw(imu_msg,
				imu_hr.xgyro, -imu_hr.ygyro, -imu_hr.zgyro,
				imu_hr.xacc, -imu_hr.yacc, -imu_hr.zacc);

		imu_msg->header = header;
		imu_raw_pub.publish(imu_msg);
	}

	if (imu_hr.fields_updated & (7<<6)) {
		sensor_msgs::MagneticFieldPtr magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

		// Convert from local NED plane to ENU
		magn_msg->magnetic_field.x = imu_hr.ymag * GAUSS_TO_TESLA;
		magn_msg->magnetic_field.y = imu_hr.xmag * GAUSS_TO_TESLA;
		magn_msg->magnetic_field.z = -imu_hr.zmag * GAUSS_TO_TESLA;

		magn_msg->magnetic_field_covariance = magnetic_cov;

		magn_msg->header = header;
		magn_pub.publish(magn_msg);
	}

	if (imu_hr.fields_updated & (1<<9)) {
		sensor_msgs::FluidPressurePtr atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();

		atmp_msg->fluid_pressure = imu_hr.abs_pressure * MILLIBAR_TO_PASCAL;
		atmp_msg->header = header;
		press_pub.publish(atmp_msg);
	}

	if (imu_hr.fields_updated & (1<<12)) {
		sensor_msgs::TemperaturePtr temp_msg = boost::make_shared<sensor_msgs::Temperature>();

		temp_msg->temperature = imu_hr.temperature;
		temp_msg->header = header;
		temp_pub.publish(temp_msg);
	}
}

void IMUPubPlugin::handle_raw_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	if (has_hr_imu || has_scaled_imu)
		return;

	sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
	mavlink_raw_imu_t imu_raw;
	mavlink_msg_raw_imu_decode(msg, &imu_raw);

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;

	/* NOTE: APM send SCALED_IMU data as RAW_IMU */
	fill_imu_msg_raw(imu_msg,
			imu_raw.xgyro * MILLIRS_TO_RADSEC,
			-imu_raw.ygyro * MILLIRS_TO_RADSEC,
			-imu_raw.zgyro * MILLIRS_TO_RADSEC,
			imu_raw.xacc * MILLIG_TO_MS2,
			-imu_raw.yacc * MILLIG_TO_MS2,
			-imu_raw.zacc * MILLIG_TO_MS2);

	if (!uas->is_ardupilotmega()) {
		ROS_WARN_THROTTLE_NAMED(60, "imu", "RAW_IMU: linear acceleration known on APM only");
		linear_accel_vec.x = 0.0;
		linear_accel_vec.y = 0.0;
		linear_accel_vec.z = 0.0;
	}

	imu_msg->header = header;
	imu_raw_pub.publish(imu_msg);

	/* -*- magnetic vector -*- */
	sensor_msgs::MagneticFieldPtr magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

	// Convert from local NED plane to ENU
	magn_msg->magnetic_field.x = imu_raw.ymag * MILLIT_TO_TESLA;
	magn_msg->magnetic_field.y = imu_raw.xmag * MILLIT_TO_TESLA;
	magn_msg->magnetic_field.z = -imu_raw.zmag * MILLIT_TO_TESLA;

	magn_msg->magnetic_field_covariance = magnetic_cov;

	magn_msg->header = header;
	magn_pub.publish(magn_msg);
}

void IMUPubPlugin::handle_scaled_imu(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	if (has_hr_imu)
		return;

	ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "Scaled IMU message used.");
	has_scaled_imu = true;

	sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
	mavlink_scaled_imu_t imu_raw;
	mavlink_msg_scaled_imu_decode(msg, &imu_raw);

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;

	fill_imu_msg_raw(imu_msg,
			imu_raw.xgyro * MILLIRS_TO_RADSEC,
			-imu_raw.ygyro * MILLIRS_TO_RADSEC,
			-imu_raw.zgyro * MILLIRS_TO_RADSEC,
			imu_raw.xacc * MILLIG_TO_MS2,
			-imu_raw.yacc * MILLIG_TO_MS2,
			-imu_raw.zacc * MILLIG_TO_MS2);

	imu_msg->header = header;
	imu_raw_pub.publish(imu_msg);

	/* -*- magnetic vector -*- */
	sensor_msgs::MagneticFieldPtr magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

	// Convert from local NED plane to ENU
	magn_msg->magnetic_field.x = imu_raw.ymag * MILLIT_TO_TESLA;
	magn_msg->magnetic_field.y = imu_raw.xmag * MILLIT_TO_TESLA;
	magn_msg->magnetic_field.z = -imu_raw.zmag * MILLIT_TO_TESLA;

	magn_msg->magnetic_field_covariance = magnetic_cov;

	magn_msg->header = header;
	magn_pub.publish(magn_msg);
}

void IMUPubPlugin::handle_scaled_pressure(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	if (has_hr_imu)
		return;

	mavlink_scaled_pressure_t press;
	mavlink_msg_scaled_pressure_decode(msg, &press);

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;

	sensor_msgs::TemperaturePtr temp_msg = boost::make_shared<sensor_msgs::Temperature>();
	temp_msg->temperature = press.temperature / 100.0;
	temp_msg->header = header;
	temp_pub.publish(temp_msg);

	sensor_msgs::FluidPressurePtr atmp_msg = boost::make_shared<sensor_msgs::FluidPressure>();
	atmp_msg->fluid_pressure = press.press_abs * 100.0;
	atmp_msg->header = header;
	press_pub.publish(atmp_msg);
}


}; // namespace mavplugin
