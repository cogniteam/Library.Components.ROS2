/**
 *  File name: ImuModule.cpp
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Aug 18, 2019
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <lynx_driver/modules/ImuModule.h>


namespace lynx {
namespace modules {


ImuModule::ImuModule()
    : configServer_(ros::NodeHandle("~/imu")) {
    
}

ImuModule::~ImuModule() {

}

void ImuModule::setup() {

    cacheDirectory_ = string(getenv("HOME")) + "/.lynx";

    magScales_.x = 1.0;
    magScales_.y = 1.0;
    magScales_.z = 1.0;

    if (!boost::filesystem::exists(cacheDirectory_)) {
        boost::filesystem::create_directories(cacheDirectory_);
    }
    
    //
    // Load calibrations
    //

    try {
        ifstream gyroCacheFile((cacheDirectory_ + "/gyro_biases.txt").c_str());
        gyroCacheFile >> gyroBiases_.x;
        gyroCacheFile >> gyroBiases_.y;
        gyroCacheFile >> gyroBiases_.z;
        ROS_INFO("Gyro calibration loaded");
    } catch (...) {

    }

    try {
        ifstream magCacheFile((cacheDirectory_ + "/mag_biases.txt").c_str());
        magCacheFile >> magBiases_.x;
        magCacheFile >> magBiases_.y;
        magCacheFile >> magBiases_.z;
        magCacheFile >> magScales_.x;
        magCacheFile >> magScales_.y;
        magCacheFile >> magScales_.z;
        ROS_INFO("Mag calibration loaded");
    } catch (...) {

    }

    ros::NodeHandle node;
    ros::NodeHandle nodePrivate("~");

    nodePrivate.param<string>("tf_prefix", tfPrefix_, "");

    lastMagRawMsg_.header.stamp.sec = 0;
    lastMagRawMsg_.header.stamp.nsec = 0;

    configServer_.setCallback(
            boost::bind(&ImuModule::configCallback, this, _1, _2));

    imuStatePublisher_ = node.advertise<lynx_msgs::StateImu>(
            "events/imu/state", 1, true);

	imuRawPublisher_ = node.advertise<sensor_msgs::Imu>(
			"imu_raw", 100, false);

    /**
     * Applies calibration to raw imu messages 
     * published by the publisher above or from a bag file
     */
    imuRawSubscriber_ = node.subscribe(
            "imu_raw", 1000, &ImuModule::imuRawCallback, this);

	imuPublisher_ = node.advertise<sensor_msgs::Imu>(
			"imu", 1000, false);
            
	magneticRawPublisher_ = node.advertise<sensor_msgs::MagneticField>(
			"mag_raw", 1000, false);

    /**
     * Applies calibration to raw mag messages 
     * published by the publisher above or from a bag file
     */
    magneticRawSubscriber_ = node.subscribe("mag_raw", 
            1000, &ImuModule::magneticRawCallback, this);
            
	magneticPublisher_ = node.advertise<sensor_msgs::MagneticField>("mag", 
            1000, false);

    calibrationCommandsSubscriber_ = node.subscribe("imu/calibration", 
            1000, &ImuModule::calibrationCommandCallback, this);

    magPointPublisher_ = node.advertise<visualization_msgs::Marker>("mag_point", 
            1000, false);

    magRawPointPublisher_ = node.advertise<visualization_msgs::Marker>("mag_raw_point", 
            1000, false);
}

void ImuModule::handleMessage(const mavlink_message_t& mavlinkMessage) {

    
    if (mavlinkMessage.msgid == MAVLINK_MSG_ID_IMU) {
        ROS_INFO_ONCE("-- imu stream started");

        mavlink_imu_t imuMessage;
        mavlink_msg_imu_decode(&mavlinkMessage, &imuMessage);

        sensor_msgs::Imu imuRawRosMsg;
        sensor_msgs::MagneticField magRawRosMsg;
        
        imuRawRosMsg.header.stamp = ros::Time::now();

        string frameId = "";

        if (tfPrefix_ != "") {
            frameId = tfPrefix_ + "/base_link";
        } else {
            frameId = "base_link";
        }

        imuRawRosMsg.header.frame_id = frameId;
        magRawRosMsg.header.frame_id = frameId;

        magRawRosMsg.header.stamp = imuRawRosMsg.header.stamp;

        imuRawRosMsg.linear_acceleration.x = 9.78 * imuMessage.accel_x * 0.061 / 1000.0;
        imuRawRosMsg.linear_acceleration.y = 9.78 * imuMessage.accel_y * 0.061 / 1000.0;
        imuRawRosMsg.linear_acceleration.z = 9.78 * imuMessage.accel_z * 0.061 / 1000.0;
        imuRawRosMsg.angular_velocity.x = angles::from_degrees(imuMessage.gyro_x * 8.75 / 1000.0);
        imuRawRosMsg.angular_velocity.y = angles::from_degrees(imuMessage.gyro_y * 8.75 / 1000.0);
        imuRawRosMsg.angular_velocity.z = angles::from_degrees(imuMessage.gyro_z * 8.75 / 1000.0);
        magRawRosMsg.magnetic_field.x = imuMessage.mag_x / 6842.0;
        magRawRosMsg.magnetic_field.y = imuMessage.mag_y / 6842.0;
        magRawRosMsg.magnetic_field.z = imuMessage.mag_z / 6842.0;

        //
        // Raw messages
        //
        imuRawPublisher_.publish(imuRawRosMsg);

        //
        // Limit mag publish rate to magPublishRate_ value
        //
        if ((lastMagRawMsg_.header.stamp.sec == 0 && lastMagRawMsg_.header.stamp.nsec == 0) ||
                (magRawRosMsg.header.stamp - lastMagRawMsg_.header.stamp).toSec() > (1.0 / config_.mag_publish_rate)) {

            magneticRawPublisher_.publish(magRawRosMsg);
            lastMagRawMsg_ = magRawRosMsg;
        }

    }

    if (mavlinkMessage.msgid == MAVLINK_MSG_ID_STATE_IMU) {
        ROS_INFO_ONCE("-- imu state stream started");

        mavlink_state_imu_t imuState;
        mavlink_msg_state_imu_decode(&mavlinkMessage, &imuState);

        if (!imuState.lsm6_connected) {
            ROS_WARN("LSM6 (accels and gyros) connection failed");
        } else {
            ROS_INFO("LSM6 (accels and gyros) connected");
        }

        if (!imuState.lis3mdl_connected) {
            ROS_WARN("LIS3MDL (magnetometers) connection failed");
        } else {
            ROS_INFO("LIS3MDL (magnetometers) connected");
        }

        lynx_msgs::StateImu imuStateRosMsg;

        imuStateRosMsg.lis3mdl_connected = imuState.lis3mdl_connected;
        imuStateRosMsg.lsm6_connected = imuState.lsm6_connected;
        imuStatePublisher_.publish(imuStateRosMsg);
    }

}

void ImuModule::calibrationCommandCallback(
            const std_msgs::String::Ptr& command) {
    
    if (command->data == "calibration_g") {
        state_ = ImuState::GyroCalibration;
        ROS_INFO("Gyro calibration started");

        gyroBiasesSum_.x = 0;
        gyroBiasesSum_.y = 0;
        gyroBiasesSum_.z = 0;
        gyroSamplesCollected_ = 0;

    } else if (command->data == "calibration_m") {
        state_ = ImuState::MagnetometerCalibration;

        magMax_.x = -9999;
        magMax_.y = -9999;
        magMax_.z = -9999;
        magMin_.x = 9999;
        magMin_.y = 9999;
        magMin_.z = 9999;

        ROS_INFO("Magnetometer calibration started");
    } else {

        if (state_ == ImuState::GyroCalibration) {
            gyroBiases_.x = gyroBiasesSum_.x / (double)gyroSamplesCollected_;
            gyroBiases_.y = gyroBiasesSum_.y / (double)gyroSamplesCollected_;
            gyroBiases_.z = gyroBiasesSum_.z / (double)gyroSamplesCollected_;

            ofstream cacheFile((cacheDirectory_ + "/gyro_biases.txt").c_str(), ios::out | ios::trunc);
            cacheFile << gyroBiases_.x << endl;
            cacheFile << gyroBiases_.y << endl;
            cacheFile << gyroBiases_.z << endl;
            cacheFile.close();

            ROS_INFO("Gyro calibrated");
        }

        if (state_ == ImuState::MagnetometerCalibration) {
            magBiases_.x = (magMin_.x + magMax_.x) / 2.0;
            magBiases_.y = (magMin_.y + magMax_.y) / 2.0;
            magBiases_.z = (magMin_.z + magMax_.z) / 2.0;

            magScales_.x = (magMax_.x - magMin_.x) / 2.0;
            magScales_.y = (magMax_.y - magMin_.y) / 2.0;
            magScales_.z = (magMax_.z - magMin_.z) / 2.0;

            double averageScale = (magScales_.x + magScales_.y + magScales_.z) / 3.0;

            magScales_.x = averageScale / magScales_.x;
            magScales_.y = averageScale / magScales_.y;
            magScales_.z = averageScale / magScales_.z;

            ofstream cacheFile((cacheDirectory_ + "/mag_biases.txt").c_str(), ios::out | ios::trunc);
            cacheFile << magBiases_.x << endl;
            cacheFile << magBiases_.y << endl;
            cacheFile << magBiases_.z << endl;
            cacheFile << magScales_.x << endl;
            cacheFile << magScales_.y << endl;
            cacheFile << magScales_.z << endl;
            cacheFile.close();

            ROS_INFO("Magnetometer hard-iron calibrated");
        }

        state_ = ImuState::Normal;
    }
}

void ImuModule::imuRawCallback(const sensor_msgs::Imu::Ptr& imu) {

    auto imuRawRosMsg = *imu;

    //
    // Calibrate gyro biases
    //
    imu->angular_velocity.x -= gyroBiases_.x;
    imu->angular_velocity.y -= gyroBiases_.y;
    imu->angular_velocity.z -= gyroBiases_.z;

    imuPublisher_.publish(imu);

    /**
     * Collect gyro readings
     */
    if (state_ == ImuState::GyroCalibration) {
        gyroBiasesSum_.x += imuRawRosMsg.angular_velocity.x;
        gyroBiasesSum_.y += imuRawRosMsg.angular_velocity.y;
        gyroBiasesSum_.z += imuRawRosMsg.angular_velocity.z;
        gyroSamplesCollected_++;
    }

}

void ImuModule::magneticRawCallback(
        const sensor_msgs::MagneticField::Ptr& magneticField) {

    auto magneticFieldRaw = *magneticField;

    //
    // Calibrate magnetometer biases
    //
    magneticField->magnetic_field.x = (magneticField->magnetic_field.x - magBiases_.x) * magScales_.x;
    magneticField->magnetic_field.y = (magneticField->magnetic_field.y - magBiases_.y) * magScales_.y;
    magneticField->magnetic_field.z = (magneticField->magnetic_field.z - magBiases_.z) * magScales_.z;

    magneticPublisher_.publish(magneticField);

    //
    // Mag points
    //
    visualization_msgs::Marker magPoint;
    visualization_msgs::Marker magRawPoint;

    magPoint.type = visualization_msgs::Marker::SPHERE;
    magPoint.action = visualization_msgs::Marker::ADD;
    
    magPoint.id = rand();
    magRawPoint.id = rand();

    magPoint.ns = "imu/mag";
    magRawPoint.ns = "imu/mag_raw";

    magRawPoint.type = magPoint.type;
    magRawPoint.action = magPoint.action;

    magPoint.header.stamp = magneticField->header.stamp;
    magPoint.header.frame_id = (ros::this_node::getNamespace() != "/") ? 
            ros::this_node::getNamespace().substr(1) + "/odom" : 
            "odom";

    magRawPoint.header = magPoint.header;

    magPoint.pose.position.x = magneticField->magnetic_field.x;
    magPoint.pose.position.y = magneticField->magnetic_field.y;
    magPoint.pose.position.z = magneticField->magnetic_field.z;
    magPoint.pose.orientation.w = 1.0;

    magRawPoint.pose.position.x = magneticFieldRaw.magnetic_field.x;
    magRawPoint.pose.position.y = magneticFieldRaw.magnetic_field.y;
    magRawPoint.pose.position.z = magneticFieldRaw.magnetic_field.z;
    magRawPoint.pose.orientation.w = 1.0;

    magPoint.scale.x = 0.05;
    magPoint.scale.y = 0.05;
    magPoint.scale.z = 0.05;

    magRawPoint.scale = magPoint.scale;

    magPoint.color.a = 1.0;
    magRawPoint.color.a = 1.0;
    magPoint.color.g = 1.0;
    magRawPoint.color.r = 1.0;

    magPointPublisher_.publish(magPoint);
    magRawPointPublisher_.publish(magRawPoint);

    //
    // Find mag min/max limits
    //
    if (state_ == ImuState::MagnetometerCalibration) {

        magMax_.x = fmax(magMax_.x, magneticFieldRaw.magnetic_field.x);
        magMax_.y = fmax(magMax_.y, magneticFieldRaw.magnetic_field.y);
        magMax_.z = fmax(magMax_.z, magneticFieldRaw.magnetic_field.z);
        magMin_.x = fmin(magMin_.x, magneticFieldRaw.magnetic_field.x);
        magMin_.y = fmin(magMin_.y, magneticFieldRaw.magnetic_field.y);
        magMin_.z = fmin(magMin_.z, magneticFieldRaw.magnetic_field.z);

        ROS_INFO("(%2.2f,%2.2f), (%2.2f,%2.2f), (%2.2f,%2.2f)",
                magMax_.x ,magMin_.x ,magMax_.y ,magMin_.y ,magMax_.z ,magMin_.z);
    }

}

} /* namespace modules */
} /* namespace lynx */


