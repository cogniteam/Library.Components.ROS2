/**
 *  File name: ImuModule.h
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


#ifndef INCLUDE_LYNX_DRIVER_IMUMODULE_H_
#define INCLUDE_LYNX_DRIVER_IMUMODULE_H_


#include <fstream>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <lynx_driver/modules/ModuleBase.hpp>
#include <lynx_driver/ImuConfig.h>
#include <lynx_msgs/StateImu.h>


namespace lynx {
namespace modules {


using namespace std;


class ImuModule : public ModuleBase {

public:

	ImuModule();

	virtual ~ImuModule();

public:

    virtual string getName() const override {
        return "IMU";
    }

    virtual bool canHandleMessage(int mavlinkMessageId) const override {
        return mavlinkMessageId == MAVLINK_MSG_ID_IMU ||
                mavlinkMessageId == MAVLINK_MSG_ID_STATE_IMU;
    }

    virtual void setup() override;

    virtual void handleMessage(const mavlink_message_t& mavlinkMessage) override;

private:

    enum ImuState {
        Normal,
        GyroCalibration,
        MagnetometerCalibration
    };

private:

    void calibrationCommandCallback(
            const std_msgs::String::Ptr& command);

    void imuRawCallback(const sensor_msgs::Imu::Ptr& imu);

    void magneticRawCallback(
            const sensor_msgs::MagneticField::Ptr& magneticField);

    void configCallback(lynx_driver::ImuConfig& config, uint32_t level) {
        config_ = config;
    }

private:

    ImuState state_ = ImuState::Normal;

    ros::Publisher imuStatePublisher_;

	ros::Publisher imuPublisher_;

	ros::Publisher imuRawPublisher_;

    ros::Subscriber imuRawSubscriber_;

	ros::Publisher magneticPublisher_;

	ros::Publisher magneticRawPublisher_;

    ros::Subscriber magneticRawSubscriber_;

    ros::Subscriber calibrationCommandsSubscriber_;

    sensor_msgs::MagneticField lastMagRawMsg_;

    /**
     * Gyro calibration
     */
    geometry_msgs::Vector3 gyroBiases_;

    geometry_msgs::Vector3 gyroBiasesSum_;
    
    int gyroSamplesCollected_ = 0;

    /**
     * Magnetometer calibration
     */
    geometry_msgs::Vector3 magBiases_;

    geometry_msgs::Vector3 magScales_;

    geometry_msgs::Vector3 magMin_;

    geometry_msgs::Vector3 magMax_;

    ros::Publisher magPointPublisher_;
    
    ros::Publisher magRawPointPublisher_;

    /**
     * Calibration cache directory
     */
    string cacheDirectory_;

    dynamic_reconfigure::Server<lynx_driver::ImuConfig> configServer_;

    lynx_driver::ImuConfig config_;

    string tfPrefix_;
};


} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_DRIVER_IMUMODULE_H_ */
