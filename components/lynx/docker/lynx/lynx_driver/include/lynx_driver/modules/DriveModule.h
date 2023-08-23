/**
 *  File name: DriveModule.h
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Aug 21, 2019
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


#ifndef INCLUDE_LYNX_DRIVER_DRIVEMODULE_H_
#define INCLUDE_LYNX_DRIVER_DRIVEMODULE_H_


#include <fstream>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int32.h>
#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>

#include <lynx_msgs/StateDrive.h>
#include <lynx_driver/PidConfig.h>
#include <lynx_driver/modules/ModuleBase.hpp>
#include <lynx_driver/tools/PidController.hpp>
#include <lynx_driver/tools/LynxOdometryEstimator.hpp>
#include <lynx_driver/DriveConfig.h>
#include <lynx_msgs/Encoder.h>


namespace lynx {
namespace modules {


using namespace std;
using namespace lynx::tools;


class DriveModule : public ModuleBase {

public:

	DriveModule();

	virtual ~DriveModule();

public:

    virtual string getName() const override {
        return "Drive";
    }

    virtual void setup() override;

public:

    virtual bool canHandleMessage(int mavlinkMessageId) const {
        return mavlinkMessageId == MAVLINK_MSG_ID_ENCODER ||
            mavlinkMessageId == MAVLINK_MSG_ID_STATE_PID;
    }

    // TODO Move to cpp
    virtual void handleMessage(const mavlink_message_t& mavlinkMessage); 

private:

	void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped& cmd);

    void driveConfigCallback(lynx_driver::DriveConfig& config, uint32_t level);

    void sendSteerCommand(int16_t command) {
        mavlink_message_t msg;
        mavlink_msg_command_steer_motor_pack(1, 1, &msg, 
                command);

        sendMavlinkMessage(msg);
    }

    void sendDriveCommand(int16_t command) {
        mavlink_message_t msg;
        mavlink_msg_command_drive_motor_pack(1, 1, &msg, 
                command);

        sendMavlinkMessage(msg);
    }

    void speedPidConfigCallback(lynx_driver::PidConfig& config, uint32_t level) {
        speedPid_.setConfig(config);
        speedPid_.setUpdateRate(config.rate);
    }

    /**
     * PID controller update callback - send speed command message
     * @param output 
     */
    void speedPidCallback(double output);

    void driveWatchdogTimerCallback(const ros::TimerEvent&);

    /**
     * Converts velocity from m/s to PWM value based on config pwm values
     * @param velocity 
     * @return double 
     */
    double velocityToPwm(double velocity) const;

    /**
     * Converts steering angle from radians to PWM value based on config pwm values
     * @param angle Steering angle in radians 
     * @return double 
     */
    double steeringAngleToPwm(double angle) const;

    void driveStateCallback(const lynx_msgs::StateDrive::ConstPtr& state) {
        driveState_ = *state;

        // Stop PID if offboard disabled
        if (!driveState_.offboard && speedPid_.isActive()) {
            speedPid_.setTarget(0);
            speedPid_.getState().controlVariable = driveConfig_.throttle_zero_pwm;
            speedPid_.stop();
            sendSteerCommand(1500);
        }

        // Start PID controller if offboard is activated
        if (driveState_.offboard && !speedPid_.isActive()) {
            speedPid_.setTarget(0);
            speedPid_.getState().controlVariable = driveConfig_.throttle_zero_pwm;
            speedPid_.start();
            sendSteerCommand(1500);
        }
    }

private:

	ros::Subscriber ackermannCmdSubscriber_;

    ros::Subscriber driveStateSubscriber_;

    dynamic_reconfigure::Server<lynx_driver::PidConfig> speedPidConfigServer_;

    dynamic_reconfigure::Server<lynx_driver::DriveConfig> driveConfigServer_;

    PidController speedPid_;

    lynx_driver::DriveConfig driveConfig_;

    LynxOdometryEstimator odometryEstimator_;

    ros::Timer driveWatchdogTimer_;

    lynx_msgs::StateDrive vehicleState_;

    ros::Publisher encoderPublisher_;

    ros::Publisher pidPublisher_;

    lynx_msgs::StateDrive driveState_;
};


} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_DRIVER_DRIVEMODULE_H_ */
