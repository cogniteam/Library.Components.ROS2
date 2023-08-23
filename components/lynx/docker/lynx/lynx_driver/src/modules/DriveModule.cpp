/**
 *  File name: DriveModule.cpp
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


#include <lynx_driver/modules/DriveModule.h>


namespace lynx {
namespace modules {


DriveModule::DriveModule()
    : speedPidConfigServer_(ros::NodeHandle("~/speed_pid")),
      driveConfigServer_(ros::NodeHandle("~/drive")),
      speedPid_("speed") {
          
}

DriveModule::~DriveModule() {

}

void DriveModule::setup() {

    driveState_.offboard = false;

    ros::NodeHandle node;

	ackermannCmdSubscriber_ = node.subscribe("ackermann_cmd",
			100, &DriveModule::ackermannCmdCallback, this);

    driveStateSubscriber_ = node.subscribe("events/drive/state",
            1, &DriveModule::driveStateCallback, this);

    encoderPublisher_ = node.advertise<lynx_msgs::Encoder>(
            "events/drive/encoder", 1, true);
        
    pidPublisher_ = node.advertise<geometry_msgs::Vector3>(
            "events/drive/pid", 1, true);

    speedPidConfigServer_.setCallback(boost::bind(
            &DriveModule::speedPidConfigCallback, this, _1, _2));

    driveConfigServer_.setCallback(boost::bind(
            &DriveModule::driveConfigCallback, this, _1, _2));

    speedPid_.setUpdateCallback(boost::bind(
            &DriveModule::speedPidCallback, this, _1));

    //
    // Subscribe to velocity estimation updates
    //
    odometryEstimator_.setSpeedUpdateCallback([&]() {
        // Velocity in Pulses Per Second
        speedPid_.update(odometryEstimator_.getLinearVelocity());
    });

    speedPid_.start();

}

void DriveModule::driveConfigCallback(lynx_driver::DriveConfig& config, uint32_t level) {
    driveConfig_ = config;

    //
    // Update odometry & pid controller
    //
    // odometryEstimator_.setYawOffset(config.odom_yaw_offset);

    //
    // Send mavlink config drive command
    //
    mavlink_message_t msg;
    mavlink_msg_command_config_drive_pack(1, 1, &msg,
            config.throttle_zero_pwm, 
            config.steering_zero_pwm,
            config.steering_min_pwm,
            config.steering_max_pwm,
            config.throttle_deadzone, 
            config.breaks_front_on_pwm, 
            config.breaks_front_off_pwm, 
            config.breaks_rear_on_pwm,
            config.breaks_rear_off_pwm, 
            config.tick_to_m, 
            config.throttle_min_dac_value);

    sendMavlinkMessage(msg);

}

void DriveModule::ackermannCmdCallback(
		const ackermann_msgs::AckermannDriveStamped& cmd) {
    
    //
    // Linear drive
    //
    
    // if (!driveConfig_.drive_enable_pid) {
        // PID disabled, send command directly
        // sendDriveCommand(velocityToPwm(cmd.drive.speed));
        // sendDriveCommand(cmd.drive.speed * 200);
        // ROS_INFO("Command sent: %f", cmd.drive.speed * 200);
    // }

    //
    // Update PID target
    //
    speedPid_.setTarget(cmd.drive.speed);

    //
    // Steering
    //
    sendSteerCommand(steeringAngleToPwm(cmd.drive.steering_angle));

    //
    // Start watchdog
    //
    driveWatchdogTimer_.stop();
    driveWatchdogTimer_ = ros::NodeHandle().createTimer(ros::Duration(0.5), 
            &DriveModule::driveWatchdogTimerCallback, this, true, true);
}

/**
 * Converts steering angle from radians to PWM value based on config pwm values
 * @param angle Steering angle in radians 
 * @return double 
 */
double DriveModule::steeringAngleToPwm(double angle) const {
    //
    // Clamp input angle within [min, max] range
    //
    double angleClamped = fmax(angles::from_degrees(driveConfig_.steering_angle_min), 
            fmin(angles::from_degrees(driveConfig_.steering_angle_max), angle));

    //
    // Map input velocity to PWM value
    //

    // [-1..1]
    double angleRatio = 2 * ((angleClamped - angles::from_degrees(driveConfig_.steering_angle_min)) / 
            (angles::from_degrees(driveConfig_.steering_angle_max) - 
            angles::from_degrees(driveConfig_.steering_angle_min)) - 0.5);


    auto pwm = 1500 - angleRatio * ((driveConfig_.steering_max_pwm - driveConfig_.steering_min_pwm) / 2);

    return pwm;
}

/**
 * PID controller update callback - send speed command message
 * @param output 
 */
void DriveModule::speedPidCallback(double output) {

    if (fabs(speedPid_.getState().setPoint) < 0.01) {
        //
        // If zero speed received, set control to output_limit_min right away
        //
        speedPid_.getState().setPoint = 0;
        speedPid_.getState().controlVariable = driveConfig_.throttle_zero_pwm;
        speedPid_.getState().errorPrevious = 0;
        output = driveConfig_.throttle_zero_pwm;
    }

    if (driveConfig_.enable_pid) {
        sendDriveCommand(output);
    }
} 


void DriveModule::driveWatchdogTimerCallback(const ros::TimerEvent&) {
    speedPid_.setTarget(0);
    speedPid_.getState().controlVariable = driveConfig_.throttle_zero_pwm;

    
    //
    // Send 0 value to motor
    //
    sendDriveCommand(driveConfig_.throttle_zero_pwm);

    //
    // Stop steering on it's place
    //
    sendSteerCommand(1500);

}

void DriveModule::handleMessage(const mavlink_message_t& mavlinkMessage) {

    if (mavlinkMessage.msgid == MAVLINK_MSG_ID_ENCODER) {
        ROS_INFO_ONCE("-- encoder stream started");

        mavlink_encoder_t encodersMavlink;
        mavlink_msg_encoder_decode(&mavlinkMessage, &encodersMavlink);

        lynx_msgs::Encoder encoderMsg;

        encoderMsg.ticks = encodersMavlink.ticks;
        encoderMsg.velocity = encodersMavlink.velocity / 1000.0;

        encoderPublisher_.publish(encoderMsg);
    }

    if (mavlinkMessage.msgid == MAVLINK_MSG_ID_STATE_PID) {
        ROS_INFO_ONCE("-- PID state stream started");

        mavlink_state_pid_t pidStateMsg;
        mavlink_msg_state_pid_decode(&mavlinkMessage, &pidStateMsg);
    }

}   

} /* namespace modules */
} /* namespace lynx */
