/**
 *  File name: StateModule.h
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


#ifndef INCLUDE_LYNX_DRIVER_STATEMODULE_H_
#define INCLUDE_LYNX_DRIVER_STATEMODULE_H_


#include <ros/ros.h>

#include <lynx_driver/modules/ModuleBase.hpp>
#include <lynx_msgs/StateDrive.h>


namespace lynx {
namespace modules {


using namespace std;


/**
 * Publishes lynx's state message to ROS topic
 */
class StateModule : public ModuleBase {

public:

	StateModule() {
        ros::NodeHandle node;
        
        statePublisher_ = node.advertise<lynx_msgs::StateDrive>(
                "events/drive/state", 1, true);
    }

	virtual ~StateModule() {

    }

public:

    virtual string getName() const override {
        return "State";
    }

    virtual bool canHandleMessage(int mavlinkMessageId) const override {
        return mavlinkMessageId == MAVLINK_MSG_ID_STATE_DRIVE;
    }

    virtual void handleMessage(const mavlink_message_t& mavlinkMessage) override {
        
        if (mavlinkMessage.msgid == MAVLINK_MSG_ID_STATE_DRIVE) {
            ROS_INFO_ONCE("-- drive state stream started");

            mavlink_state_drive_t stateMavlinkMsg;
            mavlink_msg_state_drive_decode(&mavlinkMessage, &stateMavlinkMsg);

            lynx_msgs::StateDrive stateRosMsg;

            stateRosMsg.offboard = stateMavlinkMsg.offboard;
            stateRosMsg.throttle_voltage = (stateMavlinkMsg.throttle_voltage) * (2.75 - 0.55) / (4095) + 0.55;
            stateRosMsg.front_breaks_pwm = stateMavlinkMsg.front_breaks_pwm;
            stateRosMsg.rear_breaks_pwm = stateMavlinkMsg.rear_breaks_pwm;
            stateRosMsg.steering_pwm = stateMavlinkMsg.steering_pwm;
            stateRosMsg.lights_state = stateMavlinkMsg.lights_state;
            stateRosMsg.horn_state = stateMavlinkMsg.horn_state;
            stateRosMsg.rc_connected = stateMavlinkMsg.rc_connected;
            stateRosMsg.rc_offboard_pwm = stateMavlinkMsg.rc_offboard_pwm;
            stateRosMsg.rc_steering_pwm = stateMavlinkMsg.rc_steering_pwm;
            stateRosMsg.rc_throttle_pwm = stateMavlinkMsg.rc_throttle_pwm;
            stateRosMsg.rc_lights_pwm = stateMavlinkMsg.rc_lights_pwm;
            stateRosMsg.rc_horn_pwm = stateMavlinkMsg.rc_horn_pwm;
            
            statePublisher_.publish(stateRosMsg);
        }
    }

private:

    ros::Publisher statePublisher_;

};


} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_DRIVER_STATEMODULE_H_ */
