/**
 *  File name: ImuStateMonitor.h
 *     Author: nix <daria@cogniteam.com>
 * Created on: Nov 13, 2019
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


#include <lynx_driver/tools/monitoring_tools/IStateMonitor.h>

#include <lynx_msgs/StateImu.h>
#include <sensor_msgs/Imu.h>


namespace lynx {
namespace monitoring_tools {


class ImuStateMonitor : public IStateMonitor {

public:

    ImuStateMonitor() {
        stateHandler_.setModuleName("imu");
        ros::NodeHandle nh;
        moduleSubscriber_ = nh.subscribe(
            "events/imu/state", 1, &ImuStateMonitor::moduleCallback, this);
        imuRawSubscriber_ = nh.subscribe(
            "imu", 5, &ImuStateMonitor::imuRawCallback, this);
        
    }

    virtual ~ImuStateMonitor() {

    }

private:

    virtual lynx_msgs::ModuleState getState() override {
        if (!imuMsg_.lis3mdl_connected || !imuMsg_.lsm6_connected) {
            stateHandler_.setError();
            if (!imuMsg_.lis3mdl_connected && !imuMsg_.lsm6_connected) {
                stateHandler_.setMessage("lis3mdl and lsm6 not connected");
            } else if (!imuMsg_.lsm6_connected ) {
                stateHandler_.setMessage("lsm6 not connected");
            } else if (!imuMsg_.lis3mdl_connected) {
                stateHandler_.setMessage("lis3mdl not connected");
            }
        }
        return stateHandler_.getState();
    }

private:

    void moduleCallback(const lynx_msgs::StateImuConstPtr& imuMsg) {
        imuMsg_ = *imuMsg;
    }

    void imuRawCallback(const sensor_msgs::ImuConstPtr& imuMsg) {
        stateHandler_.setCallbackTime();
    }


private:

    lynx_msgs::StateImu imuMsg_;

    ros::Subscriber moduleSubscriber_;

    ros::Subscriber imuRawSubscriber_;



};

} /* namespace monitor_tools*/
} /* namespace lynx*/