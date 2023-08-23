/**
 *  File name: LytnxSensorsStateMonitor.h
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


#include <lynx_driver/tools/monitoring_tools/DCameraStateMonitor.hpp>
#include <lynx_driver/tools/monitoring_tools/GpsStateMonitor.hpp>
#include <lynx_driver/tools/monitoring_tools/LidarStateMonitor.hpp>
#include <lynx_driver/tools/monitoring_tools/TCameraStateMonitor.hpp>
#include <lynx_driver/tools/monitoring_tools/ImuStateMonitor.hpp>
#include <lynx_driver/tools/monitoring_tools/LowLevelStateMonitor.hpp>

#include <lynx_msgs/States.h>


namespace lynx {
namespace monitoring_tools {


class SensorsStateMonitor {

public:

    SensorsStateMonitor() {
        initStateMonitors();
        ros::NodeHandle nh;
        publishStateMessageTimer_ = nh.createTimer(
                ros::Duration(1.0), &SensorsStateMonitor::publishStateMessageTimerCallback, this); 
        statesPublisher_ = nh.advertise<lynx_msgs::States>("perception/lynx_module_states", 1, false);
    }

    virtual ~SensorsStateMonitor() {
        for (auto moduleMonitor : moduleMonitor_) {
            delete moduleMonitor;
            moduleMonitor = NULL;
        }
    }

private:

    void initStateMonitors() {
        moduleMonitor_.push_back(new LidarStateMonitor());
        moduleMonitor_.push_back(new DCameraStateMonitor());
        moduleMonitor_.push_back(new TCameraStateMonitor());
        moduleMonitor_.push_back(new GpsStateMonitor());
        moduleMonitor_.push_back(new ImuStateMonitor());
        moduleMonitor_.push_back(new LowLevelStateMonitor());
    }

    void processData() {
        lynx_msgs::States modulesState;
        for (auto moduleMonitor : moduleMonitor_) {
            auto module = moduleMonitor->getState();
            modulesState.statesVector.push_back(module);
        }
        modulesState.stamp = ros::Time::now();
        statesPublisher_.publish(modulesState);
    }

    void publishStateMessageTimerCallback(const ros::TimerEvent&) {
        processData();
    }

private:

    ros::Publisher statesPublisher_;

    ros::Timer publishStateMessageTimer_;
    
    std::vector<IStateMonitor*> moduleMonitor_;

};

} /* monitoring_tools namespace*/
} /* lynx namespace */


