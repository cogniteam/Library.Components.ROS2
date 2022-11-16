/**
 *  File name: GpsStateMonitor.hpp
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

#include <sensor_msgs/NavSatFix.h>


namespace lynx{
namespace monitoring_tools{


class GpsStateMonitor : public IStateMonitor {


public:

    GpsStateMonitor() {
        stateHandler_.setModuleName("gps");
        ros::NodeHandle nh;
        moduleSubscriber_ = nh.subscribe(
                "sensors/gps/fix", 1 , &GpsStateMonitor::moduleCallback, this);
    }

    virtual ~GpsStateMonitor() {

    }

private:

    void moduleCallback(const sensor_msgs::NavSatFixConstPtr& gpsMsg) {
        stateHandler_.setCallbackTime();
        if (gpsMsg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
            stateHandler_.setIdle();
            stateHandler_.setMessage("No GPS fix");
        }
    }

private:

    ros::Subscriber moduleSubscriber_;

};

} /* monitoring_tools names */ 
} /* lynx namespace */