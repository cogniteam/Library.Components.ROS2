/**
 *  File name: StateHandler.hpp
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

#ifndef INCLUDE_LYNX_STATE_HANDLER_INTERFACE_H_ 
#define INCLUDE_LYNX_STATE_HANDLER_INTERFACE_H_ 


#include <lynx_msgs/ModuleState.h>
#include <string>

namespace lynx{
namespace monitoring_tools{


class StateHandler {

public:

    StateHandler(double timeout = 1.0)
        : timeout_(timeout), callbackTime_(ros::Time::now()) {
    
        stateMsg_.message = "No messages received";
    }

    virtual ~StateHandler() {

    }

public:

    void setIdle() {
        stateMsg_.state = lynx_msgs::ModuleState::IDLE;
    }

    void setError() {
        stateMsg_.state = lynx_msgs::ModuleState::ERROR;
    }

    void setOnStream() {
        stateMsg_.state = lynx_msgs::ModuleState::STREAM;
    }

    void setCallbackTime() {
        callbackTime_ = ros::Time::now();
        stateMsg_.message = "OK";
        setOnStream();
    }

    void setMessage(const std::string& message) {
        stateMsg_.message = message; 
    }

    ros::Time getCallbackTime() {
        return callbackTime_;
    }

    void setModuleName(const std::string& name) {
        stateMsg_.name = name;
    }

    lynx_msgs::ModuleState getState() {
        double timeDiff = (ros::Time::now() - getCallbackTime()).toSec();

        if (timeDiff > timeout_) {
            setError();
            setMessage("No message received for " + std::to_string(timeDiff) + " sec");
        }

        return stateMsg_;
    }

private:

    lynx_msgs::ModuleState stateMsg_;

    ros::Time callbackTime_;

    double timeout_;

};

} /* namespace monitoring_tools*/
} /* namespace lynx*/

#endif /* INCLUDE_LYNX_STATE_HANDLER_INTERFACE_H_ */