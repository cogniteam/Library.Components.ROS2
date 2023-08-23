/**
 *  File name: ModuleBase.hpp
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


#ifndef INCLUDE_LYNX_DRIVER_MODULEBASE_H_
#define INCLUDE_LYNX_DRIVER_MODULEBASE_H_


#include <boost/function.hpp>

#include <lynx_driver/mavlink/lynx/mavlink.h>


namespace lynx {
namespace modules {


using namespace std;


class ModuleBase {

public:

    using MavlinkWriteFunctionType = boost::function<void (const mavlink_message_t&)>;

public:

	ModuleBase() {
        
    }

	virtual ~ModuleBase() {

    }

public:

    virtual void setup() {
        
    }

    virtual string getName() const = 0;

    virtual bool canHandleMessage(int mavlinkMessageId) const {
        return false;
    }

    virtual void handleMessage(const mavlink_message_t& mavlinkMessage) {

    }

    /**
     * Sets function to use for sending mavlink messages
     * 
     * @param writeFunction 
     */
    void setMavlinkWriteFunction(const MavlinkWriteFunctionType& writeFunction) {
        mavlinkWriteFunction_ = writeFunction;
    }

protected:

    void sendMavlinkMessage(const mavlink_message_t& message) {

        if (mavlinkWriteFunction_) {
            mavlinkWriteFunction_(boost::ref(message));
        } else {
            ROS_WARN("Mavlink write function not set!");
        }
    }

private:

    MavlinkWriteFunctionType mavlinkWriteFunction_;

};


} /* namespace modules */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_DRIVER_MODULEBASE_H_ */
