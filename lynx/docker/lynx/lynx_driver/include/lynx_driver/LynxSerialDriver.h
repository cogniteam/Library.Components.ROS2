/**
 *  File name: LynxSerialDriver.h
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Feb 8, 2019
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


#ifndef INCLUDE_LYNX_DRIVER_LYNXSERIALDRIVER_H_
#define INCLUDE_LYNX_DRIVER_LYNXSERIALDRIVER_H_


#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <lynx_driver/modules/ImuModule.h>
#include <lynx_driver/modules/DriveModule.h>
#include <lynx_driver/modules/StateModule.h>


namespace lynx {


using namespace std;


class LynxSerialDriver {

public:

	LynxSerialDriver();

	virtual ~LynxSerialDriver();

public:

	void spin();

private:

	void initializeSerial(const string& portName, int baudRate, bool reset);

	void initializeSerialUsingRosParams();

	void readSerialAsync();

	void dataReceivedCallback(const boost::system::error_code& errorCode,
			size_t bytesTransferred);

    void sendMavlinkMessage(const mavlink_message_t& mavlinkMessage);

private:

	static const int READ_BUFFER_LENGTH = 32;

private:

	string portName_;

	int baudRate_;

	boost::asio::io_service ioService_;

	boost::shared_ptr<boost::asio::serial_port> serialPortPtr_;

	uint8_t readBuffer_[READ_BUFFER_LENGTH];

	mavlink_message_t mavlinkLastMessage_;

	mavlink_status_t mavlinkLastStatus_;

    vector<modules::ModuleBase*> modules_;

};


} /* namespace lynx */


#endif /* INCLUDE_LYNX_DRIVER_LYNXSERIALDRIVER_H_ */
