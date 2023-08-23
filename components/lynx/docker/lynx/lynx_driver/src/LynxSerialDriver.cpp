/**
 *  File name: LynxSerialDriver.cpp
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


#include <lynx_driver/LynxSerialDriver.h>


namespace lynx {


LynxSerialDriver::LynxSerialDriver() {

    //
    // Create modules
    //
    modules_.push_back(new modules::ImuModule());
    modules_.push_back(new modules::DriveModule());
    modules_.push_back(new modules::StateModule());

    //
    // Set send function
    //
    for (auto&& module : modules_) {
        module->setMavlinkWriteFunction(boost::bind(
                &LynxSerialDriver::sendMavlinkMessage, this, _1));
    }

	initializeSerialUsingRosParams();

    //
    // Init modules
    //
    for (auto&& module : modules_) {
		module->setup();
    }
}

LynxSerialDriver::~LynxSerialDriver() {
	if (serialPortPtr_) {
		serialPortPtr_->cancel();
		serialPortPtr_->close();
		serialPortPtr_.reset();
	}

	ioService_.stop();
	ioService_.reset();

    for (auto&& module : modules_) {
        delete module;
        module = nullptr;
    }
}

void LynxSerialDriver::spin() {
	ros::Rate rate(200);

	while (ros::ok()) {
		ioService_.poll_one();
		ros::spinOnce();
		rate.sleep();
	}
}

void LynxSerialDriver::initializeSerialUsingRosParams() {
	ros::NodeHandle nodePrivate("~");

	string port = nodePrivate.param<string>("port", "/dev/ttyACM0");
	int baudRate = nodePrivate.param("baud_rate", 460800);
	bool reset = nodePrivate.param("reset", false);

	initializeSerial(port, baudRate, reset);
}

void LynxSerialDriver::initializeSerial(const string& portName,
		int baudRate, bool reset) {

	using namespace boost::asio;

	boost::system::error_code errorCode;

	serialPortPtr_.reset(new boost::asio::serial_port(ioService_));
	serialPortPtr_->open(portName, errorCode);
    
	if (errorCode) {
		ROS_WARN("Failed to open serial port '%s'", portName.c_str());
		ROS_WARN("Error: %s", errorCode.message().c_str());
        ROS_WARN("All driver modules still running though");
		// exit(1);
        return;
	}

	ROS_INFO("Serial port opened '%s' baud rate %i", portName.c_str(), baudRate);

	serialPortPtr_->set_option(serial_port_base::baud_rate(baudRate));
	serialPortPtr_->set_option(serial_port_base::character_size(8));
	serialPortPtr_->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
	serialPortPtr_->set_option(serial_port_base::parity(serial_port_base::parity::none));
	serialPortPtr_->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

	//
	// Reset arduino using DTR & RST pins
	//
	if (reset) {
		int data = TIOCM_RTS;
		ioctl(serialPortPtr_->native_handle(), TIOCMBIS, &data); // RST
		data = TIOCM_DTR;
		ioctl(serialPortPtr_->native_handle(), TIOCMBIS, &data); // DTR


		ioctl(serialPortPtr_->native_handle(), TIOCMBIC, &data); // RST
		ioctl(serialPortPtr_->native_handle(), TIOCMBIC, &data); // DTR
	}

	readSerialAsync();
}

void LynxSerialDriver::readSerialAsync() {

	auto&& dataReceivedCallback = boost::bind(
			&LynxSerialDriver::dataReceivedCallback, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred);

	serialPortPtr_->async_read_some(
			boost::asio::buffer(readBuffer_, READ_BUFFER_LENGTH),
			dataReceivedCallback);

}

/**
 * @note Executed on separate thread
 * @param ec
 * @param bytes_transferred
 */
void LynxSerialDriver::dataReceivedCallback(
		const boost::system::error_code& errorCode, size_t bytesTransferred) {

	if (!serialPortPtr_) {
		return;
	}

	if (errorCode) {
		readSerialAsync();
		return;
	}

	if (false /* PRINT_RECEIVED_BYTES */) {
		for (unsigned int i = 0; i < bytesTransferred; ++i) {
			cout << readBuffer_[i];
		}

		cout << endl;
	}

	/**
	 * Data received
	 */
	for (unsigned int i = 0; i < bytesTransferred; ++i) {
		auto& currentByte = readBuffer_[i];

		if (mavlink_parse_char(MAVLINK_COMM_0, currentByte, 
                &mavlinkLastMessage_, &mavlinkLastStatus_)) {
					
            for (auto&& module : modules_) {
                if (module->canHandleMessage(mavlinkLastMessage_.msgid)) {
                    module->handleMessage(mavlinkLastMessage_);
                }
            }

		}
	}

	/**
	 * Continue async read
	 */
	readSerialAsync();

}

void LynxSerialDriver::sendMavlinkMessage(
        const mavlink_message_t& mavlinkMessage) {

    boost::system::error_code errorCode;
	uint8_t sendBuffer[MAVLINK_MAX_PACKET_LEN];

	uint16_t len = mavlink_msg_to_send_buffer(
            sendBuffer, &mavlinkMessage);

	serialPortPtr_->write_some(
            boost::asio::buffer(sendBuffer, len), 
            errorCode);

}


} /* namespace lynx */

