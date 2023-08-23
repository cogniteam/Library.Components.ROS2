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


#ifndef INCLUDE_LYNX_DRIVER_PIDCONTROLLER_H_
#define INCLUDE_LYNX_DRIVER_PIDCONTROLLER_H_


#include <boost/signals2.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <lynx_driver/PidConfig.h>


namespace lynx {
namespace tools {


using namespace std;


/**
 * Holds the state of the PID controller
 */
struct PidState {

    /**
     * Desired target value
     */
    double setPoint = 0;

    /**
     * Measured value
     */
    double processVariable = 0;

    /**
     * Output control value
     */
    double controlVariable = 0;

    double errorPrevious = 0;

    double integral = 0;

    double derivative = 0;

    ros::Time updateTime = ros::Time(0);

};


class PidController {

public:

	PidController(const string& name) {
        setUpdateRate(20.0);
        setConfig(lynx_driver::PidConfig()); // Set default config values set by cfg file

        state_.updateTime = ros::Time::now();
        state_.integral = 0;
        state_.errorPrevious = 0;

        ros::NodeHandle node;

        statePublisher_ = node.advertise<geometry_msgs::Vector3>(
                "events/drive/pid", 100, false);

        updateTimer_ = ros::NodeHandle().createTimer(ros::Rate(updateRate_), 
                &PidController::updateTimerCallback, this);

    }

	virtual ~PidController() {

    }

public:

    inline PidState& getState() {
        return state_;
    }

    inline lynx_driver::PidConfig& getConfig() {
        return config_;
    }

    inline bool isActive() const {
        return active_;
    }

    /**
     * Starts PID control loop
     */
    void start() {
        
        if (active_) {
            return; // Already running
        }

        active_ = true;  
    }

    /**
     * Stops PID control loop
     */
    void stop() {

        if (!active_) {
            return; // Already stopped
        }

        active_ = false;
    }

    /**
     * Reset integral and derivative parameters
     */
    void reset() {
        state_.errorPrevious = 0;
        state_.integral = 0;
    }

    /**
     * Sets desired update rate (effective after start() is called)
     * @param rate 
     */
    inline void setUpdateRate(double rate) {

        if (rate != updateRate_) {
            updateRate_ = rate;
            updateTimer_.setPeriod(ros::Duration(1.0 / rate));
        }
    }

    /**
     * Gets update rate
     * @return double 
     */
    inline double getUpdateRate() const {
        return updateRate_;
    }

    inline void setConfig(const lynx_driver::PidConfig& config) {
        config_ = config;
    }

    void setUpdateCallback(const boost::function<void(double)>& callback) {
        updateSignal_.connect(callback);
    }

    /**
     * Update measurement
     * @param processValue 
     */
    void update(double processVariable) {
        
        // Clamp
        processVariable = fmin(fmax(
                processVariable, config_.input_limit_min), config_.input_limit_max);

        state_.processVariable = state_.processVariable * (config_.process_smooth_factor) +
                processVariable * (1.0 - config_.process_smooth_factor);
    }

    inline void setTarget(double setPoint) {
        
        //
        // Limit input values
        //
        setPoint = fmin(fmax(setPoint, config_.input_limit_min), config_.input_limit_max);

        state_.setPoint = setPoint;
    }

private:

    void updateTimerCallback(const ros::TimerEvent&) {

        //
        // Calculate PID step, update controlValue_ ...
        //
        auto updateTime = ros::Time::now();

        if (active_) {

            double dt = (updateTime - state_.updateTime).toSec();

            auto error = state_.setPoint - state_.processVariable;
            state_.integral = state_.integral + error * dt;
            state_.derivative = (error - state_.errorPrevious) / dt;

            // state_.controlVariable += config_.gain * (config_.kp * error + 
            //         config_.ki * state_.integral + 
            //         config_.kd * state_.derivative);
            
            auto newControl = config_.gain * (config_.kp * error + 
                    config_.ki * state_.integral + 
                    config_.kd * state_.derivative);
            
            double oldControlVariable = state_.controlVariable;

            if (config_.inverse_control) {
                newControl = -newControl;
            }

            //
            // Add new control to prev control command
            //

            // Clamp acceleration
            auto maxAcceleration = fabs(config_.control_max_acceleration * dt);
            
            newControl = fmax(fmin(newControl, maxAcceleration), -maxAcceleration);

            // Add to control output
            newControl += state_.controlVariable;

            // Smooth control output
            state_.controlVariable = config_.control_smooth_factor * oldControlVariable +
                    (1.0 - config_.control_smooth_factor) * newControl;

            //
            // Apply limits
            //
            state_.controlVariable = fmin(
                    config_.output_limit_max, 
                    fmax(config_.output_limit_min, state_.controlVariable));

            state_.errorPrevious = error;
        }

        state_.updateTime = updateTime;


        if (active_) {
            updateSignal_(state_.controlVariable);
        }

        //
        // Publish state to ROS topic
        //
        geometry_msgs::Vector3 pidStateMsg;
        // Target
        pidStateMsg.x = state_.setPoint;
        // Measurement
        pidStateMsg.y = state_.processVariable;
        // Control output
        pidStateMsg.z = state_.controlVariable;

        statePublisher_.publish(pidStateMsg);

    }

private:

    bool active_ = false;

    PidState state_;

    ros::Timer updateTimer_;

    double updateRate_;

    lynx_driver::PidConfig config_;

    boost::signals2::signal<void(double)> updateSignal_;

    ros::Publisher statePublisher_;

};


} /* namespace tools */
} /* namespace lynx */


#endif /* INCLUDE_LYNX_DRIVER_PIDCONTROLLER_H_ */
