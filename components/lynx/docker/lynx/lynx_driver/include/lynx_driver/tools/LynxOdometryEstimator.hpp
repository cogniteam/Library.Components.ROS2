/**
 *  File name: LynxOdometryEstimator.hpp
 *     Author: Igor Makhtes <igor@cogniteam.com>
 * Created on: Aug 24, 2019
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


#include <boost/signals2.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h> 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <lynx_msgs/Encoder.h>


namespace lynx {
namespace tools {


using namespace std;


class LynxOdometryEstimator {

public:

    LynxOdometryEstimator() {

        //
        // Reset odometry to zero
        //
        odometryEstimation_.setIdentity();
        lastFilteredImuMessage_.orientation.w = 1.0;

        ros::NodeHandle node;
        ros::NodeHandle nodePrivate("~");

        imuFilteredSubscriber_ = node.subscribe("imu/data", 1000, 
                &LynxOdometryEstimator::imuFilteredCallback, this);

        //
        // Calibrated biases but not filtered 
        //
        imuSubscriber_ = node.subscribe("imu", 1000, 
                &LynxOdometryEstimator::imuCallback, this);

        encoderSubscriber_ = node.subscribe("events/drive/encoder", 1000,
                &LynxOdometryEstimator::encoderCallback, this);

        externalOdometrySubscriber_ = node.subscribe("odom_external", 10, 
                &LynxOdometryEstimator::externalOdometryCallback, this);
        
        odometryPublisher_ = node.advertise<nav_msgs::Odometry>(
                "odom", 100, false);

        orientationEulerPublisher_ = node.advertise<geometry_msgs::Vector3>(
                "perception/odometry/orientation_euler", 100, false);

        string tfPrefix = "";

        nodePrivate.param<string>("odom_frame", odomFrame_, "odom");
        nodePrivate.param<string>("base_frame", baseFrame_, "base_link");
        nodePrivate.param<string>("tf_prefix", tfPrefix, "");

        if (tfPrefix != "") {
            odomFrame_ = tfPrefix + "/" + odomFrame_;
            baseFrame_ = tfPrefix + "/" + baseFrame_;
        }

        nodePrivate.param("publish_transform", publishTransform_, true);
        nodePrivate.param("earth_orientation", earthOrientation_, true);
        double publishRate = nodePrivate.param("publish_rate", 50.0);

        speedEstimationTimer_ = node.createTimer(ros::Rate(speedEstimationRate_), 
                &LynxOdometryEstimator::speedEstimateTimerCallback, this);

        publishTimer_ = node.createTimer(ros::Rate(publishRate), 
                &LynxOdometryEstimator::publishTimerCallback, this);

        velocityFromEncoders_ = 0;
    }

    ~LynxOdometryEstimator() {

    }

public:

    inline void setYawOffset(double offset) {
        yawOffset_ = offset;
    }

    /**
     * Gets angular velocities
     * @return geometry_msgs::Vector3 
     */
    inline geometry_msgs::Vector3 getAngularVelocity() const {
        return lastCalibratedImuMessage_.angular_velocity;
    }

    inline double getLinearVelocity() const {
        double velocity = 0;

        if (velocityFromExternalOdometry_) {
            velocity = lastExternalOdometryMessage_.twist.twist.linear.x;
        } else {
            velocity = velocityFromEncoders_;
        }

        if (fabs(velocity) < 0.01) {
            velocity = 0;
        }

        return velocity;
    }

    /**
     * Subscribes to speed change event
     * @param callback 
     * @return boost::signals2::connection 
     */
    boost::signals2::connection setSpeedUpdateCallback(
            const boost::function<void(void)>& callback) {
        return speedUpdatedSignal_.connect(callback);
    }

    /**
     * Subscribes to angular velocity change event
     * @param callback 
     * @return boost::signals2::connection 
     */
    boost::signals2::connection setAngularVelocityUpdateCallback(
            const boost::function<void(void)>& callback) {
        return angularVelocityUpdatedSignal_.connect(callback);
    }

    inline void setVelocitySmoothFactor(double factor) {
        speedSmoothFactor_ = factor;   
    }

    inline void setEncoderRatio(double tickToMeter) {
        tickToMeterRatio_ = tickToMeter;
    }

private:

    void publishTimerCallback(const ros::TimerEvent&) {
        nav_msgs::Odometry odometryMessage;

        odometryMessage.header.stamp = ros::Time::now();
        odometryMessage.header.frame_id = odomFrame_;
        odometryMessage.child_frame_id = baseFrame_;
        
        tf::poseTFToMsg(odometryEstimation_, odometryMessage.pose.pose);
        
        // Twist
        odometryMessage.twist.twist.linear.x = getLinearVelocity();
        odometryMessage.twist.twist.angular = lastCalibratedImuMessage_.angular_velocity;

        // TODO Fill twist fields

        odometryPublisher_.publish(odometryMessage);


        double roll, pitch, yaw;
        
        tf::Quaternion imuFilteredOrientation;
        tf::quaternionMsgToTF(lastFilteredImuMessage_.orientation, imuFilteredOrientation);

        tf::Matrix3x3 m(imuFilteredOrientation);
        m.getRPY(roll, pitch, yaw);

        geometry_msgs::Vector3 euilerAnglesMessage;
        euilerAnglesMessage.x = roll;
        euilerAnglesMessage.y = pitch;
        euilerAnglesMessage.z = tf::getYaw(odometryEstimation_.getRotation());
        orientationEulerPublisher_.publish(euilerAnglesMessage);

        if (publishTransform_) {
            tfPublisher_.sendTransform(
                    tf::StampedTransform(
                            odometryEstimation_, odometryMessage.header.stamp, 
                            odomFrame_, baseFrame_));
        }
    }

    void imuCallback(const sensor_msgs::Imu::Ptr& imu) {
        
        if (!imuInitialized_) {
            lastCalibratedImuMessage_ = *imu;
            imuInitialized_ = true;
            return;
        }

        if (!earthOrientation_) {
            
            //
            // Integrate gyro speed into yaw angle
            //
            double timeDelta = (imu->header.stamp - lastCalibratedImuMessage_.header.stamp).toSec();
            double yawDelta = imu->angular_velocity.z * timeDelta; //  "/ 2.0" ??

            tf::Transform deltaTransform(tf::createQuaternionFromYaw(yawDelta), 
                    tf::Vector3(0, 0, 0));

            odometryEstimation_ = odometryEstimation_ * deltaTransform;

        }

        lastCalibratedImuMessage_ = *imu;

        //
        // Raise angular velocity change event
        //
        angularVelocityUpdatedSignal_();
    }

    void imuFilteredCallback(const sensor_msgs::Imu::Ptr& imu) {


        if (earthOrientation_) {

            odometryEstimation_.setRotation(
                    tf::createQuaternionFromYaw(
                            tf::getYaw(imu->orientation) + yawOffset_));

        }

        lastFilteredImuMessage_ = *imu;

        //
        // Raise angular velocity change event
        //
        angularVelocityUpdatedSignal_();
    }

    void encoderCallback(const lynx_msgs::Encoder::Ptr& encoder) {
        
        if (!encoderInitialized_  || !imuInitialized_) {
            encoderInitialized_ = true;
            // velocityFromEncoders_ = 0;
            lastEncoderTick_ = encoder->ticks;
            lastEncoderUpdateTime_ = ros::Time::now();
            return;
        }

        //
        // Update odometry
        //
        double traveledDistance = (double)(encoder->ticks - lastEncoderTick_) * tickToMeterRatio_;

        double dt = (ros::Time::now() - lastEncoderUpdateTime_).toSec();

        if (dt <= 0) {
            //  ???
            return;
        }

        //
        // Get pitch angle for distance compensation
        //
        double pitch, _;
        
        bool orientationValid = std::abs((lastFilteredImuMessage_.orientation.w * lastFilteredImuMessage_.orientation.w
                + lastFilteredImuMessage_.orientation.x * lastFilteredImuMessage_.orientation.x
                + lastFilteredImuMessage_.orientation.y * lastFilteredImuMessage_.orientation.y
                + lastFilteredImuMessage_.orientation.z * lastFilteredImuMessage_.orientation.z) - 1.0f) 
                        < 10e-3;

        if (orientationValid) {
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(lastFilteredImuMessage_.orientation, orientation);
            tf::Matrix3x3(orientation).getRPY(_, pitch, _);
        } else {
            pitch = 0;
        }

        //
        // Project 3d orientation to 2d
        //
        traveledDistance = cos(pitch) * traveledDistance;

        tf::Transform deltaTransform(tf::Quaternion::getIdentity(), 
                tf::Vector3(traveledDistance, 0, 0));

        odometryEstimation_ *= deltaTransform;

        lastEncoderTick_ = encoder->ticks;
        lastEncoderUpdateTime_ = ros::Time::now();

    }

    void speedEstimateTimerCallback(const ros::TimerEvent&) {
        double distance = tickToMeterRatio_ * (lastEncoderTick_ - speedEstimationLastTick_);
        double speed = distance / (1.0 / speedEstimationRate_) ;

        velocityFromEncoders_ = speedSmoothFactor_ * (velocityFromEncoders_)  + 
                (1.0 - speedSmoothFactor_) * (speed);

        speedEstimationLastTick_ = lastEncoderTick_;

        //
        // Raise update event
        // 
        speedUpdatedSignal_();
    }

    void externalOdometryCallback(const nav_msgs::Odometry::Ptr& odometry) {

        if (lastExternalOdometryMessage_.header.seq == 0) {
            lastExternalOdometryMessage_ = *odometry;
            return;    
        }

        tf::Transform prevPose;
        tf::Transform currentPose;

        tf::poseMsgToTF(lastExternalOdometryMessage_.pose.pose, prevPose);
        tf::poseMsgToTF(odometry->pose.pose, currentPose);

        //
        // Increment odometry
        //

        // One step transform relative to last odom pose
        tf::Transform deltaTransform = prevPose.inverse() * currentPose;

        // Don't use rotation from odometry (IMU orietation is used)
        deltaTransform.setRotation(tf::Quaternion::getIdentity());

        odometryEstimation_ *= deltaTransform;

        if (twoDimMode_) {
            odometryEstimation_.getOrigin().setZ(0);
        }

        //
        // Raise update event
        // 
        speedUpdatedSignal_();

        lastExternalOdometryMessage_ = *odometry;
    }

private:
    
    /**
     * Extract linear velocity from external odometry topic, else, estimate velocity based on encoder readings
     */
    bool velocityFromExternalOdometry_ = false;

    /**
     * IMU (bias calibrated) messages, only angular velocities are used
     */
    ros::Subscriber imuSubscriber_;

    /**
     * Filtered IMU messages (full orientation, earth referenced)
     */
    ros::Subscriber imuFilteredSubscriber_;

    /**
     * Encoder ticks subscriber
     */
    ros::Subscriber encoderSubscriber_;

    /**
     * External odometry subscriber
     * Used to get linear velocity from odometry message
     */
    ros::Subscriber externalOdometrySubscriber_;

    /**
     * Estimated odometry publisher
     */
    ros::Publisher odometryPublisher_;

    /**
     * Global heading angle (radians)
     */
    ros::Publisher orientationEulerPublisher_;

    ros::Timer speedEstimationTimer_;

    long speedEstimationLastTick_ = 0;

    double speedEstimationRate_ = 50.0;
    
    /**
     * Traveled distance of one encoder tick
     */

    double ticksToMeter_ = 221.8; 

    double tickToMeterRatio_ = 1.0 / ticksToMeter_;

    long lastEncoderTick_ = 0;

    double velocityFromEncoders_ = 0;

    /**
     * Weighted average on encoder rotation velocity
     */
    double speedSmoothFactor_ = 0.001;

    ros::Time lastEncoderUpdateTime_;

    bool encoderInitialized_ = false;

    /**
     * If true, z value is ignored
     */
    bool twoDimMode_ = true;

    string odomFrame_;

    string baseFrame_;

    /**
     * Last IMU message with calibrated biases
     */
    sensor_msgs::Imu lastCalibratedImuMessage_;

    /**
     * Last IMU message with full orientation
     */
    sensor_msgs::Imu lastFilteredImuMessage_;

    nav_msgs::Odometry lastExternalOdometryMessage_;

    bool imuInitialized_ = false;

    double yawOffset_;

    /**
     * Estimated position
     */
    tf::Transform odometryEstimation_;

    /**
     * Odometry publish timer
     */
    ros::Timer publishTimer_;

    bool publishTransform_;

    tf::TransformBroadcaster tfPublisher_;

    /**
     * Use orientation earth-referenced orientation (east = 0*)
     */
    bool earthOrientation_;

    boost::signals2::signal<void(void)> speedUpdatedSignal_;

    boost::signals2::signal<void(void)> angularVelocityUpdatedSignal_;

};


} /* namespace tools */
} /* namespace lynx */



