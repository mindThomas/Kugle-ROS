/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "LSPC.h"
#include "MessageTypes.h"
#include <string>
#include <thread>
#include <boost/thread/recursive_mutex.hpp>
#include <future>
#include <boost/bind.hpp>
#include <fstream>      // for file output (std::ofstream)
#include <boost/filesystem.hpp>   // for directory creation (boost::filesystem)
#include <chrono>
#include <cstdint>
#include <tuple>
#include <cmath>

#include "Queue.hpp"
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <locale>
#include <stdio.h>
#include <stdlib.h>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
/*#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>*/
#include <geometry_msgs/TransformStamped.h>

/* Include messages */
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

/* Include generated Services */
#include <kugle_srvs/SetParameter.h>
#include <kugle_srvs/GetParameter.h>
#include <kugle_srvs/DumpParameters.h>
#include <kugle_srvs/StoreParameters.h>
#include <kugle_srvs/CalibrateIMU.h>
#include <kugle_srvs/CalibrateAccelerometer.h>
#include <kugle_srvs/Reboot.h>
#include <kugle_srvs/EnterBootloader.h>
#include <kugle_srvs/RestartController.h>

/* Include generated Message Types */
#include <kugle_msgs/ControllerInfo.h>
#include <kugle_msgs/Encoders.h>
#include <kugle_msgs/StateEstimate.h>
#include <kugle_msgs/Euler.h>
#include <kugle_msgs/Vector2.h>
#include <kugle_msgs/ControllerDebug.h>
#include <kugle_msgs/BalanceControllerReference.h>

/* Include generated Dynamic Reconfigure parameters */
#include <kugle_driver/ParametersConfig.h>

// For CLion to update/capture the changes made to generated services, message types and parameters, open the "build" folder and run "make"

/* Global variables */
Queue<std::tuple<lspc::ParameterLookup::type_t, uint8_t, bool>> SetParameterResponse;
Queue<std::shared_ptr<std::vector<uint8_t>>> GetParameterResponse;
Queue<std::shared_ptr<std::vector<uint8_t>>> DumpParametersResponse;
Queue<bool> StoreParametersResponse;
Queue<bool> CalibrateIMUResponse;
Queue<bool> RestartControllerResponse;

lspc::ParameterTypes::controllerMode_t currentControllerMode = lspc::ParameterTypes::UNKNOWN_MODE;

std::mutex reconfigureMutex;
std::shared_ptr<dynamic_reconfigure::Server<kugle_driver::ParametersConfig>> reconfigureServer;
kugle_driver::ParametersConfig reconfigureConfig;

bool MATLAB_log = false;

void LSPC_Callback_StateEstimates(ros::Publisher& pubOdom, ros::Publisher& pubStateEstimate, tf::TransformBroadcaster& tfBroadcaster, std::shared_ptr<tf::TransformListener> tfListener, const std::vector<uint8_t>& payload)
{
    const lspc::MessageTypesToPC::StateEstimates_t * msg = reinterpret_cast<const lspc::MessageTypesToPC::StateEstimates_t *>(payload.data());
    if (sizeof(*msg) != payload.size()) {
        ROS_DEBUG("Error parsing StateEstimates message");
        return;
    }

    tf::Quaternion q_attitude;
    q_attitude.setW(msg->q.w);
    q_attitude.setX(msg->q.x);
    q_attitude.setY(msg->q.y);
    q_attitude.setZ(msg->q.z);

    tf::Quaternion dq;
    dq.setW(msg->dq.w);
    dq.setX(msg->dq.x);
    dq.setY(msg->dq.y);
    dq.setZ(msg->dq.z);

    tf::Transform attitudeTf;
    attitudeTf.setIdentity();
    attitudeTf.setRotation(q_attitude);

    /* Lookup static transform between contact_point to ball (will be the height/z-offset from contact point to center of ball */
    tf::Vector3 ball_offset(0,0,0);
    tf::StampedTransform transformStatic;
    try{
        tfListener->lookupTransform("contact_point", "ball", ros::Time(0), transformStatic);
        ball_offset.setX(transformStatic.getOrigin().x());
        ball_offset.setY(transformStatic.getOrigin().y());
        ball_offset.setZ(transformStatic.getOrigin().z());
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("%s",ex.what());
    }

    /* Send transfrom from "odom" frame to "contact_point" frame */
    geometry_msgs::TransformStamped tf_contact_point_msg;
    tf_contact_point_msg.header.frame_id = "odom";
    tf_contact_point_msg.child_frame_id = "contact_point";
    tf_contact_point_msg.header.stamp = ros::Time::now();
    tf_contact_point_msg.transform.translation.x = msg->pos.x; // inertial frame position
    tf_contact_point_msg.transform.translation.y = msg->pos.y;
    tf_contact_point_msg.transform.translation.z = 0.0;
    tf_contact_point_msg.transform.rotation.w = 1;
    tf_contact_point_msg.transform.rotation.x = 0;
    tf_contact_point_msg.transform.rotation.y = 0;
    tf_contact_point_msg.transform.rotation.z = 0;
    tfBroadcaster.sendTransform(tf_contact_point_msg);

    /* Compute heading from received attitude quaternion */
    // We define the heading angle as the angle to the projection of the body (base_link) x-axis, projected down to the 2D plane in the inertial frame
    tf::Vector3 xVector = attitudeTf(tf::Vector3(1,0,0));
    float heading = atan2(xVector.y(), xVector.x());
    tf::Quaternion q_heading = tf::createQuaternionFromYaw(heading); // rotate around z-axis

    /* Send transfrom from "ball" frame to "heading" frame */
    geometry_msgs::TransformStamped tf_heading_msg;
    tf_heading_msg.header.frame_id = "ball";
    tf_heading_msg.child_frame_id = "heading";
    tf_heading_msg.header.stamp = ros::Time::now();
    tf_heading_msg.transform.translation.x = 0;
    tf_heading_msg.transform.translation.y = 0;
    tf_heading_msg.transform.translation.z = 0.0;
    tf_heading_msg.transform.rotation.w = q_heading.w();
    tf_heading_msg.transform.rotation.x = q_heading.x();
    tf_heading_msg.transform.rotation.y = q_heading.y();
    tf_heading_msg.transform.rotation.z = q_heading.z();
    tfBroadcaster.sendTransform(tf_heading_msg);

    /* Compute roll/pitch quaternion which represents the tilt in heading frame */
    // Remember quaternions are multipled from left to right, start with Unit quaternion all the way from left and then right multiplying
    // The combined attitude quaternion is constructed as:
    //   q = q_heading o q_tilt
    //   Since we need to first rotate into heading frame and then tilt from there
    // Hence the tilt quaternion is compute as:
    //   q_tilt = inv(q_heading) o q
    tf::Quaternion tiltQuaternion = q_heading.inverse() * q_attitude;

    /* Send transfrom from "heading" frame to "base_link" frame */
    geometry_msgs::TransformStamped tf_tilt_msg;
    tf_tilt_msg.header.frame_id = "heading";
    tf_tilt_msg.child_frame_id = "base_link";
    tf_tilt_msg.header.stamp = ros::Time::now();
    tf_tilt_msg.transform.translation.x = 0;
    tf_tilt_msg.transform.translation.y = 0;
    tf_tilt_msg.transform.translation.z = 0.0;
    tf_tilt_msg.transform.rotation.w = tiltQuaternion.w();
    tf_tilt_msg.transform.rotation.x = tiltQuaternion.x();
    tf_tilt_msg.transform.rotation.y = tiltQuaternion.y();
    tf_tilt_msg.transform.rotation.z = tiltQuaternion.z();
    tfBroadcaster.sendTransform(tf_tilt_msg);

    /* Compute angular velocity from quaternion derivative */
    // We need the angular velocity in inertial frame
    // dq = 1/2 * q o q_omega_body
    // q_omega_body = 2*inv(q) o dq
    tf::Quaternion q_omega_body = q_attitude.inverse() * dq * 2;

    /* Send odometry message */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = msg->pos.x; // inertial frame position
    odom_msg.pose.pose.position.y = msg->pos.y;
    odom_msg.pose.pose.position.z = ball_offset.z();
    odom_msg.pose.pose.orientation.w = q_attitude.w();
    odom_msg.pose.pose.orientation.x = q_attitude.x();
    odom_msg.pose.pose.orientation.y = q_attitude.y();
    odom_msg.pose.pose.orientation.z = q_attitude.z();
    odom_msg.twist.twist.linear.x = msg->vel.x; // inertial frame velocity
    odom_msg.twist.twist.linear.y = msg->vel.y;
    odom_msg.twist.twist.angular.x = q_omega_body.x();
    odom_msg.twist.twist.angular.y = q_omega_body.y();
    odom_msg.twist.twist.angular.z = q_omega_body.z();
    pubOdom.publish(odom_msg);

    /* Send state estimate message */
    kugle_msgs::StateEstimate stateEstimate_msg;
    stateEstimate_msg.receive_time = ros::Time::now();
    stateEstimate_msg.mcu_time = msg->time;
    stateEstimate_msg.q.w = msg->q.w;
    stateEstimate_msg.q.x = msg->q.x;
    stateEstimate_msg.q.y = msg->q.y;
    stateEstimate_msg.q.z = msg->q.z;
    stateEstimate_msg.dq.w = msg->dq.w;
    stateEstimate_msg.dq.x = msg->dq.x;
    stateEstimate_msg.dq.y = msg->dq.y;
    stateEstimate_msg.dq.z = msg->dq.z;
    stateEstimate_msg.position[0] = msg->pos.x;
    stateEstimate_msg.position[1] = msg->pos.y;
    stateEstimate_msg.velocity[0] = msg->vel.x;
    stateEstimate_msg.velocity[1] = msg->vel.y;
    pubStateEstimate.publish(stateEstimate_msg);
}

void LSPC_Callback_SystemInfo(const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("System info received");
    const lspc::MessageTypesToPC::SystemInfo_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::SystemInfo_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing SystemInfo message");
        return;
    }
}

std::string ParseControllerType(lspc::ParameterTypes::controllerType_t type)
{
    if (type == lspc::ParameterTypes::LQR_CONTROLLER) return "LQR_CONTROLLER";
    else if (type == lspc::ParameterTypes::SLIDING_MODE_CONTROLLER) return "SLIDING_MODE_CONTROLLER";
    else return "UNKNOWN_CONTROLLER";
}

lspc::ParameterTypes::controllerType_t ParseControllerType2(std::string type)
{
    if (!type.compare("LQR_CONTROLLER")) return lspc::ParameterTypes::LQR_CONTROLLER;
    else if (!type.compare("SLIDING_MODE_CONTROLLER")) return lspc::ParameterTypes::SLIDING_MODE_CONTROLLER;
    else return lspc::ParameterTypes::UNKNOWN_CONTROLLER;
}

std::string ParseControllerMode(lspc::ParameterTypes::controllerMode_t mode)
{
    if (mode == lspc::ParameterTypes::OFF) return "OFF";
    else if (mode == lspc::ParameterTypes::QUATERNION_CONTROL) return "QUATERNION_CONTROL";
    else if (mode == lspc::ParameterTypes::VELOCITY_CONTROL) return "VELOCITY_CONTROL";
    else if (mode == lspc::ParameterTypes::PATH_FOLLOWING) return "PATH_FOLLOWING";
    else return "UNKNOWN_MODE";
}

lspc::ParameterTypes::controllerMode_t ParseControllerMode2(std::string mode)
{
    if (!mode.compare("OFF")) return lspc::ParameterTypes::OFF;
    else if (!mode.compare("QUATERNION_CONTROL")) return lspc::ParameterTypes::QUATERNION_CONTROL;
    else if (!mode.compare("VELOCITY_CONTROL")) return lspc::ParameterTypes::VELOCITY_CONTROL;
    else if (!mode.compare("PATH_FOLLOWING")) return lspc::ParameterTypes::PATH_FOLLOWING;
    else return lspc::ParameterTypes::UNKNOWN_MODE;
}

lspc::ParameterTypes::slidingManifoldType_t ParseManifoldType2(std::string type)
{
    if (!type.compare("Q_DOT_INERTIAL_MANIFOLD")) return lspc::ParameterTypes::Q_DOT_INERTIAL_MANIFOLD;
    else if (!type.compare("Q_DOT_BODY_MANIFOLD")) return lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD;
    else if (!type.compare("OMEGA_INERTIAL_MANIFOLD")) return lspc::ParameterTypes::OMEGA_INERTIAL_MANIFOLD;
    else if (!type.compare("OMEGA_BODY_MANIFOLD")) return lspc::ParameterTypes::OMEGA_BODY_MANIFOLD;
    else return lspc::ParameterTypes::UNKNOWN_MANIFOLD;
}

std::string ParseManifoldType(lspc::ParameterTypes::slidingManifoldType_t type)
{
    if (type == lspc::ParameterTypes::Q_DOT_INERTIAL_MANIFOLD) return "Q_DOT_INERTIAL_MANIFOLD";
    else if (type == lspc::ParameterTypes::Q_DOT_BODY_MANIFOLD) return "Q_DOT_BODY_MANIFOLD";
    else if (type == lspc::ParameterTypes::OMEGA_INERTIAL_MANIFOLD) return "OMEGA_INERTIAL_MANIFOLD";
    else if (type == lspc::ParameterTypes::OMEGA_BODY_MANIFOLD) return "OMEGA_BODY_MANIFOLD";
    else return "UNKNOWN_MANIFOLD";
}

std::string ParsePowerButtonMode(lspc::ParameterTypes::powerButtonMode_t mode)
{
    if (mode == lspc::ParameterTypes::POWER_OFF) return "POWER_OFF";
    else if (mode == lspc::ParameterTypes::START_STOP_QUATERNION_CONTROLLER) return "START_STOP_QUATERNION_CONTROLLER";
    else if (mode == lspc::ParameterTypes::START_STOP_VELOCITY_CONTROLLER) return "START_STOP_VELOCITY_CONTROLLER";
    else return "UNKNOWN_BUTTON_MODE";
}

lspc::ParameterTypes::powerButtonMode_t ParsePowerButtonMode2(std::string mode)
{
    if (!mode.compare("POWER_OFF")) return lspc::ParameterTypes::POWER_OFF;
    else if (!mode.compare("START_STOP_QUATERNION_CONTROLLER")) return lspc::ParameterTypes::START_STOP_QUATERNION_CONTROLLER;
    else if (!mode.compare("START_STOP_VELOCITY_CONTROLLER")) return lspc::ParameterTypes::START_STOP_VELOCITY_CONTROLLER;
    else return lspc::ParameterTypes::UNKNOWN_BUTTON_MODE;
}


void LSPC_Callback_ControllerInfo(ros::Publisher& pubControllerInfo, const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("Controller info received");
    const lspc::MessageTypesToPC::ControllerInfo_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::ControllerInfo_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing ControllerInfo message");
        return;
    }

    currentControllerMode = msgRaw->mode;

    kugle_msgs::ControllerInfo msg;
    msg.receive_time = ros::Time::now();
    msg.type = ParseControllerType(msgRaw->type);
    msg.mode = ParseControllerMode(msgRaw->mode);
    msg.mcu_time = msgRaw->time;
    msg.compute_time = msgRaw->compute_time;
    msg.torque[0] = msgRaw->torque1;
    msg.torque[1] = msgRaw->torque2;
    msg.torque[2] = msgRaw->torque3;
    msg.delivered_torque[0] = msgRaw->delivered_torque1;
    msg.delivered_torque[1] = msgRaw->delivered_torque2;
    msg.delivered_torque[2] = msgRaw->delivered_torque3;
    pubControllerInfo.publish(msg);

    if (reconfigureMutex.try_lock()) {
        if (reconfigureConfig.type != msgRaw->type || reconfigureConfig.mode != msgRaw->mode) {
            reconfigureConfig.type = int(msgRaw->type);
            reconfigureConfig.mode = int(msgRaw->mode);
            reconfigureServer->updateConfig(reconfigureConfig);
        }
        reconfigureMutex.unlock();
    }
}

void LSPC_Callback_ControllerDebug(ros::Publisher& pubControllerDebug, const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("Controller debug received");
    const lspc::MessageTypesToPC::ControllerDebug_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::ControllerDebug_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing ControllerDebug message");
        return;
    }

    kugle_msgs::ControllerDebug msg;
    msg.receive_time = ros::Time::now();
    msg.mcu_time = msgRaw->time;

    msg.orient.roll = msgRaw->orient.roll;
    msg.orient.pitch = msgRaw->orient.pitch;
    msg.orient.yaw = msgRaw->orient.yaw;

    msg.orient_ref.roll = msgRaw->orient_ref.roll;
    msg.orient_ref.pitch = msgRaw->orient_ref.pitch;
    msg.orient_ref.yaw = msgRaw->orient_ref.yaw;

    msg.orient_integral.roll = msgRaw->orient_integral.roll;
    msg.orient_integral.pitch = msgRaw->orient_integral.pitch;
    msg.orient_integral.yaw = msgRaw->orient_integral.yaw;

    msg.omega.x = msgRaw->omega.x;
    msg.omega.y = msgRaw->omega.y;
    msg.omega.z = msgRaw->omega.z;

    msg.omega_ref.x = msgRaw->omega_ref.x;
    msg.omega_ref.y = msgRaw->omega_ref.y;
    msg.omega_ref.z = msgRaw->omega_ref.z;

    msg.vel.x = msgRaw->vel.x;
    msg.vel.y = msgRaw->vel.y;

    msg.vel_kinematics.x = msgRaw->vel_kinematics.x;
    msg.vel_kinematics.y = msgRaw->vel_kinematics.y;

    msg.vel_ref.x = msgRaw->vel_ref.x;
    msg.vel_ref.y = msgRaw->vel_ref.y;

    msg.torque[0] = msgRaw->torque[0];
    msg.torque[1] = msgRaw->torque[1];
    msg.torque[2] = msgRaw->torque[2];

    msg.S[0] = msgRaw->S[0];
    msg.S[1] = msgRaw->S[1];
    msg.S[2] = msgRaw->S[2];

    pubControllerDebug.publish(msg);
}

void LSPC_Callback_RawSensor_IMU_MPU9250(ros::Publisher& pubIMU, ros::Publisher& pubMag, const std::vector<uint8_t>& payload)
{
    const lspc::MessageTypesToPC::RawSensor_IMU_MPU9250_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::RawSensor_IMU_MPU9250_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing RawSensor_IMU_MPU9250 message");
        return;
    }

    sensor_msgs::Imu msg;

    msg.header.stamp = ros::Time::now(); // consider to use msgRaw->time   ???

    /* Raw orientation not provided by IMU */
    msg.orientation.w = 0;
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation_covariance[0] = -1;

    /* Insert accelerometer values */
    msg.linear_acceleration.x = msgRaw->accelerometer.x;
    msg.linear_acceleration.y = msgRaw->accelerometer.y;
    msg.linear_acceleration.z = msgRaw->accelerometer.z;
    for (int i = 0; i < 9; i++) msg.linear_acceleration_covariance[i] = msgRaw->accelerometer.cov[i];

    /* Insert gyroscope values */
    msg.angular_velocity.x = msgRaw->gyroscope.x;
    msg.angular_velocity.y = msgRaw->gyroscope.y;
    msg.angular_velocity.z = msgRaw->gyroscope.z;
    for (int i = 0; i < 9; i++) msg.angular_velocity_covariance[i] = msgRaw->gyroscope.cov[i];

    /* Insert magnetometer values */
    sensor_msgs::MagneticField msgMag;
    msgMag.header.stamp = msg.header.stamp;
    msgMag.magnetic_field.x = msgRaw->magnetometer.x;
    msgMag.magnetic_field.y = msgRaw->magnetometer.y;
    msgMag.magnetic_field.z = msgRaw->magnetometer.z;
    for (int i = 0; i < 9; i++) msgMag.magnetic_field_covariance[i] = msgRaw->magnetometer.cov[i];

    pubIMU.publish(msg);
    pubMag.publish(msgMag);
}

void LSPC_Callback_RawSensor_IMU_MTI200(const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("Received Raw Sensor - IMU MTI200");
}

void LSPC_Callback_RawSensor_Battery(ros::Publisher& pubBattery, const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("Received Raw Sensor - Battery");

    sensor_msgs::BatteryState msg;
    /* ToDo: Fill battery info into msg */
    //pubBattery.publish(msg);
}

void LSPC_Callback_RawSensor_Encoders(ros::Publisher& pubEncoders, const std::vector<uint8_t>& payload)
{
    const lspc::MessageTypesToPC::RawSensor_Encoders_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::RawSensor_Encoders_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing RawSensor_Encoders message");
        return;
    }

    kugle_msgs::Encoders msg;
    msg.receive_time = ros::Time::now();
    msg.mcu_time = msgRaw->time;
    msg.angle_wheel1 = msgRaw->angle1;
    msg.angle_wheel2 = msgRaw->angle2;
    msg.angle_wheel3 = msgRaw->angle3;
    pubEncoders.publish(msg);
}

void LSPC_Callback_CPUload(ros::Publisher& pubLoad, const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Microprocessor CPU Load\n" << message);

    /* Publish data to topic as well */
    std_msgs::String msg;
    msg.data = "\n" + message;
    pubLoad.publish(msg);
}

void LSPC_Callback_ArrayDump(std::shared_ptr<std::ofstream> log_file, const std::vector<uint8_t>& payload)
{
    int numberOfFloats = payload.size() / 4;
    const float * floatArray = reinterpret_cast<const float *>(payload.data());

    if (log_file->is_open()) {
        for (int i = 0; i < numberOfFloats; i++) {
            if (i > 0) *log_file << "\t";
            *log_file << std::setprecision(10) << floatArray[i];
        }
        *log_file << std::endl;
    }
}

void LSPC_Callback_Debug(ros::Publisher& pubDebug, const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Debug message:\n" << message);

    /* Publish data to topic as well */
    std_msgs::String msg;
    msg.data = "\n" + message;
    pubDebug.publish(msg);
}

long int GetCurrentMicroseconds()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int us = tp.tv_sec * 1000000 + tp.tv_usec;
    return us;
}

long int GetCurrentMicroseconds2()
{
    std::chrono::microseconds us = std::chrono::duration_cast< std::chrono::microseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );

    return us.count();
}

std::string GetFormattedTimestampCurrent()
{
    std::string output;
    std::stringstream ss;

    int64_t utime = GetCurrentMicroseconds();

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

void ROS_Callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (currentControllerMode != lspc::ParameterTypes::VELOCITY_CONTROL) return; // no need to send message over since we are not in mode where the message is used
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        //ROS_DEBUG_STREAM("Sending cmd_vel to Kugle as both VelocityReference_Heading and AngularVelocityReference_Body");
        lspc::MessageTypesFromPC::VelocityReference_t payload;
        payload.frame = lspc::ParameterTypes::HEADING_FRAME;
        payload.vel.x = msg->linear.x;
        payload.vel.y = msg->linear.y;
        payload.vel.yaw = msg->angular.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference, payloadPacked);

        lspc::MessageTypesFromPC::AngularVelocityReference_t payload2;
        payload2.frame = lspc::ParameterTypes::BODY_FRAME;
        payload2.omega.x = msg->angular.x;
        payload2.omega.y = msg->angular.y;
        payload2.omega.z = msg->angular.z;
        std::vector<uint8_t> payloadPacked2((uint8_t *)&payload2, (uint8_t *)&payload2+sizeof(payload2)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::AngularVelocityReference, payloadPacked2);
    }

    lspcMutex->unlock();
}

void ROS_Callback_cmd_vel_inertial(const geometry_msgs::Twist::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (currentControllerMode != lspc::ParameterTypes::VELOCITY_CONTROL) return; // no need to send message over since we are not in mode where the message is used
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        //ROS_DEBUG_STREAM("Sending cmd_vel_inertial to Kugle as both VelocityReference_Inertial and AngularVelocityReference_Inertial");
        lspc::MessageTypesFromPC::VelocityReference_t payload;
        payload.frame = lspc::ParameterTypes::INERTIAL_FRAME;
        payload.vel.x = msg->linear.x;
        payload.vel.y = msg->linear.y;
        payload.vel.yaw = msg->angular.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference, payloadPacked);

        lspc::MessageTypesFromPC::AngularVelocityReference_t payload2;
        payload2.frame = lspc::ParameterTypes::INERTIAL_FRAME;
        payload2.omega.x = msg->angular.x;
        payload2.omega.y = msg->angular.y;
        payload2.omega.z = msg->angular.z;
        std::vector<uint8_t> payloadPacked2((uint8_t *)&payload2, (uint8_t *)&payload2+sizeof(payload2)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::AngularVelocityReference, payloadPacked2);
    }

    lspcMutex->unlock();
}

void ROS_Callback_cmd_quaternion(const geometry_msgs::Quaternion::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (currentControllerMode != lspc::ParameterTypes::QUATERNION_CONTROL) return; // no need to send message over since we are not in mode where the message is used
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        //ROS_DEBUG_STREAM("Sending cmd_quaternion to Kugle");
        lspc::MessageTypesFromPC::QuaternionReference_t payload;
        payload.q.w = msg->w;
        payload.q.x = msg->x;
        payload.q.y = msg->y;
        payload.q.z = msg->z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::QuaternionReference, payloadPacked);
    }

    lspcMutex->unlock();
}

void ROS_Callback_cmd_combined(const kugle_msgs::BalanceControllerReference::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (currentControllerMode != lspc::ParameterTypes::QUATERNION_CONTROL) return; // no need to send message over since we are not in mode where the message is used
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        //ROS_DEBUG_STREAM("Sending cmd_quaternion to Kugle");
        lspc::MessageTypesFromPC::BalanceControllerReference_t payload;
        payload.frame = lspc::ParameterTypes::BODY_FRAME;
        payload.q.w = msg->q.w;
        payload.q.x = msg->q.x;
        payload.q.y = msg->q.y;
        payload.q.z = msg->q.z;
        payload.omega.x = msg->omega.x;
        payload.omega.y = msg->omega.y;
        payload.omega.z = msg->omega.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::BalanceControllerReference, payloadPacked);
    }

    lspcMutex->unlock();
}

void ROS_Callback_cmd_combined_inertial(const kugle_msgs::BalanceControllerReference::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (currentControllerMode != lspc::ParameterTypes::QUATERNION_CONTROL) return; // no need to send message over since we are not in mode where the message is used
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        //ROS_DEBUG_STREAM("Sending cmd_quaternion to Kugle");
        lspc::MessageTypesFromPC::BalanceControllerReference_t payload;
        payload.frame = lspc::ParameterTypes::INERTIAL_FRAME;
        payload.q.w = msg->q.w;
        payload.q.x = msg->q.x;
        payload.q.y = msg->q.y;
        payload.q.z = msg->q.z;
        payload.omega.x = msg->omega.x;
        payload.omega.y = msg->omega.y;
        payload.omega.z = msg->omega.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::BalanceControllerReference, payloadPacked);
    }

    lspcMutex->unlock();
}

bool ParseParamTypeAndID(const std::string in_type, const std::string in_param, lspc::ParameterLookup::type_t& out_type, uint8_t& out_param, lspc::ParameterLookup::ValueType_t& out_valueType, uint8_t& out_arraySize)
{
    if (!in_type.compare("debug")) out_type = lspc::ParameterLookup::debug;
    else if (!in_type.compare("behavioural")) out_type = lspc::ParameterLookup::behavioural;
    else if (!in_type.compare("controller")) out_type = lspc::ParameterLookup::controller;
    else if (!in_type.compare("estimator")) out_type = lspc::ParameterLookup::estimator;
    else if (!in_type.compare("model")) out_type = lspc::ParameterLookup::model;
    else if (!in_type.compare("test")) out_type = lspc::ParameterLookup::test;
    else {
        ROS_DEBUG("Parameter lookup: Type not found");
        return false; // not found
    }

    /* Arrays not supported yet, so set array size to 1 always */
    out_arraySize = 1;

    /* Parse parameter text */
    if (out_type == lspc::ParameterLookup::debug) {
        if (!in_param.compare("EnableDumpMessages")) {
            out_param = lspc::ParameterLookup::EnableDumpMessages;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EnableRawSensorOutput")) {
            out_param = lspc::ParameterLookup::EnableRawSensorOutput;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("UseFilteredIMUinRawSensorOutput")) {
            out_param = lspc::ParameterLookup::UseFilteredIMUinRawSensorOutput;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("DisableMotorOutput")) {
            out_param = lspc::ParameterLookup::DisableMotorOutput;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else {
            ROS_DEBUG("Parameter lookup: Parameter not found");
            return false;
        }
    }

    else if (out_type == lspc::ParameterLookup::behavioural) {
        if (!in_param.compare("IndependentHeading")) {
            out_param = lspc::ParameterLookup::IndependentHeading;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("YawVelocityBraking")) {
            out_param = lspc::ParameterLookup::YawVelocityBraking;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("StepTestEnabled")) {
            out_param = lspc::ParameterLookup::StepTestEnabled;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("SineTestEnabled")) {
            out_param = lspc::ParameterLookup::SineTestEnabled;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("PowerButtonMode")) {
            out_param = lspc::ParameterLookup::PowerButtonMode;
            out_valueType = lspc::ParameterLookup::_uint8;
        }
        else {
            ROS_DEBUG("Parameter lookup: Parameter not found");
            return false;
        }
    }

    else if (out_type == lspc::ParameterLookup::controller) {
        if (!in_param.compare("type")) {
            out_param = lspc::ParameterLookup::type;
            out_valueType = lspc::ParameterLookup::_uint8;
        }
        else if (!in_param.compare("mode")) {
            out_param = lspc::ParameterLookup::mode;
            out_valueType = lspc::ParameterLookup::_uint8;
        }
        else if (!in_param.compare("EnableTorqueLPF")) {
            out_param = lspc::ParameterLookup::EnableTorqueLPF;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("ManifoldType")) {
            out_param = lspc::ParameterLookup::ManifoldType;
            out_valueType = lspc::ParameterLookup::_uint8;
        }
        else if (!in_param.compare("ContinousSwitching")) {
            out_param = lspc::ParameterLookup::ContinousSwitching;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EquivalentControl")) {
            out_param = lspc::ParameterLookup::EquivalentControl;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("eta")) {
            out_param = lspc::ParameterLookup::eta;
            out_valueType = lspc::ParameterLookup::_float;
            out_arraySize = 3;
        }
        else if (!in_param.compare("epsilon")) {
            out_param = lspc::ParameterLookup::epsilon;
            out_valueType = lspc::ParameterLookup::_float;
            out_arraySize = 3;
        }
        else if (!in_param.compare("K")) {
            out_param = lspc::ParameterLookup::K;
            out_valueType = lspc::ParameterLookup::_float;
            out_arraySize = 3;
        }
        else if (!in_param.compare("Kx")) {
            out_param = lspc::ParameterLookup::Kx;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Ky")) {
            out_param = lspc::ParameterLookup::Ky;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Kz")) {
            out_param = lspc::ParameterLookup::Kz;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("DisableQdot")) {
            out_param = lspc::ParameterLookup::DisableQdot;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("DisableQdotInEquivalentControl")) {
            out_param = lspc::ParameterLookup::DisableQdotInEquivalentControl;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("VelocityController_AccelerationLimit")) {
            out_param = lspc::ParameterLookup::VelocityController_AccelerationLimit;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("VelocityController_MaxTilt")) {
            out_param = lspc::ParameterLookup::VelocityController_MaxTilt;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("VelocityController_MaxIntegralCorrection")) {
            out_param = lspc::ParameterLookup::VelocityController_MaxIntegralCorrection;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("VelocityController_VelocityClamp")) {
            out_param = lspc::ParameterLookup::VelocityController_VelocityClamp;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("VelocityController_IntegralGain")) {
            out_param = lspc::ParameterLookup::VelocityController_IntegralGain;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else {
            ROS_DEBUG("Parameter lookup: Parameter not found");
            return false;
        }
    }

    else if (out_type == lspc::ParameterLookup::estimator) {
        if (!in_param.compare("EnableSensorLPFfilters")) {
            out_param = lspc::ParameterLookup::EnableSensorLPFfilters;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EnableSoftwareLPFfilters")) {
            out_param = lspc::ParameterLookup::EnableSoftwareLPFfilters;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("UseMadgwick")) {
            out_param = lspc::ParameterLookup::UseMadgwick;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EstimateBias")) {
            out_param = lspc::ParameterLookup::EstimateBias;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("UseVelocityEstimator")) {
            out_param = lspc::ParameterLookup::UseVelocityEstimator;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("UseCoRvelocity")) {
            out_param = lspc::ParameterLookup::UseCoRvelocity;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EnableVelocityLPF")) {
            out_param = lspc::ParameterLookup::EnableVelocityLPF;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EstimateCOM")) {
            out_param = lspc::ParameterLookup::EstimateCOM;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("UseCOMestimateInVelocityEstimator")) {
            out_param = lspc::ParameterLookup::UseCOMestimateInVelocityEstimator;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("sigma2_bias")) {
            out_param = lspc::ParameterLookup::sigma2_bias;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("sigma2_omega")) {
            out_param = lspc::ParameterLookup::sigma2_omega;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("sigma2_heading")) {
            out_param = lspc::ParameterLookup::sigma2_heading;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("GyroscopeTrustFactor")) {
            out_param = lspc::ParameterLookup::GyroscopeTrustFactor;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else {
            ROS_DEBUG("Parameter lookup: Parameter not found");
            return false;
        }
    }

    else if (out_type == lspc::ParameterLookup::model) {
        if (!in_param.compare("l")) {
            out_param = lspc::ParameterLookup::l;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("CoR")) {
            out_param = lspc::ParameterLookup::CoR;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Mk")) {
            out_param = lspc::ParameterLookup::Mk;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Mb")) {
            out_param = lspc::ParameterLookup::Mb;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Bvk")) {
            out_param = lspc::ParameterLookup::Bvk;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Bvm")) {
            out_param = lspc::ParameterLookup::Bvm;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Bvb")) {
            out_param = lspc::ParameterLookup::Bvb;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("SaturationTorqueOfMaxOutputTorque")) {
            out_param = lspc::ParameterLookup::SaturationTorqueOfMaxOutputTorque;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else {
            ROS_DEBUG("Parameter lookup: Parameter not found");
            return false;
        }
    }

    else if (out_type == lspc::ParameterLookup::test) {
        if (!in_param.compare("tmp")) {
            out_param = lspc::ParameterLookup::tmp;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("tmp2")) {
            out_param = lspc::ParameterLookup::tmp2;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else {
            ROS_DEBUG("Parameter lookup: Parameter not found");
            return false;
        }
    }

    return true;
}

bool ROS_Service_SetParameter(kugle_srvs::SetParameter::Request &req, kugle_srvs::SetParameter::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;

    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    /* Determine parameter type and param ID */
    lspc::MessageTypesFromPC::SetParameter_t msg;
    msg.valueType = lspc::ParameterLookup::_unknown;
    if (!ParseParamTypeAndID(req.type, req.param, msg.type, msg.param, msg.valueType, msg.arraySize)) {
        lspcMutex->unlock();
        return true; // parameter not found
    }

    /* Prepare header part of set parameter message */
    std::vector<uint8_t> payloadPacked((uint8_t *)&msg, (uint8_t *)&msg+sizeof(msg)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)

    std::istringstream iss(req.value);
    std::vector<std::string> values((std::istream_iterator<std::string>(iss)),  // split string by spaces
                                     std::istream_iterator<std::string>());

    /* Parse parameter value based on type - currently this does not support arrays! */
    for (unsigned int i = 0; i < msg.arraySize; i++) {
        std::string valueString = values.at(std::min<unsigned int>(i, values.size()-1));

        if (msg.valueType == lspc::ParameterLookup::_bool) {
            if (!valueString.compare("true"))
                payloadPacked.push_back(1);
            else if (!valueString.compare("false"))
                payloadPacked.push_back(0);
            else {
                lspcMutex->unlock();
                return true; // value could not be parsed
            }
        } else if (msg.valueType == lspc::ParameterLookup::_uint8) {
            uint8_t value;
            try {
                value = uint8_t(std::stoi(valueString));
            }
            catch (std::invalid_argument &e) {   // value could not be parsed (eg. is not a number)
                // input could be a string that should be converted
                if (msg.type == lspc::ParameterLookup::controller && msg.param == lspc::ParameterLookup::mode) {
                    value = ParseControllerMode2(valueString);
                    if (value == lspc::ParameterTypes::UNKNOWN_MODE) {
                        lspcMutex->unlock();
                        return true;
                    }
                } else if (msg.type == lspc::ParameterLookup::controller && msg.param == lspc::ParameterLookup::type) {
                    value = ParseControllerType2(valueString);
                    if (value == lspc::ParameterTypes::UNKNOWN_CONTROLLER) {
                        lspcMutex->unlock();
                        return true;
                    }
                } else if (msg.type == lspc::ParameterLookup::controller && msg.param == lspc::ParameterLookup::ManifoldType) {
                    value = ParseManifoldType2(valueString);
                    if (value == lspc::ParameterTypes::UNKNOWN_MANIFOLD) {
                        lspcMutex->unlock();
                        return true;
                    }
                } else if (msg.type == lspc::ParameterLookup::behavioural && msg.param == lspc::ParameterLookup::PowerButtonMode) {
                    value = ParsePowerButtonMode2(valueString);
                    if (value == lspc::ParameterTypes::UNKNOWN_BUTTON_MODE) {
                        lspcMutex->unlock();
                        return true;
                    }
                } else { // not a parameter where we expect a string
                    lspcMutex->unlock();
                    return true;
                }
            }
            catch (std::out_of_range &e) {
                lspcMutex->unlock();
                return true;
            }
            payloadPacked.push_back(value);
        } else if (msg.valueType == lspc::ParameterLookup::_uint16) {
            uint16_t value;
            try {
                value = uint16_t(std::stoi(valueString));
            }
            catch (std::invalid_argument &e) {
                lspcMutex->unlock();
                return true;
            }  // value could not be parsed (eg. is not a number)
            catch (std::out_of_range &e) {
                lspcMutex->unlock();
                return true;
            }
            uint8_t *valuePtr = reinterpret_cast<uint8_t *>(&value);
            payloadPacked.push_back(valuePtr[0]);
            payloadPacked.push_back(valuePtr[1]);
        } else if (msg.valueType == lspc::ParameterLookup::_uint32) {
            uint32_t value;
            try {
                value = uint32_t(std::stol(valueString));
            }
            catch (std::invalid_argument &e) {
                lspcMutex->unlock();
                return true;
            }  // value could not be parsed (eg. is not a number)
            catch (std::out_of_range &e) {
                lspcMutex->unlock();
                return true;
            }
            uint8_t *valuePtr = reinterpret_cast<uint8_t *>(&value);
            payloadPacked.push_back(valuePtr[0]);
            payloadPacked.push_back(valuePtr[1]);
            payloadPacked.push_back(valuePtr[2]);
            payloadPacked.push_back(valuePtr[3]);
        } else if (msg.valueType == lspc::ParameterLookup::_float) {
            float value;
            try {
                value = std::stof(valueString);
            }
            catch (std::invalid_argument &e) {
                lspcMutex->unlock();
                return true;
            }  // value could not be parsed (eg. is not a number)
            catch (std::out_of_range &e) {
                lspcMutex->unlock();
                return true;
            }
            uint8_t *valuePtr = reinterpret_cast<uint8_t *>(&value);
            payloadPacked.push_back(valuePtr[0]);
            payloadPacked.push_back(valuePtr[1]);
            payloadPacked.push_back(valuePtr[2]);
            payloadPacked.push_back(valuePtr[3]);
        } else {
            ROS_DEBUG("Set parameter: Incorrect valuetype");
            lspcMutex->unlock();
            return true; // incorrect valuetype
        }
    }

    /* Send message to embedded controller */
    SetParameterResponse.clear(); // clear queue to prepare for waiting for response
    (*lspcObj)->send(lspc::MessageTypesFromPC::SetParameter, payloadPacked);
    lspcMutex->unlock();

    /* Wait for response */
    std::tuple<lspc::ParameterLookup::type_t, uint8_t, bool> response;
    if (SetParameterResponse.pop(response, 2)) { // 2 seconds timeout
        if (std::get<0>(response) != msg.type) { lspcMutex->unlock(); return true; }
        if (std::get<1>(response) != msg.param) { lspcMutex->unlock(); return true; }
        if (std::get<2>(response))
            res.acknowledged = true;
        else
            res.acknowledged = false;
    } else {
        ROS_DEBUG("Set parameter: Response timeout");
        return true; // timeout
    }

    return true;
}

void LSPC_Callback_SetParameterAck(const std::vector<uint8_t>& payload)
{
    ROS_DEBUG_STREAM("Set parameter acknowledged");

    const lspc::MessageTypesToPC::SetParameterAck_t * msg = reinterpret_cast<const lspc::MessageTypesToPC::SetParameterAck_t *>(payload.data());
    if (sizeof(*msg) != payload.size()) {
        ROS_DEBUG("Error parsing SetParameterAck message");
        return;
    }

    std::tuple<lspc::ParameterLookup::type_t, uint8_t, bool> response((lspc::ParameterLookup::type_t)msg->type, msg->param, msg->acknowledged);
    SetParameterResponse.push(response); // push to service thread
}

bool ROS_Service_GetParameter(kugle_srvs::GetParameter::Request &req, kugle_srvs::GetParameter::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    /* Determine parameter type and param ID */
    lspc::MessageTypesFromPC::GetParameter_t msg;
    lspc::ParameterLookup::ValueType_t valueType;
    uint8_t arraySize;
    if (!ParseParamTypeAndID(req.type, req.param, msg.type, msg.param, valueType, arraySize)) {
        lspcMutex->unlock();
        return false; // parameter not found
    }

    /* Send parameter request to embedded controller */
    GetParameterResponse.clear();
    std::vector<uint8_t> payloadPacked((uint8_t *)&msg, (uint8_t *)&msg+sizeof(msg));
    (*lspcObj)->send(lspc::MessageTypesFromPC::GetParameter, payloadPacked);
    lspcMutex->unlock();

    /* Wait for initial response */
    std::shared_ptr<std::vector<uint8_t>> response;
    if (!GetParameterResponse.pop(response, 1)) { // 1 seconds timeout
        ROS_DEBUG("Get parameter: Response timeout");
        return false; // timeout
    }

    //ROS_DEBUG_STREAM("Received GetParameter response package of size " << response->size());

    /* Parse response to identify how many packages that will follow */
    const lspc::MessageTypesToPC::GetParameter_t * msgResponse = reinterpret_cast<const lspc::MessageTypesToPC::GetParameter_t *>(response->data());
    if (response->size() <= sizeof(*msgResponse)) {
        ROS_DEBUG("Error parsing GetParameter response message - parameter value missing from response");
        return false;
    }

    if (msgResponse->param == lspc::ParameterLookup::unknown) {
        ROS_DEBUG("Get parameter: Response mismatch (unknown parameter ID)");
        return false;
    }

    if (msgResponse->param != msg.param) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect parameter ID)");
        return false;
    }

    if (msgResponse->type != msg.type) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect parameter type)");
        return false;
    }

    if (msgResponse->valueType != valueType) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect value type)");
        return false;
    }

    if (msgResponse->arraySize != arraySize) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect array size)");
        return false;
    }

    /* Convert parameter value to response string */
    void * paramPtr = &response->at(sizeof(lspc::MessageTypesToPC::GetParameter_t));
    std::ostringstream os;

    unsigned int valuesLength = response->size() - sizeof(lspc::MessageTypesToPC::GetParameter_t);
    unsigned int ptrIndex = 0;
    while (ptrIndex < valuesLength) {
        if (ptrIndex != 0) os << " "; // add a space between multiple values
        if (msgResponse->valueType == lspc::ParameterLookup::_bool) {
            bool value = reinterpret_cast<bool *>(&reinterpret_cast<uint8_t *>(paramPtr)[ptrIndex])[0];
            if (value)
                os << "true";
            else
                os << "false";
            ptrIndex += 1;
        } else if (msgResponse->valueType == lspc::ParameterLookup::_uint8) {
            uint8_t value = reinterpret_cast<uint8_t *>(paramPtr)[ptrIndex];
            if (msg.type == lspc::ParameterLookup::controller && msg.param == lspc::ParameterLookup::mode) { // controller mode requested - output parsed mode as string
                os << ParseControllerMode((lspc::ParameterTypes::controllerMode_t)value);
            } else if (msg.type == lspc::ParameterLookup::controller && msg.param == lspc::ParameterLookup::type) {
                os << ParseControllerType((lspc::ParameterTypes::controllerType_t)value);
            } else if (msg.type == lspc::ParameterLookup::controller && msg.param == lspc::ParameterLookup::ManifoldType) {
                os << ParseManifoldType((lspc::ParameterTypes::slidingManifoldType_t)value);
            } else if (msg.type == lspc::ParameterLookup::behavioural && msg.param == lspc::ParameterLookup::PowerButtonMode) {
                os << ParsePowerButtonMode((lspc::ParameterTypes::powerButtonMode_t)value);
            } else { // not a parameter where we expect a string
                os << int(value);
            }
            ptrIndex += 1;
        } else if (msgResponse->valueType == lspc::ParameterLookup::_uint16) {
            uint16_t value = reinterpret_cast<uint16_t *>(&reinterpret_cast<uint8_t *>(paramPtr)[ptrIndex])[0];
            os << int(value);
            ptrIndex += 2;
        } else if (msgResponse->valueType == lspc::ParameterLookup::_uint32) {
            uint32_t value = reinterpret_cast<uint16_t *>(&reinterpret_cast<uint8_t *>(paramPtr)[ptrIndex])[0];
            os << long(value);
            ptrIndex += 4;
        } else if (msgResponse->valueType == lspc::ParameterLookup::_float) {
            float value = reinterpret_cast<float *>(&reinterpret_cast<uint8_t *>(paramPtr)[ptrIndex])[0];
            os << std::setprecision(10) << value;
            ptrIndex += 4;
        }
    }

    res.value = os.str();

    return true;
}

void LSPC_Callback_GetParameter(const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("Get parameter info received");

    std::shared_ptr<std::vector<uint8_t>> payloadPackage = std::make_shared<std::vector<uint8_t>>(payload);
    GetParameterResponse.push(payloadPackage); // push to service thread
}


bool ROS_Service_DumpParameters(kugle_srvs::DumpParameters::Request &req, kugle_srvs::DumpParameters::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.dump = "";

    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    std::vector<uint8_t> empty;
    DumpParametersResponse.clear();
    (*lspcObj)->send(lspc::MessageTypesFromPC::DumpParameters, empty);
    lspcMutex->unlock();

    /* Wait for initial response */
    std::shared_ptr<std::vector<uint8_t>> response;
    if (!DumpParametersResponse.pop(response, 2)) { // 2 seconds timeout
        ROS_DEBUG("Dump parameters: Response timeout");
        return false; // timeout
    }

    /* Parse response to identify how many packages that will follow */
    const lspc::MessageTypesToPC::DumpParameters_t * msgResponse = reinterpret_cast<const lspc::MessageTypesToPC::DumpParameters_t *>(response->data());
    if (sizeof(*msgResponse) != response->size()) {
        ROS_DEBUG("Error parsing DumpParameters message");
        return false;
    }

    ROS_DEBUG_STREAM("Expects to receive " << int(msgResponse->packages_to_follow) << " packages to assemble " << msgResponse->parameters_size_bytes << " bytes of parameters");

    int receivedBytes = 0;

    /* Receive packages to follow */
    std::vector<uint8_t> dump;
    for (int i = 0; i < int(msgResponse->packages_to_follow); i++) {
        std::shared_ptr<std::vector<uint8_t>> dumpPackage;
        if (!DumpParametersResponse.pop(dumpPackage, 2)) { // 2 seconds timeout
            ROS_DEBUG("Dump parameters: Response timeout");
            return false; // timeout
        }
        ROS_DEBUG_STREAM("Dump parameters: Got response of size " << dumpPackage->size());
        receivedBytes += dumpPackage->size();
        dump.insert(dump.begin(), dumpPackage->begin(), dumpPackage->end()); // combine packages into one big dumpPackage vector
    }

    std::ostringstream os;
    os << "Received parameter bytes: " << receivedBytes;
    res.dump = os.str();

    // Do something with the "dumpPackage" vector here containing the dumped raw parameter bytes

    return true;
}

void LSPC_Callback_DumpParameters(const std::vector<uint8_t>& payload)
{
    ROS_DEBUG_STREAM("Dump parameters response (size " << payload.size() << ")");

    std::shared_ptr<std::vector<uint8_t>> payloadPackage = std::make_shared<std::vector<uint8_t>>(payload);
    DumpParametersResponse.push(payloadPackage); // push to service thread
}

bool ROS_Service_StoreParameters(kugle_srvs::StoreParameters::Request &req, kugle_srvs::StoreParameters::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    std::vector<uint8_t> empty;
    StoreParametersResponse.clear();
    (*lspcObj)->send(lspc::MessageTypesFromPC::StoreParameters, empty);
    lspcMutex->unlock();

    /* Wait for response */
    bool acknowledged;
    if (!StoreParametersResponse.pop(acknowledged, 2)) { // 2 seconds timeout
        ROS_DEBUG("Store parameters: Response timeout");
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    return true;
}

void LSPC_Callback_StoreParametersAck(const std::vector<uint8_t>& payload)
{
    ROS_DEBUG_STREAM("Store parameters acknowledge message received");
    const lspc::MessageTypesToPC::StoreParametersAck_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::StoreParametersAck_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing StoreParametersAck message");
        return;
    }

    StoreParametersResponse.push(msgRaw->acknowledged);
}

void reconfigureModifyParameter(std::string type, std::string param, std::string value, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    kugle_srvs::SetParameter::Request req;
    kugle_srvs::SetParameter::Response res;

    req.type = type;
    req.param = param;
    req.value = value;
    if (ROS_Service_SetParameter(req, res, lspcMutex, lspcObj))
    {
        if (res.acknowledged)
            ROS_INFO("%s.%s parameter updated successfully", type.c_str(), param.c_str());
        else
            ROS_WARN("%s.%s parameter could not be updated", type.c_str(), param.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service SetParameter");
    }
}

std::string reconfigureRetrieveParameter(std::string type, std::string param, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    kugle_srvs::GetParameter::Request req;
    kugle_srvs::GetParameter::Response res;

    req.type = type;
    req.param = param;
    if (ROS_Service_GetParameter(req, res, lspcMutex, lspcObj))
    {
        ROS_INFO("%s.%s parameter retrieved successfully", type.c_str(), param.c_str());
        return res.value;
    }
    else
    {
        ROS_ERROR("Failed to call service GetParameter");
    }

    return "";
}

float Parse2Float(std::string str)
{
    float value;
    try {
        value = std::stof(str);
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

float Parse2RoundedFloat(std::string str)
{
    float value;
    try {
        value = std::stof(str);
        value = roundf(value * 1000) / 1000; // round to 3 decimals
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

int Parse2Int(std::string str)
{
    int value;
    try {
        value = std::stoi(str);
    }
    catch (std::invalid_argument &e) {
        return 0;
    }  // value could not be parsed (eg. is not a number)
    catch (std::out_of_range &e) {
        return 0;
    }
    return value;
}

bool Parse2Bool(std::string str)
{
    if (!str.compare("true"))
        return true;
    else if (!str.compare("false"))
        return false;
    else {
        return false;
    }
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

void reconfigureCallback(kugle_driver::ParametersConfig &config, uint32_t level, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    reconfigureMutex.lock();
    ROS_DEBUG("Reconfigure Request");

    if (config.EnableDumpMessages != reconfigureConfig.EnableDumpMessages) reconfigureModifyParameter("debug", "EnableDumpMessages", config.EnableDumpMessages ? "true" : "false", lspcMutex, lspcObj);
    if (config.EnableRawSensorOutput != reconfigureConfig.EnableRawSensorOutput) reconfigureModifyParameter("debug", "EnableRawSensorOutput", config.EnableRawSensorOutput ? "true" : "false", lspcMutex, lspcObj);
    if (config.DisableMotorOutput != reconfigureConfig.DisableMotorOutput) reconfigureModifyParameter("debug", "DisableMotorOutput", config.DisableMotorOutput ? "true" : "false", lspcMutex, lspcObj);

    if (config.IndependentHeading != reconfigureConfig.IndependentHeading) reconfigureModifyParameter("behavioural", "IndependentHeading", config.IndependentHeading ? "true" : "false", lspcMutex, lspcObj);
    if (config.SineTestEnabled != reconfigureConfig.SineTestEnabled) reconfigureModifyParameter("behavioural", "SineTestEnabled", config.SineTestEnabled ? "true" : "false", lspcMutex, lspcObj);
    if (config.PowerButtonMode != reconfigureConfig.PowerButtonMode) reconfigureModifyParameter("behavioural", "PowerButtonMode", std::to_string(config.PowerButtonMode), lspcMutex, lspcObj);

    if (config.type != reconfigureConfig.type) reconfigureModifyParameter("controller", "type", std::to_string(config.type), lspcMutex, lspcObj);
    if (config.mode != reconfigureConfig.mode) reconfigureModifyParameter("controller", "mode", std::to_string(config.mode), lspcMutex, lspcObj);
    if (config.DisableQdot != reconfigureConfig.DisableQdot) reconfigureModifyParameter("controller", "DisableQdot", config.DisableQdot ? "true" : "false", lspcMutex, lspcObj);
    if (config.DisableQdotInEquivalentControl != reconfigureConfig.DisableQdotInEquivalentControl) reconfigureModifyParameter("controller", "DisableQdotInEquivalentControl", config.DisableQdotInEquivalentControl ? "true" : "false", lspcMutex, lspcObj);
    if (config.ManifoldType != reconfigureConfig.ManifoldType) reconfigureModifyParameter("controller", "ManifoldType", std::to_string(config.ManifoldType), lspcMutex, lspcObj);
    if (config.EquivalentControl != reconfigureConfig.EquivalentControl) reconfigureModifyParameter("controller", "EquivalentControl", config.EquivalentControl ? "true" : "false", lspcMutex, lspcObj);

    if (config.K_x != reconfigureConfig.K_x) reconfigureModifyParameter("controller", "K", std::to_string(config.K_x) + " " + std::to_string(config.K_y) + " " + std::to_string(config.K_z), lspcMutex, lspcObj);
    if (config.K_y != reconfigureConfig.K_y) reconfigureModifyParameter("controller", "K", std::to_string(config.K_x) + " " + std::to_string(config.K_y) + " " + std::to_string(config.K_z), lspcMutex, lspcObj);
    if (config.K_z != reconfigureConfig.K_z) reconfigureModifyParameter("controller", "K", std::to_string(config.K_x) + " " + std::to_string(config.K_y) + " " + std::to_string(config.K_z), lspcMutex, lspcObj);
    if (config.K != reconfigureConfig.K) {
        reconfigureModifyParameter("controller", "K", std::to_string(config.K), lspcMutex, lspcObj);
        config.K_x = config.K;
        config.K_y = config.K;
        config.K_z = config.K;
    }

    if (config.eta_x != reconfigureConfig.eta_x) reconfigureModifyParameter("controller", "eta", std::to_string(config.eta_x) + " " + std::to_string(config.eta_y) + " " + std::to_string(config.eta_z), lspcMutex, lspcObj);
    if (config.eta_y != reconfigureConfig.eta_y) reconfigureModifyParameter("controller", "eta", std::to_string(config.eta_x) + " " + std::to_string(config.eta_y) + " " + std::to_string(config.eta_z), lspcMutex, lspcObj);
    if (config.eta_z != reconfigureConfig.eta_z) reconfigureModifyParameter("controller", "eta", std::to_string(config.eta_x) + " " + std::to_string(config.eta_y) + " " + std::to_string(config.eta_z), lspcMutex, lspcObj);
    if (config.eta != reconfigureConfig.eta) {
        reconfigureModifyParameter("controller", "eta", std::to_string(config.eta), lspcMutex, lspcObj);
        config.eta_x = config.eta;
        config.eta_y = config.eta;
        config.eta_z = config.eta;
    }

    if (config.epsilon_x != reconfigureConfig.epsilon_x) reconfigureModifyParameter("controller", "epsilon", std::to_string(config.epsilon_x) + " " + std::to_string(config.epsilon_y) + " " + std::to_string(config.epsilon_z), lspcMutex, lspcObj);
    if (config.epsilon_y != reconfigureConfig.epsilon_y) reconfigureModifyParameter("controller", "epsilon", std::to_string(config.epsilon_x) + " " + std::to_string(config.epsilon_y) + " " + std::to_string(config.epsilon_z), lspcMutex, lspcObj);
    if (config.epsilon_z != reconfigureConfig.epsilon_z) reconfigureModifyParameter("controller", "epsilon", std::to_string(config.epsilon_x) + " " + std::to_string(config.epsilon_y) + " " + std::to_string(config.epsilon_z), lspcMutex, lspcObj);
    if (config.epsilon != reconfigureConfig.epsilon) {
        reconfigureModifyParameter("controller", "epsilon", std::to_string(config.epsilon), lspcMutex, lspcObj);
        config.epsilon_x = config.epsilon;
        config.epsilon_y = config.epsilon;
        config.epsilon_z = config.epsilon;
    }

    if (config.VelocityController_AccelerationLimit != reconfigureConfig.VelocityController_AccelerationLimit) reconfigureModifyParameter("controller", "VelocityController_AccelerationLimit", std::to_string(config.VelocityController_AccelerationLimit), lspcMutex, lspcObj);
    if (config.VelocityController_MaxTilt != reconfigureConfig.VelocityController_MaxTilt) reconfigureModifyParameter("controller", "VelocityController_MaxTilt", std::to_string(config.VelocityController_MaxTilt), lspcMutex, lspcObj);
    if (config.VelocityController_MaxIntegralCorrection != reconfigureConfig.VelocityController_MaxIntegralCorrection) reconfigureModifyParameter("controller", "VelocityController_MaxIntegralCorrection", std::to_string(config.VelocityController_MaxIntegralCorrection), lspcMutex, lspcObj);
    if (config.VelocityController_VelocityClamp != reconfigureConfig.VelocityController_VelocityClamp) reconfigureModifyParameter("controller", "VelocityController_VelocityClamp", std::to_string(config.VelocityController_VelocityClamp), lspcMutex, lspcObj);
    if (config.VelocityController_IntegralGain != reconfigureConfig.VelocityController_IntegralGain) reconfigureModifyParameter("controller", "VelocityController_IntegralGain", std::to_string(config.VelocityController_IntegralGain), lspcMutex, lspcObj);

    if (config.UseCoRvelocity != reconfigureConfig.UseCoRvelocity) reconfigureModifyParameter("estimator", "UseCoRvelocity", config.UseCoRvelocity ? "true" : "false", lspcMutex, lspcObj);
    if (config.sigma2_bias != reconfigureConfig.sigma2_bias) reconfigureModifyParameter("estimator", "sigma2_bias", to_string_with_precision(powf(10, -config.sigma2_bias),10), lspcMutex, lspcObj);
    if (config.sigma2_omega != reconfigureConfig.sigma2_omega) reconfigureModifyParameter("estimator", "sigma2_omega", to_string_with_precision(powf(10, -config.sigma2_omega),10), lspcMutex, lspcObj);
    if (config.sigma2_heading != reconfigureConfig.sigma2_heading) reconfigureModifyParameter("estimator", "sigma2_heading", to_string_with_precision(powf(10, -config.sigma2_heading),10), lspcMutex, lspcObj);
    if (config.GyroscopeTrustFactor != reconfigureConfig.GyroscopeTrustFactor) reconfigureModifyParameter("estimator", "GyroscopeTrustFactor", std::to_string(config.GyroscopeTrustFactor), lspcMutex, lspcObj);
    if (config.EnableVelocityLPF != reconfigureConfig.EnableVelocityLPF) reconfigureModifyParameter("estimator", "EnableVelocityLPF", config.EnableVelocityLPF ? "true" : "false", lspcMutex, lspcObj);

    if (config.l != reconfigureConfig.l) reconfigureModifyParameter("model", "l", std::to_string(config.l), lspcMutex, lspcObj);
    if (config.CoR != reconfigureConfig.CoR) reconfigureModifyParameter("model", "CoR", std::to_string(config.CoR), lspcMutex, lspcObj);
    if (config.SaturationTorqueOfMaxOutputTorque != reconfigureConfig.SaturationTorqueOfMaxOutputTorque) reconfigureModifyParameter("model", "SaturationTorqueOfMaxOutputTorque", std::to_string(config.SaturationTorqueOfMaxOutputTorque), lspcMutex, lspcObj);

    MATLAB_log = config.MATLAB_log;

    reconfigureConfig = config;
    reconfigureMutex.unlock();
    reconfigureServer->updateConfig(reconfigureConfig);
}

void LoadParamsIntoReconfigure(std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    std::lock_guard<std::mutex> lock(reconfigureMutex);

    reconfigureConfig.EnableDumpMessages = Parse2Bool(reconfigureRetrieveParameter("debug", "EnableDumpMessages", lspcMutex, lspcObj));
    reconfigureConfig.EnableRawSensorOutput = Parse2Bool(reconfigureRetrieveParameter("debug", "EnableRawSensorOutput", lspcMutex, lspcObj));
    reconfigureConfig.DisableMotorOutput = Parse2Bool(reconfigureRetrieveParameter("debug", "DisableMotorOutput", lspcMutex, lspcObj));

    reconfigureConfig.IndependentHeading = Parse2Bool(reconfigureRetrieveParameter("behavioural", "IndependentHeading", lspcMutex, lspcObj));
    reconfigureConfig.SineTestEnabled = Parse2Bool(reconfigureRetrieveParameter("behavioural", "SineTestEnabled", lspcMutex, lspcObj));
    reconfigureConfig.PowerButtonMode = int(ParsePowerButtonMode2(reconfigureRetrieveParameter("behavioural", "PowerButtonMode", lspcMutex, lspcObj)));

    reconfigureConfig.mode = int(ParseControllerMode2(reconfigureRetrieveParameter("controller", "mode", lspcMutex, lspcObj)));
    reconfigureConfig.type = int(ParseControllerType2(reconfigureRetrieveParameter("controller", "type", lspcMutex, lspcObj)));
    reconfigureConfig.DisableQdot = Parse2Bool(reconfigureRetrieveParameter("controller", "DisableQdot", lspcMutex, lspcObj));
    reconfigureConfig.DisableQdotInEquivalentControl = Parse2Bool(reconfigureRetrieveParameter("controller", "DisableQdotInEquivalentControl", lspcMutex, lspcObj));
    reconfigureConfig.ManifoldType = int(ParseManifoldType2(reconfigureRetrieveParameter("controller", "ManifoldType", lspcMutex, lspcObj)));
    reconfigureConfig.EquivalentControl = Parse2Bool(reconfigureRetrieveParameter("controller", "EquivalentControl", lspcMutex, lspcObj));

    // Load K
    {
        std::istringstream iss(reconfigureRetrieveParameter("controller", "K", lspcMutex, lspcObj));
        std::vector<std::string> values((std::istream_iterator<std::string>(iss)),  // split string by spaces
                                        std::istream_iterator<std::string>());
        reconfigureConfig.K = 0;
        if (values.size() == 3) {
            reconfigureConfig.K_x = Parse2RoundedFloat(values.at(0));
            reconfigureConfig.K_y = Parse2RoundedFloat(values.at(1));
            reconfigureConfig.K_z = Parse2RoundedFloat(values.at(2));
        }
    }

    // Load eta
    {
        std::istringstream iss(reconfigureRetrieveParameter("controller", "eta", lspcMutex, lspcObj));
        std::vector<std::string> values((std::istream_iterator<std::string>(iss)),  // split string by spaces
                                        std::istream_iterator<std::string>());
        reconfigureConfig.eta = 0;
        if (values.size() == 3) {
            reconfigureConfig.eta_x = Parse2RoundedFloat(values.at(0));
            reconfigureConfig.eta_y = Parse2RoundedFloat(values.at(1));
            reconfigureConfig.eta_z = Parse2RoundedFloat(values.at(2));
        }
    }

    // Load epsilon
    {
        std::istringstream iss(reconfigureRetrieveParameter("controller", "epsilon", lspcMutex, lspcObj));
        std::vector<std::string> values((std::istream_iterator<std::string>(iss)),  // split string by spaces
                                        std::istream_iterator<std::string>());
        reconfigureConfig.epsilon = 0;
        if (values.size() == 3) {
            reconfigureConfig.epsilon_x = Parse2RoundedFloat(values.at(0));
            reconfigureConfig.epsilon_y = Parse2RoundedFloat(values.at(1));
            reconfigureConfig.epsilon_z = Parse2RoundedFloat(values.at(2));
        }
    }

    reconfigureConfig.VelocityController_AccelerationLimit = Parse2RoundedFloat(reconfigureRetrieveParameter("controller", "VelocityController_AccelerationLimit", lspcMutex, lspcObj));
    reconfigureConfig.VelocityController_MaxTilt = Parse2RoundedFloat(reconfigureRetrieveParameter("controller", "VelocityController_MaxTilt", lspcMutex, lspcObj));
    reconfigureConfig.VelocityController_MaxIntegralCorrection = Parse2RoundedFloat(reconfigureRetrieveParameter("controller", "VelocityController_MaxIntegralCorrection", lspcMutex, lspcObj));
    reconfigureConfig.VelocityController_VelocityClamp = Parse2RoundedFloat(reconfigureRetrieveParameter("controller", "VelocityController_VelocityClamp", lspcMutex, lspcObj));
    reconfigureConfig.VelocityController_IntegralGain = Parse2RoundedFloat(reconfigureRetrieveParameter("controller", "VelocityController_IntegralGain", lspcMutex, lspcObj));

    reconfigureConfig.UseCoRvelocity = Parse2Bool(reconfigureRetrieveParameter("estimator", "UseCoRvelocity", lspcMutex, lspcObj));

    try {
        reconfigureConfig.sigma2_bias = -log10f(Parse2Float(reconfigureRetrieveParameter("estimator", "sigma2_bias", lspcMutex, lspcObj)));
        if (std::isinf(reconfigureConfig.sigma2_bias)) reconfigureConfig.sigma2_bias = 0;

        reconfigureConfig.sigma2_omega = -log10f(Parse2Float(reconfigureRetrieveParameter("estimator", "sigma2_omega", lspcMutex, lspcObj)));
        if (std::isinf(reconfigureConfig.sigma2_omega)) reconfigureConfig.sigma2_omega = 0;

        reconfigureConfig.sigma2_heading = -log10f(Parse2Float(reconfigureRetrieveParameter("estimator", "sigma2_heading", lspcMutex, lspcObj)));
        if (std::isinf(reconfigureConfig.sigma2_heading)) reconfigureConfig.sigma2_heading = 0;
    } catch (...)  {}
    reconfigureConfig.GyroscopeTrustFactor = Parse2RoundedFloat(reconfigureRetrieveParameter("estimator", "GyroscopeTrustFactor", lspcMutex, lspcObj));
    reconfigureConfig.EnableVelocityLPF = Parse2Bool(reconfigureRetrieveParameter("estimator", "EnableVelocityLPF", lspcMutex, lspcObj));

    reconfigureConfig.l = Parse2RoundedFloat(reconfigureRetrieveParameter("model", "l", lspcMutex, lspcObj));
    reconfigureConfig.CoR = Parse2RoundedFloat(reconfigureRetrieveParameter("model", "CoR", lspcMutex, lspcObj));
    reconfigureConfig.SaturationTorqueOfMaxOutputTorque = Parse2RoundedFloat(reconfigureRetrieveParameter("model", "SaturationTorqueOfMaxOutputTorque", lspcMutex, lspcObj));

    reconfigureServer->updateConfig(reconfigureConfig);
}


bool ROS_Service_CalibrateIMU(kugle_srvs::CalibrateIMU::Request &req, kugle_srvs::CalibrateIMU::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    lspc::MessageTypesFromPC::CalibrateIMU_t msg;
    msg.calibrate_accelerometer = false;
    msg.magic_key = 0x12345678;
    std::vector<uint8_t> payloadPacked((uint8_t *)&msg, (uint8_t *)&msg+sizeof(msg));
    (*lspcObj)->send(lspc::MessageTypesFromPC::CalibrateIMU, payloadPacked);
    lspcMutex->unlock();

    /* Wait for response */
    bool acknowledged;
    if (!CalibrateIMUResponse.pop(acknowledged, 1)) { // 1 seconds timeout
        ROS_DEBUG("Calibrate IMU: Response timeout");
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    return true;
}

bool ROS_Service_CalibrateAccelerometer(kugle_srvs::CalibrateAccelerometer::Request &req, kugle_srvs::CalibrateAccelerometer::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    lspc::MessageTypesFromPC::CalibrateIMU_t msg;
    msg.calibrate_accelerometer = true;
    msg.magic_key = 0x12345678;
    std::vector<uint8_t> payloadPacked((uint8_t *)&msg, (uint8_t *)&msg+sizeof(msg));
    (*lspcObj)->send(lspc::MessageTypesFromPC::CalibrateIMU, payloadPacked);
    lspcMutex->unlock();

    /* Wait for response */
    bool acknowledged;
    if (!CalibrateIMUResponse.pop(acknowledged, 2)) { // 2 seconds timeout
        ROS_DEBUG("Calibrate IMU: Response timeout");
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    return true;
}

void LSPC_Callback_CalibrateIMUAck(const std::vector<uint8_t>& payload)
{
    ROS_DEBUG_STREAM("Calibrate IMU acknowledge message received");
    const lspc::MessageTypesToPC::CalibrateIMUAck_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::CalibrateIMUAck_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing CalibrateIMUAck message");
        return;
    }

    CalibrateIMUResponse.push(msgRaw->acknowledged);
}


bool ROS_Service_RestartController(kugle_srvs::RestartController::Request &req, kugle_srvs::RestartController::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    lspc::MessageTypesFromPC::RestartController_t msg;
    msg.magic_key = 0x12345678;
    std::vector<uint8_t> payloadPacked((uint8_t *)&msg, (uint8_t *)&msg+sizeof(msg));
    (*lspcObj)->send(lspc::MessageTypesFromPC::RestartController, payloadPacked);
    lspcMutex->unlock();

    /* Wait for response */
    bool acknowledged;
    if (!RestartControllerResponse.pop(acknowledged, 2)) { // 2 seconds timeout
        ROS_DEBUG("Restart Controller: Response timeout");
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    return true;
}

void LSPC_Callback_RestartControllerAck(const std::vector<uint8_t>& payload)
{
    ROS_DEBUG_STREAM("Restart Controller acknowledge message received");
    const lspc::MessageTypesToPC::RestartControllerAck_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::RestartControllerAck_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing RestartControllerAck message");
        return;
    }

    RestartControllerResponse.push(msgRaw->acknowledged);
}

bool ROS_Service_Reboot(kugle_srvs::Reboot::Request &req, kugle_srvs::Reboot::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) {
        return true; // could not get lock
    }

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return true; // connection is not open
    }

    std::vector<uint8_t> empty;
    CalibrateIMUResponse.clear();
    (*lspcObj)->send(lspc::MessageTypesFromPC::Reboot, empty);
    lspcMutex->unlock();

    res.acknowledged = true;

    return true;
}

bool ROS_Service_EnterBootloader(kugle_srvs::EnterBootloader::Request &req, kugle_srvs::EnterBootloader::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    res.acknowledged = false;
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) {
        return true; // could not get lock
    }

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return true; // connection is not open
    }

    std::vector<uint8_t> empty;
    CalibrateIMUResponse.clear();
    (*lspcObj)->send(lspc::MessageTypesFromPC::EnterBootloader, empty);
    lspcMutex->unlock();

    res.acknowledged = true;

    return true;
}

void LSPC_ConnectionThread(boost::shared_ptr<ros::NodeHandle> n, std::string serial_port, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj, std::future<void> exitSignalObj)
{
    /* Configure transform publisher and listener */
    tf::TransformBroadcaster tf_broadcaster; // maybe wrap with unique_ptr ?
    std::shared_ptr<tf::TransformListener> tf_listener = std::make_shared<tf::TransformListener>();

    /* Configure ROS publishers (based on received LSPC messages) */
    ros::Publisher pub_odom = n->advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher pub_imu = n->advertise<sensor_msgs::Imu>("imu", 50);
    ros::Publisher pub_mag = n->advertise<sensor_msgs::MagneticField>("magnetometer", 50);
    ros::Publisher pub_battery = n->advertise<sensor_msgs::BatteryState>("battery", 50);
    ros::Publisher pub_encoders = n->advertise<kugle_msgs::Encoders>("encoders", 50);
    ros::Publisher pub_controller_info = n->advertise<kugle_msgs::ControllerInfo>("controller_info", 50);
    ros::Publisher pub_state_estimate = n->advertise<kugle_msgs::StateEstimate>("state_estimate", 50);
    ros::Publisher pub_mcu_debug = n->advertise<std_msgs::String>("mcu_debug", 50);
    ros::Publisher pub_mcu_load = n->advertise<std_msgs::String>("mcu_load", 50);
    ros::Publisher pub_controller_debug = n->advertise<kugle_msgs::ControllerDebug>("controller_debug", 50);

    lspcMutex->lock();
    while (exitSignalObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
        *lspcObj = new lspc::Socket; // recreate a new LSPC object
        while (!(*lspcObj)->isOpen() && ros::ok()) {
            try {
                ROS_DEBUG("Trying to connect to Kugle");
                (*lspcObj)->open("/dev/kugle");
            }
            catch (boost::system::system_error &e) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        if (!ros::ok()) break;

        ROS_DEBUG("Connected to Kugle");

        if (!boost::filesystem::is_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump"))) {
            if (boost::filesystem::exists(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump"))) {
                ROS_WARN("Kugle dump path (~/kugle_dump) already exists but without write permissions");
            } else {
                if (!boost::filesystem::create_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump")))
                    ROS_WARN("Could not create Kugle dump log folder (~/kugle_dump)");
                else
                    ROS_DEBUG("Successfully created mathdump log folder (~/kugle_dump)");
            }
        }

        if (!boost::filesystem::is_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump/sensor"))) {
            if (boost::filesystem::exists(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump/sensor"))) {
                ROS_WARN("Sensor dump path (~/kugle_dump/sensor) already exists but without write permissions");
            } else {
                if (!boost::filesystem::create_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump/sensor")))
                    ROS_WARN("Could not create sensor dump log folder (~/kugle_dump/sensor)");
                else
                    ROS_DEBUG("Successfully created sensor dump log folder (~/kugle_dump/sensor)");
            }
        }

        if (!boost::filesystem::is_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump/covariance"))) {
            if (boost::filesystem::exists(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump/covariance"))) {
                ROS_WARN("Covariance dump path (~/kugle_dump/covariance) already exists but without write permissions");
            } else {
                if (!boost::filesystem::create_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/kugle_dump/covariance")))
                    ROS_WARN("Could not create covariance dump log folder (~/kugle_dump/covariance)");
                else
                    ROS_DEBUG("Successfully created covariance dump log folder (~/kugle_dump/covariance)");
            }
        }

        std::shared_ptr<std::ofstream> mathdumpFile = std::make_shared<std::ofstream>();
        std::shared_ptr<std::ofstream> sensordumpFile = std::make_shared<std::ofstream>();
        std::shared_ptr<std::ofstream> covariancedumpFile = std::make_shared<std::ofstream>();

        lspcMutex->unlock();

        /* Load parameters from board */
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::GetParameter, &LSPC_Callback_GetParameter);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SetParameterAck, &LSPC_Callback_SetParameterAck);
        LoadParamsIntoReconfigure(lspcMutex, lspcObj);

        /* Register callbacks */
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::DumpParameters, &LSPC_Callback_DumpParameters);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::StoreParametersAck, &LSPC_Callback_StoreParametersAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SystemInfo, &LSPC_Callback_SystemInfo);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::ControllerInfo, boost::bind(&LSPC_Callback_ControllerInfo, pub_controller_info, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::ControllerDebug, boost::bind(&LSPC_Callback_ControllerDebug, pub_controller_debug, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::StateEstimates, boost::bind(&LSPC_Callback_StateEstimates, pub_odom, pub_state_estimate, tf_broadcaster, tf_listener, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_IMU_MPU9250, boost::bind(&LSPC_Callback_RawSensor_IMU_MPU9250, pub_imu, pub_mag, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_IMU_MTI200, &LSPC_Callback_RawSensor_IMU_MTI200);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_Battery, boost::bind(&LSPC_Callback_RawSensor_Battery, pub_battery, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_Encoders, boost::bind(&LSPC_Callback_RawSensor_Encoders, pub_encoders, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CalibrateIMUAck, &LSPC_Callback_CalibrateIMUAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RestartControllerAck, &LSPC_Callback_RestartControllerAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CPUload, boost::bind(&LSPC_Callback_CPUload, pub_mcu_load, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::MathDump, boost::bind(&LSPC_Callback_ArrayDump, mathdumpFile, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SensorDump, boost::bind(&LSPC_Callback_ArrayDump, sensordumpFile, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CovarianceDump, boost::bind(&LSPC_Callback_ArrayDump, covariancedumpFile, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::Debug, boost::bind(&LSPC_Callback_Debug, pub_mcu_debug, _1));

        bool MATLAB_log_prev = false;
        while ((*lspcObj)->isOpen() && ros::ok()) {
            if (MATLAB_log != MATLAB_log_prev) {
                if (MATLAB_log) {
                    std::string dumpFilename = GetFormattedTimestampCurrent() + ".txt";
                    mathdumpFile->open(std::string(getenv("HOME")) + "/kugle_dump/" + dumpFilename, std::ofstream::trunc);
                    ROS_DEBUG_STREAM("Created mathdump file: ~/kugle_dump/" << dumpFilename);

                    sensordumpFile->open(std::string(getenv("HOME")) + "/kugle_dump/sensor/" + dumpFilename, std::ofstream::trunc);
                    ROS_DEBUG_STREAM("Created sensordump file: ~/kugle_dump/sensor/" << dumpFilename);

                    covariancedumpFile->open(std::string(getenv("HOME")) + "/kugle_dump/covariance/" + dumpFilename, std::ofstream::trunc);
                    ROS_DEBUG_STREAM("Created sensordump file: ~/kugle_dump/covariance/" << dumpFilename);
                } else {
                    mathdumpFile->close();
                    sensordumpFile->close();
                    covariancedumpFile->close();
                }
                MATLAB_log_prev = MATLAB_log;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!ros::ok())
            break;

        ROS_WARN("Connection lost to Kugle");
        lspcMutex->lock();
        delete(*lspcObj); // call destructor to disconnect completely and clean up used ressources

        mathdumpFile->close();
        sensordumpFile->close();
        covariancedumpFile->close();
    }
}

int main(int argc, char **argv) {
	std::string nodeName = "kugle_driver";
	ros::init(argc, argv, nodeName.c_str());
    boost::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);  // ros::NodeHandle n
	ros::NodeHandle nParam("~"); // default/current namespace node handle
    reconfigureMutex.lock();

	ROS_DEBUG_STREAM("Starting Kugle driver...");

	std::string serial_port = "/dev/kugle";
	if (!nParam.getParam("serial_port", serial_port)) {
		ROS_WARN_STREAM("Serial port not set (Parameter: serial_port). Defaults to: /dev/kugle");
	}

	/* Create common LSPC object */
    std::shared_ptr<std::timed_mutex> lspcMutex = std::make_shared<std::timed_mutex>();
    std::shared_ptr<lspc::Socket *> lspcObj = std::make_shared<lspc::Socket *>(nullptr);  // read about shared_ptr here: https://www.acodersjourney.com/top-10-dumb-mistakes-avoid-c-11-smart-pointers/

    /* Configure subscribers */
    //std::shared_ptr<ros::Timer> reference_timeout_timer = std::make_shared<ros::Timer>(n->createTimer(ros::Duration(0.1), boost::bind(&ROS_Callback_Heartbeat, lspcMutex, lspcObj, reference_timeout_timer, _1))); // Configure 10 Hz heartbeat timer
    ros::Subscriber sub_cmd_vel = n->subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ROS_Callback_cmd_vel, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_cmd_vel_inertial = n->subscribe<geometry_msgs::Twist>("cmd_vel_inertial", 1, boost::bind(&ROS_Callback_cmd_vel_inertial, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_cmd_quaternion = n->subscribe<geometry_msgs::Quaternion>("cmd_quaternion", 1, boost::bind(&ROS_Callback_cmd_quaternion, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_cmd_combined = n->subscribe<kugle_msgs::BalanceControllerReference>("cmd_combined", 1, boost::bind(&ROS_Callback_cmd_combined, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_cmd_combined_inertial = n->subscribe<kugle_msgs::BalanceControllerReference>("cmd_combined_inertial", 1, boost::bind(&ROS_Callback_cmd_combined_inertial, _1, lspcMutex, lspcObj));

    /* Configure node services */
    ros::ServiceServer serv_set_parameter = n->advertiseService<kugle_srvs::SetParameter::Request, kugle_srvs::SetParameter::Response>("/kugle/set_parameter", boost::bind(&ROS_Service_SetParameter, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_get_parameter = n->advertiseService<kugle_srvs::GetParameter::Request, kugle_srvs::GetParameter::Response>("/kugle/get_parameter", boost::bind(&ROS_Service_GetParameter, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_dump_parameters = n->advertiseService<kugle_srvs::DumpParameters::Request, kugle_srvs::DumpParameters::Response>("/kugle/dump_parameters", boost::bind(&ROS_Service_DumpParameters, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_store_parameters = n->advertiseService<kugle_srvs::StoreParameters::Request, kugle_srvs::StoreParameters::Response>("/kugle/store_parameters", boost::bind(&ROS_Service_StoreParameters, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_calibrate_imu = n->advertiseService<kugle_srvs::CalibrateIMU::Request, kugle_srvs::CalibrateIMU::Response>("/kugle/calibrate_imu", boost::bind(&ROS_Service_CalibrateIMU, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_calibrate_accelerometer = n->advertiseService<kugle_srvs::CalibrateAccelerometer::Request, kugle_srvs::CalibrateAccelerometer::Response>("/kugle/calibrate_accelerometer", boost::bind(&ROS_Service_CalibrateAccelerometer, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_reboot = n->advertiseService<kugle_srvs::Reboot::Request, kugle_srvs::Reboot::Response>("/kugle/reboot", boost::bind(&ROS_Service_Reboot, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_enter_bootloader = n->advertiseService<kugle_srvs::EnterBootloader::Request, kugle_srvs::EnterBootloader::Response>("/kugle/enter_bootloader", boost::bind(&ROS_Service_EnterBootloader, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_restart_controller = n->advertiseService<kugle_srvs::RestartController::Request, kugle_srvs::RestartController::Response>("/kugle/restart_controller", boost::bind(&ROS_Service_RestartController, _1, _2, lspcMutex, lspcObj));

    /* Enable dynamic reconfigure */
    // Enable reconfigurable parameters - note that any parameters set on the node by roslaunch <param> tags will be seen by a dynamically reconfigurable node just as it would have been by a conventional node.
    boost::recursive_mutex configMutex;
    //std::shared_ptr<dynamic_reconfigure::Server<kugle_driver::ParametersConfig>> reconfigureServer = std::make_shared<dynamic_reconfigure::Server<kugle_driver::ParametersConfig>>(configMutex);
    reconfigureServer = std::make_shared<dynamic_reconfigure::Server<kugle_driver::ParametersConfig>>(configMutex);
    dynamic_reconfigure::Server<kugle_driver::ParametersConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2, lspcMutex, lspcObj);
    reconfigureServer->getConfigDefault(reconfigureConfig);
    reconfigureMutex.unlock();
    reconfigureServer->setCallback(f);

	/* Create LSPC connection thread */
    std::promise<void> exitSignal;
    std::future<void> exitSignalObj = exitSignal.get_future();
    std::thread connectionThread = std::thread(&LSPC_ConnectionThread, n, serial_port, lspcMutex, lspcObj, std::move(exitSignalObj));

    ros::spin();

    std::cout << "Shutting down Kugle driver" << std::endl;

    //reference_timeout_timer->stop();

    exitSignal.set_value();
    if (connectionThread.joinable())
        connectionThread.join();
}
