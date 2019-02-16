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

#include "LSPC.h"
#include "MessageTypes.h"
#include <string>
#include <thread>
#include <future>
#include <boost/bind.hpp>
#include <fstream>      // for file output (std::ofstream)
#include <boost/filesystem.hpp>   // for directory creation (boost::filesystem)
#include <chrono>
#include <cstdint>
#include <tuple>

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
#include <kugle_srvs/Reboot.h>
#include <kugle_srvs/EnterBootloader.h>
#include <kugle_srvs/RestartController.h>

/* Include generated Message Types */
#include <kugle_msgs/ControllerInfo.h>
#include <kugle_msgs/Encoders.h>
#include <kugle_msgs/StateEstimate.h>

/* Global variables */
Queue<std::tuple<lspc::ParameterLookup::type_t, uint8_t, bool>> SetParameterResponse;
Queue<std::shared_ptr<std::vector<uint8_t>>> GetParameterResponse;
Queue<std::shared_ptr<std::vector<uint8_t>>> DumpParametersResponse;
Queue<bool> StoreParametersResponse;
Queue<bool> CalibrateIMUResponse;
Queue<bool> RestartControllerResponse;

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
    else if (mode == lspc::ParameterTypes::ANGULAR_VELOCITY_CONTROL) return "ANGULAR_VELOCITY_CONTROL";
    else if (mode == lspc::ParameterTypes::VELOCITY_CONTROL) return "VELOCITY_CONTROL";
    else if (mode == lspc::ParameterTypes::PATH_FOLLOWING) return "PATH_FOLLOWING";
    else return "UNKNOWN_MODE";
}

lspc::ParameterTypes::controllerMode_t ParseControllerMode2(std::string mode)
{
    if (!mode.compare("OFF")) return lspc::ParameterTypes::OFF;
    else if (!mode.compare("QUATERNION_CONTROL")) return lspc::ParameterTypes::QUATERNION_CONTROL;
    else if (!mode.compare("ANGULAR_VELOCITY_CONTROL")) return lspc::ParameterTypes::ANGULAR_VELOCITY_CONTROL;
    else if (!mode.compare("VELOCITY_CONTROL")) return lspc::ParameterTypes::VELOCITY_CONTROL;
    else if (!mode.compare("PATH_FOLLOWING")) return lspc::ParameterTypes::PATH_FOLLOWING;
    else return lspc::ParameterTypes::UNKNOWN_MODE;
}

void LSPC_Callback_ControllerInfo(ros::Publisher& pubControllerInfo, const std::vector<uint8_t>& payload)
{
    //ROS_DEBUG_STREAM("Controller info received");
    const lspc::MessageTypesToPC::ControllerInfo_t * msgRaw = reinterpret_cast<const lspc::MessageTypesToPC::ControllerInfo_t *>(payload.data());
    if (sizeof(*msgRaw) != payload.size()) {
        ROS_DEBUG("Error parsing ControllerInfo message");
        return;
    }

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
        ROS_DEBUG("Error parsing ControllerInfo message");
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

void LSPC_Callback_MathDump(std::shared_ptr<std::ofstream> log_file, const std::vector<uint8_t>& payload)
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
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(1))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        ROS_DEBUG_STREAM("Sending cmd_vel to Kugle");
        lspc::MessageTypesFromPC::VelocityReference_Heading_t payload;
        payload.vel.x = msg->linear.x;
        payload.vel.y = msg->linear.y;
        payload.vel.yaw = msg->angular.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference_Heading, payloadPacked);
    }

    lspcMutex->unlock();
}

void ROS_Callback_cmd_vel_inertial(const geometry_msgs::Twist::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(1))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        ROS_DEBUG_STREAM("Sending cmd_vel_inertial to Kugle");
        lspc::MessageTypesFromPC::VelocityReference_Inertial_t payload;
        payload.vel.x = msg->linear.x;
        payload.vel.y = msg->linear.y;
        payload.vel.yaw = msg->angular.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference_Inertial, payloadPacked);
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
        if (!in_param.compare("EnableLogOutput")) {
            out_param = lspc::ParameterLookup::EnableLogOutput;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("EnableRawSensorOutput")) {
            out_param = lspc::ParameterLookup::EnableRawSensorOutput;
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
        else if (!in_param.compare("ContinousSwitching")) {
            out_param = lspc::ParameterLookup::ContinousSwitching;
            out_valueType = lspc::ParameterLookup::_bool;
        }
        else if (!in_param.compare("eta")) {
            out_param = lspc::ParameterLookup::eta;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("epsilon")) {
            out_param = lspc::ParameterLookup::epsilon;
            out_valueType = lspc::ParameterLookup::_float;
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
        else if (!in_param.compare("Use2Lvelocity")) {
            out_param = lspc::ParameterLookup::Use2Lvelocity;
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
        else if (!in_param.compare("Mk")) {
            out_param = lspc::ParameterLookup::Mk;
            out_valueType = lspc::ParameterLookup::_float;
        }
        else if (!in_param.compare("Mb")) {
            out_param = lspc::ParameterLookup::Mb;
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
    for (std::string valueString : values) {
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
        lspcMutex->unlock();
        return true; // timeout
    }

    lspcMutex->unlock();
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

    /* Wait for initial response */
    std::shared_ptr<std::vector<uint8_t>> response;
    if (!GetParameterResponse.pop(response, 2)) { // 2 seconds timeout
        ROS_DEBUG("Get parameter: Response timeout");
        lspcMutex->unlock();
        return false; // timeout
    }

    ROS_DEBUG_STREAM("Received GetParameter response package of size " << response->size());

    /* Parse response to identify how many packages that will follow */
    const lspc::MessageTypesToPC::GetParameter_t * msgResponse = reinterpret_cast<const lspc::MessageTypesToPC::GetParameter_t *>(response->data());
    if (response->size() <= sizeof(*msgResponse)) {
        ROS_DEBUG("Error parsing GetParameter response message - parameter value missing from response");
        return false;
    }

    if (msgResponse->param != msg.param) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect parameter ID)");
        lspcMutex->unlock();
        return false;
    }

    if (msgResponse->type != msg.type) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect parameter type)");
        lspcMutex->unlock();
        return false;
    }

    if (msgResponse->valueType != valueType) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect value type)");
        lspcMutex->unlock();
        return false;
    }

    if (msgResponse->arraySize != arraySize) {
        ROS_DEBUG("Get parameter: Response mismatch (incorrect array size)");
        lspcMutex->unlock();
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

    lspcMutex->unlock();
    return true;
}

void LSPC_Callback_GetParameter(const std::vector<uint8_t>& payload)
{
    ROS_DEBUG_STREAM("Get parameter info received");

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

    /* Wait for initial response */
    std::shared_ptr<std::vector<uint8_t>> response;
    if (!DumpParametersResponse.pop(response, 2)) { // 2 seconds timeout
        ROS_DEBUG("Dump parameters: Response timeout");
        lspcMutex->unlock();
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
            lspcMutex->unlock();
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

    lspcMutex->unlock();
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

    /* Wait for response */
    bool acknowledged;
    if (!StoreParametersResponse.pop(acknowledged, 2)) { // 2 seconds timeout
        ROS_DEBUG("Store parameters: Response timeout");
        lspcMutex->unlock();
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    lspcMutex->unlock();
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

bool ROS_Service_CalibrateIMU(kugle_srvs::CalibrateIMU::Request &req, kugle_srvs::CalibrateIMU::Response &res, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj)
{
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(100))) return false; // could not get lock

    if (!(*lspcObj)->isOpen()) {
        lspcMutex->unlock();
        return false; // connection is not open
    }

    lspc::MessageTypesFromPC::CalibrateIMU_t msg;
    msg.magic_key = 0x12345678;
    std::vector<uint8_t> payloadPacked((uint8_t *)&msg, (uint8_t *)&msg+sizeof(msg));
    (*lspcObj)->send(lspc::MessageTypesFromPC::CalibrateIMU, payloadPacked);

    /* Wait for response */
    bool acknowledged;
    if (!CalibrateIMUResponse.pop(acknowledged, 2)) { // 2 seconds timeout
        ROS_DEBUG("Calibrate IMU: Response timeout");
        lspcMutex->unlock();
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    lspcMutex->unlock();
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

    /* Wait for response */
    bool acknowledged;
    if (!RestartControllerResponse.pop(acknowledged, 2)) { // 2 seconds timeout
        ROS_DEBUG("Restart Controller: Response timeout");
        lspcMutex->unlock();
        return false; // timeout
    }

    res.acknowledged = acknowledged;

    lspcMutex->unlock();
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

    res.acknowledged = true;

    lspcMutex->unlock();
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

    res.acknowledged = true;

    lspcMutex->unlock();
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
        std::shared_ptr<std::ofstream> mathdumpFile = std::make_shared<std::ofstream>();

        std::string mathdumpFilename = GetFormattedTimestampCurrent() + ".txt";
        mathdumpFile->open(std::string(getenv("HOME")) + "/kugle_dump/" + mathdumpFilename, std::ofstream::trunc);
        ROS_DEBUG_STREAM("Created mathdump file: ~/kugle_dump/" << mathdumpFilename);

        lspcMutex->unlock();

        /* Register callbacks */
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::GetParameter, &LSPC_Callback_GetParameter);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SetParameterAck, &LSPC_Callback_SetParameterAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::DumpParameters, &LSPC_Callback_DumpParameters);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::StoreParametersAck, &LSPC_Callback_StoreParametersAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SystemInfo, &LSPC_Callback_SystemInfo);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::ControllerInfo, boost::bind(&LSPC_Callback_ControllerInfo, pub_controller_info, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::StateEstimates, boost::bind(&LSPC_Callback_StateEstimates, pub_odom, pub_state_estimate, tf_broadcaster, tf_listener, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_IMU_MPU9250, boost::bind(&LSPC_Callback_RawSensor_IMU_MPU9250, pub_imu, pub_mag, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_IMU_MTI200, &LSPC_Callback_RawSensor_IMU_MTI200);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_Battery, boost::bind(&LSPC_Callback_RawSensor_Battery, pub_battery, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_Encoders, boost::bind(&LSPC_Callback_RawSensor_Encoders, pub_encoders, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CalibrateIMUAck, &LSPC_Callback_CalibrateIMUAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RestartControllerAck, &LSPC_Callback_RestartControllerAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CPUload, boost::bind(&LSPC_Callback_CPUload, pub_mcu_load, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::MathDump, boost::bind(&LSPC_Callback_MathDump, mathdumpFile, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::Debug, boost::bind(&LSPC_Callback_Debug, pub_mcu_debug, _1));

        while ((*lspcObj)->isOpen() && ros::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!ros::ok())
            break;

        ROS_WARN("Connection lost to Kugle");
        lspcMutex->lock();
        delete(*lspcObj); // call destructor to disconnect completely and clean up used ressources

        mathdumpFile->close();
    }
}

int main(int argc, char **argv) {
	std::string nodeName = "kugle_driver";
	ros::init(argc, argv, nodeName.c_str());
    boost::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);  // ros::NodeHandle n
	ros::NodeHandle nParam("~"); // default/current namespace node handle

	ROS_DEBUG_STREAM("Starting Kugle driver...");

	std::string serial_port = "/dev/kugle";
	if (!nParam.getParam("serial_port", serial_port)) {
		ROS_WARN_STREAM("Serial port not set (Parameter: serial_port). Defaults to: /dev/kugle");
	}

	/* Create LSPC connection thread */
    std::shared_ptr<std::timed_mutex> lspcMutex = std::make_shared<std::timed_mutex>();
    std::shared_ptr<lspc::Socket *> lspcObj = std::make_shared<lspc::Socket *>(nullptr);  // read about shared_ptr here: https://www.acodersjourney.com/top-10-dumb-mistakes-avoid-c-11-smart-pointers/

    std::promise<void> exitSignal;
    std::future<void> exitSignalObj = exitSignal.get_future();
    std::thread connectionThread = std::thread(&LSPC_ConnectionThread, n, serial_port, lspcMutex, lspcObj, std::move(exitSignalObj));

    /* Configure subscribers */
    //std::shared_ptr<ros::Timer> reference_timeout_timer = std::make_shared<ros::Timer>(n->createTimer(ros::Duration(0.1), boost::bind(&ROS_Callback_Heartbeat, lspcMutex, lspcObj, reference_timeout_timer, _1))); // Configure 10 Hz heartbeat timer
    ros::Subscriber sub_cmd_vel = n->subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ROS_Callback_cmd_vel, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_cmd_vel_inertial = n->subscribe<geometry_msgs::Twist>("cmd_vel_inertial", 1, boost::bind(&ROS_Callback_cmd_vel_inertial, _1, lspcMutex, lspcObj));

    /* Configure node services */
	ros::ServiceServer serv_set_parameter = n->advertiseService<kugle_srvs::SetParameter::Request, kugle_srvs::SetParameter::Response>("/kugle/set_parameter", boost::bind(&ROS_Service_SetParameter, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_get_parameter = n->advertiseService<kugle_srvs::GetParameter::Request, kugle_srvs::GetParameter::Response>("/kugle/get_parameter", boost::bind(&ROS_Service_GetParameter, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_dump_parameters = n->advertiseService<kugle_srvs::DumpParameters::Request, kugle_srvs::DumpParameters::Response>("/kugle/dump_parameters", boost::bind(&ROS_Service_DumpParameters, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_store_parameters = n->advertiseService<kugle_srvs::StoreParameters::Request, kugle_srvs::StoreParameters::Response>("/kugle/store_parameters", boost::bind(&ROS_Service_StoreParameters, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_calibrate_imu = n->advertiseService<kugle_srvs::CalibrateIMU::Request, kugle_srvs::CalibrateIMU::Response>("/kugle/calibrate_imu", boost::bind(&ROS_Service_CalibrateIMU, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_reboot = n->advertiseService<kugle_srvs::Reboot::Request, kugle_srvs::Reboot::Response>("/kugle/reboot", boost::bind(&ROS_Service_Reboot, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_enter_bootloader = n->advertiseService<kugle_srvs::EnterBootloader::Request, kugle_srvs::EnterBootloader::Response>("/kugle/enter_bootloader", boost::bind(&ROS_Service_EnterBootloader, _1, _2, lspcMutex, lspcObj));
    ros::ServiceServer serv_restart_controller = n->advertiseService<kugle_srvs::RestartController::Request, kugle_srvs::RestartController::Response>("/kugle/restart_controller", boost::bind(&ROS_Service_RestartController, _1, _2, lspcMutex, lspcObj));

    ros::spin();

    std::cout << "Shutting down Kugle driver" << std::endl;

    //reference_timeout_timer->stop();

    exitSignal.set_value();
    if (connectionThread.joinable())
        connectionThread.join();
}
