/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

#include <kugle_msgs/BalanceControllerReference.h>
#include <kugle_srvs/SetParameter.h>

ros::Publisher velocityReferencePub;
ros::Publisher balanceControllerReferencePub;
ros::Subscriber joystickSub;
double maximum_linear_velocity;
double rate_limit_linear_velocity;
double maximum_angular_velocity;
double rate_limit_angular_velocity;
double maximum_angle_degree;
double rate_limit_angle_degree;

typedef struct VelocityReference_t {
    double x;
    double y;
    double yawVel;
} VelocityReference_t;

ros::Time ReferenceTime;

VelocityReference_t VelocityReference;
VelocityReference_t VelocityReferenceRateLimited;
VelocityReference_t VelocityReference_prev;

double RPYReference[3];
double RPYReferenceRateLimited[2];
double RPYReference_prev[2];

bool AngleControlMode = false;
bool ControllerEnabled = false;

bool Cross_prev = false;
bool Cross_waitForRelease = false;
ros::Time Cross_press_time;

bool Square_prev = false;
bool Square_waitForRelease = false;
ros::Time Square_press_time;

void ToggleControllerMode()
{
    ControllerEnabled = !ControllerEnabled;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<kugle_srvs::SetParameter>("/kugle/set_parameter");
    kugle_srvs::SetParameter srv;

    srv.request.type = "controller";
    srv.request.param = "mode";

    if (ControllerEnabled && AngleControlMode)
        srv.request.value = "QUATERNION_CONTROL";
    else if (ControllerEnabled && !AngleControlMode)
        srv.request.value = "VELOCITY_CONTROL";
    else
        srv.request.value = "OFF";

    if (client.call(srv))
    {
        std::cout << "Switched control mode to: " << srv.request.value << std::endl;
    }
    else
    {
        std::cout << "Failed to switch control mode" << std::endl;
    }
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    ReferenceTime = ros::Time::now();
    VelocityReference.x = maximum_linear_velocity * msg->axes[1]; // left stick Y
    VelocityReference.y = maximum_linear_velocity * msg->axes[0]; // left stick X
    VelocityReference.yawVel = maximum_angular_velocity * msg->axes[2]; // right stick X
    RPYReference[0] = -maximum_angle_degree * msg->axes[0]; // left stick X
    RPYReference[1] = maximum_angle_degree * msg->axes[1]; // left stick Y

    if (msg->buttons[1]) { // cross button
        if (!Cross_prev) {
            Cross_press_time = ros::Time::now();
            Cross_prev = true;
        }
        else if (!Cross_waitForRelease) {
            ros::Duration diff = ros::Time::now() - Cross_press_time;
            if (diff.toSec() > 0.05) { // button pressed and held
                ToggleControllerMode();
                Cross_waitForRelease = true;
            }
        }
    } else {
        Cross_prev = false;
        Cross_waitForRelease = false;
        Cross_press_time.init();
    }

    if (msg->buttons[0]) { // square button
        if (!Square_prev) {
            Square_press_time = ros::Time::now();
            Square_prev = true;
        }
        else if (!Square_waitForRelease) {
            ros::Duration diff = ros::Time::now() - Square_press_time;
            if (diff.toSec() > 0.05) { // button pressed and held
                AngleControlMode = !AngleControlMode;
                if (AngleControlMode)
                    std::cout << "Angle control mode selected" << std::endl;
                else
                    std::cout << "Velocity control mode selected" << std::endl;
                ControllerEnabled = !ControllerEnabled;
                ToggleControllerMode();
                Square_waitForRelease = true;
            }
        }
    } else {
        Square_prev = false;
        Square_waitForRelease = false;
        Square_press_time.init();
    }

    /*
        this->buttonSq = joy->buttons[0];
        this->buttonX = joy->buttons[1];
        this->buttonO = joy->buttons[2];
        this->buttonTr = joy->buttons[3];
        this->buttonTouch = joy->buttons[13];
        this->l1 = joy->buttons[4];
        this->r1 = joy->buttons[5];

        this->arrowsX = joy->axes[9];
        this->arrowsY = joy->axes[10];
        this->l2 = joy->axes[3];
        this->r2 = joy->axes[4];

        this->leftStickX = joy->axes[0];
        this->leftStickY = joy->axes[1];
        this->rightStickX = joy->axes[2];
        this->rightStickY = joy->axes[5];
    */
}

double RateLimiter(double in, double prev, double dt, double rate_limit)
{
    double acceleration = (in - prev) / dt;
    double out = prev + dt * copysign(fmin(fabs(acceleration), rate_limit), acceleration);
    return out;
}

void PublishVelocity(double loop_time, double time_out)
{
    geometry_msgs::Twist velocityMsg;
    ros::Time current_ros_time = ros::Time::now();
    ros::Duration diff = current_ros_time - ReferenceTime;

    velocityMsg.linear.x = 0;
    velocityMsg.linear.y = 0;
    velocityMsg.angular.x = 0;
    velocityMsg.angular.y = 0;
    velocityMsg.angular.z = 0;

    if (diff.toSec() < time_out) { // update message with joystick-based velocity reference
        // Perform rate limitation of output
        VelocityReferenceRateLimited.x = RateLimiter(VelocityReference.x, VelocityReference_prev.x, loop_time, rate_limit_linear_velocity);
        VelocityReferenceRateLimited.y = RateLimiter(VelocityReference.y, VelocityReference_prev.y, loop_time, rate_limit_linear_velocity);
        VelocityReferenceRateLimited.yawVel = RateLimiter(VelocityReference.yawVel, VelocityReference_prev.yawVel, loop_time, rate_limit_angular_velocity);
        VelocityReference_prev = VelocityReferenceRateLimited;

        // Prepare message to send
        velocityMsg.linear.x = VelocityReferenceRateLimited.x;
        velocityMsg.linear.y = VelocityReferenceRateLimited.y;
        velocityMsg.angular.z = VelocityReferenceRateLimited.yawVel;
    }

    if (!AngleControlMode)
        velocityReferencePub.publish(velocityMsg);
}

void PublishQuaternion(double loop_time, double time_out)
{
    kugle_msgs::BalanceControllerReference balanceControllerReferenceMsg;
    tf2::Quaternion q;
    ros::Time current_ros_time = ros::Time::now();
    ros::Duration diff = current_ros_time - ReferenceTime;

    balanceControllerReferenceMsg.q.w = 1;
    balanceControllerReferenceMsg.q.x = 0;
    balanceControllerReferenceMsg.q.y = 0;
    balanceControllerReferenceMsg.q.z = 0;
    balanceControllerReferenceMsg.omega.x = 0;
    balanceControllerReferenceMsg.omega.y = 0;
    balanceControllerReferenceMsg.omega.z = 0;

    if (diff.toSec() < time_out) { // update message with joystick-based velocity reference
        // Perform rate limitation of output
        RPYReferenceRateLimited[0] = RateLimiter(RPYReference[0], RPYReference_prev[0], loop_time, rate_limit_angle_degree);
        RPYReferenceRateLimited[1] = RateLimiter(RPYReference[1], RPYReference_prev[1], loop_time, rate_limit_angle_degree);

        RPYReference_prev[0] = RPYReferenceRateLimited[0];
        RPYReference_prev[1] = RPYReferenceRateLimited[1];

        // Integrate yaw velocity to generate yaw angle
        RPYReference[2] += loop_time * VelocityReferenceRateLimited.yawVel;

        // Compute quaternion based on
        q.setRPY(M_PI/180.0f*RPYReferenceRateLimited[0], M_PI/180.0f*RPYReferenceRateLimited[1], RPYReference[2]);

        balanceControllerReferenceMsg.q.w = q.w();
        balanceControllerReferenceMsg.q.x = q.x();
        balanceControllerReferenceMsg.q.y = q.y();
        balanceControllerReferenceMsg.q.z = q.z();
        balanceControllerReferenceMsg.omega.x = 0;
        balanceControllerReferenceMsg.omega.y = 0;
        balanceControllerReferenceMsg.omega.z = VelocityReferenceRateLimited.yawVel;
    }

    if (AngleControlMode)
        balanceControllerReferencePub.publish(balanceControllerReferenceMsg);
}

int main(int argc, char **argv) {
    std::string nodeName = "joystick_mapper";
    ros::init(argc, argv, nodeName.c_str());
    ros::NodeHandle n; // default/current namespace node handle
    ros::NodeHandle nParam("~"); // private node handle

    // Publish to reference topic
    velocityReferencePub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    balanceControllerReferencePub = n.advertise<kugle_msgs::BalanceControllerReference>("cmd_combined", 1000);

    // Subscribe to joystick topic
    joystickSub = n.subscribe("joy", 1000, &joystickCallback);

    nParam.param("angle_mode", AngleControlMode, false);
    if (AngleControlMode)
        std::cout << "Angle control mode selected" << std::endl;
    else
        std::cout << "Velocity control mode selected" << std::endl;

    // Get maximum velocity
    nParam.param("maximum_linear_velocity", maximum_linear_velocity, double(0.5));
    nParam.param("rate_limit_linear_velocity", rate_limit_linear_velocity, double(0.5));
    nParam.param("maximum_angular_velocity", maximum_angular_velocity, double(0.5));
    nParam.param("rate_limit_angular_velocity", rate_limit_angular_velocity, double(0.5));
    nParam.param("maximum_angle_degree", maximum_angle_degree, double(0.5));
    nParam.param("rate_limit_angle_degree", rate_limit_angle_degree, double(0.5));

    // Get publish rate
    int publish_rate;
    nParam.param("publish_rate", publish_rate, int(10));

    VelocityReference.x = 0;
    VelocityReference.y = 0;
    VelocityReference.yawVel = 0;
    VelocityReference_prev.x = 0;
    VelocityReference_prev.y = 0;
    VelocityReference_prev.yawVel = 0;
    RPYReference[0] = 0;
    RPYReference[1] = 0;
    RPYReference[2] = 0;
    RPYReference_prev[0] = 0;
    RPYReference_prev[1] = 0;

    //ros::spin();
    ros::Rate loop_rate(publish_rate); // publish at specified publish rate
    while (ros::ok())
    {
        ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)

        PublishVelocity(1.0/publish_rate, 0.5); // timeout after 500 ms
        PublishQuaternion(1.0/publish_rate, 0.5); // timeout after 500 ms

        loop_rate.sleep();
    }
}
