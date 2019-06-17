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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "MPC.h"
#include "Trajectory.h"
#include "Path.h"

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kugle_msgs/StateEstimate.h>
#include <std_srvs/Empty.h>

#include <boost/math/quaternion.hpp> // see https://www.boost.org/doc/libs/1_66_0/libs/math/doc/html/quaternions.html
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

/* For plotting/visualization */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

boost::math::quaternion<double> Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll);
void Quaternion_quat2eul_zyx(boost::math::quaternion<double> q, float yaw_pitch_roll[3]);
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
void StateEstimateCallback(const kugle_msgs::StateEstimate::ConstPtr& msg);
void GetLocalizationBasedPositionEstimate(tf::TransformListener * tfListener, std::string map_frame, std::string base_link_frame);
void GazeboReset(ros::NodeHandle &n);
void GazeboPause(ros::NodeHandle &n);
void GazeboResume(ros::NodeHandle &n);

nav_msgs::Odometry odom;
kugle_msgs::StateEstimate stateEstimate;
struct {
    double time;
    Eigen::Vector2d position;
    boost::math::quaternion<double> quaternion;
} localizationEstimate;

int main(int argc, char **argv) {
	std::string nodeName = "mpc_test";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n;
    ros::NodeHandle nParam("~"); // default/current namespace node handle
    geometry_msgs::Twist cmd_vel;

	double rate = 10;
    if (!nParam.getParam("rate", rate)) {
        ROS_WARN_STREAM("Loop rate not set (Parameter: rate). Defaults to: 10 Hz");
    }

    std::string map_frame = "map";
    if (!nParam.getParam("map_frame", map_frame)) {
        ROS_WARN_STREAM("Map frame not set (Parameter: map_frame). Defaults to: map");
    }

    std::string base_link_frame = "base_link";
    if (!nParam.getParam("base_link_frame", base_link_frame)) {
        ROS_WARN_STREAM("base_link frame not set (Parameter: base_link_frame). Defaults to: base_link");
    }

    bool use_localization = false;
    if (!nParam.getParam("use_localization", use_localization)) {
        ROS_WARN_STREAM("Use localization flag not set (Parameter: use_localization). Defaults to: false");
    }

    double desired_velocity = 1.0;
    if (!nParam.getParam("desired_velocity", use_localization)) {
        ROS_WARN_STREAM("Desired velocity not set (Parameter: desired_velocity). Defaults to: 1.0 m/s");
    }

    ros::Rate loop_rate(rate); // Loop rate
    ros::Publisher pub_cmd_vel_angular_inertial = n.advertise<geometry_msgs::Twist>("cmd_vel_inertial", 1000);
    ros::Subscriber sub_odom = n.subscribe("odom", 1000, &OdometryCallback);
    ros::Subscriber sub_stateestimate = n.subscribe("StateEstimate", 1000, &StateEstimateCallback);
    tf::TransformListener tfListener;

    // Wait for first state estimate message
    stateEstimate.mcu_time = 0;
    localizationEstimate.time = 0;
    while (ros::ok())
    {
        if (stateEstimate.mcu_time > 0 && (localizationEstimate.time > 0 || !use_localization)) break;

        if (use_localization)
            GetLocalizationBasedPositionEstimate(&tfListener, map_frame, base_link_frame);

        if (use_localization)
            std::cout << "Waiting for localization" << std::endl;
        else
            std::cout << "Waiting for State Estimate message" << std::endl;

        /*std::cout << "Position estimate" << std::endl;
        std::cout << "odom = " << std::endl << odom.pose.pose.position;
        std::cout << "stateEst = " << std::endl << "(" << stateEstimate.position[0] << ", " << stateEstimate.position[1] << ")" << std::endl;
        std::cout << "tf = " << std::endl << localizationEstimate.position << std::endl;
        std::cout << std::endl;*/

        ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
        loop_rate.sleep();
    }

	MPC::MPC mpc;
    mpc.setDesiredVelocity(desired_velocity);

    boost::math::quaternion<double> RobotQuaternion;
    Eigen::Vector2d RobotPos;
    Eigen::Vector2d RobotVelocity; // inertial frame
    MPC::MPC::state_t state;

    RobotQuaternion = boost::math::quaternion<double>(stateEstimate.q.w, stateEstimate.q.x, stateEstimate.q.y, stateEstimate.q.z);
    if (use_localization) {
        RobotPos(0) = localizationEstimate.position[0];
        RobotPos(1) = localizationEstimate.position[1];
    } else {
        RobotPos(0) = stateEstimate.position[0];
        RobotPos(1) = stateEstimate.position[1];
    }
    RobotVelocity(0) = stateEstimate.velocity[0];
    RobotVelocity(1) = stateEstimate.velocity[1];

    MPC::Trajectory trajectory = MPC::Trajectory::GenerateCircleTrajectory(RobotPos);
	mpc.setTrajectory(trajectory, RobotPos, Eigen::Vector2d(0.01,0), RobotQuaternion);
	//mpc.setXYreferencePosition(RobotPos(0), RobotPos(1));
	mpc.setCurrentState(RobotPos, Eigen::Vector2d(0.01,0), RobotQuaternion);
	//mpc.setObstacles(obstacles);


	while (ros::ok())
	{
		/*if ((i % 10) == 0) {
		    mpc.setTrajectory(trajectory, state.position, state.velocity, state.quaternion);
		    mpc.setObstacles(obstacles);
		    //mpc.setObstacles(obstacles);
		}*/

        if (use_localization)
            GetLocalizationBasedPositionEstimate(&tfListener, map_frame, base_link_frame);

        RobotQuaternion = boost::math::quaternion<double>(stateEstimate.q.w, stateEstimate.q.x, stateEstimate.q.y, stateEstimate.q.z);
        if (use_localization) {
            RobotPos(0) = localizationEstimate.position[0];
            RobotPos(1) = localizationEstimate.position[1];
        } else {
            RobotPos(0) = stateEstimate.position[0];
            RobotPos(1) = stateEstimate.position[1];
        }
        RobotVelocity(0) = stateEstimate.velocity[0];
        RobotVelocity(1) = stateEstimate.velocity[1];

		mpc.setTrajectory(trajectory, RobotPos, RobotVelocity, RobotQuaternion);
		mpc.setCurrentState(RobotPos, RobotVelocity, RobotQuaternion);

		cv::Mat imgTrajectory = cv::Mat( 500, 1333, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
		trajectory.plot(imgTrajectory, cv::Scalar(0, 0, 255), false, false, -8, -3, 8, 3);
		mpc.PlotRobot(imgTrajectory, cv::Scalar(255, 0, 0), false, -8, -3, 8, 3);
		mpc.PlotObstacles(imgTrajectory, cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), false, -8, -3, 8, 3);
		cv::imshow("Trajectory", imgTrajectory);

		cv::Mat imgWindowTrajectory = cv::Mat( 500, 500, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
		mpc.getCurrentTrajectory().plot(imgWindowTrajectory, cv::Scalar(0, 255, 0), true, false, -4, -4, 4, 4);
		mpc.PlotRobotInWindow(imgWindowTrajectory, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
		mpc.PlotObstaclesInWindow(imgWindowTrajectory, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
		cv::imshow("Window", imgWindowTrajectory);

		cv::Mat imgWindowPath = cv::Mat( 500, 500, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
		mpc.getCurrentPath().plot(imgWindowPath, cv::Scalar(0, 255, 0), true, -4, -4, 4, 4);
		mpc.getCurrentPath().PlotPoint(mpc.getClosestPointOnPath(), imgWindowPath, cv::Scalar(0, 0, 255), true, -4, -4, 4, 4);
		mpc.PlotRobotInWindow(imgWindowPath, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
		mpc.PlotObstaclesInWindow(imgWindowPath, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
		cv::imshow("Path", imgWindowPath);

		mpc.Step();

		Eigen::Vector2d angularVelocityReference(0.0, 0.0);
		if (mpc.getStatus() != MPC::MPC::SUCCESS) {
		    std::cout << "MPC Solver failed" << std::endl;
            GazeboPause(n);
		    cv::waitKey(0);
            GazeboResume(n);
		}

		state = mpc.getHorizonState();
		std::cout << "Predicted states:" << std::endl;
		std::cout << "path distance = " << state.pathDistance << std::endl;
		std::cout << "position = " << std::endl << state.position << std::endl;
		std::cout << "velocity = " << std::endl << state.velocity << std::endl;
		std::cout << "quaternion = " << std::endl << state.quaternion << std::endl;
		std::cout << std::endl;

		angularVelocityReference = mpc.getInertialAngularVelocity();
		std::cout << "Control output (angular velocity):" << std::endl;
		std::cout << "   x = " << angularVelocityReference[0] << std::endl;
		std::cout << "   y = " << angularVelocityReference[1] << std::endl;

		cv::Mat imgPredicted = cv::Mat( 500, 500, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
		mpc.PlotPredictedTrajectory(imgPredicted, -4, -4, 4, 4);
		mpc.PlotObstaclesInWindow(imgPredicted, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
		mpc.PlotRobotInWindow(imgPredicted, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
		cv::imshow("Predicted", imgPredicted);

        cmd_vel.angular.x = angularVelocityReference[0];
        cmd_vel.angular.y = angularVelocityReference[1];
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;

        pub_cmd_vel_angular_inertial.publish(cmd_vel);

        cv::waitKey(1);
		ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)
		loop_rate.sleep();
	}
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received odometry, translational velocity: (" << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ")");
    odom = *msg;
}

void StateEstimateCallback(const kugle_msgs::StateEstimate::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received StateEstimate message");
    stateEstimate = *msg;
}

void GetLocalizationBasedPositionEstimate(tf::TransformListener * tfListener, std::string map_frame, std::string base_link_frame)
{
    ros::Time currentTime = ros::Time::now();

    // Look-up current estimate of robot in map
    tf::StampedTransform tf_base_link;
    try {
        tfListener->lookupTransform(map_frame, base_link_frame, ros::Time(0), tf_base_link);

        Eigen::Vector2d robot_center(tf_base_link.getOrigin().x(), tf_base_link.getOrigin().y());
        tf::Quaternion q = tf_base_link.getRotation();

        localizationEstimate.quaternion =  boost::math::quaternion<double>(q.w(), q.x(), q.y(), q.z());
        localizationEstimate.position = robot_center;
        localizationEstimate.time = currentTime.toSec();
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}

void GazeboReset(ros::NodeHandle &n)
{
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("Success calling /gazebo/reset_world");
    }
    else
    {
        ROS_ERROR("Failed calling /gazebo/reset_world");
    }
}

void GazeboPause(ros::NodeHandle &n)
{
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("Success calling /gazebo/pause_physics");
    }
    else
    {
        ROS_ERROR("Failed calling /gazebo/pause_physics");
    }
}

void GazeboResume(ros::NodeHandle &n)
{
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty srv;
    if (client.call(srv))
    {
        ROS_INFO("Success calling /gazebo/unpause_physics");
    }
    else
    {
        ROS_ERROR("Failed calling /gazebo/unpause_physics");
    }
}

boost::math::quaternion<double> Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll)
{
	const float cx = cosf(roll/2);
	const float cy = cosf(pitch/2);
	const float cz = cosf(yaw/2);
	const float sx = sinf(roll/2);
	const float sy = sinf(pitch/2);
	const float sz = sinf(yaw/2);

	double q[4];
	q[0] = cz*cy*cx+sz*sy*sx;
	q[1] = cz*cy*sx-sz*sy*cx;
	q[2] = cz*sy*cx+sz*cy*sx;
	q[3] = sz*cy*cx-cz*sy*sx;

	return boost::math::quaternion<double>(q[0], q[1], q[2], q[3]);
}

void Quaternion_quat2eul_zyx(boost::math::quaternion<double> q, float yaw_pitch_roll[3])
{
	// Normalize quaternion
	q /= norm(q);

	float qw = q.R_component_1();
	float qx = q.R_component_2();
	float qy = q.R_component_3();
	float qz = q.R_component_4();

	float aSinInput = -2*(qx*qz-qw*qy);
	aSinInput = fmax(fmin(aSinInput, 1.f), -1.f);

	yaw_pitch_roll[0] = atan2( 2*(qx*qy+qw*qz), qw*qw + qx*qx - qy*qy - qz*qz ); // yaw
	yaw_pitch_roll[1] = asin( aSinInput ); // pitch
	yaw_pitch_roll[2] = atan2( 2*(qy*qz+qw*qx), qw*qw - qx*qx - qy*qy + qz*qz ); // roll
}