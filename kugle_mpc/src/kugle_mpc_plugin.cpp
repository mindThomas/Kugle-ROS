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

#include <kugle_mpc/kugle_mpc_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

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

#include <boost/math/quaternion.hpp> // see https://www.boost.org/doc/libs/1_66_0/libs/math/doc/html/quaternions.html
#include <boost/thread/thread.hpp>

/* For plotting/visualization */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


PLUGINLIB_EXPORT_CLASS(KugleMPC, nav_core::BaseLocalPlanner);

KugleMPC::KugleMPC(){
	//nothing ot fill in here; "initialize" will do the initializations
}

KugleMPC::~KugleMPC() {
    stopThread_ = true;
    mpc_signalling_cv.notify_all();
    if (publisherThread_.joinable())
        publisherThread_.join();
    if (mpcThread_.joinable())
        mpcThread_.join();
}

//put inits here:
void KugleMPC::initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros){
	ros::NodeHandle nh(name);
    ros::NodeHandle private_nh("~/" + name);

	tfListener_ = tf;
	costmap_ = costmap_ros;
	global_plan_changed_ = false;

	pub_global_plan_ = nh.advertise<nav_msgs::Path>("global_plan", 1000);
    pub_local_plan_ = nh.advertise<nav_msgs::Path>("local_plan", 1000);
    pub_cmd_vel_angular_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_angular", 1000);

    private_nh.param("map_frame", map_frame_, std::string("map"));
	private_nh.param("mpc_frame", mpc_frame_, std::string("odom"));
	private_nh.param("heading_frame", heading_frame_, std::string("heading"));
    private_nh.param("base_link_frame", base_link_frame_, std::string("base_link"));

    private_nh.param("width", window_width_, double(4.0));
    private_nh.param("height", window_height_, double(4.0));

    sub_odom_ = nh.subscribe("odom", 1000, &KugleMPC::OdometryCallback, this);

    currentAttitude_.w = 1;
    currentAttitude_.x = 0;
    currentAttitude_.y = 0;
    currentAttitude_.z = 0;

    prevTime_ = ros::Time::now();

    stopThread_ = false;
    mpcThread_ = boost::thread(boost::bind(&KugleMPC::MPC_Thread, this));
    publisherThread_ = boost::thread(boost::bind(&KugleMPC::AngularVelocityPublisherThread, this));

}


void KugleMPC::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received odometry, translational velocity: (" << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ")");
    currentAttitude_ = msg->pose.pose.orientation;
    currentVelocityHeadingFrame_ = msg->twist.twist.linear; // notice that this velocity is in heading frame
    odomTime_ = msg->header.stamp;
}

bool KugleMPC::isGoalReached(){
	return goal_reached_;
}

bool KugleMPC::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan){
	//The "plan" that comes in here is a bunch of poses of variable length, calculated by the global planner(?).
	//We're just ignoring it entirely, but an actual planner would probably take this opportunity
	//to store it somewhere and maybe update components that reference it.
	ROS_INFO("GOT A PLAN OF SIZE %lu", plan.size());

	nav_msgs::Path planMsg;
	planMsg.header.stamp = ros::Time::now();
	planMsg.header.frame_id = map_frame_;
	planMsg.poses.insert(planMsg.poses.begin(), plan.begin(), plan.end());
	pub_global_plan_.publish(planMsg);

    //if (abs(global_plan_size_ - plan.size()) > 10) { // the plan needs to change by at least 10 elements (increase og decrease) for us to update the trajectory
    //if (goal_reached_ || global_plan_.size() == 0) {
    if (fabs(goal_point_.point[0] - plan.back().pose.position.x) > 0.05 || fabs(goal_point_.point[1] - plan.back().pose.position.y) > 0.05) { // goal point changed
        ROS_INFO("GLOBAL PLAN CHANGED!");

        global_plan_mutex_.lock();
        global_plan_.clear();
        for (unsigned int i = 0; i < plan.size() - 1; i++) // add points up to the last point, since the last point has to be added as a goal point
        {
            global_plan_.AddPoint(Eigen::Vector2d(plan.at(i).pose.position.x, plan.at(i).pose.position.y));
        }
        goal_point_ = MPC::TrajectoryPoint(global_plan_.back().seq+1, plan.back().pose.position.x, plan.back().pose.position.y, true);
        global_plan_.AddPoint(goal_point_); // set goal point

        //global_plan_.plot(false, false, -10, -10, 10, 10);

        global_plan_size_ = plan.size();
        goal_reached_ = false;

        global_plan_changed_ = true;
        global_plan_mutex_.unlock();
    }

	return true;
}

bool KugleMPC::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    // This function is called periodically with the "controller_frequency" rate set in "move_base_params.yaml"

    //This is the meat-and-potatoes of the plugin, where velocities are actually generated.
    //in this minimal case, simply specify constants; more generally, choose vx and omega_z
    // intelligently based on the goal and the environment
    // When isGoalReached() is false, computeVelocityCommands will be called each iteration
    // of the controller--which is a settable parameter.  On each iteration, values in
    // cmd_vel should be computed and set, and these values will be published by move_base
    // to command robot motion
    cmd_vel.angular.x = appliedAngularVelocityReference_[0];
    cmd_vel.angular.y = appliedAngularVelocityReference_[1];
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    return true;
}

void KugleMPC::MPC_Thread()
{
    unsigned int updateTrajectoryPrescaler = 0;
    bool enablePositionHold = true;

    goal_reached_ = true;

    while (!stopThread_) {
        mpc_processing_mutex_.lock();
        {
            std::unique_lock<std::mutex> lck(mpc_signalling_mutex_);
            mpc_signalling_cv.wait(lck);
        }

        ros::Time currentTime = ros::Time::now();

        // Look-up current estimate of robot in map
        tf::StampedTransform tf_base_link;
        try {
            tfListener_->lookupTransform(map_frame_, base_link_frame_, ros::Time(0), tf_base_link);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        ROS_INFO_STREAM("Difference in time between TF time and current time: "
                                << ros::Duration(currentTime - tf_base_link.stamp_).toSec() * 1000 << " ms");
        ROS_INFO_STREAM("Difference in time between ODOM time and current time: "
                                << ros::Duration(currentTime - odomTime_).toSec() * 1000 << " ms");
        ROS_INFO_STREAM(
                "Delta time since last execution (dt) = " << ros::Duration(currentTime - prevTime_).toSec() * 1000
                                                          << " ms");
        if (std::abs((ros::Duration(currentTime - prevTime_).toSec() / mpc_.getSampleTime()) - 1.0) >
            0.1) { // more than 10% difference
            ROS_WARN("move_base configured frequency/rate does not match discretized MPC sample time!");
        }
        prevTime_ = currentTime;

        /* Assemble state variables to be fed into MPC */
        Eigen::Vector2d robot_center(tf_base_link.getOrigin().x(), tf_base_link.getOrigin().y());
        tf::Quaternion robot_quaternion = tf_base_link.getRotation();
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(robot_quaternion);
        mat.getEulerYPR(yaw, pitch, roll);

        boost::math::quaternion<double> attitude_quaternion(robot_quaternion.w(), robot_quaternion.x(),
                                                            robot_quaternion.y(), robot_quaternion.z());
        double RobotYaw = mpc_.extractHeading(attitude_quaternion);
        Eigen::Matrix2d R_heading = Eigen::Rotation2Dd(RobotYaw).toRotationMatrix();
        Eigen::Vector2d robot_velocity_inertial =
                R_heading * Eigen::Vector2d(currentVelocityHeadingFrame_.x, currentVelocityHeadingFrame_.y);

        std::cout << "Current states:" << std::endl;
        std::cout << "position = " << std::endl << robot_center << std::endl;
        std::cout << "velocity = " << std::endl << robot_velocity_inertial << std::endl;
        std::cout << "quaternion = " << std::endl << attitude_quaternion << std::endl;
        std::cout << std::endl;

        global_plan_mutex_.lock();

        if (global_plan_changed_) {
            global_plan_changed_ = false;
            enablePositionHold = false;
            updateTrajectoryPrescaler = 0;
            /* Update MPC with changed trajectory */
            //mpc_.setTrajectory(global_plan_, robot_center, robot_velocity_inertial, attitude_quaternion);

            /*global_plan_.WindowExtract(local_plan_, robot_center, yaw, window_width_, window_height_);
            local_plan_.plot(true, false, -window_height_/2, -window_width_/2, window_height_/2, window_width_/2); // notice that width and height is swapped, since x is plotted up
            local_path_ = MPC::Path(local_plan_, 6, true, false);
            local_path_.plot(true, -window_height_/2, -window_width_/2, window_height_/2, window_width_/2);*/


            /*if ()
                mpc_.setTrajectory(global_plan_, robot_center, robot_velocity_inertial, attitude_quaternion);*/

        }


        int lastSeqID = global_plan_.GetLastSequenceID();
        MPC::TrajectoryPoint goal;
        bool foundPoint = global_plan_.find(lastSeqID, goal);

        double distanceToGoal = 0;
        if (foundPoint) {
            distanceToGoal = goal.EuclideanDistance(robot_center);
            std::cout << "Distance to goal: " << distanceToGoal << std::endl;
            if (mpc_.getCurrentTrajectory().includesGoal())
                std::cout << "Trajectory includes goal" << std::endl;

            if (distanceToGoal < 0.05 && fabs(robot_velocity_inertial[0]) < 0.01 && fabs(robot_velocity_inertial[1]) < 0.01)
                goal_reached_ = true;
        } else {
            goal_reached_ = false;
        }

        //if (distanceToGoal > 0.5 && global_plan_.size() > 10 /* || !mpc_.getCurrentTrajectory().includesGoal()*/) {
        if ((updateTrajectoryPrescaler % 2) == 0 && global_plan_.size() > 0) {
            mpc_.setTrajectory(global_plan_, robot_center, robot_velocity_inertial, attitude_quaternion);
        }

        if (enablePositionHold) {
            mpc_.setXYreferencePosition(robot_center[0], robot_center[1]);
            enablePositionHold = false;
        }

        global_plan_mutex_.unlock();

        /*if (distanceToGoal <= 1.0) {  // we have less than 1.0 meters to goal => just set the goal reference
            mpc_.setXYreferencePosition(goal.point[0], goal.point[1]);
        }*/

        /*std::cout << "Global plan distance: " << global_plan_.distance() << std::endl;
        int lastSeqID = global_plan_.GetLastSequenceID();
        std::cout << "Global plan lastSeqID: " << lastSeqID << std::endl;

        if (lastSeqID > 5) { // more than 5 points in trajectory - fit a path to the trajectory and do path following
            mpc_.setTrajectory(global_plan_, robot_center, robot_velocity_inertial, attitude_quaternion);
        } else { // less than 5 points, just set the final point/goal as the desired position
            MPC::TrajectoryPoint pRef;
            bool foundPoint = global_plan_.find(lastSeqID, pRef);
            if (foundPoint && pRef.seq != -1)
                mpc_.setXYreferencePosition(pRef.point[0], pRef.point[1]);
        }*/

        mpc_.setCurrentState(robot_center, robot_velocity_inertial, attitude_quaternion);
        std::cout << "Current path position = " << mpc_.getCurrentPathPosition() << std::endl;

        mpc_.Step();


        MPC::MPC::state_t state = mpc_.getHorizonState();
        Eigen::Vector2d angularVelocityReference(0.0, 0.0);

        if (mpc_.getStatus() == MPC::MPC::SUCCESS) {
            if (mpc_.getSolveTime() < mpc_.getSampleTime()) { // only use the MPC result if it is not outdated
                angularVelocityOutputs_mutex_.lock();
                angularVelocityReference = mpc_.getInertialAngularVelocity();
                angularVelocityOutputs_ = mpc_.getInertialAngularVelocityHorizon();
                angularVelocityOutputs_mutex_.unlock();
                PublishPredictedTrajectory();
            }
        } else {
            std::cout << "MPC Solver failed -- applying previously computed control action (if any)" << std::endl;
            mpc_.Reset();
        }

        std::cout << "Predicted states:" << std::endl;
        std::cout << "path length = " << state.pathDistance << std::endl;
        std::cout << "position = " << std::endl << state.position << std::endl;
        std::cout << "velocity = " << std::endl << state.velocity << std::endl;
        std::cout << "quaternion = " << std::endl << state.quaternion << std::endl;
        std::cout << std::endl;

        mpc_processing_mutex_.unlock();

        cv::Mat imgPredicted = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
        mpc_.getCurrentTrajectory().plot(imgPredicted, cv::Scalar(0, 0, 255), true, false, -4, -4, 4, 4);
        mpc_.PlotPredictedTrajectory(imgPredicted, -4, -4, 4, 4);
        mpc_.getCurrentPath().plot(imgPredicted, cv::Scalar(0, 255, 0), true, -4, -4, 4, 4);
        //mpc_.PlotObstaclesInWindow(imgPredicted, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        mpc_.PlotRobotInWindow(imgPredicted, cv::Scalar(255, 0, 0), true, -4, -4, 4, 4);
        cv::imshow("Predicted", imgPredicted);

        updateTrajectoryPrescaler++;
        cv::waitKey(1);


        /*if (mpc_.getStatus() == MPC::MPC::SUCCESS) {
            MPC::MPC::state_t state = mpc_.getHorizonState();
            std::cout << "Predicted states:" << std::endl;
            std::cout << "path distance = " << state.pathDistance << std::endl;
            std::cout << "position = " << std::endl << state.position << std::endl;
            std::cout << "velocity = " << std::endl << state.velocity << std::endl;
            std::cout << "quaternion = " << std::endl << state.quaternion << std::endl;
            std::cout << std::endl;

            angularVelocityReference = mpc_.getInertialAngularVelocity();
            std::cout << "Control output (angular velocity):" << std::endl;
            std::cout << "   x = " << angularVelocityReference[0] << std::endl;
            std::cout << "   y = " << angularVelocityReference[1] << std::endl;
            std::cout << std::endl;

            PublishPredictedTrajectory();

        } else {
            std::cout << "MPC Solver failed - Resetting..." << std::endl;
            mpc_.Reset();

            std::cout << "MPC Performing recovery by setting current position as reference" << std::endl;
            mpc_.setXYreferencePosition(robot_center[0], robot_center[1]);
            mpc_.setCurrentState(robot_center, robot_velocity_inertial, attitude_quaternion);

            mpc_.Step();

            MPC::MPC::state_t state = mpc_.getHorizonState();
            std::cout << "Predicted states:" << std::endl;
            std::cout << "path distance = " << state.pathDistance << std::endl;
            std::cout << "position = " << std::endl << state.position << std::endl;
            std::cout << "velocity = " << std::endl << state.velocity << std::endl;
            std::cout << "quaternion = " << std::endl << state.quaternion << std::endl;
            std::cout << std::endl;

            angularVelocityReference = mpc_.getInertialAngularVelocity();
            std::cout << "Control output (angular velocity):" << std::endl;
            std::cout << "   x = " << angularVelocityReference[0] << std::endl;
            std::cout << "   y = " << angularVelocityReference[1] << std::endl;
            std::cout << std::endl;
        }*/
    }
}

void KugleMPC::tic()
{
    gettimeofday(&timing_start_, 0);
}

double KugleMPC::toc()
{
    struct timeval toc;
    struct timeval temp;

    gettimeofday(&toc, 0);

    if ((toc.tv_usec - timing_start_.tv_usec) < 0)
    {
        temp.tv_sec = toc.tv_sec - timing_start_.tv_sec - 1;
        temp.tv_usec = 1000000 + toc.tv_usec - timing_start_.tv_usec;
    }
    else
    {
        temp.tv_sec = toc.tv_sec - timing_start_.tv_sec;
        temp.tv_usec = toc.tv_usec - timing_start_.tv_usec;
    }

    return (double)temp.tv_sec + (double)temp.tv_usec / 1e6;
}

void KugleMPC::AngularVelocityPublisherThread()
{
    ros::Rate loop_rate(1.0 / mpc_.getSampleTime());
    int64_t milliseconds_wait = 1000.0 * mpc_.getSampleTime();
    while (!stopThread_ && ros::ok()) {
        tic();
        {
            std::unique_lock<std::mutex> lck(mpc_signalling_mutex_);
            mpc_signalling_cv.notify_all();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }

        if (mpc_processing_mutex_.try_lock_for(std::chrono::milliseconds(milliseconds_wait))) {
            ROS_INFO("[OK]  MPC control output computed within time");
            mpc_processing_mutex_.unlock();
        } else {
            ROS_WARN("[ERR]  MPC computation took too long");
        }

        double computationTime = toc();
        ROS_INFO_STREAM("MPC computation time: " << computationTime * 1000 << " ms");

        PublishAngularVelocityControl();
        loop_rate.sleep();
    }
}

void KugleMPC::PublishPredictedTrajectory()
{
	nav_msgs::Path planMsg;
	planMsg.header.stamp = ros::Time::now();
	planMsg.header.frame_id = map_frame_;

	for (unsigned int i = 0; i < MPC::MPC::HorizonLength; i++) {
		auto state = mpc_.getHorizonState(i);

		geometry_msgs::PoseStamped pose;
		pose.header.seq = i;
        pose.header.stamp = planMsg.header.stamp;
		pose.header.frame_id = planMsg.header.frame_id;
		pose.pose.position.x = state.position[0];
        pose.pose.position.y = state.position[1];
        pose.pose.position.z = 0;
        pose.pose.orientation.w = state.quaternion.R_component_1();
        pose.pose.orientation.x = state.quaternion.R_component_2();
        pose.pose.orientation.y = state.quaternion.R_component_3();
        pose.pose.orientation.z = state.quaternion.R_component_4();

        planMsg.poses.push_back(pose);
	}

	pub_local_plan_.publish(planMsg);
}

void KugleMPC::PublishAngularVelocityControl()
{
    geometry_msgs::Twist cmd_vel;

    angularVelocityOutputs_mutex_.lock();
    // Apply next control action from angularVelocityOutputs vector (previous successfully computed horizon)
    if (angularVelocityOutputs_.size() > 0) {
        appliedAngularVelocityReference_[0] = angularVelocityOutputs_.back().first;
        appliedAngularVelocityReference_[1] = angularVelocityOutputs_.back().second;
        angularVelocityOutputs_.pop_back();
    } else {
        appliedAngularVelocityReference_[0] = 0;
        appliedAngularVelocityReference_[1] = 0;
        global_plan_.clear(); // clear current trajectory to enable position hold
    }
    angularVelocityOutputs_mutex_.unlock();

    std::cout << "Setting control output (angular velocity):" << std::endl;
    std::cout << "   x = " << appliedAngularVelocityReference_[0] << std::endl;
    std::cout << "   y = " << appliedAngularVelocityReference_[1] << std::endl;
    std::cout << std::endl;

    //This is the meat-and-potatoes of the plugin, where velocities are actually generated.
    //in this minimal case, simply specify constants; more generally, choose vx and omega_z
    // intelligently based on the goal and the environment
    // When isGoalReached() is false, computeVelocityCommands will be called each iteration
    // of the controller--which is a settable parameter.  On each iteration, values in
    // cmd_vel should be computed and set, and these values will be published by move_base
    // to command robot motion
    cmd_vel.angular.x = appliedAngularVelocityReference_[0];
    cmd_vel.angular.y = appliedAngularVelocityReference_[1];
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    pub_cmd_vel_angular_.publish(cmd_vel);
}