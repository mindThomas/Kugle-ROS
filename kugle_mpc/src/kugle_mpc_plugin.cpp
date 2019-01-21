//package name, header name for new plugin library
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

PLUGINLIB_EXPORT_CLASS(KugleMPC, nav_core::BaseLocalPlanner);

KugleMPC::KugleMPC(){
	//nothing ot fill in here; "initialize" will do the initializations
}

//put inits here:
void KugleMPC::initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros){
	ros::NodeHandle nh(name);
    ros::NodeHandle private_nh("~/" + name);
	
	old_size = 0;
	tfListener_ = tf;
	costmap_ = costmap_ros;
	global_plan_changed_ = false;

	pub_global_plan_ = nh.advertise<nav_msgs::Path>("global_plan", 1000);
    pub_local_plan_ = nh.advertise<nav_msgs::Path>("local_plan", 1000);

    private_nh.param("map_frame", map_frame_, std::string("map"));
	private_nh.param("mpc_frame", mpc_frame_, std::string("odom"));
	private_nh.param("heading_frame", heading_frame_, std::string("heading"));
    private_nh.param("base_link_frame", base_link_frame_, std::string("base_link"));

    private_nh.param("width", window_width_, double(4.0));
    private_nh.param("height", window_height_, double(4.0));

    sub_odom_ = nh.subscribe("odom", 1000, &KugleMPC::OdometryCallback, this);

    currentAttitude_.w = 1;
    currentAttitude_.x = 1;
    currentAttitude_.y = 1;
    currentAttitude_.z = 1;

    prevTime_ = ros::Time::now();
}


void KugleMPC::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Received odometry, translational velocity: (" << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ")");
    currentAttitude_ = msg->pose.pose.orientation;
    currentVelocityHeadingFrame_ = msg->twist.twist.linear; // notice that this velocity is in heading frame
    odomTime_ = msg->header.stamp;
}

bool KugleMPC::isGoalReached(){
	//For demonstration purposes, sending a single navpoint will cause five seconds of activity before exiting.
	return ros::Time::now() > tg;
}

bool KugleMPC::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan){
	//The "plan" that comes in here is a bunch of poses of varaible length, calculated by the global planner(?).
	//We're just ignoring it entirely, but an actual planner would probably take this opportunity
	//to store it somewhere and maybe update components that refrerence it.
	ROS_INFO("GOT A PLAN OF SIZE %lu", plan.size());
	//If we wait long enough, the global planner will ask us to follow the same plan again. This would reset the five-
	//second timer if I just had it refresh every time this function was called, so I check to see if the new plan is
	//"the same" as the old one first.
	if(plan.size() != old_size){
		old_size = plan.size();
		tg = ros::Time::now() + ros::Duration(5.0);
	}

	nav_msgs::Path planMsg;
	planMsg.header.stamp = ros::Time::now();
	planMsg.header.frame_id = map_frame_;
	planMsg.poses.insert(planMsg.poses.begin(), plan.begin(), plan.end());
	pub_global_plan_.publish(planMsg);

	global_plan_.clear();
	for (auto& pose : plan)
	{
		global_plan_.AddPoint(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
	}

	//global_plan_.plot(false, false, -10, -10, 10, 10);

	global_plan_changed_ = true;
	
	return true;
}

bool KugleMPC::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){
	// This function is called periodically with the "controller_frequency" rate set in "move_base_params.yaml"

	if (global_plan_.size() == 0) {
		// No plan received
		cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
		return true;
	}

	ros::Time currentTime = ros::Time::now();

	// Look-up current estimate of robot in map
	tf::StampedTransform tf_base_link;
	try {
		tfListener_->lookupTransform(map_frame_, base_link_frame_, ros::Time(0), tf_base_link);
	}
	catch (tf::TransformException &ex) {
		ROS_WARN("%s",ex.what());
	}

	ROS_INFO_STREAM("Difference in time between TF time and current time: " << ros::Duration(currentTime - tf_base_link.stamp_).toSec() * 1000 << " ms");
    ROS_INFO_STREAM("Difference in time between ODOM time and current time: " << ros::Duration(currentTime - odomTime_).toSec() * 1000 << " ms");
    ROS_INFO_STREAM("Delta time since last execution (dt) = " << ros::Duration(currentTime - prevTime_).toSec() * 1000 << " ms");
    if (std::abs((ros::Duration(currentTime - prevTime_).toSec() / mpc_.getSampleTime()) - 1.0) > 0.05) {
        ROS_WARN("move_base configured frequency/rate does not match discretized MPC sample time!");
    }
    prevTime_ = currentTime;

    /* Assemble state variables to be fed into MPC */
    Eigen::Vector2d robot_center(tf_base_link.getOrigin().x(), tf_base_link.getOrigin().y());
    tf::Quaternion robot_quaternion = tf_base_link.getRotation();
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(robot_quaternion);
    mat.getEulerYPR(yaw, pitch, roll);

    boost::math::quaternion<double> attitude_quaternion(robot_quaternion.w(), robot_quaternion.x(), robot_quaternion.y(), robot_quaternion.z());
    double RobotYaw = mpc_.extractHeading(attitude_quaternion);
    Eigen::Matrix2d R_heading = Eigen::Rotation2Dd(RobotYaw).toRotationMatrix();
    Eigen::Vector2d robot_velocity_inertial = R_heading * Eigen::Vector2d(currentVelocityHeadingFrame_.x, currentVelocityHeadingFrame_.y);

    std::cout << "Current states:" << std::endl;
    std::cout << "position = " << std::endl << robot_center << std::endl;
    std::cout << "velocity = " << std::endl << robot_velocity_inertial << std::endl;
    std::cout << "quaternion = " << std::endl << attitude_quaternion << std::endl;
    std::cout << std::endl;


	if (global_plan_changed_) {
		global_plan_changed_ = false;

        /* Update MPC with changed trajectory */
        //mpc_.setTrajectory(global_plan_, robot_center, robot_velocity_inertial, attitude_quaternion);

		/*global_plan_.WindowExtract(local_plan_, robot_center, yaw, window_width_, window_height_);
		local_plan_.plot(true, false, -window_height_/2, -window_width_/2, window_height_/2, window_width_/2); // notice that width and height is swapped, since x is plotted up
		local_path_ = MPC::Path(local_plan_, 6, true, false);
		local_path_.plot(true, -window_height_/2, -window_width_/2, window_height_/2, window_width_/2);*/
	}

    mpc_.setTrajectory(global_plan_, robot_center, robot_velocity_inertial, attitude_quaternion);

    mpc_.setCurrentState(robot_center, robot_velocity_inertial, attitude_quaternion);

    mpc_.Step();

    MPC::MPC::state_t state = mpc_.getHorizonState();
    std::cout << "Estimated states:" << std::endl;
    std::cout << "position = " << std::endl << state.position << std::endl;
    std::cout << "velocity = " << std::endl << state.velocity << std::endl;
    std::cout << "quaternion = " << std::endl << state.quaternion << std::endl;
    std::cout << std::endl;

    Eigen::Vector2d angularVelocityReference = mpc_.getInertialAngularVelocity();
    std::cout << "Control output (angular velocity):" << std::endl;
    std::cout << "   x = " << angularVelocityReference[0] << std::endl;
    std::cout << "   y = " << angularVelocityReference[1] << std::endl;
    std::cout << std::endl;

    PublishPredictedTrajectory();

	//This is the meat-and-potatoes of the plugin, where velocities are actually generated.
	//in this minimal case, simply specify constants; more generally, choose vx and omega_z
	// intelligently based on the goal and the environment
	// When isGoalReached() is false, computeVelocityCommands will be called each iteration
	// of the controller--which is a settable parameter.  On each iteration, values in
	// cmd_vel should be computed and set, and these values will be published by move_base
	// to command robot motion
	cmd_vel.angular.x = angularVelocityReference[0];
	cmd_vel.angular.y = angularVelocityReference[1];
	cmd_vel.angular.z = 0;
	cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
	return true;
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