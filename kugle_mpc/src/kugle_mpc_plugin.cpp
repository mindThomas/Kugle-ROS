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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

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

	pub_plan_ = nh.advertise<nav_msgs::Path>("global_plan", 1000);

    private_nh.param("map_frame", map_frame_, std::string("map"));
	private_nh.param("mpc_frame", mpc_frame_, std::string("odom"));
	private_nh.param("heading_frame", heading_frame_, std::string("heading"));

    private_nh.param("width", window_width_, double(4.0));
    private_nh.param("height", window_height_, double(4.0));
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
	pub_plan_.publish(planMsg);

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
		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = 0;
		cmd_vel.linear.z = 0;
		cmd_vel.angular.z = 0;
		return true;
	}


	// Look-up current estimate of robot in map
	tf::StampedTransform tf_heading;
	try {
		tfListener_->lookupTransform(map_frame_, heading_frame_, ros::Time(0), tf_heading);
	}
	catch (tf::TransformException &ex) {
		ROS_WARN("%s",ex.what());
	}


	if (global_plan_changed_) {
		global_plan_changed_ = false;

		Eigen::Vector2d robot_center(tf_heading.getOrigin().x(), tf_heading.getOrigin().y());
		tfScalar yaw, pitch, roll;
		tf::Matrix3x3 mat(tf_heading.getRotation());
		mat.getEulerYPR(yaw, pitch, roll);

		global_plan_.WindowExtract(local_plan_, robot_center, yaw, window_width_, window_height_);
		local_plan_.plot(true, false, -window_height_/2, -window_width_/2, window_height_/2, window_width_/2); // notice that width and height is swapped, since x is plotted up
		local_path_ = MPC::Path(local_plan_, 6, true, false);
		local_path_.plot(true, -window_height_/2, -window_width_/2, window_height_/2, window_width_/2);
	}

	//This is the meat-and-potatoes of the plugin, where velocities are actually generated.
	//in this minimal case, simply specify constants; more generally, choose vx and omega_z
	// intelligently based on the goal and the environment
	// When isGoalReached() is false, computeVelocityCommands will be called each iteration
	// of the controller--which is a settable parameter.  On each iteration, values in
	// cmd_vel should be computed and set, and these values will be published by move_base
	// to command robot motion
	cmd_vel.linear.x = 0.2;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.z = 0.2;
	return true;
}
