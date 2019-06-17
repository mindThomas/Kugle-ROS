#ifndef KUGLE_MPC_PLUGIN_H
#define KUGLE_MPC_PLUGIN_H

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <mutex>
#include <condition_variable>

#include <sys/stat.h>
#include <sys/time.h>

#include "MPC.h"
#include "Trajectory.h"
#include "Path.h"
 
class KugleMPC : public nav_core::BaseLocalPlanner {
	public:
		KugleMPC();
		~KugleMPC();
		//TestPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros);
		bool isGoalReached();
		bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);
		bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

	private:
		void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void PublishPredictedTrajectory();
        void PublishAngularVelocityControl();
        void MPC_Thread();
        void AngularVelocityPublisherThread();

        void tic();
        double toc();

	private:
		tf::TransformListener * tfListener_;
		costmap_2d::Costmap2DROS * costmap_;

		double window_width_;
        double window_height_;

		std::string map_frame_;
        std::string mpc_frame_;
        std::string heading_frame_;
        std::string base_link_frame_;
        std::string odom_topic_;

		ros::Publisher pub_global_plan_;
        ros::Publisher pub_local_plan_;
		ros::Publisher pub_cmd_vel_angular_;
		ros::Subscriber sub_odom_;

		geometry_msgs::Quaternion currentAttitude_;
		geometry_msgs::Vector3 currentVelocityHeadingFrame_;
		ros::Time odomTime_;

		bool global_plan_changed_;
		unsigned int global_plan_size_;
        MPC::Trajectory global_plan_;
        std::mutex global_plan_mutex_;
        MPC::TrajectoryPoint goal_point_;
        MPC::Trajectory local_plan_;
        MPC::Path local_path_;

        ros::Time prevTime_;
        MPC::MPC mpc_;
        boost::thread mpcThread_;
        boost::thread publisherThread_;
        bool stopThread_;
        bool goal_reached_;

        std::mutex mpc_signalling_mutex_;
        std::timed_mutex mpc_processing_mutex_;
        std::condition_variable mpc_signalling_cv;

		double desired_velocity_;
		std::vector<std::pair<double,double>> angularVelocityOutputs_;
        std::mutex angularVelocityOutputs_mutex_;
        Eigen::Vector2d appliedAngularVelocityReference_;

        struct timeval timing_start_;
};

#endif
