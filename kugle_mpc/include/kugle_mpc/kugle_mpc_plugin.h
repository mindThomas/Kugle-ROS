#ifndef KUGLE_MPC_PLUGIN_H
#define KUGLE_MPC_PLUGIN_H

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>

#include "MPC.h"
#include "Trajectory.h"
#include "Path.h"
 
class KugleMPC : public nav_core::BaseLocalPlanner {
	public:
		KugleMPC();
		//TestPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros);
		bool isGoalReached();
		bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);
		bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
		
	private:
		tf::TransformListener * tfListener_;
		costmap_2d::Costmap2DROS * costmap_;

		double window_width_;
        double window_height_;

		std::string map_frame_;
        std::string mpc_frame_;
        std::string heading_frame_;

		ros::Publisher pub_plan_;

		bool global_plan_changed_;
        MPC::Trajectory global_plan_;
        MPC::Trajectory local_plan_;
        MPC::Path local_path_;

		// Test only
        ros::Time tg;
        unsigned int old_size;
};

#endif
