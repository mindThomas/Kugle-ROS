#ifndef KUGLE_MPC_PLUGIN_H
#define KUGLE_MPC_PLUGIN_H

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
 
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
		ros::Time tg;
		unsigned int old_size;
		tf::TransformListener * handed_tf;
		costmap_2d::Costmap2DROS * costmap_handle;

		ros::Publisher pub_plan;
	};	

#endif
