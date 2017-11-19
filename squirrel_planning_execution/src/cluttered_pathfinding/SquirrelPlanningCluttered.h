#ifndef SQUIRREL_PLANNING_EXECUTION_CLUTTERED_PATHFINDING_SQUIRREDPLANNINGCLUTTERED_H
#define SQUIRREL_PLANNING_EXECUTION_CLUTTERED_PATHFINDING_SQUIRREDPLANNINGCLUTTERED_H

#include <ros/ros.h>
#include <rosplan_knowledge_msgs/GenerateProblemService.h>
#include <nav_msgs/OccupancyGrid.h>

namespace mongodb_store
{
	class MessageStoreProxy;
};

namespace KCL_rosplan
{
	class SquirrelPlanningCluttered
	{
	public:
		SquirrelPlanningCluttered(ros::NodeHandle &nh);

		void setupSimulation();
		
		bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		
		void testFindingPaths();
		
		void storeNavigationGrid(const nav_msgs::OccupancyGrid::ConstPtr& og);
		
		bool ready() const { return received_navigation_grid_; }
		
		inline mongodb_store::MessageStoreProxy& getMessageStore() { return message_store; }
		
	private:
		ros::NodeHandle* node_handle;
		mongodb_store::MessageStoreProxy message_store;
		
		/* knowledge service clients */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient query_knowledge_client;
		
		/* find paths in the cluttered environment. */
		ros::ServiceClient find_path_service;
		
		ros::ServiceClient find_objects_service;
		
		/* the current navigation mesh. */
		ros::Subscriber nav_grid_sub;
		nav_msgs::OccupancyGrid current_navigation_grid_;
		bool received_navigation_grid_;
		
		/* dummy problem generator. */
		ros::ServiceServer pddl_generation_service;
	};
};

#endif
