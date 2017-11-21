#include <std_msgs/Int8.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include <actionlib/client/simple_action_client.h>
#include <mongodb_store/message_store.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_dispatch_msgs/PlanAction.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>

#include "SquirrelPlanningCluttered.h"

#include <squirrel_navigation_msgs/ClutterPlannerSrv.h>

#include "squirrel_object_perception_msgs/BCylinder.h"
#include "squirrel_object_perception_msgs/SceneObject.h"
#include "squirrel_manipulation_msgs/GetObjectPositions.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	SquirrelPlanningCluttered::SquirrelPlanningCluttered(ros::NodeHandle &nh)
		: node_handle(&nh), message_store(nh), received_navigation_grid_(false)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		// Dummy problem generator.
		//pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::SquirrelPlanningCluttered::generatePDDLProblemFile, this);
		
		// Test finding paths.
		nav_grid_sub = nh.subscribe("/map", 1, &KCL_rosplan::SquirrelPlanningCluttered::storeNavigationGrid, this);
		find_path_service = nh.serviceClient<squirrel_navigation_msgs::ClutterPlannerSrv>("/clutter_service");
		
		find_objects_service = nh.serviceClient<squirrel_manipulation_msgs::GetObjectPositions>("/getObjectsPositions");
		
		setupSimulation();
	}
	
	void SquirrelPlanningCluttered::setupSimulation()
	{
		// We will make some fictional objects and associated waypoints.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		squirrel_manipulation_msgs::GetObjectPositions op;
		
		if (!find_objects_service.call(op))
		{
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not call the server to get all the objects in the domain!");
			exit(1);
		}
		
		std::vector<std::string> waypoints;
		waypoints.push_back("kenny_waypoint");
		std::vector<std::string> objects;
		for (unsigned int i = 0; i < op.response.objectids.size(); ++i)
		{
			// Create the object, both in PDDL and in the knowledge base.
			objects.push_back(op.response.objectids[i]);
			
			// Give every object its own waypoint.
			std::stringstream ss;
			ss << op.response.objectids[i] << "_wp";
			waypoints.push_back(ss.str());
			
			// Put the location in the database.
			const geometry_msgs::Pose& p = op.response.objectposes[i];
			
			geometry_msgs::PoseStamped pose;
			pose.header.seq = 0;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "/map";
			pose.pose = p;
			message_store.insertNamed(ss.str(), pose);
			
			// Create 4 possible locations around each object for pushing.
			geometry_msgs::Pose push_location = op.response.objectposes[i];
			
			// North.
			pose.pose = push_location;
			pose.pose.position.y += op.response.diameters[i] / 2.0f + 1.0f;
			
			ss.str(std::string());
			ss << op.response.objectids[i] << "_north_wp";
			waypoints.push_back(ss.str());
			
			message_store.insertNamed(ss.str(), pose);
			
			// South.
			pose.pose = push_location;
			pose.pose.position.y -= op.response.diameters[i] / 2.0f + 1.0f;
			
			ss.str(std::string());
			ss << op.response.objectids[i] << "_south_wp";
			waypoints.push_back(ss.str());
			
			message_store.insertNamed(ss.str(), pose);
			
			// West.
			pose.pose = push_location;
			pose.pose.position.y -= op.response.diameters[i] / 2.0f - 1.0f;
			
			ss.str(std::string());
			ss << op.response.objectids[i] << "_west_wp";
			waypoints.push_back(ss.str());
			
			message_store.insertNamed(ss.str(), pose);
			
			// East.
			pose.pose = push_location;
			pose.pose.position.y -= op.response.diameters[i] / 2.0f + 1.0f;
			
			ss.str(std::string());
			ss << op.response.objectids[i] << "_east_wp";
			waypoints.push_back(ss.str());
			
			message_store.insertNamed(ss.str(), pose);
		}
		
		/*
		waypoints.push_back("kenny_waypoint");
		std::vector<std::string> wind_directions;
		wind_directions.push_back("north");
		wind_directions.push_back("south");
		wind_directions.push_back("west");
		wind_directions.push_back("east");
		*/
		// Create a objects.
		for (std::vector<std::string>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const std::string& object = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "object";
			knowledge_item.instance_name = object;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not add the object %s to the knowledge base.", object.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (SquirrelPlanningCluttered) Added %s to the knowledge base.", object.c_str());
			
			// Give every toy a location.
			std::stringstream ss;
			ss << object << "_wp";
			
			waypoints.push_back(ss.str());
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "object_at";
			knowledge_item.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = object;
			knowledge_item.values.push_back(kv);
			
			kv.key = "wp";
			kv.value = ss.str();
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not add the fact (object_at %s %s) to the knowledge base.", object.c_str(), ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (SquirrelPlanningCluttered) Added the fact (object_at %s %s) to the knowledge base.", object.c_str(), ss.str().c_str());
			knowledge_item.values.clear();
		}
		
		for (std::vector<std::string>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
		{
			const std::string& waypoint_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			knowledge_item.instance_name = waypoint_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not add the waypoint %s to the knowledge base.", waypoint_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (SquirrelPlanningCluttered) Added %s to the knowledge base.", waypoint_predicate.c_str());
		}
		
		// Add kenny!
		{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "kenny";
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not add the robot kenny to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SquirrelPlanningCluttered) Added kenny to the knowledge base.");
		}
		
		{
		// Set kenny at it's starting waypoint.
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_at";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp";
		kv.value = "kenny_waypoint";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not add the fact (robot_at robot kenny_waypoint) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SquirrelPlanningCluttered) Added the fact (robot_at robot kenny_waypoint) to the knowledge base.");
		knowledge_item.values.clear();
		}
		
		// Locate the location of the robot.
		{
		tf::StampedTransform transform;
		tf::TransformListener tfl;
		try {
			tfl.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
			tfl.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		} catch ( tf::TransformException& ex ) {
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Error find the transform between /map and /base_link.");
			//exit(1);
		}
		
		// Add this information to the knowledge base.
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = transform.getOrigin().getX();
		pose.pose.position.y = transform.getOrigin().getY();
		pose.pose.position.z = transform.getOrigin().getZ();
		pose.pose.orientation.x = transform.getRotation().getX();
		pose.pose.orientation.y = transform.getRotation().getY();
		pose.pose.orientation.z = transform.getRotation().getZ();
		pose.pose.orientation.w = transform.getRotation().getW();
		message_store.insertNamed("kenny_waypoint", pose);
		}

		// Add target waypoint.
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = 1.84;
		pose.pose.position.y = -8.34;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		message_store.insertNamed("target_wp", pose);
		}
		
		// Add a simple goal.
		{
		rosplan_knowledge_msgs::KnowledgeItem waypoint_goal;
		waypoint_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		waypoint_goal.attribute_name = "object_at";
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = "blue_ball";
		waypoint_goal.values.push_back(kv);
		
		kv.key = "wp";
		kv.value = "target_wp";
		waypoint_goal.values.push_back(kv);
			
		rosplan_knowledge_msgs::KnowledgeUpdateService add_goal;
		add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		add_goal.request.knowledge = waypoint_goal;
		
		if (!update_knowledge_client.call(add_goal)) {
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Could not add the fact (robot_at kenny red_ball_wp) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SquirrelPlanningCluttered) Added the fact (robot_at kenny red_ball_wp) to the knowledge base.");
		}
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/

	/**
	 * Generate a contingent problem.
	 */
	bool SquirrelPlanningCluttered::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
		return true;
	}
	
	void SquirrelPlanningCluttered::storeNavigationGrid(const nav_msgs::OccupancyGrid::ConstPtr& og)
	{
		ROS_INFO("KCL: (SquirrelPlanningCluttered) Received the navigation grid.");
		current_navigation_grid_ = *og;
		received_navigation_grid_ = true;
	}
	/*
	void SquirrelPlanningCluttered::testFindingPaths()
	{
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > start_wps;
		if (!message_store.queryNamed<geometry_msgs::PoseStamped>("test1", start_wps) ||
		    start_wps.empty())
		{
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Failed to fetch the pose stamped: test1");
		}
		
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > goal_wps;
		if (!message_store.queryNamed<geometry_msgs::PoseStamped>("test2", goal_wps) ||
		    goal_wps.empty())
		{
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Failed to fetch the pose stamped: test2");
		}
		
		ROS_INFO("Start (%f, %f, %f).", start_wps[0]->pose.position.x, start_wps[0]->pose.position.y, start_wps[0]->pose.position.z);
		ROS_INFO("Goal (%f, %f, %f).", goal_wps[0]->pose.position.x, goal_wps[0]->pose.position.y, goal_wps[0]->pose.position.z);
		
		squirrel_navigation_msgs::ClutterPlannerSrv cps;
		cps.request.start = *start_wps[0];
		cps.request.goal = *goal_wps[0];
		cps.request.grid = current_navigation_grid_;
		
		if (!find_path_service.call(cps))
		{
			ROS_ERROR("KCL: (SquirrelPlanningCluttered) Failed to call the waypoint service");
			exit(-1);
		}
		
		
		ROS_INFO("KCL: (SquirrelPlanningCluttered) Found a path with %lu nodes.", cps.response.path.size());
		for (std::vector<geometry_msgs::PoseStamped>::const_iterator ci = cps.response.path.begin(); ci != cps.response.path.end(); ++ci)
		{
			const geometry_msgs::PoseStamped& ps = *ci;
			ROS_INFO("(%f, %f, %f).", ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
		}
	}
	*/
}; // close namespace

/*-------------*/
/* Main method */
/*-------------*/
/*
void initMongoDBData(mongodb_store::MessageStoreProxy& message_store)
{
	// Create a Scene Object for objects in the environment.
	squirrel_object_perception_msgs::SceneObject scene_object;
	scene_object.id = "toy1";
	scene_object.category = "unknown";
	
	geometry_msgs::PoseStamped object_pose;
	object_pose.header.seq = 0;
	object_pose.header.stamp = ros::Time::now();
	object_pose.header.frame_id = "/map";
	object_pose.pose.position.x = 0.77f;
	object_pose.pose.position.y = 3.52f;
	object_pose.pose.position.z = 0.0f;
	
	object_pose.pose.orientation.x = 0.0f;
	object_pose.pose.orientation.y = 0.0f;
	object_pose.pose.orientation.z = 0.0f;
	object_pose.pose.orientation.w = 1.0f;
	scene_object.pose = object_pose.pose;
	
	squirrel_object_perception_msgs::BCylinder c;
	c.diameter = 0.3f;
	c.height = 0.2f;
	scene_object.bounding_cylinder = c;
	std::string near_waypoint_mongodb_id(message_store.insertNamed("toy1", scene_object));
	
	// Create some waypoints
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 0.87f;
	pose.pose.position.y = 1.04f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("wp1", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 4.46f;
	pose.pose.position.y = 4.84f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("wp2", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 0.926f;
	pose.pose.position.y = 6.17f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("wp3", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 0.926f;
	pose.pose.position.y = 6.17f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("push_wp", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 0.926f;
	pose.pose.position.y = 6.17f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("object_wp", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 3.6f;
	pose.pose.position.y = 5.4f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("test1", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 3.6f;
	pose.pose.position.y = 1.0f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("test2", pose));
	}
	
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = 0.77f;
	pose.pose.position.y = 3.52f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("toy1_wp", pose));
	}
}
*/
int main(int argc, char **argv) {

	ros::init(argc, argv, "squirrel_planning_cluttered_environment");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::SquirrelPlanningCluttered rpsr(nh);
	
	// Setup the environment.
	//initMongoDBData(rpsr.getMessageStore());
	ROS_INFO("KCL: (SquirrelPlanningCluttered) Waiting for the grid to be published...");
	while (!rpsr.ready())
	{
		ros::spinOnce();
	}
	ROS_INFO("KCL: (SquirrelPlanningCluttered) Grid published, good to go!");
	
	//rpsr.testFindingPaths();
/*
	// Setup all the simulated actions.
	KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(nh);
	KCL_rosplan::FinaliseClassificationPDDLAction finalise_classify_action(nh);
	
	// Setup the recursive actions.
	KCL_rosplan::ExamineAreaPDDLAction examine_area_action(nh);
	KCL_rosplan::ExploreAreaPDDLAction explore_area_action(nh);
	KCL_rosplan::ObserveClassifiableOnAttemptPDDLAction observe_classifiable_on_attempt_action(nh);
	KCL_rosplan::TidyAreaPDDLAction tidy_are_action(nh);
*/
	// Lets start the planning process.
	std::string data_path;
	nh.getParam("/data_path", data_path);
	
	std::string planner_path;
	nh.getParam("/planner_path", planner_path);
	
	std::stringstream ss;
	//ss << data_path << "tidy_room_domain-nt.pddl";
	ss << data_path << "cleanroom_popf_domain.pddl";
	std::string domain_path = ss.str();
	
	ss.str(std::string());
	ss << data_path << "cleanroom_popf_problem_generated.pddl";
	std::string problem_path = ss.str();
	
	std::string planner_command;
	nh.getParam("/rosplan_planning_system/planner_command", planner_command);
	
	ROS_INFO("KCL: (SquirrelPlanningCluttered) Planner command: %s", planner_command.c_str());
	
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command;
	psrv.start_action_id = 0;

	ROS_INFO("KCL: (SquirrelPlanningCluttered) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (SquirrelPlanningCluttered) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (SquirrelPlanningCluttered) Goal sent");
	
	ros::spin();
	return 0;
}
