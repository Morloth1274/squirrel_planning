cmake_minimum_required(VERSION 2.8.3)
project(squirrel_planning_execution)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  rosplan_knowledge_msgs
  rosplan_dispatch_msgs
  rosplan_planning_system
  nav_msgs
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  occupancy_grid_utils
  squirrel_speech_msgs
  squirrel_object_perception_msgs
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib rosplan_knowledge_msgs rosplan_planning_system nav_msgs mongodb_store geometry_msgs diagnostic_msgs visualization_msgs tf occupancy_grid_utils squirrel_speech_msgs squirrel_object_perception_msgs
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Pointing service
set(tidyroom_SOURCES
	src/TidyRoom.cpp)

## map sources
set(rpsquirrelroadmap_SOURCES
  src/RPSquirrelRoadmap.cpp
  src/RPSimpleMapVisualization.cpp)

## recurse sources
set(rpsquirrelRecursion_SOURCES
  src/RPSquirrelRecursion.cpp
  src/ContingentTacticalClassifyPDDLGenerator.cpp
  src/ContingentStrategicClassifyPDDLGenerator.cpp
  src/ClassicalTidyPDDLGenerator.cpp
  src/pddl_actions/PlannerInstance.cpp
  src/pddl_actions/NextTurnPDDLAction.cpp
  src/pddl_actions/ShedKnowledgePDDLAction.cpp
  src/pddl_actions/FinaliseClassificationPDDLAction.cpp
  src/pddl_actions/ExamineAreaPDDLAction.cpp
  src/pddl_actions/ExploreAreaPDDLAction.cpp
  src/pddl_actions/TidyAreaPDDLAction.cpp
  src/pddl_actions/ObserveClassifiableOnAttemptPDDLAction.cpp
  src/ViewConeGenerator.cpp)
  
## robot knows game (year 3, 1st scenario)
set(robotKnows_SOURCES
  src/RobotKnowsGame.cpp
  src/ContingentTacticalClassifyPDDLGenerator.cpp
  src/ContingentStrategicClassifyPDDLGenerator.cpp
  src/ClassicalTidyPDDLGenerator.cpp
  src/pddl_actions/PlannerInstance.cpp
  src/pddl_actions/NextTurnPDDLAction.cpp
  src/pddl_actions/ShedKnowledgePDDLAction.cpp
  src/pddl_actions/FinaliseClassificationPDDLAction.cpp
  src/pddl_actions/ExamineAreaPDDLAction.cpp
  src/pddl_actions/ExploreAreaPDDLAction.cpp
  src/pddl_actions/TidyAreaPDDLAction.cpp
  src/pddl_actions/ObserveClassifiableOnAttemptPDDLAction.cpp
  src/ViewConeGenerator.cpp)
  
set(simulatedPDDLActionsNode_SOURCES
  src/SimulatedPDDLActionsNode.cpp
  src/pddl_actions/PlannerInstance.cpp
  src/pddl_actions/GotoPDDLAction.cpp
  src/pddl_actions/PushObjectPDDLAction.cpp
  src/pddl_actions/PickupPDDLAction.cpp
  src/pddl_actions/PutObjectInBoxPDDLAction.cpp
  src/pddl_actions/TidyObjectPDDLAction.cpp
  src/pddl_actions/ClearObjectPDDLAction.cpp
  src/pddl_actions/ExploreWaypointPDDLAction.cpp
  src/pddl_actions/ClassifyObjectPDDLAction.cpp
  src/pddl_actions/DropObjectPDDLAction.cpp
  src/pddl_actions/GiveObjectPDDLAction.cpp
  src/pddl_actions/TakeObjectPDDLAction.cpp
  src/pddl_actions/EmotePDDLAction.cpp
  src/pddl_actions/ChildrenPDDLAction.cpp
  src/pddl_actions/SimulatedObservePDDLAction.cpp
  src/pddl_actions/ExamineObjectInHandPDDLAction.cpp
  src/pddl_actions/FollowChildPDDLAction.cpp)
  
set(sortingGame_SOURCES
  src/SortingGame.cpp
  src/pddl_actions/PlannerInstance.cpp
  src/pddl_actions/NextTurnPDDLAction.cpp
  src/pddl_actions/ShedKnowledgePDDLAction.cpp
  src/pddl_actions/FinaliseClassificationPDDLAction.cpp
  src/pddl_actions/SimulatedObservePDDLAction.cpp)
  
set(speechSimulator_SOURCES
  src/SpeechSimulator.cpp)

set(viewConeTester_SOURCES
  src/ViewConeGenerator.cpp
  src/view_cone_test_suite/ViewConeCaller.cpp)
  
set(knowRuleTester_SOURCES
  src/pddl_generators/RobotKnowsSortingGamePDDLGenerator.cpp)
  
set(recommenderTester_SOURCES
  src/RecommenderSystem.cpp
  src/pddl_actions/ListenToFeedbackPDDLAction.cpp
  src/pddl_actions/InspectObjectPDDLAction.cpp
  src/PlanToSensePDDLGenerator.cpp
  src/PlanToAskPDDLGenerator.cpp)
 
#set(sortClusteredRoom_SOURCES
#  src/cluttered_pathfinding/SquirrelPlanningCluttered.cpp)
  

## Declare cpp executables
add_executable(tidyroom ${tidyroom_SOURCES})
add_executable(simpledemo src/TidyRoomGrasping.cpp)
add_executable(rpsquirrelRoadmap ${rpsquirrelroadmap_SOURCES})
add_executable(rpsquirrelRecursion ${rpsquirrelRecursion_SOURCES})
add_executable(simulatedPDDLActionsNode ${simulatedPDDLActionsNode_SOURCES})
add_executable(sortingGame ${sortingGame_SOURCES})
add_executable(speechSimulator ${speechSimulator_SOURCES})
add_executable(viewConeTester ${viewConeTester_SOURCES})
#add_executable(knowRuleTester ${knowRuleTester_SOURCES})
add_executable(recommenderTester ${recommenderTester_SOURCES})
add_executable(robotKnowsGame ${robotKnows_SOURCES})
#add_executable(sortClusteredRoom ${sortClusteredRoom_SOURCES})

add_dependencies(tidyroom ${catkin_EXPORTED_TARGETS})
add_dependencies(simpledemo ${catkin_EXPORTED_TARGETS})
add_dependencies(rpsquirrelRoadmap ${catkin_EXPORTED_TARGETS})
add_dependencies(rpsquirrelRecursion ${catkin_EXPORTED_TARGETS})
add_dependencies(simulatedPDDLActionsNode ${catkin_EXPORTED_TARGETS})
add_dependencies(sortingGame ${catkin_EXPORTED_TARGETS})
add_dependencies(speechSimulator ${catkin_EXPORTED_TARGETS})
add_dependencies(viewConeTester ${catkin_EXPORTED_TARGETS})
#add_dependencies(knowRuleTester ${catkin_EXPORTED_TARGETS})
add_dependencies(recommenderTester ${catkin_EXPORTED_TARGETS})
add_dependencies(robotKnowsGame ${catkin_EXPORTED_TARGETS})
#add_dependencies(sortClusteredRoom ${catkin_EXPORTED_TARGETS})

target_link_libraries(tidyroom ${catkin_LIBRARIES})
target_link_libraries(simpledemo ${catkin_LIBRARIES})
target_link_libraries(rpsquirrelRoadmap ${catkin_LIBRARIES})
target_link_libraries(rpsquirrelRecursion ${catkin_LIBRARIES})
target_link_libraries(simulatedPDDLActionsNode ${catkin_LIBRARIES})
target_link_libraries(sortingGame ${catkin_LIBRARIES})
target_link_libraries(speechSimulator ${catkin_LIBRARIES})
target_link_libraries(viewConeTester ${catkin_LIBRARIES})
#target_link_libraries(knowRuleTester ${catkin_LIBRARIES})
target_link_libraries(recommenderTester ${catkin_LIBRARIES})
target_link_libraries(robotKnowsGame ${catkin_LIBRARIES})
#target_link_libraries(sortClusteredRoom ${catkin_LIBRARIES})

##########
## Test ##
##########
# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest

#add_executable(occupancy_grid_publisher src/view_cone_test_suite/OccupancyGridPublisher.cpp)
#target_link_libraries(occupancy_grid_publisher ${catkin_LIBRARIES})

#add_executable(view_cone_test src/view_cone_test_suite/ViewConeCaller.cpp src/ViewConeGenerator.cpp)
#target_link_libraries(view_cone_test ${catkin_LIBRARIES})

#add_executable(plan_simulator src/test_suite/ExplorePDDLAction.cpp src/test_suite/GotoPDDLAction.cpp src/test_suite/PlannerInstance.cpp src/test_suite/TidyRooms.cpp)
#target_link_libraries(plan_simulator ${catkin_LIBRARIES})
