#include "find_path.h"

bool find_path(const Eigen::Vector3f &position,
               const Eigen::Vector3f &position_tolerance,
               const Flap &goal_flap,
               const Eigen::Vector4f &quaternion,
               const arm_navigation_msgs::PlanningScene &planning_diff) {
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  arm_navigation_msgs::MoveArmGoal goalA;

  // Attached collision obstacles
  // goalA.planning_scene_diff = planning_diff;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh.param<std::string>("planner_id",goalA.motion_plan_request.planner_id,std::string(""));
  nh.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("ompl_planning/plan_kinematic_path"));
  goalA.motion_plan_request.goal_constraints.set_position_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";

  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = .65;//position.x();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = -.188;//position.y();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = -.75;//position.z();

  // make_flap_trimeshes(goal_flap, goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape);
/*
  arm_navigation_msgs::Shape &trimesh_object = goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape;

    trimesh_object.type = arm_navigation_msgs::Shape::MESH;

        geometry_msgs::Point p0;
        p0.x = 0;
        p0.y = 0;
        p0.z = 0;

        geometry_msgs::Point p1;
        p1.x = 0;
        p1.y = 0;
        p1.z = 10;

        geometry_msgs::Point p2;
        p2.x = 10;
        p2.y = 0;
        p2.z = 0;

        geometry_msgs::Point p3;
        p3.x = 0;
        p3.y = 10;
        p3.z = 0;


        trimesh_object.vertices.push_back(p0);
        trimesh_object.vertices.push_back(p1);
        trimesh_object.vertices.push_back(p2);
        trimesh_object.vertices.push_back(p3);

    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);

    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);

    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(0);

    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(3);
        */
  // /*
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(10);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(10);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(10);
// */
  /*
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(position_tolerance.x());
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(position_tolerance.y());
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(position_tolerance.z());
  */

  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = 0;//quaternion.x();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = 0;//quaternion.y();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = 0;//quaternion.z();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = 1;//quaternion.w();

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.2;  //0.04
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.2;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.2;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
      return 0;
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success) {
          ROS_INFO("Action finished: %s",state.toString().c_str());
          return 1;
      } else {
        ROS_INFO("Action failed: %s",state.toString().c_str());
        return 0;
      }
    }
  }
  return 0;
  // ros::shutdown();
}