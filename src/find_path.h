// #ifndef FIND_PATH_H
// #define PATH_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include "flap.h"
#include "actionlib/client/simple_action_client.h"
#include "arm_navigation_msgs/MoveArmAction.h"
#include "add_planar_obstacle.h"


bool find_path(const Eigen::Vector3f &position,
               const Eigen::Vector3f &position_tolerance,
               const Flap &goal_flap,
               const Eigen::Vector4f &quaternion,
               const arm_navigation_msgs::PlanningScene &planning_diff);

// #endif