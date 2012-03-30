// #ifndef ADD_PLANAR_OBSTACLE_H
// #define ADD_PLANAR_OBSTACLE_H

#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <planning_environment/models/collision_models.h>

#include "flap.h"
#include "geometry_utils.h"

#ifndef SET_PLANNING_SCENE_DIFF_NAME_H
#define SET_PLANNING_SCENE_DIFF_NAME "/environment_server/set_planning_scene_diff"
#endif

void make_flap_trimeshes(const Flap &rect, arm_navigation_msgs::Shape &trimesh_object);

arm_navigation_msgs::PlanningScene add_obstacles(const std::vector<Flap> &rects);

void remove_obstacles();

// #endif