#include "add_planar_obstacle.h"
#include <planning_environment/util/construct_object.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <urdf/model.h>
#include <resource_retriever/retriever.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>
#include <algorithm>
#include <sstream>
#include <climits>
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>

#include <LinearMath/btConvexHull.h>

void make_flap_trimeshes(const Flap &rect, arm_navigation_msgs::Shape &trimesh_object)
{
    trimesh_object.type = arm_navigation_msgs::Shape::MESH;
    for (unsigned int i = 0; i < 5; i++)
    {
        geometry_msgs::Point p;
        p.x = rect.corners.at(i).x();
        p.y = rect.corners.at(i).y();
        p.z = rect.corners.at(i).z();

        if (i == 4)
        { // to make volumn nonzero
            p.z += 0.01;
        }

        trimesh_object.vertices.push_back(p);
        std::cout << "geometry point: " << p << std::endl;
    }

    // Bottom plane
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);

    // Side plane
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(4);
/*
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
 */
}

void add_single_obstacle(arm_navigation_msgs::CollisionObject &collision_object, const Flap &rect)
{
    arm_navigation_msgs::Shape trimesh_object;
    make_flap_trimeshes(rect, trimesh_object);

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    collision_object.shapes.push_back(trimesh_object);
    collision_object.poses.push_back(pose);
}

void add_single_obstacle_old(arm_navigation_msgs::CollisionObject &collision_object, const Flap &rect)
{
    arm_navigation_msgs::Shape trimesh_object;

    trimesh_object.type = arm_navigation_msgs::Shape::MESH;

    /*
btVector3 scale(1.0,1.0,1.0);
btVector3 v1(0.0613,0.035,-0.017);
btVector3 v2(0.055, 0.036, -0.019);
btVector3 v3(0.056, 0.037, -0.0184);
std::vector<tf::Vector3> verts;
verts.push_back(v1);
verts.push_back(v2);
verts.push_back(v3);


ConvexDecomposition::HullDesc hd(ConvexDecomposition::QF_TRIANGLES, 3, verts);
ConvexDecomposition::HullDescHullResult hr;
ConvexDecomposition::HullDescHullLibrary hl;
if (hl.CreateConvexHull(hd, hr) == QE_OK) {
    std::cout <<"ok" << std::endl;
} else {
    std::cout << "damn" << std::endl;
}


shapes::Mesh *mesh = shapes::createMeshFromVertices (verts);
     */
    /*
    btVector3 scale(1.0,1.0,1.0);
    //filename is either an absolute path or a package:// address: package://pr2_description/meshes/gripper_v0/l_finger.dae
    shapes::Mesh *mesh = shapes::createMeshFromFilename("package://pr2-box-packaging/ship.stl", &scale);
    // unsigned int tris[3] = {0, 1, 2};
    // mesh->triangles = tris;
    // double vers[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};
    // mesh->vertices = vers;

    if(!planning_environment::constructObjectMsg(mesh, trimesh_object)) {
      ROS_WARN_STREAM("Object construction fails");
    }
     */


    /*
    geometry_msgs::Point p0;
    p0.x = 0;
    p0.y = 0;
    p0.z = 1;
    geometry_msgs::Point p1;
    p1.x = 0;
    p1.y = 1;
    p1.z = 0;
    geometry_msgs::Point p2;
    p2.x = 1;
    p2.y = 0;
    p2.z = 0;
    geometry_msgs::Point p3;
    p0.x = 0;
    p0.y = 0;
    p0.z = 0;

    trimesh_object.vertices.push_back(p0);
    trimesh_object.vertices.push_back(p1);
    trimesh_object.vertices.push_back(p2);
    trimesh_object.vertices.push_back(p3);


    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
     */
    /*
    for (unsigned int i = 0; i < 5; i++)
    {

        geometry_msgs::Point p;
        p.x = rect.corners.at(i).x();
        p.y = rect.corners.at(i).y();
        p.z = rect.corners.at(i).z();

        if (i == 4) {  // to make volumn nonzero
            p.z += 0.01;
        }

        trimesh_object.vertices.push_back(p);
        std::cout << "geometry point: " << p << std::endl;
    }

    // Bottom plane
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(3);

    // Side plane
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(4);
    /*
    trimesh_object.triangles.push_back(1);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(4);
    trimesh_object.triangles.push_back(3);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(4);
    trimesh_object.triangles.push_back(2);
    trimesh_object.triangles.push_back(0);
    trimesh_object.triangles.push_back(4);
     */


    /*
        trimesh_object.type = arm_navigation_msgs::Shape::MESH;
        for (int i = 0; i < 3; i++)
        {
            geometry_msgs::Point p;
            p.x = 0; //rect.corners.at(i).x();
            p.y = 0; //rect.corners.at(i).y();
            p.z = 0; //rect.corners.at(i).z();
            if (i == 0)
            {
                p.x = 1;
            }
            else if (i == 1)
            {
                p.y = 1;
            }
            else
            {
                p.z = 1;
            }
            trimesh_object.vertices.push_back(p);
            std::cout << "geometry point: " << p << std::endl;
        }
        std::cout << "trimesh object vertices length: " << trimesh_object.vertices.size() << std::endl;
        for (int i = 0; i < 3; i++)
        {
            trimesh_object.triangles.push_back(i);
        }
        std::cout << "trimesh object triangles length: " << trimesh_object.triangles.size() << std::endl;
        for (int i = 1; i < 4; i++)
        {
            // trimesh_object.triangles.push_back(i);
        }

        for (int i = 0; i < trimesh_object.triangles.size(); i++)
        {
            std::cout << "geometry point: " << trimesh_object.vertices.at(trimesh_object.triangles.at(i)) << std::endl;

        }
     */
    /*
 arm_navigation_msgs::Shape planar_object;
 planar_object.type = arm_navigation_msgs::Shape::BOX;
 planar_object.dimensions.push_back(rect.width);
 planar_object.dimensions.push_back(rect.height);
 planar_object.dimensions.push_back(FLAP_THICKNESS);

 // object.dimensions.resize(2);
 // object.dimensions[0] = .1;
 // object.dimensions[1] = 1.0;
     * */

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    /*
  pose.position.x = .6;
  pose.position.y = -.6;
  pose.position.z = .375;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
     * */
    /*
    pose.position.x = rect.corners.at(4).x();
    pose.position.y = rect.corners.at(4).x();
    pose.position.z = rect.corners.at(4).x();

    Eigen::Vector4f orientation = VectorToQuaternion(rect.normal);
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
     */
    collision_object.shapes.push_back(trimesh_object);
    collision_object.poses.push_back(pose);

}

void remove_obstacles() {

    ros::NodeHandle rh;

    ros::Publisher vis_marker_publisher_;
    ros::Publisher vis_marker_array_publisher_;

    vis_marker_publisher_ = rh.advertise<visualization_msgs::Marker > ("state_validity_markers", 128);
    vis_marker_array_publisher_ = rh.advertise<visualization_msgs::MarkerArray > ("state_validity_markers_array", 128);

    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    ros::ServiceClient get_planning_scene_client =
            rh.serviceClient<arm_navigation_msgs::GetPlanningScene > (SET_PLANNING_SCENE_DIFF_NAME);

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::CollisionObject collision_object;

    collision_object.id = "pole";
    collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
    collision_object.header.frame_id = "odom_combined";
    collision_object.header.stamp = ros::Time::now();

    if (!get_planning_scene_client.call(planning_scene_req, planning_scene_res))
    {
        ROS_WARN("Can't get planning scene");
        // return -1;
    }
}

arm_navigation_msgs::PlanningScene add_obstacles(const std::vector<Flap> &rects)
{
    ros::NodeHandle rh;

    ros::Publisher vis_marker_publisher_;
    ros::Publisher vis_marker_array_publisher_;

    vis_marker_publisher_ = rh.advertise<visualization_msgs::Marker > ("state_validity_markers", 128);
    vis_marker_array_publisher_ = rh.advertise<visualization_msgs::MarkerArray > ("state_validity_markers_array", 128);

    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
    ros::ServiceClient get_planning_scene_client =
            rh.serviceClient<arm_navigation_msgs::GetPlanningScene > (SET_PLANNING_SCENE_DIFF_NAME);

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::CollisionObject collision_object;

    collision_object.id = "pole";
    collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    collision_object.header.frame_id = "odom_combined";
    collision_object.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < rects.size(); i++)
    {
        add_single_obstacle(collision_object, rects.at(i));

        std::cout << "add_planar_obstacle: adding object " << i << std::endl;
        for (int k = 0; k < 4; k++)
        {
            // std::cout << "its corners: " << rects.at(i).corners.at(k) << std::endl;
        }

    }
    planning_scene_req.planning_scene_diff.collision_objects.push_back(collision_object);

    std::cout << "collision_object size: " << planning_scene_req.planning_scene_diff.collision_objects.size() << std::endl;

    /*
    if (!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) {
        ROS_WARN("Can't get planning scene");
        // return -1;
    }
*/
    
    return planning_scene_res.planning_scene;
}
