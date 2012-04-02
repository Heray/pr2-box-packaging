
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#define FLAP_THICKNESS 0.005  // 5mm

#ifndef FLAP_H
#define FLAP_H

struct Flap {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr segment;
    pcl::ModelCoefficients::ConstPtr normal;
    std::vector<Eigen::Vector3f> corners;
    std::vector<Eigen::Vector3f> path_finding_points;
    double height;
    double width;
};

#endif

#ifndef SET_PATH_FINDING_POINTS_H
#define SET_PATH_FINDING_POINTS_H

void set_path_finding_points(Flap &flap) {
    flap.path_finding_points.push_back((flap.corners.at(0) + flap.corners.at(4)) / 2.0);
    flap.path_finding_points.push_back((flap.corners.at(1) + flap.corners.at(4)) / 2.0);
    flap.path_finding_points.push_back((flap.corners.at(2) + flap.corners.at(4)) / 2.0);
    flap.path_finding_points.push_back((flap.corners.at(3) + flap.corners.at(4)) / 2.0);
    flap.path_finding_points.push_back(flap.corners.at(4));
}
#endif
