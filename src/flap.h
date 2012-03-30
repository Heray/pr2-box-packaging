
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
    double height;
    double width;
};

#endif