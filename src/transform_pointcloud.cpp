#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include "pcl/io/io.h"
#include "pcl/point_types.h"
#include "pcl/surface/mls.h"
#include "pcl_ros/publisher.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl/common/transform.h>

#include <iostream>
#include <fstream>

typedef pcl::PointXYZ Point;

double StrToDouble(const std::string& s) {
  double result;
  std::istringstream ss(s);
  ss >> result;
  if (!ss) {
    std::cout << s << std::endl;
    throw std::invalid_argument("StrToInt");
  }
  return result;
}

void GetMatrixFromFile(const char* fname, Eigen::Matrix4f& tm) {
  std::ifstream file(fname);
  if (file != NULL) {
    std::string line;
    std::getline(file, line);
    std::getline(file, line);

    // Read file. create vector of AABB's
    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of(" "));
    double m00 = StrToDouble(strs[0]);
    double m01 = StrToDouble(strs[1]);
    double m02 = StrToDouble(strs[2]);
    double m03 = StrToDouble(strs[3]);
    double m10 = StrToDouble(strs[4]);
    double m11 = StrToDouble(strs[5]);
    double m12 = StrToDouble(strs[6]);
    double m13 = StrToDouble(strs[7]);
    double m20 = StrToDouble(strs[8]);
    double m21 = StrToDouble(strs[9]);
    double m22 = StrToDouble(strs[10]);
    double m23 = StrToDouble(strs[11]);

    tm.row(0) = Eigen::Vector4f(m00,m01,m02,m03);
    tm.row(1) = Eigen::Vector4f(m10,m11,m12,m13);
    tm.row(2) = Eigen::Vector4f(m20,m21,m22,m23);      
  } else {
    std::cout << "input file could not be found!" << std::endl;
  }
}

void TransformPointCloud(const char* point_cloud_name,
                         const char* matrix_name) {
  pcl::PointCloud<Point> cloud_out;
  pcl::PointCloud<Point> cloud_raw;

  std::string pcd_name = std::string(point_cloud_name);
  pcl::io::loadPCDFile(pcd_name, cloud_raw);

  Eigen::Matrix4f matrix_4f = Eigen::Matrix4f::Identity();
  GetMatrixFromFile(matrix_name, matrix_4f);
  pcl::transformPointCloud(cloud_raw, cloud_out, matrix_4f);
  pcl::io::savePCDFile(std::string("point_cloud_transformed.pcd"),
                       cloud_out);
}

/*
int main (int argc, char** argv) {
  ros::init(argc, argv, "pcl_surface_mls_test");
  TransformPointCloud(argv[1], argv[2]);
  return (0);
}
*/
