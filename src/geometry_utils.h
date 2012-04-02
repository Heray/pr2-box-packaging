
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <Eigen/Geometry>
#include <math.h>

#include "flap.h"

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

struct XYZ {
   double x,y,z;
};

#endif

void Normalise(XYZ *r);
XYZ ArbitraryRotate(XYZ p,double theta,XYZ r);
XYZ ArbitraryRotate2(XYZ p,double theta,XYZ p1,XYZ p2);
Eigen::Vector3f RotatePoint(Eigen::Vector3f const &pt, double theta, Eigen::Vector3f const &axis_p1, Eigen::Vector3f const &axis_p2);
Eigen::Vector4f VectorToQuaternion(Eigen::Vector3f const &vDirection);
Eigen::Vector4f VectorToQuaternion(pcl::ModelCoefficients::ConstPtr v);

Eigen::Vector4f RectQuaternion(const Flap &rect);