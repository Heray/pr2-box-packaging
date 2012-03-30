
#ifndef CLOSE_FLAPS_H
#define CLOSE_FLAPS_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv/cv.h>
#include <pcl/filters/project_inliers.h>

#include "flap.h"
#include "geometry_utils.h"
#include "extract_planes.h"
#include "plane_to_rectangle.h"
#include "find_path.h"
#include "add_planar_obstacle.h"

typedef pcl::PointXYZ Point;
#define PI 3.1415926

#endif
