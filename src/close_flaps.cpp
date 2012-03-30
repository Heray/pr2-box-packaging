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
// #define PI 3.1415926

void writeRectangleToPC(const Flap &flap, int counter) {
    pcl::PointCloud<Point> rect_cloud; // = new pcl::PointCloud<Point>;
    for (unsigned int i = 0; i < flap.corners.size(); i++) {
        Point p;
        p.x = flap.corners.at(i).x();
        p.y = flap.corners.at(i).y();
        p.z = flap.corners.at(i).z();
        rect_cloud.push_back(p);
    }

    std::stringstream cfg_debug_s;
    cfg_debug_s << "rectangle_" << counter << ".pcd";
    pcl::io::savePCDFile(cfg_debug_s.str(), rect_cloud);

    std::stringstream cfg_debug_s2;
    cfg_debug_s2 << "flap_" << counter << ".pcd";
    pcl::io::savePCDFile(cfg_debug_s2.str(), *(flap.segment));

}

Eigen::Vector3f getFlapsCenter(const std::vector<Flap> &flaps) {
    double x = 0;
    double y = 0;
    double z = 0;
    for (unsigned int i = 0; i < flaps.size(); i++) {
        for (unsigned int j = 0 ; j < flaps.at(i).corners.size(); j++) {
            x += flaps.at(i).corners.at(j).x();
            y += flaps.at(i).corners.at(j).y();
            z += flaps.at(i).corners.at(j).z();
        }
    }
    Eigen::Vector3f res (x, y, z);
    return res;
}

void getRectangles(const std::string &filename, std::vector<Flap> &rects) {
  Extractor p;

  // Get planes from point cloud
  std::vector<pcl::PointCloud<Point>::ConstPtr > segments;
  std::vector<pcl::ModelCoefficients::Ptr> coefficients;
  p.initializeFromFilename(filename);
  int i = 0;
  while ((i < 9) && (p.compute_object(i, segments, coefficients))) {
    std::vector<Eigen::Vector3f> corners;
    Flap rect = {segments.at(i), coefficients.at(i), corners};
    // this_flap.segment = segments.at(i);
    // this_flap.normal = coefficients.at(i);
    // this_flap.corners = corners;
    fitPlaneToRect(rect);

    rects.push_back(rect);
    i++;
  }

  std::cout << "num planes found: " << rects.size() << std::endl;
}

double flap_z(const Flap &flap) {
    double mean_z = 0;
    for (unsigned int i = 0; i < flap.segment->size(); i++) {
        mean_z += flap.segment->at(i).z;
    }
    return mean_z/flap.segment->size();
}

bool flap_z_compare(const Flap &f1, const Flap &f2) { return flap_z(f1) > flap_z(f2); }

void getFlaps(std::vector<Flap> & rects, std::vector<Flap> &flaps) {
  // Find flaps
  // TODO!
  // Currently just get the top 4 planes as flaps
  std::sort(rects.begin(), rects.end(), flap_z_compare);
  /*
  std::vector<pcl::PointCloud<Point>::ConstPtr > flap_segments;
  for (unsigned int i = 0; i < 4; i++) {
      flap_segments.push_back(segments.at(i));
  }
  std::cout << "flap_segment size: " << flap_segments.size() << std::endl;
  */
  // Get rectangles from planes
  for (unsigned int i = 0; i < 4; i++) {
      flaps.push_back(rects.at(i));
      // fitPlaneToRect(flaps.at(i));
      
      writeRectangleToPC(flaps.at(i), i);
  }

  for (unsigned int i = 0; i < 4; i++) {
      for (unsigned int j = 0; j < flaps.at(i).corners.size(); j++) {
          std::cout << "point " << i << " " << j << ": " << flaps.at(i).corners.at(j) << std::endl;
      }
      printf("\n");
  }

  // Find the lowest two points of each rectangle (flap).
  // Find paths to close each flap

}

void addPath(const Eigen::Vector3f &pt, const Eigen::Vector3f &normal, pcl::PointCloud<pcl::PointNormal> &path) {
    pcl::PointNormal start_path;
    start_path.x = pt.x(); start_path.y = pt.y(); start_path.z = pt.z();
    start_path.normal[0] = normal.x(); start_path.normal[1] = normal.y(); start_path.normal[2] = normal.z();
    path.push_back(start_path);
}
void addPath(const Eigen::Vector3f &pt, const pcl::ModelCoefficients::ConstPtr normal, pcl::PointCloud<pcl::PointNormal> &path) {
    Eigen::Vector3f n (normal->values.at(0), normal->values.at(1), normal->values.at(2));
    addPath(pt, n, path);
}

void findPathForFlap(const Flap &flap,
                     Eigen::Vector3f const &box_center,
                     const std::vector<Flap> &obstacle_planes,
                     int counter) {
    std::vector<Eigen::Vector3f> corners = flap.corners;
    Eigen::Vector3f flap_center = corners.at(4);

    // Find the bottom of the flap
    // flap.sort(flap_z_compare);
    // std::sort(corners.begin(), corners.end(), point_z_compare);

    // Find the center point of the bottom
    Eigen::Vector3f bottom_center((corners.at(0) + corners.at(1)) / 2);

    // Find the distance between the center of the flap to the center of the bottom
    double dist = (flap_center - bottom_center).norm();

    // Get the height and width of the flap
    double width = (corners.at(0) - corners.at(1)).norm();
    double height = 2.0 * dist;
    std::cout << "edge: " << corners.at(0) << ", " << corners.at(1) << std::endl;
    std::cout << "center: " << bottom_center << std::endl;
    std::cout << "width: " << width << " height: " << height << std::endl;

    // Find the vertical point
    Eigen::Vector3f vertical_mid (bottom_center.x(), bottom_center.y(), bottom_center.z() + dist);

    // Find the ending horizontal point
    Eigen::Vector3f horiz_end_pos = RotatePoint(vertical_mid, PI/2.0, corners.at(0), corners.at(1));
    Eigen::Vector3f horiz_end_neg = RotatePoint(vertical_mid, -PI/2.0, corners.at(0), corners.at(1));
    double dist_pos = (horiz_end_pos - box_center).norm();
    double dist_neg = (horiz_end_neg - box_center).norm();
    Eigen::Vector3f horiz_end = horiz_end_pos;
    if (dist_pos > dist_neg ) {
        horiz_end = horiz_end_neg;
    }

    // Find the orientation of the vertical plane and the horizontal plane
    Eigen::Vector3f vertical_orientation = (horiz_end - bottom_center);
    vertical_orientation /= vertical_orientation.norm();
    Eigen::Vector3f horiz_orientation = RotatePoint(vertical_mid, PI, corners.at(0), corners.at(1)) - bottom_center;
    horiz_orientation /= horiz_orientation.norm();

    // Add obstacles to collision checker
    arm_navigation_msgs::PlanningScene planning_diff = add_obstacles(obstacle_planes);

    // Start planning
    Eigen::Vector3f tolerance_plane (width, height, FLAP_THICKNESS);

    // First move the arm to the starting position
    Eigen::Vector3f start_pt (flap_center.x(), flap_center.y(), flap_center.z() - FLAP_THICKNESS);
    VectorToQuaternion(flap.normal);
    find_path(start_pt, tolerance_plane, flap, VectorToQuaternion(flap.normal), planning_diff);

    /*
    // Then move the arm to the vertical mid
    find_path(vertical_mid, tolerance_plane, VectorToQuaternion(vertical_orientation));
    VectorToQuaternion(vertical_orientation);

    // Finally move the arm to the horizaontal end
    find_path(horiz_end, tolerance_plane, VectorToQuaternion(horiz_orientation));
    VectorToQuaternion(horiz_orientation);

    // debug info
    pcl::PointCloud<pcl::PointNormal> path;
    addPath(start_pt, flap.normal, path);
    addPath(vertical_mid, vertical_orientation, path);
    addPath(horiz_end, horiz_orientation, path);

    std::stringstream cfg_debug_s;
    cfg_debug_s << "flap_path_" << counter << "_debug.pcd";
    pcl::io::savePCDFile(cfg_debug_s.str(), path);
    */
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "move_arm_joint_goal_test");

    // Get the flaps (both corners and orientations)
    std::vector<Flap> rects;  // all planes
    std::vector<Flap> flaps;  // just flaps
    
    getRectangles(std::string(argv[1]), rects);

    getFlaps(rects, flaps);

    Eigen::Vector3f flaps_center = getFlapsCenter(flaps);

    // For each flap, plan a path!
    for (unsigned int i = 0; i < 4; i++) {
        std::cout << "findPathForflap!!!" << std::endl;
        // Build obstacle rects
        std::vector<Flap> obstacle_planes;
        for (unsigned int k = 0; k < rects.size(); k++) {
            if (i != k) {
                obstacle_planes.push_back(rects.at(k));
            }
        }

        findPathForFlap(flaps.at(i), flaps_center, obstacle_planes, i);
    }
}