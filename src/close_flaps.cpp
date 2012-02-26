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

#include "extract_planes.cpp"
#include "plane_to_rectangle.cpp"

typedef pcl::PointXYZRGB Point;

struct flap {
    std::vector<Eigen::Vector3f> corners;
    pcl::ModelCoefficients normal;
};

void writeRectangleToPC(std::vector<Eigen::Vector3f> rectangle, int counter) {
    pcl::PointCloud<Point> rect_cloud; // = new pcl::PointCloud<Point>;
    for (unsigned int i = 0; i < rectangle.size(); i++) {
        Point p;
        p.x = rectangle.at(i).x();
        p.y = rectangle.at(i).y();
        p.z = rectangle.at(i).z();
        rect_cloud.push_back(p);
    }

    std::stringstream cfg_debug_s;
    cfg_debug_s << "rectangle_" << counter << ".pcd";
    pcl::io::savePCDFile(cfg_debug_s.str(), rect_cloud);
}


void getFlapRectangles(const std::string &filename,
                       std::vector<std::vector<Eigen::Vector3f> > &rectangles) {
  Extractor p;

  // Get planes from point cloud
  std::vector<pcl::PointCloud<Point>::ConstPtr > segments;
  std::vector<pcl::ModelCoefficients::Ptr > coefficients;
  p.initializeFromFilename(filename);
  int i = 0;
  while ((i < 5) && (p.compute_object(i, segments, coefficients))) {
    i++;
  }

  // Find flaps
  // TODO!
  std::vector<pcl::PointCloud<Point>::ConstPtr > flap_segments;
  for (unsigned int i = 0; i < 4; i++) {
      flap_segments.push_back(segments.at(i));
  }
  std::cout << "flap_segment size: " << flap_segments.size() << std::endl;

  // Get rectangles from planes
  for (unsigned int i = 0; i < flap_segments.size(); i++) {
      std::vector<Eigen::Vector3f> rectangle = fitTableTopBbx(flap_segments.at(i), coefficients.at(i));
      rectangles.push_back(rectangle);
      
      writeRectangleToPC(rectangle, i);
  }

  for (unsigned int i = 0; i < rectangles.size(); i++) {
      for (unsigned int j = 0; j < rectangles.at(i).size(); j++) {
          std::cout << "point " << i << " " << j << ": " << rectangles.at(i).at(j) << std::endl;
      }
      printf("\n");
  }

  // Find the lowest two points of each rectangle (flap).
  // Find paths to close each flap

}

void findPathForFlap(std::vector<Eigen::Vector3f> flap)

int main(int argc, char** argv) {
    std::vector<std::vector<Eigen::Vector3f> > flaps;
    getFlapRectangles(std::string(argv[1]), flaps);
}