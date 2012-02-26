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

typedef pcl::PointXYZRGB Point;


std::vector<Eigen::Vector3f> fitTableTopBbx(pcl::PointCloud<Point>::ConstPtr& cloud,
                                            pcl::ModelCoefficients::ConstPtr table_coefficients_const_)
{
     
     // Project points onto the table plane
     pcl::ProjectInliers<Point>    proj;
     // pcl::ModelCoefficients::Ptr table_coefficients_const_;
     // table_coefficients_const_.reset(new pcl::ModelCoefficients);
     // table_coefficients_const_->set_values_size(4);
     // table_coefficients_const_->values[3] = 1.0;
     proj.setModelType(pcl::SACMODEL_PLANE);
     pcl::PointCloud<Point>    projected_cloud;
     proj.setInputCloud(cloud);
     proj.setModelCoefficients(table_coefficients_const_);
     proj.filter(projected_cloud);

     // store the table top plane parameters
     Eigen::Vector3f plane_normal;
     plane_normal.x() = table_coefficients_const_->values[0];
     plane_normal.y() = table_coefficients_const_->values[1];
     plane_normal.z() = table_coefficients_const_->values[2];
     // compute an orthogonal normal to the plane normal
     Eigen::Vector3f v = plane_normal.unitOrthogonal();
     // take the cross product of the two normals to get
     // a thirds normal, on the plane
     Eigen::Vector3f u = plane_normal.cross(v);

     // project the 3D point onto a 2D plane
     std::vector<cv::Point2f>    points;
     // choose a point on the plane
     Eigen::Vector3f p0(projected_cloud.points[0].x,
                         projected_cloud.points[0].y,
                         projected_cloud.points[0].z);
     for(unsigned int ii=0; ii<projected_cloud.points.size(); ii++)
     {
       Eigen::Vector3f p3d(projected_cloud.points[ii].x,
                            projected_cloud.points[ii].y,
                            projected_cloud.points[ii].z);

       // subtract all 3D points with a point in the plane
       // this will move the origin of the 3D coordinate system
       // onto the plane

       // std::cout << p3d << std::endl;
       
       p3d = p3d - p0;

       cv::Point2f p2d;
       p2d.x = p3d.dot(u);
       p2d.y = p3d.dot(v);
       // std::cout << p2d << std::endl;
       points.push_back(p2d);
     }

     cv::Mat points_mat(points);
     cv::RotatedRect rrect = cv::minAreaRect(points_mat);
     cv::Point2f rrPts[4];
     rrect.points(rrPts);

     std::vector<Eigen::Vector3f>    table_top_bbx;

     //store the table top bounding points in a vector
     for(unsigned int ii=0; ii<4; ii++) {
         Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0);
         table_top_bbx.push_back(pbbx);
     }
     Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0);
     table_top_bbx.push_back(center);

     return table_top_bbx;
}
