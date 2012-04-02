#include "plane_to_rectangle.h"

typedef pcl::PointXYZ Point;

bool point_z_compare(Eigen::Vector3f f1, Eigen::Vector3f f2) { return f1.z() < f2.z(); }

void fitPlaneToRect(Flap &flap)
{
    pcl::PointCloud<Point>::ConstPtr cloud = flap.segment;
    pcl::ModelCoefficients::ConstPtr table_coefficients_const_ = flap.normal;

    // Project points onto the table plane
    pcl::ProjectInliers<Point> proj;
    // pcl::ModelCoefficients::Ptr table_coefficients_const_;
    // table_coefficients_const_.reset(new pcl::ModelCoefficients);
    // table_coefficients_const_->set_values_size(4);
    // table_coefficients_const_->values[3] = 1.0;
    proj.setModelType(pcl::SACMODEL_PLANE);
    pcl::PointCloud<Point> projected_cloud;
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
    std::vector<cv::Point2f> points;
    // choose a point on the plane
    Eigen::Vector3f p0(projected_cloud.points[0].x,
                       projected_cloud.points[0].y,
                       projected_cloud.points[0].z);
    for (unsigned int ii = 0; ii < projected_cloud.points.size(); ii++)
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


    //store the table top bounding points in a vector
    for (unsigned int ii = 0; ii < 4; ii++)
    {
        Eigen::Vector3f pbbx(rrPts[ii].x * u + rrPts[ii].y * v + p0);
        flap.corners.push_back(pbbx);
    }
    Eigen::Vector3f center(rrect.center.x * u + rrect.center.y * v + p0);
    flap.corners.push_back(center);


    //****** Get height and width *******
    Eigen::Vector3f flap_center = flap.corners.at(4);

    // Find the bottom of the flap
    std::sort(flap.corners.begin(), flap.corners.end(), point_z_compare);

    // Correct the point order if necessary
    if ((flap.corners.at(2)-flap.corners.at(0)).norm() > (flap.corners.at(3)-flap.corners.at(0)).norm()) {
        Eigen::Vector3f temp = flap.corners.at(2);
        flap.corners.at(2) = flap.corners.at(3);
        flap.corners.at(3) = temp;
        std::cout << "Wrong Corner Order!!!!";
    }

    // Find the center point of the bottom
    Eigen::Vector3f bottom_center((flap.corners.at(0) + flap.corners.at(1)) / 2);

    // Find the distance between the center of the flap to the center of the bottom
    double dist = (flap_center - bottom_center).norm();

    // Set the height and width of the flap
    flap.width = (flap.corners.at(0) - flap.corners.at(1)).norm();
    flap.height = 2.0 * dist;
}
