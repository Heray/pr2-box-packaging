// #ifndef EXTRACT_PLANES_H
// #define EXTRACT_PLANES_H


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <stdio.h>

#define PI 3.14159265
#define RANGE 0.1  //angle range
#define UNIT 1.0  // m

typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

class Extractor
{
public:

public:
    pcl::PCDReader reader;
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_plane, seg_cylinder;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<Point> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::KdTreeFLANN<Point>::Ptr tree;
    KdTreePtr clusters_tree_;
    pcl::PassThrough<Point> pass;
    pcl::VoxelGrid<Point> vg;
    double LEAFSIZE;
    unsigned int CUT_THRESHOLD;

    // Datasets
    pcl::PointCloud<Point>::Ptr cloud;
    pcl::PointCloud<Point>::Ptr cloud_filtered;
    pcl::PointIndices::Ptr main_cluster_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::ModelCoefficients::Ptr coefficients_object_, coefficients_plane_, coefficients_cylinder_;
    pcl::PointIndices::Ptr inliers_object_, inliers_plane_, inliers_cylinder_;
    pcl::EuclideanClusterExtraction<Point> cluster_;

    Extractor();

    float compute_plane();
    float compute_cylinder();
    void initializeFromFilename(const std::string & pcd_name);
    void initializeFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp);
    void initializeCFGdebug(const std::string & pcd_name);
    void initialize(bool toFilter);
    bool compute_object(const int i,
                        std::vector<pcl::PointCloud<Point>::ConstPtr > &segments,
                        std::vector<pcl::ModelCoefficients::Ptr > &coefficients);

};

// #endif