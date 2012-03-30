#include "extract_planes.h"

/*
class Extractor
{
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
*/
    Extractor::Extractor() {
        tree.reset(new pcl::KdTreeFLANN<Point > ());
        cloud.reset(new pcl::PointCloud<Point>);
        cloud_filtered.reset(new pcl::PointCloud<Point>);
        cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
        coefficients_plane_.reset(new pcl::ModelCoefficients);
        coefficients_cylinder_.reset(new pcl::ModelCoefficients);
        inliers_plane_.reset(new pcl::PointIndices);
        inliers_cylinder_.reset(new pcl::PointIndices);

        // Filter Pass
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-100, 100);

        // VoxelGrid for Downsampling
        LEAFSIZE = 0.005f;
        vg.setLeafSize(LEAFSIZE, LEAFSIZE, LEAFSIZE);

        // Any object < CUT_THRESHOLD will be abandoned.
        //CUT_THRESHOLD = (int) (LEAFSIZE * LEAFSIZE * 7000000); // 700
        CUT_THRESHOLD = (int) (LEAFSIZE * LEAFSIZE * 20000000); // 1500 for nonfiltering

        // Clustering
        cluster_.setClusterTolerance(0.06 * UNIT);
        cluster_.setMinClusterSize(50.0);
        cluster_.setSearchMethod(clusters_tree_);

        // Normals
        ne.setSearchMethod(tree);
        ne.setKSearch(50); // 50 by default

        // plane SAC
        seg_plane.setOptimizeCoefficients(true);
        seg_plane.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg_plane.setNormalDistanceWeight(0.1); // 0.1
        seg_plane.setMethodType(pcl::SAC_RANSAC);
        seg_plane.setMaxIterations(1000); // 10000
        seg_plane.setDistanceThreshold(0.05); // 0.03

        // cylinder SAC
        seg_cylinder.setOptimizeCoefficients(true);
        seg_cylinder.setModelType(pcl::SACMODEL_CYLINDER);
        seg_cylinder.setMethodType(pcl::SAC_RANSAC);
        seg_cylinder.setNormalDistanceWeight(0.1);
        seg_cylinder.setMaxIterations(10000);
        seg_cylinder.setDistanceThreshold(0.02); // 0.05
        seg_cylinder.setRadiusLimits(0.02, 0.07); // [0, 0.1]
    }

    float Extractor::compute_plane() {
        seg_plane.setInputCloud(cloud_filtered);
        seg_plane.setInputNormals(cloud_normals);
        seg_plane.segment(*inliers_plane_, *coefficients_plane_);
        std::cerr << "Plane coefficients: " << *coefficients_plane_ << std::endl;

        //seg.getDistancesToModel(*plane_coefficients_, distances);
        // compute score
        // Ideally this should be the distances and angular distance
        // At here I will just use the sie of the pc.

        float cost = (float) inliers_plane_->indices.size();
        return cost;
    }

    float Extractor::compute_cylinder() {
        seg_cylinder.setInputCloud(cloud_filtered);
        seg_cylinder.setInputNormals(cloud_normals);

        // Obtain the cylinder inliers and coefficients
        seg_cylinder.segment(*inliers_cylinder_, *coefficients_cylinder_);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder_ << std::endl;

        float cost = (float) inliers_cylinder_->indices.size();
        return cost;
    }

    void Extractor::initializeFromFilename(const std::string & pcd_name) {
        reader.read(pcd_name, *cloud);
        std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

        initialize(true);
    }

    void Extractor::initializeFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp) {
        std::cout << "Initializing from Point Cloud" << std::endl;
        cloud->points.resize(cloudp->size());
        for (unsigned int i = 0; i < cloudp->size(); i++) {
            cloud->points.at(i).x = cloudp->points.at(i).x;
            cloud->points.at(i).y = cloudp->points.at(i).y;
            cloud->points.at(i).z = cloudp->points.at(i).z;
            // cloud->points.at(i).rgba = i;
        }
        initialize(false);
    }

    // For debugging
    void Extractor::initializeCFGdebug(const std::string & pcd_name) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cfg;
        cloud_cfg.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        reader.read(pcd_name, *cloud_cfg);
        std::cerr << "PointCloud has: " << cloud_cfg->points.size() << " data points." << std::endl;

        pcl::VoxelGrid<pcl::PointXYZRGB> pvg;
        pvg.setLeafSize(LEAFSIZE, LEAFSIZE, LEAFSIZE);
        pvg.setInputCloud(cloud_cfg);
        pvg.filter(*cloud_cfg);
        std::cout << "PointCloud after prefiltering has: " <<
                cloud_cfg->points.size() << " data points." << std::endl;

        initializeFromCloud(cloud_cfg);
    }

    void Extractor::initialize(bool toFilter) {
        // A passthrough filter to remove spurious NaNs
        pass.setInputCloud(cloud);
        pass.filter(*cloud_filtered);
        std::cout << "PointCloud after pass through has: " <<
                cloud_filtered->points.size() << " data points." << std::endl;


        // Downsample the dataset using a leaf size of 1cm
        // After filtering the point cloud, all indices do not point to the
        // original points. Therefore disable this if called from initializeFromPointCLoud
        if (toFilter) {
            vg.setInputCloud(cloud_filtered);
            vg.filter(*cloud_filtered);
            std::cout << "PointCloud after filtering has: " <<
                    cloud_filtered->points.size() << " data points." << std::endl;
        }
        // Estimate point normals
        ne.setInputCloud(cloud_filtered);
        ne.compute(*cloud_normals);
        ROS_INFO("%lu normals estimated", cloud_normals->points.size());
    }

    bool Extractor::compute_object(const int i, // std::vector<pcl::PointIndices > &segments) {
                        std::vector<pcl::PointCloud<Point>::ConstPtr > &segments,
                        std::vector<pcl::ModelCoefficients::Ptr > &coefficients) {
        // Call get_plane, get_cylinder, get_sphere
        float plane_score = compute_plane(); // plane_inliers_ has the pc.
        // float cylinder_score = compute_cylinder(); // cylinder_inliers_ has the pc

        // Select the lowest score and store it in object_inliers_.
        // inliers_object_ = (plane_score > cylinder_score ? inliers_plane_ : inliers_cylinder_);

        inliers_object_ = inliers_plane_;
        // inliers_object_ = inliers_cylinder_;

        std::cerr << "object inliers has " << inliers_object_->indices.size()<< " points." << std::endl;

        // Test if the point cloud is too small
        if (inliers_object_->indices.size() < CUT_THRESHOLD) {
            std::cerr << "object inliers has " << inliers_object_->indices.size()
                    << " < " << CUT_THRESHOLD << " points. Aborting..." << std::endl;
            return false;
        }

        /*
        // Debugging info
        pcl::PointCloud<Point>::Ptr best_object_(new pcl::PointCloud<Point>);
        pcl::copyPointCloud(*cloud_filtered, *inliers_object_, *best_object_);
        std::stringstream debug_s0;
        debug_s0 << "no_cluster_object_" << i << ".pcd";
        pcl::io::savePCDFile(debug_s0.str(), *best_object_);
         */

        // Euclidean Clustering
        std::vector<pcl::PointIndices> clusters;
        cluster_.setInputCloud(cloud_filtered);
        cluster_.setIndices(inliers_object_);
        cluster_.extract(clusters);
        std::cout << "Number of Euclidian clusters found: " << clusters.size() << std::endl;

        // Check if there is no clusters
        if (clusters.size() == 0) {
            std::cerr << "No clusters found. Aborting..." << std::endl;
            return false;
        }

        main_cluster_.reset(new pcl::PointIndices(clusters.at(0)));

        // Extract the inliers from the input cloud
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(main_cluster_);
        extract.setNegative(false);

        pcl::PointCloud<Point>::Ptr cloud_plane(new pcl::PointCloud<Point > ());
        extract.filter(*cloud_plane);

        // Test if cluster[0] is too small
        if (main_cluster_->indices.size() < CUT_THRESHOLD) {
            std::cerr << "object cluster[0] has " << main_cluster_->indices.size()
                    << " < " << CUT_THRESHOLD << " points. Not writing to disk..." << std::endl;
        } else {
            // Write the inliers to disk
            std::stringstream debug_s;
            if (inliers_object_ == inliers_cylinder_) {
                debug_s << "cylinder_object_" << i << ".pcd";
            } else if (inliers_object_ == inliers_plane_) {
                debug_s << "plane_object_" << i << ".pcd";
            } else {
                debug_s << "unknown_object_" << i << ".pcd";
            }
            pcl::io::savePCDFile(debug_s.str(), *cloud_plane);
            std::cout << "PointCloud representing the extracted component: " <<
                    cloud_plane->points.size() << " data points." << std::endl;


            // Get coefficient
            // pcl::ModelCoefficients temp_coefficient (*coefficients_plane_);
            pcl::ModelCoefficients::Ptr temp_coefficient_;
            temp_coefficient_.reset(new pcl::ModelCoefficients(*coefficients_plane_));

            coefficients.push_back(temp_coefficient_);
            segments.push_back(cloud_plane);

            /*
            // Insert the points to segments for cfg3D
            pcl::PointIndices::Ptr originalIndices(new pcl::PointIndices());
            originalIndices->indices.resize(cloud_plane->size());

            for (unsigned int j = 0; j < originalIndices->indices.size(); j++) {
                // originalIndices->indices.at(j) = cloud_plane->points.at(j).rgba;
                originalIndices->indices.at(j) = cloud_plane->points.at(j).rgb;
            }
            segments.push_back(*originalIndices);
            

            // Debugging for cfg3D
            pcl::PointCloud<Point>::Ptr cfg_segment_cloud(new pcl::PointCloud<Point > ());

            pcl::ExtractIndices<Point> cfg_extract;
            cfg_extract.setNegative(false);
            cfg_extract.setInputCloud(cloud);
            cfg_extract.setIndices(originalIndices);
            cfg_extract.filter(*cfg_segment_cloud);

            std::stringstream cfg_debug_s;
            cfg_debug_s << "cfg_debug_segment_" << i << ".pcd";
            pcl::io::savePCDFile(cfg_debug_s.str(), *cfg_segment_cloud);
            */

            /*
            cfg_segment_cloud->points.resize(originalIndices->indices.size());
            for (unsigned int j = 0; j < originalIndices->indices.size(); j++) {
                unsigned int index = originalIndices->indices.at(j);
                cfg_segment_cloud->points.at(j).x = cloud->points.at(index).x;
                cfg_segment_cloud->points.at(j).y = cloud->points.at(index).y;
                cfg_segment_cloud->points.at(j).z = cloud->points.at(index).z;
            }
             */

        }

        // Remove cluster[0], update cloud_filtered and cloud_normals
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        extract_normals.setNegative(true);
        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(main_cluster_);
        extract_normals.filter(*cloud_normals);

        return true;
    }
// };

/*
int main(int argc, char** argv) {
    ros::init(argc, argv, "Extractor");
    Extractor p;

    std::vector<pcl::PointIndices> segments;
    p.initializeCFGdebug(std::string(argv[1]));
    int i = 0;
    while (i < 100 && p.compute_object(i, segments)) {
        i++;
    }

    return (0);
}
*/
