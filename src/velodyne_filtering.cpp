#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PassThrough<pcl::PointXYZ> p_filter;
ros::Publisher pub;

void pc_cleanup(const sensor_msgs::PointCloud2ConstPtr &input) {
    pcl::PCLPointCloud2 *inputPC = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr inputPCPtr(inputPC);
    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr buoy_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*input, *inputPC);
    pcl::fromPCLPointCloud2(*inputPC, *working_cloud);

    p_filter.setInputCloud(working_cloud);
    p_filter.setFilterFieldName("x");
    p_filter.setFilterLimits(0.1, 30);
    p_filter.filter(*working_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(working_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(0.3); // 2cm
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(30);
    ec.setSearchMethod(tree);
    ec.setInputCloud(working_cloud);
    ec.extract(cluster_indices);

    /* First step in segmenting buoys */
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pbuoy_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	    buoy_cluster->points.push_back(working_cloud->points[*pit]); //*
	pbuoy_cluster->width = pbuoy_cluster->points.size();
	pbuoy_cluster->height = 1;
	pbuoy_cluster->is_dense = true;

	// std::cout << "PointCloud representing Buoys: " << buoy_cluster->points.size() << " data points." << std::endl;
	j++;
    }

    /* SAC Segmentation */
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices inlier_indices;

    seg.setInputCloud(buoy_cluster);
    seg.setDistanceThreshold(0.010);
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    seg.setOptimizeCoefficients(true);
    seg.setRadiusLimits(0.05, 0.5);
    seg.setMaxIterations(1000000);

    std::cout << std::endl;
    ROS_INFO("Running RANSAC");
    seg.segment(inlier_indices, *coefficients);

    pcl::PointCloud<pcl::PointXYZRGB> seg_cloud;
    pcl::copyPointCloud(*buoy_cluster, seg_cloud);

    if (inlier_indices.indices.size() == 0) {
	ROS_INFO("RANSAC nothing found\n");
    } else {
	ROS_INFO("RANSAC found shape with [%d] points", (int)inlier_indices.indices.size());
	for (int c = 0; c < coefficients->values.size(); ++c)
	    ROS_INFO("Coeff %d = [%f]", (int)c + 1, (float)coefficients->values[c]);

	for (int m = 0; m < inlier_indices.indices.size(); ++m) {
	    seg_cloud.points[inlier_indices.indices[m]].r = 255;
	    seg_cloud.points[inlier_indices.indices[m]].g = 123;
	    seg_cloud.points[inlier_indices.indices[m]].b = 67;
	}
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(seg_cloud, output);

    /* Set field options */
    output.header.frame_id = "velodyne";
    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_filtering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, pc_cleanup);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/filtered_points", 1);
    ros::spin();
}
