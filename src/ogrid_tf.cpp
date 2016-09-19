/* C-std Libs */
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

/* PCL Libs */
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>

/* ROS Libs */
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

/* TF bullshit */
#include <tf/tf.h>
#include <tf/transform_listener.h>

/* Boost */
#include <boost/circular_buffer.hpp>

ros::Publisher pub;
// clang-format off
boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ> > tpc_buff(10);
// clang-format on

void cloud_merging_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    /* TF-Shenanigans */
    tf::TransformListener tf_listener;
    tf::StampedTransform transform_;

    /* Converting ROS message into friendlier PCL-native format */
    pcl::PCLPointCloud2 *input_pc = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr input_pc_ptr(input_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*input, *input_pc);
    pcl::fromPCLPointCloud2(*input_pc, *working_cloud);

    ROS_INFO("Attempting to transform pointcloud");
    (*working_cloud).header.frame_id = "/velodyne";
    (*working_cloud).header.stamp = (*input).header.stamp.toSec();

    if (tf_listener.waitForTransform("enu", "velodyne", (*input).header.stamp, ros::Duration(5.0))) {
	ROS_INFO("TF Available");
    }
    tf_listener.lookupTransform("enu", "velodyne", (*input).header.stamp, transform_);
    // pcl_ros::transformPointCloud("enu", *working_cloud, *transformed_cloud, tf_listener);
    // tf_listener.lookupTransform("velodyne", "ins", ros::Time(0), transform_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_clustering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_merging_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/clustered", 1);
    ros::spin();
}
