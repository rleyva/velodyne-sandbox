/* C-std Libs */
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

/* PCL Libs */
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
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
#include <boost/timer/timer.hpp>

ros::Publisher pub;
// clang-format off
boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ> > tpc_buff(10);
// clang-format on

// void transform_pc(const pcl::PCLPointCloud::ConstPtr &input, pcl::PCLPointCloud &output) { ROS_INFO("Transforming..."); }

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
    //(*working_cloud).header.frame_id = "/velodyne";
    //(*working_cloud).header.stamp = (*input).header.stamp.toSec();

    if (tf_listener.canTransform("enu", "velodyne", (*input).header.stamp)) {
	ROS_INFO("Tranform velodyne->enu is VALID");
    } else {
	ROS_INFO("Tranform velodyne->enu is NOT valid");
    }

    if (tf_listener.waitForTransform("enu", "velodyne", (*input).header.stamp, ros::Duration(0.5))) {
	ROS_INFO("TF Available");
    } else {
	ROS_INFO("TF not available");
    }

    /* TODO: Sort this out; tf should correspond to message stamp */
    tf_listener.lookupTransform("enu", "velodyne", ros::Time(0), transform_);

    double yaw, pitch, roll;
    transform_.getBasis().getRPY(roll, pitch, yaw);

    tf::Vector3 v = transform_.getOrigin();
    tf::Quaternion q = transform_.getRotation();

    Eigen::Matrix3d r_;
    Eigen::Matrix4d rotation_mat;

    Eigen::Quaterniond orientation(q.getW(), q.getX(), q.getY(), q.getZ());
    Eigen::Vector3d translation(v.getX(), v.getY(), v.getZ());
    r_ = orientation.toRotationMatrix();

    /*
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << "]"
	      << std::endl
	      << "            in RPY (radian) [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl
	      << "            in RPY (degree) [" << roll * 180.0 / M_PI << ", " << pitch * 180.0 / M_PI << ", "
	      << yaw * 180.0 / M_PI << "]" << std::endl;
    */

    // clang-format off
    rotation_mat << r_(0,0), r_(0,1), r_(0,2), translation(0),
		    r_(1,0), r_(1,1), r_(1,2), translation(1),
		    r_(2,0), r_(2,1), r_(2,2), translation(2),
		    0,       0,       0,       1;
    // clang-format on

    std::cout << "\nRotation Matrix: " << std::endl << rotation_mat << std::endl << std::endl;

    pcl::transformPointCloud(*working_cloud, *transformed_cloud, rotation_mat);
    // pcl::transformPointCloud(*working_cloud, *transformed_cloud, translation, orientation);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "enu";
    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_clustering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_merging_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/clustered", 1);
    ros::spin();
}
