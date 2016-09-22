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

#include <pcl/features/normal_3d.h>
//#include <pcl/features/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

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

bool publish = false;
int pc_count = 0;
ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ> acloud;
pcl::PointCloud<pcl::PointXYZ> fccloud; /* Concatenated cloud */
pcl::PointCloud<pcl::PointXYZ>::Ptr tccloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tcprev(new pcl::PointCloud<pcl::PointXYZ>);

// clang-format off
// boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(ccloud);
// clang-format on

/*pcl::PointCloud<pcl::PointXYZ> align_pair(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
					  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, bool downsample = false) {
     TODO: Add downsampling

     Isn't this just a pcl::PointNormal type?
    // pcl::PointCloudWithNormals::Ptr points_with_normals_src(new pcl::PointCloudWithNormals);
    // pcl::PointCloudWithNormals::Ptr points_with_normals_tgt(new pcl::PointCloudWithNormals);

    std::cout << "Starting to align via IPC" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    /* Make thread groud to handle multiple normal calculations

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    float alpha[4] = {1.0, 1.0, 1.0, 1.0};

    pcl::copyPointCloud(*cloud_src, );
    pcl::copyPointCloud(*cloud_tgt, tgt);

    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setTransformationEpsilon(1e-3);
    reg.setMaxCorrespondenceDistance(1.0);
    reg.setMaximumIterations(50);
    reg.setInputTarget(cloud_tgt);
    reg.setInputSource(cloud_src);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

    reg.align(*aligned);
    std::cout << "has converged: " << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;
    std::cout << std::endl << reg.getFinalTransformation() << std::endl;

    /* Transform back into target frame
    pcl::return *aligned;
}*/

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

    std::cout << "\n VELODYNE->ENU Rotation Matrix: " << std::endl << rotation_mat << std::endl << std::endl;

    /* Transform to ENU frame as a nice warm up for ICP */
    pcl::transformPointCloud(*working_cloud, *transformed_cloud, rotation_mat);

    /* Point cloud alignment */
    pcl::PointCloud<pcl::PointXYZ> icp_cloud;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setTransformationEpsilon(1e-5);
    reg.setMaxCorrespondenceDistance(0.05);
    reg.setMaximumIterations(100);

    /* Working with initial conditions */
    if (pc_count == 0) {
	/* Initial cloud, append to end of buffer and continue */
	ROS_INFO("Waiting on second point cloud");
	tcprev = transformed_cloud;
    } else {
	pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
	ROS_INFO("Received cloud, attempting ICP");
	publish = true;

	reg.setInputTarget(tcprev);
	reg.setInputSource(transformed_cloud);

	/* TODO: Add exception catch for non-convergence */
	reg.align(*tcloud);
	std::cout << "ICP has converged: " << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;
	std::cout << std::endl << reg.getFinalTransformation() << std::endl;

	/* Transform ICP'ed cloud into ENU frame */
	// pcl::transformPointCloud(*transformed_cloud, *tcloud, reg.getFinalTransformation());
	tcprev = tcloud;
	fccloud = fccloud + *tcprev;
    }

    if (publish) {
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(fccloud, output);
	// pcl::toROSMsg(icp_cloud, output);

	output.header.stamp = ros::Time::now();
	output.header.frame_id = "enu";
	pub.publish(output);
    }
    pc_count++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_clustering");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_merging_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/clustered", 1);
    ros::spin();
}
