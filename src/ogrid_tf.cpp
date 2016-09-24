/* C-std Libs */
#include <iostream>
#include <math.h>
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
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

/* ROS Libs */
#include <message_filters/subscriber.h>
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
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>

int tf_count = 0;
int tf_delta = 0;
int pc_count = 0;
bool publish = false;

ros::Publisher pub;
ros::Time lt;

pcl::PointCloud<pcl::PointXYZ>::Ptr ccloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cprev(new pcl::PointCloud<pcl::PointXYZ>);

void downsample_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr output_src) {
    ROS_INFO("Downsampling incoming cloud");
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_src);
    vg.setLeafSize(0.5, 0.5, 0.5);
    vg.filter(*output_src);
}

void remove_outliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr output_src) {
    ROS_INFO("Removing outliers");
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*output_src);
}

bool fetch_latest_tf(const std::string &source_frame, const std::string &target_frame, const ros::Time &ts,
		     const tf::TransformListener &tf_listener, tf::StampedTransform &transform) {
    /* Thread that will handle waiting for TF to be made available.
     * Returns true if transform is successful
     *
     * TODO: Do not return most recent transform; return one closer to the actual timestamp */

    bool tf_status = false;
    ROS_INFO("Attempting to transform pointcloud");

    if (tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1))) {
	lt = ros::Time(0);
	ROS_INFO("TF Available");
	tf_status = true;
    } else {
	ROS_INFO("TF not available");
    }
    tf_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

    /* Check tf-message delta */
    ros::Duration t_ = (ros::Time::now() - ts);
    std::cout << "ROS Time: " << ros::Time::now().toSec() << std::endl;
    std::cout << "Message Time: " << ts.toSec() << std::endl;
    std::cout << "Timing delta: " << t_.toSec() << std::endl;

    return tf_status;
}

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

    /* Cleaning up point cloud */
    downsample_cloud(working_cloud, working_cloud);
    remove_outliers(working_cloud, working_cloud);

    boost::thread tf_helper(&fetch_latest_tf, "/velodyne", "/enu", boost::ref((*input).header.stamp), boost::ref(tf_listener),
			    boost::ref(transform_));
    tf_helper.join();

    double yaw, pitch, roll;
    transform_.getBasis().getRPY(roll, pitch, yaw);

    tf::Vector3 v = transform_.getOrigin();
    tf::Quaternion q = transform_.getRotation();

    Eigen::Matrix3d r_;
    Eigen::Matrix4d rotation_mat;

    Eigen::Quaterniond orientation(q.getW(), q.getX(), q.getY(), q.getZ());
    Eigen::Vector3d translation(v.getX(), v.getY(), v.getZ());
    r_ = orientation.toRotationMatrix();

    // clang-format off
    rotation_mat << r_(0, 0), r_(0, 1), r_(0, 2), translation(0), 
		    r_(1, 0), r_(1, 1), r_(1, 2), translation(1), 
		    r_(2, 0), r_(2, 1), r_(2, 2), translation(2), 
		    0,        0,        0,        1;
    // clang-format on

    std::cout << "\n Velodyne -> ENU Rotation Matrix: " << std::endl << rotation_mat << std::endl << std::endl;

    /* Transform to ENU frame as a nice warm up for ICP */
    pcl::transformPointCloud(*working_cloud, *transformed_cloud, rotation_mat);

    /* Point cloud alignment */
    pcl::PointCloud<pcl::PointXYZ> icp_cloud;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setTransformationEpsilon(1e-8);
    reg.setMaxCorrespondenceDistance(1);
    reg.setEuclideanFitnessEpsilon(1);
    reg.setMaximumIterations(200);

    /* Working with initial conditions */
    if (pc_count == 0) {
	/* Initial cloud, append to end of buffer and continue */
	ROS_INFO("Waiting on second point cloud");
	cprev = transformed_cloud;
    } else {
	ROS_INFO("Received cloud, attempting ICP");
	// publish = true;

	/*
	 * We want to output 3 point clouds in the following format:
	 * 1. Original point cloud in WHITE
	 * 2. Original + 1 cloud in RED
	 * 3. ICP fit in GREEN
	 *
	 * This should only happen if the ICP convergence score is >> 1
	 */

	pcl::PointCloud<pcl::PointXYZ> tcloud;

	reg.setInputTarget(cprev);	     /* Original cloud is set as the target */
	reg.setInputSource(transformed_cloud); /* Most recently recieved cloud is used as the fit-ee */
	reg.align(tcloud);		       /* Aligned cloud is saved as tcloud */

	std::cout << "ICP has converged: " << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;
	std::cout << std::endl << reg.getFinalTransformation() << std::endl;

	/* TODO: After X frames, choose new inital cloud to fit onto */
	if (reg.getFitnessScore() < 1) {
	    std::cout << "Fitness is GOOD" << std::endl;

	    sensor_msgs::PointCloud2 output;
	    output.header.frame_id = "enu";
	    output.header.stamp = lt;

	    pcl::PointCloud<pcl::PointXYZRGB> cc_;

	    pcl::copyPointCloud(*cprev, cc_);
	    for (int m = 0; m < cc_.size(); ++m) {
		cc_.points[m].r = 255;
		cc_.points[m].g = 255;
		cc_.points[m].b = 255;
	    }
	    pcl::toROSMsg(cc_, output);
	    pub.publish(output);
	    ROS_INFO("Published original cloud");
	    ros::spinOnce();

	    pcl::copyPointCloud(*transformed_cloud, cc_);
	    for (int m = 0; m < cc_.size(); ++m) {
		cc_.points[m].r = 255;
		cc_.points[m].g = 0;
		cc_.points[m].b = 0;
	    }
	    pcl::toROSMsg(cc_, output);
	    pub.publish(output);
	    ROS_INFO("Publshed newly recieved cloud");
	    ros::spinOnce();

	    pcl::copyPointCloud(tcloud, cc_);
	    for (int m = 0; m < cc_.size(); ++m) {
		cc_.points[m].r = 0;
		cc_.points[m].g = 255;
		cc_.points[m].b = 0;
	    }
	    pcl::toROSMsg(cc_, output);
	    pub.publish(output);
	    ROS_INFO("Published transformed cloud");
	    ros::spinOnce();
	}

	/* Transform ICP'ed cloud into ENU frame */
	// pcl::transformPointCloud(*transformed_cloud, *tcloud, reg.getFinalTransformation());
	// tcprev = tcloud;
	// fccloud = fccloud + *tcprev;
    }
    /*
	if (publish) {
	    sensor_msgs::PointCloud2 output;
	    pcl::toROSMsg(*tcprev, output);
	    output.header.stamp = tl;
	    output.header.frame_id = "enu";
	    pub.publish(output);
	}*/
    pc_count++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cloud_registration");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 4, cloud_merging_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/clustered", 4);
    ros::spin();
}
