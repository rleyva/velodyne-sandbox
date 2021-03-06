#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

ros::Publisher pub;

void ransac_fitting_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    /* Converting ROS message into PCL-native cloud */
    pcl::PCLPointCloud2 *input_pc = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr input_ptr(input_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /* TODO: Look into fromROSMsg function */
    pcl_conversions::toPCL(*input, *input_pc);
    pcl::fromPCLPointCloud2(*input_pc, *working_cloud);

    /* Pass-through filtering - Filtering NaN points*/
    // pcl::PassThrough<PointXYZ> pass;
    // pass.setInputCloud(*working_cloud);
    // pass.setFilterFieldname("z");
    // pass.setFilerLimits(0.0, 1.0);
    // pass.filter(*working_cloud);

    /* kd-tree Generation  */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ktree(new pcl::search::KdTree<pcl::PointXYZ>);
    ktree->setInputCloud(working_cloud);

    /* MLS smoothening */
    // pcl::PointCloud<pcl::PointXYZ> mls_points;
    // pcl::PointCloud<pcl::Normal>::Ptr mls_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setInputCloud(working_cloud);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(ktree);
    mls.setSearchRadius(0.3);
    mls.setComputeNormals(true);

    mls.process(mls_points);

    std::cout << "Point Cloud Size: " << working_cloud->size() << std::endl;
    std::cout << "MLS Cloud Size: " << mls_points.points.size() << std::endl;

    /* Outlier removal */
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud(mls_points.makeShared());
    sor.setStddevMulThresh(1.0);
    sor.filter(mls_points);

    std::cout << "Outlier Removal Size: " << mls_points.points.size() << std::endl;

    /* SAC segmentation */
    /*   This will operate on the MLS cloud previously generated */
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg;
    // pcl::SACSegmentation<pcl::PointNormal> seg;

    seg.setInputCloud(mls_points.makeShared());
    seg.setInputNormals(mls_points.makeShared());
    seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    // seg.setModelType(pcl::SACMODEL_SPHERE);
    // seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_LMEDS);
    // seg.setDistanceThreshold(0.);
    seg.setNormalDistanceWeight(0.1);
    seg.setOptimizeCoefficients(true);
    seg.setRadiusLimits(0.1, 0.15);
    seg.setEpsAngle(30 / (180 / 3.141592654));
    seg.setMaxIterations(1000000);

    ROS_INFO("Running RANSAC");

    pcl::PointIndices inlier_indices;
    seg.segment(inlier_indices, *coefficients);

    ROS_INFO("Finished Segmenting");

    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
    pcl::copyPointCloud(mls_points, rgb_cloud);

    if (inlier_indices.indices.size() == 0) {
	ROS_INFO("RANSAC nothing found");
    } else {
	ROS_INFO("RANSAC found shape with [%d] points", (int)inlier_indices.indices.size());
	for (int c = 0; c < coefficients->values.size(); ++c)
	    ROS_INFO("Coeff %d = [%f]", (int)c + 1, (float)coefficients->values[c]);

	for (int m = 0; m < inlier_indices.indices.size(); ++m) {
	    rgb_cloud.points[inlier_indices.indices[m]].r = 0;
	    rgb_cloud.points[inlier_indices.indices[m]].g = 255;
	    rgb_cloud.points[inlier_indices.indices[m]].b = 0;
	}
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(rgb_cloud, output);
    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ransac_fitting");
    ros::NodeHandle nh;

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normal Viewer"));
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // viewer->addPointCloud<pcl::PointXYZ>(working_cloud, "cloud");
    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(working_cloud, doncloud_filtered, 5, 0.05, "shit");

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, ransac_fitting_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/filtered_points", 1);
    ros::spin();
}
