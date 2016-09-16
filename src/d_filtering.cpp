#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <boost/circular_buffer.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>

/* TODO: Make structure containing time stamp from original message along with PointXYZ cloud */

ros::Publisher pub;

boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclBuffer(3);
boost::circular_buffer<tf::StampedTransform> tfBuffer(3);

// pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

/* NOTE: This could be implemented with ROS message_filters::Cache        */
/*       We're using a ground up implmentation for some added flexibility */
void temporal_filter_cb(const sensor_msgs::PointCloud2ConstPtr &input) {

    pcl::PCLPointCloud2 *inputPC = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr inputPCPtr(inputPC);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*input, *inputPC);
    pcl::fromPCLPointCloud2(*inputPC, *cloud_filtered);

    if (!pclBuffer.full()) {
	std::cout << "Adding converted cloud..." << std::endl;
	pclBuffer.push_back(cloud_filtered);
	return;
    } else {
	std::cout << "Buffer is full, carrying out other functions" << std::endl;
    }
}

void normal_filter_cb(const sensor_msgs::PointCloud2ConstPtr &input) {

    pcl::PCLPointCloud2 *inputPC = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr inputPCPtr(inputPC);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*input, *inputPC);
    pcl::fromPCLPointCloud2(*inputPC, *working_cloud);

    double small_scale = 2;
    double large_scale = 6;
    double threshold = 2;
    double segradius = 2;

    pcl::search::Search<pcl::PointXYZ>::Ptr tree;

    if (working_cloud->isOrganized())
	tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    else
	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));

    tree->setInputCloud(working_cloud);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(working_cloud);
    ne.setSearchMethod(tree);

    // ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (),
    //		     std::numeric_limits<float>::max ());

    ne.setViewPoint(0, 0, 0);

    std::cout << "Calculating normals for scale..." << small_scale << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(small_scale);
    ne.compute(*normals_small_scale);

    std::cout << "Calculating normals for scale..." << large_scale << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch(large_scale);
    ne.compute(*normals_large_scale);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*working_cloud, *doncloud);

    std::cout << "Calculating DoN... " << std::endl;
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud(working_cloud);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    don.computeFeature(*doncloud);
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());

    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
	new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)));

    pcl::ConditionalRemoval<pcl::PointNormal> condrem(range_cond);
    condrem.setInputCloud(doncloud);

    // pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

    condrem.filter(*doncloud_filtered);
    doncloud = doncloud_filtered;

    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree(new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud(doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

    ec.setClusterTolerance(segradius);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(segtree);
    ec.setInputCloud(doncloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++) {
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointNormal>);

	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
	    cloud_cluster_don->points.push_back(doncloud->points[*pit]);
	}

	cloud_cluster_don->width = int(cloud_cluster_don->points.size());
	cloud_cluster_don->height = 1;
	cloud_cluster_don->is_dense = true;
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*working_cloud, output);
    pub.publish(output);
}

void radius_outlier_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    // boost::timer::auto_cpu_timer t;

    pcl::PCLPointCloud2 *rosPC2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr rosPC2Ptr(rosPC2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PCLPointCloud2> vf;
    pcl::PCLPointCloud2 filteredCloud;

    pcl_conversions::toPCL(*input, *rosPC2);

    vf.setInputCloud(rosPC2Ptr);
    vf.setLeafSize(0.1, 0.1f, 0.1f); /* Toy around with these values */
    vf.filter(filteredCloud);

    pcl::fromPCLPointCloud2(filteredCloud, *cloud_filtered);

    // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (rosPC2Ptr);
    // sor.setMeanK (10);			/* Toy around with this value */
    // sor.setStddevMulThresh (1);		/* Also toy around with this value */
    // sor.filter (filteredCloud);

    // pcl::search::KdTree<pcl::PCLPointCloud2>::Ptr tree (new pcl::search::KdTree<pcl::PCLPointCloud2>);
    // pcl::PointCloud<pcl::PointNormal> mls_points;
    // pcl::MovingLeastSquares<pcl::PCLPointCloud2, pcl::PointNormal> mls;

    // mls.setComputeNormals (true);
    // mls.setInputCloud (rosPC2Ptr);
    // mls.setPolynomialFit (true);
    // mls.setSearchMethod (tree);
    // mls.setSearchRadius (0.03);

    // mls.process (mls_points);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)cloud_filtered->points.size();

    while (cloud_filtered->points.size() > 0.3 * nr_points) {
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
	    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	    break;
	}

	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_p);
	std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points."
		  << std::endl;

	extract.setNegative(true);
	extract.filter(*cloud_f);
	cloud_filtered.swap(cloud_f);
	i++;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish(output);
}

void statistical_outlier_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    /* Converts ROS point cloud & carries out statitstical filtering */
    /* Boost timer has ~5ns resolution */
    // boost::timer::auto_cpu_timer t;

    pcl::PCLPointCloud2 *rosPC2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr rosPC2Ptr(rosPC2);
    pcl::PCLPointCloud2 filteredCloud;

    pcl_conversions::toPCL(*input, *rosPC2);

    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(rosPC2Ptr);
    sor.setMeanK(10);	  /* Toy around with this value */
    sor.setStddevMulThresh(1); /* Also toy around with this value */
    sor.filter(filteredCloud);

    /*
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < rosPC2Ptr->points.size (); ++i)
	 std::cerr << "    " << rosPC2Ptr->points[i].x << " "
			     << rosPC2Ptr->points[i].y << " "
			     << rosPC2->points[i].z << std::endl;

    /* Display pointcloud after filtering */
    /*
    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < filteredCloud->points.size (); ++i)
	 std::cerr << "    " << filteredCloud->points[i].x << " "
			     << filteredCloud->points[i].y << " "
			     << filteredCloud->points[i].z << std::endl;
    */

    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(filteredCloud, output);
    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_filtering");
    ros::NodeHandle nh;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normal Viewer"));
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    viewer->addPointCloud<pcl::PointXYZ>(working_cloud, "cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(working_cloud, doncloud_filtered, 5, 0.05, "shit");

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, normal_filter_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/filtered_points", 1);
    ros::spin();
}
