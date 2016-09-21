#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "buoy_marker_pub");
    ros::NodeHandle nh;
    ros::Rate r(1);

    uint32_t buoy = visualization_msgs::Marker::SPHERE;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    while (ros::ok()) {
	visualization_msgs::Marker marker;
	/* TODO: Make this an argument handled via launch */
	marker.header.frame_id = "/enu";
	marker.header.stamp = ros::Time::now();

	/* TODO: Change to "for buoy in... */
	marker.ns = "buoy";
	marker.type = buoy;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;

	/* TODO: Use RANSAC values to generae sphere sizes */
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;

	/* TODO: Set color based on confidence or color detected */
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;

	marker.lifetime = ros::Duration();

	while (marker_pub.getNumSubscribers() < 1) {
	    if (!ros::ok()) {
		return 0;
	    }
	    sleep(1);
	}
	marker_pub.publish(marker);
	r.sleep();
    }
}
