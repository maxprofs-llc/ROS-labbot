#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

using namespace message_filters;

void synchronizedCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::LaserScanConstPtr& laser)
{
  ROS_INFO("Messages synchronized!");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_synchronizer_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odometry_publisher", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);

  typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, scan_sub);
  sync.registerCallback(boost::bind(&synchronizedCallback, _1, _2));

  ros::spin();

  return 0;
}
