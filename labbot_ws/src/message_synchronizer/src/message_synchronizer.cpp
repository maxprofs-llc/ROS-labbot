#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

void hokuyoCallback(const sensor_msgs::LaserScan& laser);
void odometryCallback(const nav_msgs::Odometry& odom);
void synchronizedCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::LaserScanConstPtr& laser);

class MessageSynchronizer
{
public:
   /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle* n;
  ros::Publisher pub_synchronized;
  ros::Subscriber sub_odometry;
  ros::Subscriber sub_hokuyo;

  typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
  Synchronizer<MySyncPolicy> * ptrSync;

  message_filters::Subscriber<nav_msgs::Odometry>* message_filter_odometry;
  message_filters::Subscriber<sensor_msgs::LaserScan>* message_filter_hokuyo;

  MessageSynchronizer()
  {
  }

  void init()
  {
    n = new ros::NodeHandle();
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    pub_synchronized = n->advertise<std_msgs::String>("messages_synchronized", 1000);

    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called chatterCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    //sub_odometry = n.subscribe("/odometry_publisher", 10, odometryCallback);
    //sub_hokuyo = n.subscribe("/scan", 10, hokuyoCallback);

    //message_filters::Subscriber<nav_msgs::Odometry> message_filter_odometry(n, "/odometry_publisher", 1);
    //message_filters::Subscriber<sensor_msgs::LaserScan> message_filter_hokuyo(n, "/scan", 1);

    message_filter_odometry = new message_filters::Subscriber<nav_msgs::Odometry>(*n, "/odometry_publisher", 1);
    message_filter_hokuyo = new message_filters::Subscriber<sensor_msgs::LaserScan>(*n, "/scan", 1);
    
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), message_filter_odometry, message_filter_hokuyo);
    //sync.registerCallback(boost::bind(&synchronizedCallback, _1, _2));
    //ptrSync = &sync;
    ptrSync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *message_filter_odometry, *message_filter_hokuyo);
    ptrSync->registerCallback(boost::bind(&synchronizedCallback, _1, _2));   
  }
};

MessageSynchronizer ms;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "messages_synchronizer");

  ms.init();

  ros::spin();
/*
  ros::Rate loop_rate(10); 

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
*/


  return 0;
}

void hokuyoCallback(const sensor_msgs::LaserScan& laser)
{
  ROS_INFO("Hokuyo reporting! Size[%ld]", laser.ranges.size());
}

void odometryCallback(const nav_msgs::Odometry& odom)
{
  ROS_INFO("Odometry received! Position x: [%f]", odom.pose.pose.position.x);
}

void synchronizedCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::LaserScanConstPtr& laser)
{
  ROS_INFO("Messages synchronized! Hokuyo size[%ld], odometry position x: [%f]", laser->ranges.size(), odom->pose.pose.position.x);
  
  std_msgs::String msg;

  std::stringstream ss;
  ss << "It works!";
  msg.data = ss.str();

  ms.pub_synchronized.publish(msg);
}
