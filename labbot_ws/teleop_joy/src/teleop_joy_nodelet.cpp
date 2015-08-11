#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace teleop_joy
{

class teleop_joy_nodelet : public nodelet::Nodelet
{
  public:
    teleop_joy_nodelet() {};
    ~teleop_joy_nodelet() {};
    virtual void onInit()
    {
	ros::NodeHandle &n=getNodeHandle();
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_joy", 1);
  	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &teleop_joy_nodelet::Callback, this);
    }

  private:
    void Callback(const sensor_msgs::Joy::ConstPtr& joy)
    {
  	geometry_msgs::TwistPtr vel(new geometry_msgs::Twist());	//przód-tył prawy joy(1;-1) 
  	vel->linear.x=joy->axes[4];
  	vel->angular.z=joy->axes[0];	//obrót (1;-1)
  	vel_pub.publish(vel);
    }

    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
};

}
PLUGINLIB_EXPORT_CLASS(teleop_joy::teleop_joy_nodelet, nodelet::Nodelet);
