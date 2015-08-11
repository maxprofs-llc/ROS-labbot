//based on turtle teleop tutorial:http://wiki.ros.org/joy/Tutorials/WritingTeleopNode
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Teleop
{
  public:
    Teleop();
  private:
    void Callback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
};

Teleop::Teleop()
{
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_joy", 1);
  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::Callback, this);
}

void Teleop::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;	
  vel.linear.x=joy->axes[4];	//move forward or backward - right joystick <1;-1> 
  vel.angular.z=joy->axes[0];	//turn - left joystick <1;-1>
  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy_node");
  Teleop teleop;
  ros::spin();
}
