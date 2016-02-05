// code based on the turtle teleop tutorial available at:
// http://wiki.ros.org/joy/Tutorials/WritingTeleopNode
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Teleop
{
	public:
		Teleop();
	private:
		void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy);
		
		ros::NodeHandle nh;
		ros::Publisher velocityPublisher;
		ros::Subscriber joySubscriber;
};

Teleop::Teleop()
{
	this->velocityPublisher = this->nh.advertise<geometry_msgs::Twist>("cmd_joy", 10);
	this->joySubscriber = this->nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::CallbackJoy, this);
}

void Teleop::CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist velocities;	
	
	velocities.linear.x=joy->axes[4];	// move forward or backward - Up/Down Axis stick right <1;-1> 
	velocities.angular.z=joy->axes[0];	// turn - Left/Right Axis stick left <1;-1>
	
	this->velocityPublisher.publish(velocities);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_joy_node");
	Teleop teleop;
	ros::spin();
}
