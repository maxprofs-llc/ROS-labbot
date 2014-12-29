// based on https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers_tutorials/trunk/turtle_teleop/src/teleop_turtle_joy.cpp
// from tutorial http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class LabbotTeleoperation
{
	public:
	LabbotTeleoperation()
	{

		//gamepadStickRightX = 3;
		//gamepadStickRightY = 4;

		motorRightScale = 40;
		motorLeftScale = 40;

		msgToLabbotPublisher = nh.advertise<labbot::msgToLabbot>("toLabbot", 1);

		gamepadSubscriber = nh.subscribe<geometry_msgs::Twist>("Twist", 10, &LabbotTeleoperation::gamepadCallback, this);
	}

	private:
	void gamepadCallback(const geometry_msgs::Twist::ConstPtr& twist);

	ros::NodeHandle nh;

	//int gamepadStickRightX, gamepadStickRightY;
	float motorRightScale, motorLeftScale;
	ros::Publisher msgToLabbotPublisher;
	ros::Subscriber gamepadSubscriber;
  
};

void LabbotTeleoperation::gamepadCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	
	//msg.motorRightSpeed = motorRightScale * joy->axes[gamepadStcikRightY];
	//msg.motorLeftSpeed = motorLeftScale * joy->axes[gamepadStcikLeftY];

	float motorRightSpeed = 0;
	float motorLeftSpeed = 0;

	// get the data from gamepad
	float x = twist->linear.x;
	float y = twist->angular.z;

	// calculate angle and size of vector
	float angle = atan2(y, x);
	float power = sqrt(x*x + y*y);

	// set the speed
	motorRightSpeed = x - y;
	motorLeftSpeed = x + y;

	// scale the output
	motorRightSpeed = motorRightSpeed * motorRightScale;
	motorLeftSpeed = motorLeftSpeed * motorLeftScale;

	if(motorRightSpeed > motorRightScale)
	{
		motorRightSpeed = motorRightScale;
	}

	if(motorLeftSpeed > motorLeftScale)
	{
		motorLeftSpeed = motorLeftScale;
	}

	// fill the msg
	labbot::msgToLabbot msg;
	msg.motorRightSpeed = motorRightSpeed;
	msg.motorLeftSpeed = motorLeftSpeed;
	// publish the message
	msgToLabbotPublisher.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Labbot_teleopeartor");
	LabbotTeleoperation teleoperator;

	ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%

