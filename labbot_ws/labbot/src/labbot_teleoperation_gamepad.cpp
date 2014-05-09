// based on https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers_tutorials/trunk/turtle_teleop/src/teleop_turtle_joy.cpp
// from tutorial http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <sensor_msgs/Joy.h>

class LabbotTeleoperation
{
	public:
	LabbotTeleoperation()
	{
		gamepadStcikRightY = 1;
		gamepadStcikLeftY = 4;

		motorRightScale = 40;
		motorLeftScale = 40;

		//nh.param("axis_linear", linear, linear);
		//nh.param("axis_angular", angular, angular);

		msgToLabbotPublisher = nh.advertise<labbot::msgToLabbot>("toLabbot", 1);

		gamepadSubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &LabbotTeleoperation::gamepadCallback, this);
	}

	private:
	void gamepadCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh;

	int gamepadStcikLeftY, gamepadStcikRightY;
	float motorRightScale, motorLeftScale;
	ros::Publisher msgToLabbotPublisher;
	ros::Subscriber gamepadSubscriber;
  
};

void LabbotTeleoperation::gamepadCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	labbot::msgToLabbot msg;
	msg.motorRightSpeed = motorRightScale * joy->axes[gamepadStcikRightY];
	msg.motorLeftSpeed = motorLeftScale * joy->axes[gamepadStcikLeftY];
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

