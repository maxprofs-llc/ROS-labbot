// based on https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers_tutorials/trunk/turtle_teleop/src/teleop_turtle_joy.cpp
// from tutorial http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <sensor_msgs/Joy.h>
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
class LabbotTeleoperation
{
public:
  LabbotTeleoperation()
  {
    angular = 4;
    linear = 1;

    motorRightScale = 80;
    motorLeftScale = 80;

  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
// %EndTag(PARAMS)%
// %Tag(PUB)%
  msgToLabbotPublisher = nh.advertise<labbot::msgToLabbot>("toLabbot", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
  gamepadSubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &LabbotTeleoperation::gamepadCallback, this);
// %EndTag(SUB)%
  }

private:
  void gamepadCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh;

  int linear, angular;
  float motorRightScale, motorLeftScale;
  ros::Publisher msgToLabbotPublisher;
  ros::Subscriber gamepadSubscriber;
  
};
// %EndTag(CLASSDEF)%

// %Tag(CALLBACK)%
void LabbotTeleoperation::gamepadCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  labbot::msgToLabbot msg;
  msg.motorRightSpeed = motorRightScale * joy->axes[angular];
  msg.motorLeftSpeed = motorLeftScale * joy->axes[linear];
  msgToLabbotPublisher.publish(msg);
}
// %EndTag(CALLBACK)%
// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Labbot_teleopeartor");
  LabbotTeleoperation teleoperator;

  ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%

