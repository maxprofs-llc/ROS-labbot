#include <ros/ros.h>
#include <labbot/msgToLabbot.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class LabbotTeleoperation
{
  public:
    LabbotTeleoperation()
    {
    	motorRightScale = 40;
    	motorLeftScale = 40;

    	msgToLabbotPublisher = nh.advertise<labbot::msgToLabbot>("toLabbot", 1);

    	gamepadSubscriber = nh.subscribe<geometry_msgs::Twist>("Twist", 10, &LabbotTeleoperation::gamepadCallback, this);
    }

  private:
    void gamepadCallback(const geometry_msgs::Twist::ConstPtr& twist);

    ros::NodeHandle nh;

    float motorRightScale, motorLeftScale;
    ros::Publisher msgToLabbotPublisher;
    ros::Subscriber gamepadSubscriber;
};

void LabbotTeleoperation::gamepadCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  float motorRightSpeed = 0;
  float motorLeftSpeed = 0;

  // get the data from gamepad
  float x = twist->linear.x;
  float y = twist->angular.z;

  // set the speed
  motorRightSpeed = x - y;
  motorLeftSpeed = x + y;

  // scale the output
  motorRightSpeed = motorRightSpeed * motorRightScale;
  motorLeftSpeed = motorLeftSpeed * motorLeftScale;

  //check if over scale
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
