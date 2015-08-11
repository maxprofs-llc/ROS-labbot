#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Mux
{
  public:
    Mux();
  private:
    void Callback1(const geometry_msgs::Twist::ConstPtr& joy);
    void Callback2(const geometry_msgs::Twist::ConstPtr& key);
    void Callback3(const geometry_msgs::Twist::ConstPtr& test);
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber test_sub;
    ros::Time current_time, last_time;
    geometry_msgs::Twist vel_msg;
    int priorytet;
    float czas;
};

Mux::Mux()
{
  czas=0;	//variable czas (ang. time) counts the time
  priorytet=0;	//variable priorytet (ang. priority) holds the priority of current commandline
  vel_pub = n.advertise<geometry_msgs::Twist>("Twist", 1);	//chosen commandline is send to topic Twist
  joy_sub = n.subscribe<geometry_msgs::Twist>("cmd_joy", 10, &Mux::Callback1, this);	//we subscribe the topic from joy
  key_sub = n.subscribe<geometry_msgs::Twist>("cmd_key", 10, &Mux::Callback2, this);	//we subscribe the topic from keyboard 
  test_sub = n.subscribe<geometry_msgs::Twist>("cmd_test", 10, &Mux::Callback3, this);	//we subscribe the topic from test_odometry

  ros::Rate r(100);	//loop is set to run at 100Hz
  while(n.ok())
  {
    ros::spinOnce(); 
    
    current_time = ros::Time::now();
    czas+=(current_time - last_time).toSec();	//czas is valued in seconds
    if(czas>2)	//every two seconds (you can change it as you wish) czas is set to zero
    {
      czas=0;	//czas is set to zero and counts from start
      if(priorytet>0)	//if the priority is more than 0
        priorytet--;	//it is decremented till it's set to zero
    }

    last_time = ros::Time::now();
    r.sleep();
  }
}

void Mux::Callback1(const geometry_msgs::Twist::ConstPtr& joy)
{
  vel_msg.linear.x=joy->linear.x;
  vel_msg.angular.z=joy->angular.z;
  priorytet=2;	//priority is set to two
  czas=0;	//every time message from joy is received, czas (I remind it holds time) is set to zero, so those 2 seconds, which reset priority level will make it after those messages will stop to appear
  vel_pub.publish(vel_msg);	//we publish commands on Twist topic
}

void Mux::Callback2(const geometry_msgs::Twist::ConstPtr& key)
{
  if(priorytet==0 || priorytet==1)	//if the priority is set to zero or one, so thera are no messages from joy
  {
    priorytet=1;	//priority is set to one
    czas=0;	//same here
    vel_msg.linear.x=key->linear.x;	//same here
    vel_msg.angular.z=key->angular.z;
    vel_pub.publish(vel_msg);	//we publish command on Twist topic
  }
}

void Mux::Callback3(const geometry_msgs::Twist::ConstPtr& test)
{
  if(priorytet==0)	//if the priority is set to zero, so thera are no messages from joy or key
  {
    vel_msg.linear.x=test->linear.x;	//same here
    vel_msg.angular.z=test->angular.z;
    vel_pub.publish(vel_msg);	//we publish command on Twist topic
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_mux_node");
  Mux mux;
}
