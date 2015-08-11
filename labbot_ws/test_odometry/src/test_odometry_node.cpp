#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <ros/console.h>

class Test
{
  public:
    Test();
  private:
    void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy);
    void CallbackOdometry(const nav_msgs::Odometry::ConstPtr& odom);
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber odom_sub;
    ros::Time current_time, last_time;
    int etap;
    float x, y;
    float czas;
    double teta;
    geometry_msgs::Twist vel;
};

Test::Test()
{
  etap=0;
  teta=0;
  x=0;
  y=0;
  vel_pub = n.advertise<geometry_msgs::Twist>("Twist", 50);
  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 50, &Test::CallbackJoy, this);
  odom_sub = n.subscribe<nav_msgs::Odometry>("labbot_odometry", 50, &Test::CallbackOdometry, this);
  czas=0;	//variable czas (eng. time)
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(100);	//loop is set to run at 100Hz
  while(n.ok())
  {
    ros::spinOnce(); 
    
    //starting test for the right ride
    if(etap==1)
    {
	vel.angular.z=0.15;
	if(teta>=89.5)
	{
	  etap=101;
	  vel.angular.z=0;
	  czas=0;
	}
    }
    if(etap==101)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=2;
    }
    if(etap==2)
    {
      	vel.linear.x=0.15;
      	if(teta<89.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta>90.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta>=89.5 && teta<=90.5)
      	{
          vel.angular.z=0;
      	}
      	if(y>=2.498)
      	{
	  etap=201;
	  vel.linear.x=0;
          czas=0;
      	}
    }
    if(etap==201)
    {
      	vel.linear.x=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
        {etap=3;}
    }
    if(etap==3)
    {
      	if(teta<0)
          teta+=360;
      	vel.angular.z=0.15;
      	if(teta>=179.5)
        {
	  etap=301;
	  vel.angular.z=0;
          czas=0;
	}
    }
    if(etap==301)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=4;
    }
    if(etap==4)
    {
      	vel.linear.x=0.15;
      	if(teta<0)
          teta+=360;
      	if(teta<179.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta>180.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta>=179.5 && teta<=180.5)
      	{
          vel.angular.z=0;
      	}
      	if(x<=-2.498)
        {
	  etap=401;
	  vel.linear.x=0;
          czas=0;
	}
    }
    if(etap==401)
    {
      	vel.linear.x=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=5;
    } 
    if(etap==5)
    {
      	if(teta>0)
         teta-=360;
      	vel.angular.z=0.15;
      	if(teta>=-89.5)
        {
	  etap=501;
	  vel.angular.z=0;
          czas=0;
	}
    }
    if(etap==501)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=6;
    }
    if(etap==6)
    {
      	vel.linear.x=0.15;
      	if(teta<-90.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta>-89.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta>=-90.5 && teta<=-89.5)
      	{
          vel.angular.z=0;
      	}
      	if(y<=0.002)
        {
	  etap=601;
	  vel.linear.x=0;
          czas=0;
	}
    }
    if(etap==601)
    {
      	vel.linear.x=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=7;
    }
    if(etap==7)
    {
      	vel.angular.z=0.15;
      	if(teta>-0.5)
        {
	  etap=701;
	  vel.angular.z=0;
          czas=0;
	}
    }
    if(etap==701)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=8;
    }
    if(etap==8)
    {
      	vel.linear.x=0.15;
      	if(teta<-0.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta>0.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta>=-0.5 && teta<=0.5)
      	{
          vel.angular.z=0;
      	}
      	if(x>=-0.002)
        {
	  etap=801;
	  vel.linear.x=0;
	  czas=0;
	}
    }
    if(etap==801)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=9;
    }

    
    //starting ride for the left side
    if(etap==-1)
    {
      	vel.angular.z=-0.15;
      	if(teta<=-89.5)
        {
	  etap=-101;
	  vel.angular.z=0;
	  czas=0;
	}
    }
    if(etap==-101)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-2;
    }
    if(etap==-2)
    {
      	vel.linear.x=0.15;
      	if(teta>-89.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta<-90.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta<=-89.5 && teta>=-90.5)
      	{
          vel.angular.z=0;
     	}
      	if(y<=-2.498)
        {
	  etap=-201;
	  vel.linear.x=0;
          czas=0;
	}
    }
    if(etap==-201)
    {
      	vel.linear.x=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-3;
    }
    if(etap==-3)
    {
      	if(teta>0)
          teta-=360;
      	vel.angular.z=-0.15;
      	if(teta<=-179.5)
        {
	  etap=-301;
	  vel.angular.z=0;
          czas=0;
	}
    }
    if(etap==-301)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-4;
    }
    if(etap==-4)
    {
      	vel.linear.x=0.15;
      	if(teta>0)
          teta-=360;
      	if(teta>-179.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta<-180.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta<=-179.5 && teta>=-180.5)
      	{
          vel.angular.z=0;
      	}
      	if(x<=-2.498)
        {
	  etap=-401;
	  vel.linear.x=0;
          czas=0;
	}
    }
    if(etap==-401)
    {
      	vel.linear.x=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-5;
    } 
    if(etap==-5)
    {
      	if(teta<0)
         teta+=360;
      	vel.angular.z=-0.15;
      	if(teta<=90.5)
        {
	  etap=-501;
	  vel.angular.z=0;
          czas=0;
	}
    }
    if(etap==-501)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-6;
    }
    if(etap==-6)
    {
      	vel.linear.x=0.15;
      	if(teta>90.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta<89.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta<=90.5 && teta>=89.5)
      	{
          vel.angular.z=0;
      	}
      	if(y>=-0.002)
        {
	  etap=-601;
	  vel.linear.x=0;
          czas=0;
	}
    }
    if(etap==-601)
    {
      	vel.linear.x=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-7;
    }
    if(etap==-7)
    {
      	vel.angular.z=-0.15;
      	if(teta<0.5)
        {
	  etap=-701;
	  vel.angular.z=0;
          czas=0;
	}
    }
    if(etap==-701)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-8;
    }
    if(etap==-8)
    {
      	vel.linear.x=0.15;
      	if(teta>0.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta<-0.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta<=0.5 && teta>=-0.5)
      	{
          vel.angular.z=0;
      	}
      	if(x>=-0.002)
        {
	  etap=-801;
	  vel.linear.x=0;
	  czas=0;
	}
    }
    if(etap==-801)
    {
      	vel.angular.z=0;
      	current_time = ros::Time::now();
      	czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	if(czas>2)
          etap=-9;
    }

    //starting forward drive test
    if(etap==50)
    {
      	vel.linear.x=0.15;
      	if(teta<-0.5)
      	{
          vel.angular.z=0.03;
      	}
      	if(teta>0.5)
      	{
          vel.angular.z=-0.03;
      	}
      	if(teta>=-0.5 && teta<=0.5)
      	{
          vel.angular.z=0;
      	}
      	if(x>=5)
        {
	  etap=1111;
	  vel.linear.x=0;
          czas=0;
	}
    }

    //stoping test
    if(etap==1111)
    {
	vel.linear.x=0;
	vel.linear.z=0;
    }

    vel_pub.publish(vel);
    last_time = ros::Time::now();
    r.sleep();
  }
}

void Test::CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->buttons[1]==1)  //pressing B button we start the test of riding rectangle to the right
    etap=-1;
  if(joy->buttons[2]==1)  //pressing X button we start the test of riding rectangle to the left
    etap=1;
  if(joy->buttons[3]==1)  //pressing Y button we start the test of riding 5m forward
    etap=50;
  if(joy->buttons[0]==1)  //pressing A button we stop any test (safety button)
    etap=1111;
}

void Test::CallbackOdometry(const nav_msgs::Odometry::ConstPtr& odom)
{
  double th;
  x=odom->pose.pose.position.x;  //reading position in x axis
  y=odom->pose.pose.position.y;  //reading position in y axis
  th = tf::getYaw(odom->pose.pose.orientation);  //transforming quaternion to radians values by using a tf library function 
  teta=th*180/3.141592654;		//transforming to degrees value
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_odometry_node");
  Test test;
}
