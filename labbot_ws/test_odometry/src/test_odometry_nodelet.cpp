#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <ros/console.h>
#include <ecl/threads/thread.hpp>

namespace test_odometry
{

class test_odometry_nodelet:public nodelet::Nodelet
{
  public:
    test_odometry_nodelet(): shutdown_requested_(false) {}
    ~test_odometry_nodelet() 
    {
	NODELET_DEBUG_STREAM("Waiting for update thread to finish.");
	shutdown_requested_ = true;
	update_thread_.join();
    }

    virtual void onInit()
    {
    	ros::NodeHandle &n=getNodeHandle(); 
    	etap=0;
     	teta=0;
  	x=0;
  	y=0;
  	vel_pub = n.advertise<geometry_msgs::Twist>("Twist", 50);
  	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 50, boost::bind(&test_odometry_nodelet::CallbackJoy, this, _1));
  	odom_sub = n.subscribe<nav_msgs::Odometry>("labbot_odometry", 50, boost::bind(&test_odometry_nodelet::CallbackOdometry, this, _1));
  	czas=0;	//variable czas (eng. time)
  	test_current_time = ros::Time::now();
  	test_last_time = ros::Time::now();
	update_thread_.start(&test_odometry_nodelet::Test, *this);
    }

  private:
    void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
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

    void CallbackOdometry(const nav_msgs::Odometry::ConstPtr& odom)
    {
    	double th;
    	x=odom->pose.pose.position.x;  //reading position in x axis
    	y=odom->pose.pose.position.y;  //reading position in y axis
    	th = tf::getYaw(odom->pose.pose.orientation);  //transforming quaternion to radians values by using a tf library function 
    	teta=th*180/3.141592654;		//transforming to degrees value
    }

    void Test()
    {
  	geometry_msgs::TwistPtr vel (new geometry_msgs::Twist());
  	ros::Rate r(100);	//loop is set to run at 100Hz
  	while(! shutdown_requested_ && ros::ok())
  	{
    	  //starting test for the right ride
    	  if(etap==1)
    	  {
      	    vel->angular.z=0.15;
      	  if(teta>=89.5)
          {
	    etap=101;
	    vel->angular.z=0;
	    czas=0;
	  }
    	}
    	if(etap==101)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=2;}
        }
    	if(etap==2)
    	{
          vel->linear.x=0.15;
      	  if(teta<89.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta>90.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta>=89.5 && teta<=90.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(y>=2.498)
          {
	    etap=201;
	    vel->linear.x=0;
            czas=0;
	  }
      	}
    	if(etap==201)
    	{
      	  vel->linear.x=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=3;}
   	}
    	if(etap==3)
    	{
      	  if(teta<0)
          teta+=360;
      	  vel->angular.z=0.15;
      	  if(teta>=179.5)
          {
	    etap=301;
	    vel->angular.z=0;
            czas=0;
	  }
    	}
    	if(etap==301)
    	{
          vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=4;}
    	}
    	if(etap==4)
    	{
      	  vel->linear.x=0.15;
      	  if(teta<0)
            teta+=360;
      	  if(teta<179.5)
      	  {
            vel->angular.z=0.03;
          }
      	  if(teta>180.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta>=179.5 && teta<=180.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(x<=-2.498)
          {
	    etap=401;
	    vel->linear.x=0;
            czas=0;
	  }
    	}
    	if(etap==401)
    	{
      	  vel->linear.x=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=5;}
    	} 
    	if(etap==5)
    	{
      	  if(teta>0)
            teta-=360;
      	  vel->angular.z=0.15;
      	  if(teta>=-89.5)
          {
	    etap=501;
	    vel->angular.z=0;
            czas=0;
	  }
     	}
    	if(etap==501)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=6;}
    	}
    	if(etap==6)
    	{
      	  vel->linear.x=0.15;
      	  if(teta<-90.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta>-89.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta>=-90.5 && teta<=-89.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(y<=0.002)
          {
	    etap=601;
	    vel->linear.x=0;
            czas=0;
	  }
    	}
    	if(etap==601)
    	{
      	  vel->linear.x=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=7;}
    	}
    	if(etap==7)
    	{
      	  vel->angular.z=0.15;
      	  if(teta>-0.5)
          {
	    etap=701;
	    vel->angular.z=0;
            czas=0;
	  }
    	}
    	if(etap==701)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=8;}
    	}
    	if(etap==8)
    	{
      	  vel->linear.x=0.15;
      	  if(teta<-0.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta>0.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta>=-0.5 && teta<=0.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(x>=-0.002)
          {
	    etap=801;
	    vel->linear.x=0;
	    czas=0;
	  }
    	}
    	if(etap==801)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=9;}
    	}

    	//starting ride for the left side
    	if(etap==-1)
    	{
      	  vel->angular.z=-0.15;
      	  if(teta<=-89.5)
          {
	    etap=-101;
	    vel->angular.z=0;
	    czas=0;
	  }
    	}
    	if(etap==-101)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-2;}
    	}
    	if(etap==-2)
    	{
      	  vel->linear.x=0.15;
      	  if(teta>-89.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta<-90.5)
      	  {
            vel->angular.z=0.03;
       	  }
      	  if(teta<=-89.5 && teta>=-90.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(y<=-2.498)
          {
	    etap=-201;
	    vel->linear.x=0;
            czas=0;
	  }
    	}
    	if(etap==-201)
    	{
      	  vel->linear.x=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-3;}
    	}
    	if(etap==-3)
    	{
      	  if(teta>0)
          teta-=360;
      	  vel->angular.z=-0.15;
      	  if(teta<=-179.5)
          {
	    etap=-301;
	    vel->angular.z=0;
            czas=0;
	  }
    	}
    	if(etap==-301)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-4;}
    	}
    	if(etap==-4)
    	{
      	  vel->linear.x=0.15;
      	  if(teta>0)
            teta-=360;
      	  if(teta>-179.5)
      	  {
            vel->angular.z=-0.03;
          }
      	  if(teta<-180.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta<=-179.5 && teta>=-180.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(x<=-2.498)
          {
	    etap=-401;
	    vel->linear.x=0;
            czas=0;
	  }
    	}
    	if(etap==-401)
    	{
      	  vel->linear.x=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-5;}
    	} 
    	if(etap==-5)
    	{
      	  if(teta<0)
            teta+=360;
      	  vel->angular.z=-0.15;
      	  if(teta<=90.5)
          {
	    etap=-501;
	    vel->angular.z=0;
            czas=0;
	  }
    	}
    	if(etap==-501)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-6;}
    	}
   	if(etap==-6)
    	{
      	  vel->linear.x=0.15;
      	  if(teta>90.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta<89.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta<=90.5 && teta>=89.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(y>=-0.002)
          {
	    etap=-601;
	    vel->linear.x=0;
            czas=0;
	  }
    	}
    	if(etap==-601)
    	{
      	  vel->linear.x=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-7;}
    	}
    	if(etap==-7)
    	{
      	  vel->angular.z=-0.15;
      	  if(teta<0.5)
          {
	    etap=-701;
	    vel->angular.z=0;
            czas=0;
	  }
    	}
    	if(etap==-701)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-8;}
    	}
    	if(etap==-8)
    	{
      	  vel->linear.x=0.15;
      	  if(teta>0.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta<-0.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta<=0.5 && teta>=-0.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(x>=-0.002)
          {
	    etap=-801;
	    vel->linear.x=0;
	    czas=0;
	  }
    	}
    	if(etap==-801)
    	{
      	  vel->angular.z=0;
      	  test_current_time = ros::Time::now();
      	  czas+=(test_current_time - test_last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      	  if(czas>2)
            {etap=-9;}
    	}

    	//starting forward drive test
    	if(etap==50)
    	{
      	  vel->linear.x=0.15;
      	  if(teta<-0.5)
      	  {
            vel->angular.z=0.03;
      	  }
      	  if(teta>0.5)
      	  {
            vel->angular.z=-0.03;
      	  }
      	  if(teta>=-0.5 && teta<=0.5)
      	  {
            vel->angular.z=0;
      	  }
      	  if(x>=5)
          {
	    etap=1111;
	    vel->linear.x=0;
            czas=0;
	  }
    	}
	
	//stoping test
	if(etap==1111)
	{
	  vel->linear.x=0;
	  vel->linear.z=0;
	}

    	vel_pub.publish(vel);
    	test_last_time = ros::Time::now();
    	r.sleep();
    }
  }

  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;
  ros::Subscriber odom_sub;
  ros::Time test_current_time, test_last_time;
  int etap;
  float x, y;
  float czas;
  double teta;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

}
PLUGINLIB_EXPORT_CLASS(test_odometry::test_odometry_nodelet, nodelet::Nodelet);
