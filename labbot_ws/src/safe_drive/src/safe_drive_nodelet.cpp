#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ecl/threads/thread.hpp>

namespace safe_drive
{

class safe_drive_nodelet : public nodelet::Nodelet
{
  public:
    safe_drive_nodelet() : shutdown_requested_(false) { };
    ~safe_drive_nodelet()
    {
	NODELET_DEBUG_STREAM("Waiting for update thread to finish.");
	shutdown_requested_ = true;
	update_thread_.join();
    }

    virtual void onInit()
    {
	ros::NodeHandle &n=getNodeHandle(); 
	stop_pub = n.advertise<geometry_msgs::Twist>("Twist", 1);	//declaration of publisher on topic Twist
	twist_sub = n.subscribe<geometry_msgs::Twist>("Twist", 1, boost::bind(&safe_drive_nodelet::Callback, this, _1));	//declaration of subscribing the Twist topic 
	czas=0;	//variable czas (eng. time)
	ros::Time::init();
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	update_thread_.start(&safe_drive_nodelet::Update, *this);	//running Update in new thread
    }
    
  private:
    void Update()
    {        
	geometry_msgs::TwistPtr stop(new geometry_msgs::Twist());
	stop->linear.x=0;
	stop->angular.z=0;
	ros::Rate r(10);    //loop is rated at 10Hz
	while (!shutdown_requested_ && ros::ok())
	{
   	  current_time = ros::Time::now();
    	  czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
    
     	  if(czas>2)	//if there were no messages from Twist in last five seconds
    	  {
      	    stop_pub.publish(stop);	//send a message to stop
          }

      	  last_time = ros::Time::now();
    	  r.sleep();
  	}
    }

    void Callback(const geometry_msgs::Twist::ConstPtr& move)
    {
	czas=0;	//when a massege received czas is set to zero, and time is counted again
    }

    ros::Publisher stop_pub;
    ros::Subscriber twist_sub;
    ros::Time current_time, last_time;
    float czas;
    ecl::Thread update_thread_;
    bool shutdown_requested_;
};

}
PLUGINLIB_EXPORT_CLASS(safe_drive::safe_drive_nodelet, nodelet::Nodelet);
