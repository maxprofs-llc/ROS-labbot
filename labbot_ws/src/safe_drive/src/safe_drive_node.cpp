#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Safety
{
  public:
    Safety();
  private:
    void Callback(const geometry_msgs::Twist::ConstPtr& move);
    ros::NodeHandle n;
    ros::Publisher stop_pub;
    ros::Subscriber twist_sub;
    ros::Time current_time, last_time;
    float czas;
};

Safety::Safety()
{
  
  stop_pub = n.advertise<geometry_msgs::Twist>("Twist", 1);	//declaration of publisher on topic Twist
  geometry_msgs::Twist stop;

  czas=0;	//variable czas (eng. time)
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10);    //loop is rated at 10Hz
  while(n.ok())
  {
    ros::spinOnce(); 
    
    current_time = ros::Time::now();
    czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds

    twist_sub = n.subscribe<geometry_msgs::Twist>("Twist", 1, &Safety::Callback, this);	//declaration of subscribing the Twist topic 
    
    if(czas>5)	//if there were no messages from Twist in last five seconds
    {
      stop_pub.publish(stop);	//send a message to stop
    }

    last_time = ros::Time::now();
    r.sleep();
  }
}

void Safety::Callback(const geometry_msgs::Twist::ConstPtr& move)
{
   czas=0;	//when a massege received czas is set to zero, and time is counted again
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safe_drive_node");
  Safety safety;
}
