#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Mux
{
  public:
    Mux();
  private:
    void CallbackJoy(const geometry_msgs::Twist::ConstPtr& joy);
    void CallbackKeyboard(const geometry_msgs::Twist::ConstPtr& key);
    void CallbackOther(const geometry_msgs::Twist::ConstPtr& other);
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber other_sub;
    ros::Time current_time, last_time;
    geometry_msgs::Twist pyk1;
    int priorytet;
    float czas;//, lastlin, lastang;
};

Mux::Mux()
{
  //lastlin=0;	//holds last value of linear comand
  //lastang=0;	//holds last value of angular comand
  czas=0;	//variable czas (ang. time) counts the time
  priorytet=0;	//variable priorytet (ang. priority) holds the priority of current commandline
  vel_pub = n.advertise<geometry_msgs::Twist>("Twist", 1);	//chosen commandline is send to topic Twist
  
  joy_sub = n.subscribe<geometry_msgs::Twist>("cmd_joy", 10, &Mux::CallbackJoy, this);	//we subscribe the topic from joy
  key_sub = n.subscribe<geometry_msgs::Twist>("cmd_key", 10, &Mux::CallbackKeyboard, this);	//we subscribe the topic from keyboard 
  other_sub = n.subscribe<geometry_msgs::Twist>("cmd_other", 10, &Mux::CallbackOther, this);	//we subscribe the topic from some other source

  ros::Rate r(100);	//loop is set to run at 100Hz
  while(n.ok()){
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

void Mux::CallbackJoy(const geometry_msgs::Twist::ConstPtr& joy)
{
  pyk1.linear.x=joy->linear.x;//*0.6321+0.3679*lastlin;	//comands are smoothed
  pyk1.angular.z=joy->angular.z;//*0.6321+0.3679*lastang;
  priorytet=2;	//priority is set to two
  czas=0;	//every time message from joy is received, czas (I remind it holds time) is set to zero, so those 2 seconds, which reset priority level will make it after those messages will stop to appear
  //lastlin=pyk1.linear.x;
  //lastang=pyk1.angular.z;
  vel_pub.publish(pyk1);	//we publish commands on Twist topic
}

void Mux::CallbackKeyboard(const geometry_msgs::Twist::ConstPtr& key)
{
  if(priorytet==0 || priorytet==1)	//if the priority is set to zero or one, so thera are no messages from joy
  {
    priorytet=1;	//priority is set to one
    czas=0;	//same here
    pyk1.linear.x=key->linear.x;//*0.6321+0.3679*lastlin;	//same here
    pyk1.angular.z=key->angular.z;//*0.6321+0.3679*lastang;
    //lastlin=pyk2.linear.x;
    //lastang=pyk2.angular.z;
    vel_pub.publish(pyk1);	//we publish command on Twist topic
  }
}

void Mux::CallbackOther(const geometry_msgs::Twist::ConstPtr& other)
{
  if(priorytet==0)	//if the priority is set to zero, so thera are no messages from joy or key
  {
    pyk1.linear.x=other->linear.x;//*0.6321+0.3679*lastlin;	//same here
    pyk1.angular.z=other->angular.z;//*0.6321+0.3679*lastang;
    //lastlin=pyk2.linear.x;
    //lastang=pyk2.angular.z;
    vel_pub.publish(pyk1);	//we publish command on Twist topic
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_mux_node");
  Mux mux;
}
