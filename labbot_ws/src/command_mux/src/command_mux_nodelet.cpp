#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>

namespace command_mux
{

class command_mux_nodelet : public nodelet::Nodelet
{
public:
    command_mux_nodelet() : shutdown_requested_(false) { };
    ~command_mux_nodelet()
    {
        NODELET_DEBUG_STREAM("Waiting for update thread to finish.");
        shutdown_requested_ = true;
        update_thread_->join();
    }

    virtual void onInit()
    {
        ros::NodeHandle &n=getNodeHandle();
        czas=0;	//variable czas (ang. time) counts the time
        priorytet=0;	//variable priorytet (ang. priority) holds the priority of current commandline
        vel_pub = n.advertise<geometry_msgs::Twist>("Twist", 1);	//chosen commandline is send to topic Twist
        joy_sub = n.subscribe<geometry_msgs::Twist>("cmd_joy", 10, boost::bind(&command_mux_nodelet::Callback_joy, this, _1));	//we subscribe the topic from joy
        key_sub = n.subscribe<geometry_msgs::Twist>("cmd_key", 10, boost::bind(&command_mux_nodelet::Callback_key, this, _1));	//we subscribe the topic from keyboard
        other_sub = n.subscribe<geometry_msgs::Twist>("cmd_test", 10, boost::bind(&command_mux_nodelet::Callback_other, this, _1));	//we subscribe the topic from test_odometry
        update_thread_.reset(new std::thread(&command_mux_nodelet::Update, this));	//running Update in new thread
    }

private:
    void Update()
    {
        ros::Rate r(100);	//loop is set to run at 100Hz
        while (!shutdown_requested_ && ros::ok())
        {
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
    
    void Callback_joy(const geometry_msgs::Twist::ConstPtr& joy)
    {
        geometry_msgs::TwistPtr vel_msg(new geometry_msgs::Twist());
        vel_msg->linear.x=joy->linear.x;
        vel_msg->angular.z=joy->angular.z;
        priorytet=2;	//priority is set to two
        czas=0;	//every time message from joy is received, czas (I remind it holds time) is set to zero, so those 2 seconds, which reset priority level will make it after those messages will stop to appear
        vel_pub.publish(vel_msg);	//we publish commands on Twist topic
    }
    
    void Callback_key(const geometry_msgs::Twist::ConstPtr& key)
    {
        geometry_msgs::TwistPtr vel_msg(new geometry_msgs::Twist());
        if(priorytet==0 || priorytet==1)	//if the priority is set to zero or one, so thera are no messages from joy
        {
            priorytet=1;	//priority is set to one
            czas=0;	//same here
            vel_msg->linear.x=key->linear.x;
            vel_msg->angular.z=key->angular.z;
            vel_pub.publish(vel_msg);	//we publish command on Twist topic
        }
    }
    
    void Callback_other(const geometry_msgs::Twist::ConstPtr& other)
    {
        geometry_msgs::TwistPtr vel_msg(new geometry_msgs::Twist());
        if(priorytet==0)	//if the priority is set to zero, so thera are no messages from joy or key
        {
            vel_msg->linear.x=other->linear.x;
            vel_msg->angular.z=other->angular.z;
            vel_pub.publish(vel_msg);	//we publish command on Twist topic
        }
    }
    
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber other_sub;
    ros::Time current_time, last_time;
    int priorytet;
    float czas;
    std::unique_ptr<std::thread> update_thread_;
    bool shutdown_requested_;
};

}
PLUGINLIB_EXPORT_CLASS(command_mux::command_mux_nodelet, nodelet::Nodelet);
