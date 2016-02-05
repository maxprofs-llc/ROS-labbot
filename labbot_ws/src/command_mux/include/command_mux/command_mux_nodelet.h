#ifdef COMMAND_MUX_H
#define COMMAND_MUX_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace command_mux_nodelet
{

class Mux : public nodelet::Nodelet
{
  public:
    Mux();
    ~Mux();
    void onInit();
  private:
    void Spin();
    void Callback1(const geometry_msgs::Twist::ConstPtr& joy);
    void Callback2(const geometry_msgs::Twist::ConstPtr& key);
    void Callback3(const geometry_msgs::Twist::ConstPtr& test);
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber test_sub;
    ros::Time current_time, last_time;
    geometry_msgs::Twist pyk1;
    int priorytet;
    float czas;//, lastlin, lastang;
};
}
