#include <ros/ros.h>
#include <labbot/msgFromLabbot.h>
#include <std_msgs/Float32.h>

class Jazda
{
  public:
    Jazda();
  private:
    void Callback(const labbot::msgFromLabbot::ConstPtr& msgFromLabbot);
    ros::NodeHandle n;
    float lticks, rticks;
    ros::Publisher tr_pub;
    ros::Publisher tl_pub;
    ros::Subscriber t_sub;
};

Jazda::Jazda()
{
  lticks=0;
  rticks=0;
  std_msgs::Float32 left;
  std_msgs::Float32 right;
  tr_pub = n.advertise<std_msgs::Float32>("rtiki", 1);
  tl_pub = n.advertise<std_msgs::Float32>("ltiki", 1);

  ros::Rate r(10.0);  //pętla wykonuję się w częstotliwości 10Hz
  while(n.ok()){
    ros::spinOnce();
    t_sub = n.subscribe<labbot::msgFromLabbot>("fromLabbot", 50, &Jazda::Callback);
    left.data=lticks;
    right.data=rticks;
    tr_pub.publish(right);
    tl_pub.publish(left);
    r.sleep();
  }
}

void Jazda::Callback(const labbot::msgFromLabbot::ConstPtr& msgFromLabbot)
{
  lticks += msgFromLabbot->motorLeftInput;
  rticks += msgFromLabbot->motorRightInput;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiki");
  Jazda jazda;
}
