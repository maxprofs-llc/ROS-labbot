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
  void Callback1(const sensor_msgs::Joy::ConstPtr& joy);
  void Callback2(const nav_msgs::Odometry::ConstPtr& odom);
  ros::NodeHandle n;
  ros::Publisher vel_pub;
  ros::Publisher xyteta_pub;
  ros::Publisher zmiana_pub;
  ros::Subscriber joy_sub;
  ros::Subscriber odom_sub;
  ros::Time current_time, last_time;
  int etap;
  float x, y;
  float czas;
  double teta, teta2, deltateta;
  geometry_msgs::Twist pyk;
  geometry_msgs::Vector3 pstryk;
  geometry_msgs::Vector3 plum;
};

Test::Test()
{
  etap=0;
  teta=0;
  teta2=0;
  x=0;
  y=0;
  vel_pub = n.advertise<geometry_msgs::Twist>("Twist", 50);
  xyteta_pub = n.advertise<geometry_msgs::Vector3>("polozenie", 50);
  zmiana_pub = n.advertise<geometry_msgs::Vector3>("roznica", 50);
  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 50, &Test::Callback1, this);
  odom_sub = n.subscribe<nav_msgs::Odometry>("labbot_odometry", 50, &Test::Callback2, this);
  czas=0;	//variable czas (eng. time)
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate r(100);	//loop is set to run at 100Hz
  while(n.ok()){
    ros::spinOnce(); 
    
    //starting test for the right ride
    if(etap==1)
    {
      pyk.angular.z=0.15;
      if(teta>=89.5)
        {
	  etap=101;
	  pyk.angular.z=0;
	  czas=0;
	}
    }
    if(etap==101)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=2;}
    }
    if(etap==2)
    {
      pyk.linear.x=0.15;//2*(4-y);
      if(teta<89)
      {
        pyk.angular.z=0.03;
      }
      if(teta>91)
      {
        pyk.angular.z=-0.03;
      }
      if(teta>=89 && teta<=91)
      {
        pyk.angular.z=0;
      }
      if(y>=0.998)
        {
	  etap=201;
	  pyk.linear.x=0;
          czas=0;
	}
    }
    if(etap==201)
    {
      pyk.linear.x=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=3;}
    }
    if(etap==3)
    {
      if(teta<0)
        teta+=360;
      pyk.angular.z=0.15;
      if(teta>=179.5)
        {
	  etap=301;
	  pyk.angular.z=0;
          czas=0;
	}
    }
    if(etap==301)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=4;}
    }
    if(etap==4)
    {
      pyk.linear.x=0.15;//2*(4+x);
      if(teta<0)
        teta+=360;
      if(teta<179)
      {
        pyk.angular.z=0.03;
      }
      if(teta>181)
      {
        pyk.angular.z=-0.03;
      }
      if(teta>=179 && teta<=181)
      {
        pyk.angular.z=0;
      }
      if(x<=-0.998)
        {
	  etap=401;
	  pyk.linear.x=0;
          czas=0;
	}
    }
    if(etap==401)
    {
      pyk.linear.x=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=5;}
    } 
    if(etap==5)
    {
      if(teta>0)
         teta-=360;
      pyk.angular.z=0.15;
      if(teta>=-89.5)
        {
	  etap=501;
	  pyk.angular.z=0;
          czas=0;
	}
    }
    if(etap==501)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=6;}
    }
    if(etap==6)
    {
      pyk.linear.x=0.15;//2*(y);
      if(teta<-91)
      {
        pyk.angular.z=0.03;
      }
      if(teta>-89)
      {
        pyk.angular.z=-0.03;
      }
      if(teta>=-91 && teta<=-89)
      {
        pyk.angular.z=0;
      }
      if(y<=0.002)
        {
	  etap=601;
	  pyk.linear.x=0;
          czas=0;
	}
    }
    if(etap==601)
    {
      pyk.linear.x=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=7;}
    }
    if(etap==7)
    {
      pyk.angular.z=0.15;
      if(teta>-0.5)
        {
	  etap=701;
	  pyk.angular.z=0;
          czas=0;
	}
    }
    if(etap==701)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=8;}
    }
    if(etap==8)
    {
      pyk.linear.x=0.15;//2*(-x);
      if(teta<-1)
      {
        pyk.angular.z=0.03;
      }
      if(teta>1)
      {
        pyk.angular.z=-0.03;
      }
      if(teta>=-1 && teta<=1)
      {
        pyk.angular.z=0;
      }
      if(x>=-0.002)
        {
	  etap=801;
	  pyk.linear.x=0;
	  czas=0;
	}
    }
    if(etap==801)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=9;}
    }

    
    //starting ride for the left side
    if(etap==-1)
    {
      pyk.angular.z=-0.15;
      if(teta<=-89.5)
        {
	  etap=-101;
	  pyk.angular.z=0;
	  czas=0;
	}
    }
    if(etap==-101)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-2;}
    }
    if(etap==-2)
    {
      pyk.linear.x=0.15;//2*(4-y);
      if(teta>-89)
      {
        pyk.angular.z=-0.03;
      }
      if(teta<-91)
      {
        pyk.angular.z=0.03;
      }
      if(teta<=-89 && teta>=-91)
      {
        pyk.angular.z=0;
      }
      if(y<=-0.998)
        {
	  etap=-201;
	  pyk.linear.x=0;
          czas=0;
	}
    }
    if(etap==-201)
    {
      pyk.linear.x=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-3;}
    }
    if(etap==-3)
    {
      if(teta>0)
        teta-=360;
      pyk.angular.z=-0.15;
      if(teta<=-179.5)
        {
	  etap=-301;
	  pyk.angular.z=0;
          czas=0;
	}
    }
    if(etap==-301)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-4;}
    }
    if(etap==-4)
    {
      pyk.linear.x=0.15;//2*(4+x);
      if(teta>0)
        teta-=360;
      if(teta>-179)
      {
        pyk.angular.z=-0.03;
      }
      if(teta<-181)
      {
        pyk.angular.z=0.03;
      }
      if(teta<=-179 && teta>=-181)
      {
        pyk.angular.z=0;
      }
      if(x<=-0.998)
        {
	  etap=-401;
	  pyk.linear.x=0;
          czas=0;
	}
    }
    if(etap==-401)
    {
      pyk.linear.x=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-5;}
    } 
    if(etap==-5)
    {
      if(teta<0)
         teta+=360;
      pyk.angular.z=-0.15;
      if(teta<=90.5)
        {
	  etap=-501;
	  pyk.angular.z=0;
          czas=0;
	}
    }
    if(etap==-501)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-6;}
    }
    if(etap==-6)
    {
      pyk.linear.x=0.15;//2*(y);
      if(teta>91)
      {
        pyk.angular.z=-0.03;
      }
      if(teta<89)
      {
        pyk.angular.z=0.03;
      }
      if(teta<=91 && teta>=89)
      {
        pyk.angular.z=0;
      }
      if(y>=-0.002)
        {
	  etap=-601;
	  pyk.linear.x=0;
          czas=0;
	}
    }
    if(etap==-601)
    {
      pyk.linear.x=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-7;}
    }
    if(etap==-7)
    {
      pyk.angular.z=-0.15;
      if(teta<0.5)
        {
	  etap=-701;
	  pyk.angular.z=0;
          czas=0;
	}
    }
    if(etap==-701)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-8;}
    }
    if(etap==-8)
    {
      pyk.linear.x=0.15;//2*(-x);
      if(teta>1)
      {
        pyk.angular.z=-0.03;
      }
      if(teta<-1)
      {
        pyk.angular.z=0.03;
      }
      if(teta<=1 && teta>=-1)
      {
        pyk.angular.z=0;
      }
      if(x>=-0.002)
        {
	  etap=-801;
	  pyk.linear.x=0;
	  czas=0;
	}
    }
    if(etap==-801)
    {
      pyk.angular.z=0;
      current_time = ros::Time::now();
      czas+=(current_time - last_time).toSec();	//czas is counting the time as the program run, it is valued in seconds
      if(czas>2)
        {etap=-9;}
    }

    //starting forward drive test
    if(etap==50)
    {
      pyk.linear.x=0.15;//2*(4-y);
      if(teta<-1)
      {
        pyk.angular.z=0.03;
      }
      if(teta>1)
      {
        pyk.angular.z=-0.03;
      }
      if(teta>=-1 && teta<=1)
      {
        pyk.angular.z=0;
      }
      if(x>=5)
        {
	  etap=1111;
	  pyk.linear.x=0;
          czas=0;
	}
    }
	
	
	//stoping test
	if(etap==1111)
	{
		pyk.linear.x=0;
		pyk.linear.z=0;
	}


    /*if(pyk.linear.x>1)
       pyk.linear.x=1;
    if(pyk.linear.y>1)
       pyk.linear.y=1;*/

/*if(teta>80 && teta<100)
{
	ROS_DEBUG("%f", teta);
}*/

    vel_pub.publish(pyk);
    pstryk.x=x;
    pstryk.y=y;
    pstryk.z=teta;
    xyteta_pub.publish(pstryk);
    last_time = ros::Time::now();
    r.sleep();
  }
}

void Test::Callback1(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist pyk;
  if(joy->buttons[1]==1)  //wciśnięcie przycisku B rozpoczyna test w prawą stronę
    etap=-1;
  if(joy->buttons[2]==1)  //wciśnięcie przycisku X rozpoczyna test w lewą stronę
    etap=1;
  if(joy->buttons[3]==1)  //wciśnięcie przycisku Y rozpoczyna test jazdy na wprost 5m
    etap=50;
  if(joy->buttons[0]==1)  //wciśnięcie przycisku A zatrzymuje działanie testu
    etap=1111;

}

void Test::Callback2(const nav_msgs::Odometry::ConstPtr& odom)
{
  double th;
  x=odom->pose.pose.position.x;  //odczytujemy położenie w osi x
  y=odom->pose.pose.position.y;  //odczytujemy położenie w osi y
  th = tf::getYaw(odom->pose.pose.orientation);  //transformacja kwaternionu na wartości kątowe za pomocą funkcji z biblioteki pakietu TF 
  teta=th*180/3.141592654;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_odometry_node");
  Test test;
}












