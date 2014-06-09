#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// for msgFromLabbot msgs
#include <labbot/msgFromLabbot.h>

// jeśli sinus
#include <math.h>

// based on http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom tutorial

double dLTicks = 0;
double dRTicks = 0;
// TODO - dodać zabezpieczenie przed przepełnieniem
double highWrap = 32767*0.7;
double lowWrap = -32767*0.7;


void msgFromLabbotCallback(const labbot::msgFromLabbot::ConstPtr& msgFromLabbot);


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Labbot_odometry");

	ros::NodeHandle nh;
	// first arg - topic name, second arg - queue size
	ros::Publisher labbotOdometryPublisher = nh.advertise<nav_msgs::Odometry>("labbot_odometry", 50);
	tf::TransformBroadcaster labbotTransformBroadcaster;
	// first arg - topic name, second arg - queue size
	ros::Subscriber msgFromLabbotSubscriber = nh.subscribe<labbot::msgFromLabbot>("fromLabbot", 50, &msgFromLabbotCallback);

	// starting position
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;
	// starting velocities
	double vx = 0.0;
	double vy = 0.0;
	double wth = 0.0;
	
	// robot constant (dimensions, encoder properties)
	double ticksPerMeter = 775.8832547454;	// ticks per meter
	double baseWidth = 0.285;				// distance between wheels in meters

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(10.0);						// frequency of main loop
	while(nh.ok())
	{
		ros::spinOnce();					// check for incoming messages
		current_time = ros::Time::now();

		
		double dt = (current_time - last_time).toSec();	// time from last execution
		double dLS = dLTicks / ticksPerMeter;			// distance travelled by left wheel in dt time
		dLTicks = 0;									// mark number of ticks as read
		double dRS = dRTicks / ticksPerMeter;			// distance travelled by right wheel in dt time
		dRTicks = 0;									// mark number of ticks as read
		double dS = (dRS + dLS) / 2.0;					// distance travelled by center of the robot
		double dTh = (dRS - dLS) / baseWidth;			// approximate rotation (works for small angles)
		
		double dx = 0;
		double dy = 0;

		
		if(dS != 0)
		{
		// TODO sprawdzić czy działa dobrzez względem https://code.google.com/p/differential-drive/source/browse/nodes/diff_tf.py
		// i tego:
		/*
		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
		*/
			dx = dS * cos(th + dTh / 2.0);
			dy = dS * sin(th + dTh / 2.0);
		}
		if(dTh != 0)
		{
			th = th + dTh;
		}
		
		// calculate position
		x += dx;
		y += dy;
		
		// calculate velocities
		vx = dx / dt;
		vy = dy / dt;
		wth = dTh / dt;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "labbot_odometry";
		odom_trans.child_frame_id = "labbot_base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		labbotTransformBroadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "labbot_odometry";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = wth;

		//publish the message
		labbotOdometryPublisher.publish(odom);

		last_time = current_time;
		r.sleep();
	}
}

void msgFromLabbotCallback(const labbot::msgFromLabbot::ConstPtr& msgFromLabbot)
{
// TODO dodać zabezpieczenie przed przepelnieniem pozniej
// TODO zrobić
// ??? int16 ma takie bajery? to nie int16, tylko constPtr, przerobić frmLabbot z float na int16!!!
// głupi jestem, to są porównania ze zdefiniowanymi wartościami
/*
  int wdol, wgore;
  if(enk1->data>high_wrap & ltick<low_wrap)//zabezpieczenie przed przepełnieniem
    wdol=1;
  else
    wdol=0;
  if(enk1->data<low_wrap & ltick>high_wrap)
    wgore=1;
  else
    wgore=0;
  d_ltick=enk1->data-ltick+wgore*2*32767-wdol*2*32767;//wyznaczamy różnicę między obecnym stanem licznika enkodera a stanem poprzednim
  ltick=enk1->data;
  */
	// adding makes sure that no data was lost (zeroing is done in main loop)
	dLTicks += msgFromLabbot->motorLeftInput;
	dRTicks += msgFromLabbot->motorRightInput;
}
