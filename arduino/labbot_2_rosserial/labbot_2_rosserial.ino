// required for Arduino Leonardo
#define USE_USBCON
#include <Arduino.h>

#include <ros.h>
#include <labbot/msgFromLabbot.h>
#include <labbot/msgToLabbot.h>

ros::NodeHandle  nh;

labbot::msgFromLabbot sendMsg;
labbot::msgToLabbot recvMsg;

#include <labbot_motors.h>
#include <Encoder.h>
#include <DualMC33926MotorShield.h>

CLabbotMotors motors(2, 3, 0, 1);

// function copied from cmdMessenger 
// https://github.com/thijse/Arduino-Libraries/tree/master/CmdMessenger
// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval) {
	if (millis() - prevTime > interval) {
		prevTime = millis();
		return true;
	}
	else
		return false;
}

const unsigned long motorsUpdateInterval = 10; // 0.01 second interval, 100 Hz frequency
unsigned long previousMotorsUpdateTime = 0;

void receivedMessagesCallback(const labbot::msgToLabbot& msg)
{
	double newMotorRightSpeed = 0;
	double newMotorLeftSpeed = 0;

	const double maxSpeed = 70;

	if (msg.motorRightSpeed > maxSpeed)
	{
		newMotorRightSpeed = maxSpeed;
	}
	else if (msg.motorRightSpeed < -maxSpeed)
	{
		newMotorRightSpeed = -maxSpeed;
	}
	else
	{
		newMotorRightSpeed = msg.motorRightSpeed;
	}

	if (msg.motorLeftSpeed > maxSpeed)
	{
		newMotorLeftSpeed = maxSpeed;
	}
	else if (msg.motorLeftSpeed < -maxSpeed)
	{
		newMotorLeftSpeed = -maxSpeed;
	}
	else
	{
		newMotorLeftSpeed = msg.motorLeftSpeed;
	}

	motors.SetDesiredSpeed(newMotorRightSpeed, newMotorLeftSpeed);
}

ros::Publisher fromLabbot("fromLabbot", &sendMsg);
ros::Subscriber<labbot::msgToLabbot> toLabbot("toLabbot", &receivedMessagesCallback);

#define ROS_enabled

void setup()
{
	/* add setup code here */
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	delay(1000);
	for (int i = 0; i < 10; ++i)
	{
		digitalWrite(13, HIGH);
		delay(200);
		digitalWrite(13, LOW);
		delay(200);
	}

	#ifdef ROS_enabled
		// ros objects
		nh.initNode();
		nh.advertise(fromLabbot);
		nh.subscribe(toLabbot);
	#else
		Serial.begin(115200);
	#endif
}

#ifndef ROS_enabled
	unsigned long timeOfUpdate = 0;
	unsigned long timeOfCommunication = 0;
#endif

void loop()
{
	if (hasExpired(previousMotorsUpdateTime, motorsUpdateInterval))
	{
		#ifndef ROS_enabled
			unsigned long timeStart = micros();
		#endif

		motors.Update();

		#ifndef ROS_enabled
			timeOfUpdate = (micros() - timeStart);
			timeStart = micros();	
		#endif

		sendMsg.motorLeftInput = motors.GetCurrentSpeedMotorLeft();
		sendMsg.motorLeftSetpoint = motors.GetDesiredSpeedValueMotorLeft();
		sendMsg.motorLefttOutput = motors.GetSteerValueMotorLeft();
		sendMsg.motorRightInput = motors.GetCurrentSpeedMotorRight();
		sendMsg.motorRightSetpoint = motors.GetDesiredSpeedValueMotorRight();
		sendMsg.motorRightOutput = motors.GetSteerValueMotorRight();

		#ifdef ROS_enabled
			fromLabbot.publish(&sendMsg);
		#else
			//  this is the test
			Serial.print(motors.time1);
			Serial.print(", ");
			Serial.print(motors.time2);
			Serial.print(", ");
			Serial.print(motors.time3);
			Serial.print(", ");
			Serial.print(motors.time4);
			Serial.print(", ");
			Serial.print(timeOfUpdate);
			Serial.print(", ");
			Serial.println(timeOfCommunication);

			Serial.print(motors.GetCurrentSpeedMotorLeft());
			Serial.print(", ");
			Serial.print(motors.GetDesiredSpeedValueMotorLeft());
			Serial.print(", ");
			Serial.print(motors.GetSteerValueMotorLeft());
			Serial.print(", ");
			Serial.print(motors.GetCurrentSpeedMotorRight());
			Serial.print(", ");
			Serial.print(motors.GetDesiredSpeedValueMotorRight());
			Serial.print(", ");
			Serial.println(motors.GetSteerValueMotorRight());

			timeOfCommunication = (micros() - timeStart);
		#endif	
	}

	#ifdef ROS_enabled
		// do all the nodehandler duties once
		nh.spinOnce();
	#endif
}
