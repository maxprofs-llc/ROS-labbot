#define USE_USBCON

#include <ArduinoHardware.h>
#include <Arduino.h>

#include <ros.h>
#include <labbot/msgFromLabbot.h>
#include <labbot/msgToLabbot.h>

ros::NodeHandle  nh;

labbot::msgFromLabbot sendMsg;
labbot::msgToLabbot recvMsg;


#include <Encoder.h>
#include <DualMC33926MotorShield.h>

typedef float f32_t;

class CLabbotMotor
{
	public:
		int32_t desiredSpeed;
		int32_t currentSpeed;
		int32_t steerValue;

		Encoder encoder;

		f32_t Kp;
		f32_t Ki;
		f32_t Kd;

	public:
		int32_t encoderBuffer[8];

	#define AVG_CALCULATE_NUMBER_OF_BITS_TO_SHIFT(bufferSize) \
		(\
		bufferSize == 1 ? 0 : \
		bufferSize == 2 ? 1 : \
		bufferSize == 4 ? 2 : \
		bufferSize == 8 ? 3 : \
		bufferSize == 16 ? 4 : \
		bufferSize == 32 ? 5 : \
		bufferSize == 64 ? 6 : \
		bufferSize == 128 ? 7 : \
		0	\
		)

		int32_t previousSpeed;
		f32_t I_part;

	#define AVG_ENCODER_BUFFER_SIZE 8

		int32_t avgEncoderValue(int32_t newValue)
		{
			// buffer size = 8
	#define AVG_ENCODER_BIT_SHIFT			AVG_CALCULATE_NUMBER_OF_BITS_TO_SHIFT(8)				// 3
			static int32_t buffer[AVG_ENCODER_BUFFER_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			static uint8_t currentIndex = 0;

			buffer[currentIndex] = newValue;
			if (currentIndex >= (AVG_ENCODER_BUFFER_SIZE - 1))
			{
				currentIndex = 0;
			}
			else
			{
				currentIndex++;
			}

			int32_t average = 0;
			uint8_t i;
			for (i = 0; i<AVG_ENCODER_BUFFER_SIZE; i++)
			{
				average += buffer[i];
			}
			average = (average >> AVG_ENCODER_BIT_SHIFT);

			return average;
		}

		inline void PID_compute()
		{
			const int32_t outputMax = 100;
			const int32_t outputMin = -100;

			// static value in class method is accessible by all objects
			//static int32_t previousSpeed;
			//static f32_t I_part = 0;
			// PV - process variable
			// SP - set point
			// CV - control variable
			// error = PV - SP
			f32_t error = (f32_t)(desiredSpeed - currentSpeed);
			f32_t P_part = Kp * error;
			I_part = I_part + Ki * error;
			if (I_part > outputMax)
			{
				I_part = outputMax;
			}
			else if (I_part < outputMin)
			{
				I_part = outputMin;
			}
			// TODO: I_part should be limited
			f32_t D_part = Kd * (f32_t)(currentSpeed - previousSpeed);

			int32_t output = (int32_t)(P_part + I_part + D_part);

			if (output > outputMax)
			{
				output = outputMax;
			}
			else if (output < outputMin)
			{
				output = outputMin;
			}

			// TODO: is this still needed?
			if (desiredSpeed < 5 && desiredSpeed > -5)
			{
				output = 0;
			}


			// scale output (-100-100) to (-400 to 400)
			steerValue = output;

			previousSpeed = currentSpeed;
		}

	public:
		void Update()
		{

		}
};

class CLabbotMotors
{
	private:
		CLabbotMotor right;
		CLabbotMotor left;

		DualMC33926MotorShield motorsDriver;

		f32_t Kp;
		f32_t Ki;
		f32_t Kd;



	public:


		CLabbotMotors(uint8_t encoderRightPin1, uint8_t encoderRightPin2, uint8_t encoderLeftPin1, uint8_t encoderLeftPin2) //: encoderRight(encoderRightPin1, encoderRightPin2), encoderLeft(encoderLeftPin1, encoderLeftPin2)
		{
			right.encoder.Init(encoderRightPin1, encoderRightPin2);
			left.encoder.Init(encoderLeftPin1, encoderLeftPin2);

			this->motorsDriver.init();

			//this->motorRightDirection = MOTOR_DIRECTION_FORWARD;
			//this->motorLeftDirection = MOTOR_DIRECTION_FORWARD;

			this->right.desiredSpeed = 0;
			this->right.currentSpeed = 0;
			this->right.steerValue = 0;
			this->left.desiredSpeed = 0;
			this->left.currentSpeed = 0;
			this->left.steerValue = 0;

			this->SetTunnings(2.0F, 4.0F, 0.0F);

			time1 = 0;
			time2 = 0;
			time3 = 0;
			time4 = 0;

			this->SetDesiredSpeed(0, 0);
			this->Update();
		}
		/*
		CLabbotMotors(Encoder* encRight, Encoder* encLeft)
		{
		encoderRight = encRight;
		encoderLeft = encLeft;

		this->motorsDriver.init();

		//this->motorRightDirection = MOTOR_DIRECTION_FORWARD;
		//this->motorLeftDirection = MOTOR_DIRECTION_FORWARD;

		this->desiredSpeedRight = 0;
		this->currentSpeedRight = 0;
		this->steerValueRight = 0;
		this->desiredSpeedLeft = 0;
		this->currentSpeedLeft = 0;
		this->steerValueLeft = 0;

		this->Kp = 4.0F;
		this->Ki = 8.0F / 100.0F;
		this->Kd = 0.0F * 100.0F;
		}
		*/


		~CLabbotMotors()
		{
			//delete this->encoderRight;
			//delete this->encoderLeft;
		}

		unsigned long time1;
		unsigned long time2;
		unsigned long time3;
		unsigned long time4;

		void Update()
		{
			unsigned long timeStart = micros();
			// read current data from encoder
			/*
			int32_t currentEncoderRightValue = this->encoderRight->read();
			this->encoderRight->write(0);
			int32_t currentEncoderLeftValue = this->encoderLeft->read();
			this->encoderLeft->write(0);
			*/

			int32_t currentEncoderLeftValue = this->left.encoder.read();
			this->left.encoder.write(0);
			int32_t currentEncoderRightValue = -this->right.encoder.read();
			this->right.encoder.write(0);

			time1 = (micros() - timeStart);


			timeStart = micros();
			// TODO: put this above, decide what type is required (int32_t, int8_t)
			// what does speed means? rpm? m/s?
			//this->right.currentSpeed = this->right.avgEncoderValue(currentEncoderRightValue);
			//this->left.currentSpeed = this->left.avgEncoderValue(currentEncoderLeftValue);

			this->right.currentSpeed = currentEncoderRightValue;
			this->left.currentSpeed = currentEncoderLeftValue;

			time2 = (micros() - timeStart);

			timeStart = micros();
			// update the PID
			this->right.PID_compute();
			this->left.PID_compute();

			time3 = (micros() - timeStart);



			/*
			if (motorRightDirection == MOTOR_DIRECTION_FORWARD)
			{
			this->steerValueRight = -this->steerValueRight;
			}
			// else do nothing
			if (motorLeftDirection == MOTOR_DIRECTION_FORWARD)
			{
			this->steerValueLeft = -this->steerValueLeft;
			}
			// else do nothing
			*/
			timeStart = micros();
			// set the driver pins to spin the motor in appropriate direction with requested speed
			this->motorsDriver.setSpeeds(this->right.steerValue * 4, -(this->left.steerValue * 4));
			//this->motorsDriver.setM1Speed((this->steerValueRight<<2));
			//this->motorsDriver.setM2Speed(-(this->steerValueLeft<<2));
			time4 = (micros() - timeStart);
		}

		void SetDesiredSpeed(int32_t speedRight, int32_t speedLeft)
		{
			this->right.desiredSpeed = speedRight;
			this->left.desiredSpeed = speedLeft;
		}

		void SetTunnings(f32_t _Kp, f32_t _Ki, f32_t _Kd)
		{
			Kp = _Kp;
			Ki = _Ki / 100.0F;
			Kd = _Kd * 100.0F;

			this->right.Kp = Kp;
			this->right.Ki = Ki;
			this->right.Kd = Kd;

			this->left.Kp = Kp;
			this->left.Ki = Ki;
			this->left.Kd = Kd;
		}


		inline int32_t GetCurrentSpeedMotorRight()
		{
			return this->right.currentSpeed;
		}
		inline int32_t GetCurrentSpeedMotorLeft()
		{
			return this->left.currentSpeed;
		}
		inline int32_t GetSteerValueMotorRight()
		{
			return this->right.steerValue;
		}
		inline int32_t GetSteerValueMotorLeft()
		{
			return this->left.steerValue;
		}
		inline int32_t GetDesiredSpeedValueMotorRight()
		{
			return this->right.desiredSpeed;
		}
		inline int32_t GetDesiredSpeedValueMotorLeft()
		{
			return this->left.desiredSpeed;
		}

		inline int32_t GetKp()
		{
			return (int32_t)Kp;
		}
		inline int32_t GetKi()
		{
			return (int32_t)(Ki*100.0F);
		}
		inline int32_t GetKd()
		{
			return (int32_t)(Kd);
		}
};

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
const unsigned long publishInterval = 100;
unsigned long previousPublishTime = 0;

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

	// ros objects
	nh.initNode();
	nh.advertise(fromLabbot);
	nh.subscribe(toLabbot);

	motors.SetTunnings(2.0F, 4.0F, 0.0F);
}

void loop()
{
	// send data to PC after defined interval
	if (hasExpired(previousPublishTime, publishInterval))
	{
			sendMsg.motorLeftInput = motors.GetCurrentSpeedMotorLeft();
			sendMsg.motorLeftSetpoint = motors.GetDesiredSpeedValueMotorLeft();
			sendMsg.motorLefttOutput = motors.GetSteerValueMotorLeft();
			sendMsg.motorRightInput = motors.GetCurrentSpeedMotorRight();
			sendMsg.motorRightSetpoint = motors.GetDesiredSpeedValueMotorRight();
			sendMsg.motorRightOutput = motors.GetSteerValueMotorRight();

			fromLabbot.publish(&sendMsg);
	}

	if (hasExpired(previousMotorsUpdateTime, motorsUpdateInterval))
	{
		motors.Update();
	}

	// do all the nodehandler duties once
	nh.spinOnce();
}
