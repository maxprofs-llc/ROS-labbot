// required for Arduino Leonardo
#define USE_USBCON
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

	private:
		f32_t Kp;
		f32_t Ki;
		f32_t Kd;
		
		int32_t previousSpeed;
		f32_t I_part;

	public:

	// currenlty not used - averaging encoder values
	// TODO - is int32_t really needed?
	/*
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

	#define AVG_ENCODER_BUFFER_SIZE 8
	#define AVG_ENCODER_BIT_SHIFT			AVG_CALCULATE_NUMBER_OF_BITS_TO_SHIFT(8)				// 3

		int32_t encoderBuffer[AVG_ENCODER_BUFFER_SIZE];
		int8_t currentIndex;
		
		int32_t avgEncoderValue(int32_t newValue)
		{
			// C version with statics - in C++ classes it has to be moved to field of class
			// static int32_t buffer[AVG_ENCODER_BUFFER_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			// static uint8_t currentIndex = 0;
			
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
	*/
	
		CLabbotMotor()
		{
			/*
			this->currentIndex = 0;
			for(uint8_t i=0; i<AVG_ENCODER_BUFFER_SIZE; ++i)
			{
				this->buffer[i] = 0;
			}
			*/
			
			this->desiredSpeed = 0;
			this->currentSpeed = 0;
			this->steerValue = 0;
			this->previousSpeed = 0;
			this->I_part - 0.0F;
			
			this->Kp = 1.0F;
			this->Ki = 0.0F;
			this->Kd = 0.0F;
		}
		
		void Init(uint8_t encoderPin1, uint8_t encoderPin2, f32_t _Kp, f32_t _Ki, f32_t _Kd)
		{
			this->desiredSpeed = 0;
			this->currentSpeed = 0;
			this->steerValue = 0;

			this->encoder.Init(encoderPin1, encoderPin2);

			this->SetTunings(_Kp, _Ki, _Kd);

			this->previousSpeed = 0.0F;
			this->I_part = 0.0F;
		}

		void SetTunings(f32_t _Kp, f32_t _Ki, f32_t _Kd)
		{
			this->Kp = _Kp;
			this->Ki = _Ki;
			this->Kd = _Kd;
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
			if(desiredSpeed == 0 && currentSpeed == 0)
			{
					I_part = 0;
			}
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

		unsigned long time1;
		unsigned long time2;
		unsigned long time3;
		unsigned long time4;

		CLabbotMotors(uint8_t encoderRightPin1, uint8_t encoderRightPin2, uint8_t encoderLeftPin1, uint8_t encoderLeftPin2)
		{
			ScaleTunings(2.0F, 4.0F, 0.0F);

			this->right.Init(encoderRightPin1, encoderRightPin2, Kp, Ki, Kd);
			this->left.Init(encoderLeftPin1, encoderLeftPin2, Kp, Ki, Kd);

			this->motorsDriver.init();

			this->time1 = 0;
			this->time2 = 0;
			this->time3 = 0;
			this->time4 = 0;
			
			this->SetDesiredSpeed(0, 0);
			this->Update();
		}

		void Update()
		{
			unsigned long timeStart = micros();
			// read current data from encoder
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
			timeStart = micros();
			// set the driver pins to spin the motor in appropriate direction with requested speed (and scale the output -100-100 to -400-400 by multiplying by 4 (bitshift by two)
			this->motorsDriver.setSpeeds((this->right.steerValue << 2), -((this->left.steerValue << 2 )));
			time4 = (micros() - timeStart);
		}

		void SetDesiredSpeed(int32_t speedRight, int32_t speedLeft)
		{
			this->right.desiredSpeed = speedRight;
			this->left.desiredSpeed = speedLeft;
		}

		void ScaleTunings(f32_t _Kp, f32_t _Ki, f32_t _Kd)
		{
			this->Kp = _Kp;
			this->Ki = _Ki / 100.0F;
			this->Kd = _Kd * 100.0F;
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
	//Serial.begin(115200);

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
}

//unsigned long timeOfUpdate = 0;
//unsigned long timeOfCommunication = 0;

void loop()
{


	if (hasExpired(previousMotorsUpdateTime, motorsUpdateInterval))
	{
		//unsigned long timeStart = micros();

		motors.Update();

		//timeOfUpdate = (micros() - timeStart);

		//timeStart = micros();	

		sendMsg.motorLeftInput = motors.GetCurrentSpeedMotorLeft();
		sendMsg.motorLeftSetpoint = motors.GetDesiredSpeedValueMotorLeft();
		sendMsg.motorLefttOutput = motors.GetSteerValueMotorLeft();
		sendMsg.motorRightInput = motors.GetCurrentSpeedMotorRight();
		sendMsg.motorRightSetpoint = motors.GetDesiredSpeedValueMotorRight();
		sendMsg.motorRightOutput = motors.GetSteerValueMotorRight();


		 // this is the test
		//Serial.print(motors.time1);
		//Serial.print(", ");
		//Serial.print(motors.time2);
		//Serial.print(", ");
		//Serial.print(motors.time3);
		//Serial.print(", ");
		//Serial.print(motors.time4);
		//Serial.print(", ");
		//Serial.print(timeOfUpdate);
		//Serial.print(", ");
		//Serial.println(timeOfCommunication);

		//Serial.print(motors.GetCurrentSpeedMotorLeft());
		//Serial.print(", ");
		//Serial.print(motors.GetDesiredSpeedValueMotorLeft());
		//Serial.print(", ");
		//Serial.print(motors.GetSteerValueMotorLeft());
		//Serial.print(", ");
		//Serial.print(motors.GetCurrentSpeedMotorRight());
		//Serial.print(", ");
		//Serial.print(motors.GetDesiredSpeedValueMotorRight());
		//Serial.print(", ");
		//Serial.println(motors.GetSteerValueMotorRight());

		//timeOfCommunication = (micros() - timeStart);


		fromLabbot.publish(&sendMsg);
	}

	// do all the nodehandler duties once
	nh.spinOnce();
}
