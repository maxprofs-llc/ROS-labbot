#ifndef C_LABBOT_MOTORS_h
#define C_LABBOT_MOTORS_h

#include <Arduino.h>
#include "labbot_motor.h"
#include <DualMC33926MotorShield.h>
//#include "../DualMC33926MotorShield/DualMC33926MotorShield.h"

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

#endif