#ifndef C_LABBOT_MOTOR_h
#define C_LABBOT_MOTOR_h

#include <Arduino.h>
//#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
//#include "../Encoder/Encoder.h"

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

			steerValue = output;

			previousSpeed = currentSpeed;
		}
};

#endif