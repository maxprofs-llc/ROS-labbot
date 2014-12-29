#ifndef _ROS_labbot_msgFromLabbot_h
#define _ROS_labbot_msgFromLabbot_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace labbot
{

  class msgFromLabbot : public ros::Msg
  {
    public:
      float motorRightInput;
      float motorRightSetpoint;
      float motorRightOutput;
      float motorLeftInput;
      float motorLeftSetpoint;
      float motorLefttOutput;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_motorRightInput;
      u_motorRightInput.real = this->motorRightInput;
      *(outbuffer + offset + 0) = (u_motorRightInput.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorRightInput.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorRightInput.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorRightInput.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorRightInput);
      union {
        float real;
        uint32_t base;
      } u_motorRightSetpoint;
      u_motorRightSetpoint.real = this->motorRightSetpoint;
      *(outbuffer + offset + 0) = (u_motorRightSetpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorRightSetpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorRightSetpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorRightSetpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorRightSetpoint);
      union {
        float real;
        uint32_t base;
      } u_motorRightOutput;
      u_motorRightOutput.real = this->motorRightOutput;
      *(outbuffer + offset + 0) = (u_motorRightOutput.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorRightOutput.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorRightOutput.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorRightOutput.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorRightOutput);
      union {
        float real;
        uint32_t base;
      } u_motorLeftInput;
      u_motorLeftInput.real = this->motorLeftInput;
      *(outbuffer + offset + 0) = (u_motorLeftInput.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorLeftInput.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorLeftInput.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorLeftInput.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorLeftInput);
      union {
        float real;
        uint32_t base;
      } u_motorLeftSetpoint;
      u_motorLeftSetpoint.real = this->motorLeftSetpoint;
      *(outbuffer + offset + 0) = (u_motorLeftSetpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorLeftSetpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorLeftSetpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorLeftSetpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorLeftSetpoint);
      union {
        float real;
        uint32_t base;
      } u_motorLefttOutput;
      u_motorLefttOutput.real = this->motorLefttOutput;
      *(outbuffer + offset + 0) = (u_motorLefttOutput.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorLefttOutput.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorLefttOutput.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorLefttOutput.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorLefttOutput);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_motorRightInput;
      u_motorRightInput.base = 0;
      u_motorRightInput.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorRightInput.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorRightInput.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorRightInput.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorRightInput = u_motorRightInput.real;
      offset += sizeof(this->motorRightInput);
      union {
        float real;
        uint32_t base;
      } u_motorRightSetpoint;
      u_motorRightSetpoint.base = 0;
      u_motorRightSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorRightSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorRightSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorRightSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorRightSetpoint = u_motorRightSetpoint.real;
      offset += sizeof(this->motorRightSetpoint);
      union {
        float real;
        uint32_t base;
      } u_motorRightOutput;
      u_motorRightOutput.base = 0;
      u_motorRightOutput.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorRightOutput.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorRightOutput.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorRightOutput.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorRightOutput = u_motorRightOutput.real;
      offset += sizeof(this->motorRightOutput);
      union {
        float real;
        uint32_t base;
      } u_motorLeftInput;
      u_motorLeftInput.base = 0;
      u_motorLeftInput.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorLeftInput.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorLeftInput.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorLeftInput.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorLeftInput = u_motorLeftInput.real;
      offset += sizeof(this->motorLeftInput);
      union {
        float real;
        uint32_t base;
      } u_motorLeftSetpoint;
      u_motorLeftSetpoint.base = 0;
      u_motorLeftSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorLeftSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorLeftSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorLeftSetpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorLeftSetpoint = u_motorLeftSetpoint.real;
      offset += sizeof(this->motorLeftSetpoint);
      union {
        float real;
        uint32_t base;
      } u_motorLefttOutput;
      u_motorLefttOutput.base = 0;
      u_motorLefttOutput.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorLefttOutput.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorLefttOutput.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorLefttOutput.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorLefttOutput = u_motorLefttOutput.real;
      offset += sizeof(this->motorLefttOutput);
     return offset;
    }

    const char * getType(){ return "labbot/msgFromLabbot"; };
    const char * getMD5(){ return "4bf0f6e2fd2b507f5c1d51ceec5b93b5"; };

  };

}
#endif