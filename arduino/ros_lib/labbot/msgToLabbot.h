#ifndef _ROS_labbot_msgToLabbot_h
#define _ROS_labbot_msgToLabbot_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace labbot
{

  class msgToLabbot : public ros::Msg
  {
    public:
      float motorRightSpeed;
      float motorLeftSpeed;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_motorRightSpeed;
      u_motorRightSpeed.real = this->motorRightSpeed;
      *(outbuffer + offset + 0) = (u_motorRightSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorRightSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorRightSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorRightSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorRightSpeed);
      union {
        float real;
        uint32_t base;
      } u_motorLeftSpeed;
      u_motorLeftSpeed.real = this->motorLeftSpeed;
      *(outbuffer + offset + 0) = (u_motorLeftSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motorLeftSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motorLeftSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motorLeftSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motorLeftSpeed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_motorRightSpeed;
      u_motorRightSpeed.base = 0;
      u_motorRightSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorRightSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorRightSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorRightSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorRightSpeed = u_motorRightSpeed.real;
      offset += sizeof(this->motorRightSpeed);
      union {
        float real;
        uint32_t base;
      } u_motorLeftSpeed;
      u_motorLeftSpeed.base = 0;
      u_motorLeftSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motorLeftSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motorLeftSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motorLeftSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motorLeftSpeed = u_motorLeftSpeed.real;
      offset += sizeof(this->motorLeftSpeed);
     return offset;
    }

    const char * getType(){ return "labbot/msgToLabbot"; };
    const char * getMD5(){ return "227e24617faca74e2a267f0d326d13ec"; };

  };

}
#endif