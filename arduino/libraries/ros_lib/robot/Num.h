#ifndef _ROS_robot_Num_h
#define _ROS_robot_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot
{

  class Num : public ros::Msg
  {
    public:
      int64_t lwheel;
      int64_t rwheel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_lwheel;
      u_lwheel.real = this->lwheel;
      *(outbuffer + offset + 0) = (u_lwheel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lwheel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lwheel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lwheel.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_lwheel.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_lwheel.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_lwheel.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_lwheel.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->lwheel);
      union {
        int64_t real;
        uint64_t base;
      } u_rwheel;
      u_rwheel.real = this->rwheel;
      *(outbuffer + offset + 0) = (u_rwheel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rwheel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rwheel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rwheel.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rwheel.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rwheel.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rwheel.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rwheel.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rwheel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_lwheel;
      u_lwheel.base = 0;
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_lwheel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->lwheel = u_lwheel.real;
      offset += sizeof(this->lwheel);
      union {
        int64_t real;
        uint64_t base;
      } u_rwheel;
      u_rwheel.base = 0;
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rwheel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rwheel = u_rwheel.real;
      offset += sizeof(this->rwheel);
     return offset;
    }

    const char * getType(){ return "robot/Num"; };
    const char * getMD5(){ return "8e39f53b445bcd99541fff2210df0010"; };

  };

}
#endif