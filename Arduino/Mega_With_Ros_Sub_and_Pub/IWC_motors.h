#ifndef _ROS_rover_msgs_IWC_motors_h
#define _ROS_rover_msgs_IWC_motors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_msgs
{

  class IWC_motors : public ros::Msg
  {
    public:
      typedef uint8_t _right_front_speed_type;
      _right_front_speed_type right_front_speed;
      typedef uint8_t _right_front_dir_type;
      _right_front_dir_type right_front_dir;
      typedef uint8_t _right_middle_speed_type;
      _right_middle_speed_type right_middle_speed;
      typedef uint8_t _right_middle_dir_type;
      _right_middle_dir_type right_middle_dir;
      typedef uint8_t _right_rear_speed_type;
      _right_rear_speed_type right_rear_speed;
      typedef uint8_t _right_rear_dir_type;
      _right_rear_dir_type right_rear_dir;
      typedef uint8_t _left_front_speed_type;
      _left_front_speed_type left_front_speed;
      typedef uint8_t _left_front_dir_type;
      _left_front_dir_type left_front_dir;
      typedef uint8_t _left_middle_speed_type;
      _left_middle_speed_type left_middle_speed;
      typedef uint8_t _left_middle_dir_type;
      _left_middle_dir_type left_middle_dir;
      typedef uint8_t _left_rear_speed_type;
      _left_rear_speed_type left_rear_speed;
      typedef uint8_t _left_rear_dir_type;
      _left_rear_dir_type left_rear_dir;

    IWC_motors():
      right_front_speed(0),
      right_front_dir(0),
      right_middle_speed(0),
      right_middle_dir(0),
      right_rear_speed(0),
      right_rear_dir(0),
      left_front_speed(0),
      left_front_dir(0),
      left_middle_speed(0),
      left_middle_dir(0),
      left_rear_speed(0),
      left_rear_dir(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->right_front_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_front_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_right_front_dir;
      u_right_front_dir.real = this->right_front_dir;
      *(outbuffer + offset + 0) = (u_right_front_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_front_dir);
      *(outbuffer + offset + 0) = (this->right_middle_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_middle_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_right_middle_dir;
      u_right_middle_dir.real = this->right_middle_dir;
      *(outbuffer + offset + 0) = (u_right_middle_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_middle_dir);
      *(outbuffer + offset + 0) = (this->right_rear_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_rear_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_right_rear_dir;
      u_right_rear_dir.real = this->right_rear_dir;
      *(outbuffer + offset + 0) = (u_right_rear_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_rear_dir);
      *(outbuffer + offset + 0) = (this->left_front_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_front_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_left_front_dir;
      u_left_front_dir.real = this->left_front_dir;
      *(outbuffer + offset + 0) = (u_left_front_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_front_dir);
      *(outbuffer + offset + 0) = (this->left_middle_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_middle_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_left_middle_dir;
      u_left_middle_dir.real = this->left_middle_dir;
      *(outbuffer + offset + 0) = (u_left_middle_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_middle_dir);
      *(outbuffer + offset + 0) = (this->left_rear_speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_rear_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_left_rear_dir;
      u_left_rear_dir.real = this->left_rear_dir;
      *(outbuffer + offset + 0) = (u_left_rear_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_rear_dir);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->right_front_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->right_front_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_right_front_dir;
      u_right_front_dir.base = 0;
      u_right_front_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_front_dir = u_right_front_dir.real;
      offset += sizeof(this->right_front_dir);
      this->right_middle_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->right_middle_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_right_middle_dir;
      u_right_middle_dir.base = 0;
      u_right_middle_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_middle_dir = u_right_middle_dir.real;
      offset += sizeof(this->right_middle_dir);
      this->right_rear_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->right_rear_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_right_rear_dir;
      u_right_rear_dir.base = 0;
      u_right_rear_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_rear_dir = u_right_rear_dir.real;
      offset += sizeof(this->right_rear_dir);
      this->left_front_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->left_front_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_left_front_dir;
      u_left_front_dir.base = 0;
      u_left_front_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_front_dir = u_left_front_dir.real;
      offset += sizeof(this->left_front_dir);
      this->left_middle_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->left_middle_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_left_middle_dir;
      u_left_middle_dir.base = 0;
      u_left_middle_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_middle_dir = u_left_middle_dir.real;
      offset += sizeof(this->left_middle_dir);
      this->left_rear_speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->left_rear_speed);
      union {
        uint8_t real;
        uint8_t base;
      } u_left_rear_dir;
      u_left_rear_dir.base = 0;
      u_left_rear_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_rear_dir = u_left_rear_dir.real;
      offset += sizeof(this->left_rear_dir);
     return offset;
    }

    const char * getType(){ return PSTR( "rover_msgs/IWC_motors" ); };
    const char * getMD5(){ return PSTR( "9127df6b15bf89b164efabcc5866d72b" ); };

  };

}
#endif
