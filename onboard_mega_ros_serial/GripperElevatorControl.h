#ifndef _ROS_rover_msgs_GripperElevatorControl_h
#define _ROS_rover_msgs_GripperElevatorControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_msgs
{

  class GripperElevatorControl : public ros::Msg
  {
    public:
      typedef int32_t _gripper_type;
      _gripper_type gripper;
      typedef int32_t _elevator_speed_type;
      _elevator_speed_type elevator_speed;
      typedef int32_t _elevator_direction_type;
      _elevator_direction_type elevator_direction;

    GripperElevatorControl():
      gripper(0),
      elevator_speed(0),
      elevator_direction(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_gripper;
      u_gripper.real = this->gripper;
      *(outbuffer + offset + 0) = (u_gripper.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper);
      union {
        int32_t real;
        uint32_t base;
      } u_elevator_speed;
      u_elevator_speed.real = this->elevator_speed;
      *(outbuffer + offset + 0) = (u_elevator_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevator_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevator_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevator_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->elevator_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_elevator_direction;
      u_elevator_direction.real = this->elevator_direction;
      *(outbuffer + offset + 0) = (u_elevator_direction.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_elevator_direction.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_elevator_direction.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_elevator_direction.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->elevator_direction);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_gripper;
      u_gripper.base = 0;
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gripper = u_gripper.real;
      offset += sizeof(this->gripper);
      union {
        int32_t real;
        uint32_t base;
      } u_elevator_speed;
      u_elevator_speed.base = 0;
      u_elevator_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevator_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevator_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevator_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->elevator_speed = u_elevator_speed.real;
      offset += sizeof(this->elevator_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_elevator_direction;
      u_elevator_direction.base = 0;
      u_elevator_direction.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_elevator_direction.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_elevator_direction.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_elevator_direction.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->elevator_direction = u_elevator_direction.real;
      offset += sizeof(this->elevator_direction);
     return offset;
    }

    const char * getType(){ return PSTR( "rover_msgs/GripperElevatorControl" ); };
    const char * getMD5(){ return PSTR( "62129799ed1b088acd2a2d076ff39117" ); };

  };

}
#endif
