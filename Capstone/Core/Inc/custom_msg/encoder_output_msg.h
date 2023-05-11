#ifndef _ROS_custom_msg_encoder_output_msg_h
#define _ROS_custom_msg_encoder_output_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msg
{

  class encoder_output_msg : public ros::Msg
  {
    public:
      typedef float _output_rpm_m1_type;
      _output_rpm_m1_type output_rpm_m1;
      typedef float _output_controller_m1_type;
      _output_controller_m1_type output_controller_m1;
      typedef float _error_m1_type;
      _error_m1_type error_m1;
      typedef float _output_rpm_m2_type;
      _output_rpm_m2_type output_rpm_m2;
      typedef float _output_controller_m2_type;
      _output_controller_m2_type output_controller_m2;
      typedef float _error_m2_type;
      _error_m2_type error_m2;

    encoder_output_msg():
      output_rpm_m1(0),
      output_controller_m1(0),
      error_m1(0),
      output_rpm_m2(0),
      output_controller_m2(0),
      error_m2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_output_rpm_m1;
      u_output_rpm_m1.real = this->output_rpm_m1;
      *(outbuffer + offset + 0) = (u_output_rpm_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_rpm_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_rpm_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_rpm_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_rpm_m1);
      union {
        float real;
        uint32_t base;
      } u_output_controller_m1;
      u_output_controller_m1.real = this->output_controller_m1;
      *(outbuffer + offset + 0) = (u_output_controller_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_controller_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_controller_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_controller_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_controller_m1);
      union {
        float real;
        uint32_t base;
      } u_error_m1;
      u_error_m1.real = this->error_m1;
      *(outbuffer + offset + 0) = (u_error_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_m1);
      union {
        float real;
        uint32_t base;
      } u_output_rpm_m2;
      u_output_rpm_m2.real = this->output_rpm_m2;
      *(outbuffer + offset + 0) = (u_output_rpm_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_rpm_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_rpm_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_rpm_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_rpm_m2);
      union {
        float real;
        uint32_t base;
      } u_output_controller_m2;
      u_output_controller_m2.real = this->output_controller_m2;
      *(outbuffer + offset + 0) = (u_output_controller_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_controller_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_controller_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_controller_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_controller_m2);
      union {
        float real;
        uint32_t base;
      } u_error_m2;
      u_error_m2.real = this->error_m2;
      *(outbuffer + offset + 0) = (u_error_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_m2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_output_rpm_m1;
      u_output_rpm_m1.base = 0;
      u_output_rpm_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_rpm_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_rpm_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_rpm_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_rpm_m1 = u_output_rpm_m1.real;
      offset += sizeof(this->output_rpm_m1);
      union {
        float real;
        uint32_t base;
      } u_output_controller_m1;
      u_output_controller_m1.base = 0;
      u_output_controller_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_controller_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_controller_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_controller_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_controller_m1 = u_output_controller_m1.real;
      offset += sizeof(this->output_controller_m1);
      union {
        float real;
        uint32_t base;
      } u_error_m1;
      u_error_m1.base = 0;
      u_error_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error_m1 = u_error_m1.real;
      offset += sizeof(this->error_m1);
      union {
        float real;
        uint32_t base;
      } u_output_rpm_m2;
      u_output_rpm_m2.base = 0;
      u_output_rpm_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_rpm_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_rpm_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_rpm_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_rpm_m2 = u_output_rpm_m2.real;
      offset += sizeof(this->output_rpm_m2);
      union {
        float real;
        uint32_t base;
      } u_output_controller_m2;
      u_output_controller_m2.base = 0;
      u_output_controller_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_controller_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_controller_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_controller_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_controller_m2 = u_output_controller_m2.real;
      offset += sizeof(this->output_controller_m2);
      union {
        float real;
        uint32_t base;
      } u_error_m2;
      u_error_m2.base = 0;
      u_error_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error_m2 = u_error_m2.real;
      offset += sizeof(this->error_m2);
     return offset;
    }

    const char * getType(){ return "custom_msg/encoder_output_msg"; };
    const char * getMD5(){ return "6b87ede69122fe3791ae8516e827ad27"; };

  };

}
#endif
