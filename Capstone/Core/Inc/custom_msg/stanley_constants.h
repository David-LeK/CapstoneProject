#ifndef _ROS_custom_msg_stanley_constants_h
#define _ROS_custom_msg_stanley_constants_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msg
{

  class stanley_constants : public ros::Msg
  {
    public:
      typedef float _V_desired_type;
      _V_desired_type V_desired;
      typedef float _K_type;
      _K_type K;
      typedef float _input_Kp_m1_type;
      _input_Kp_m1_type input_Kp_m1;
      typedef float _input_Ki_m1_type;
      _input_Ki_m1_type input_Ki_m1;
      typedef float _input_Kd_m1_type;
      _input_Kd_m1_type input_Kd_m1;
      typedef float _input_Kp_m2_type;
      _input_Kp_m2_type input_Kp_m2;
      typedef float _input_Ki_m2_type;
      _input_Ki_m2_type input_Ki_m2;
      typedef float _input_Kd_m2_type;
      _input_Kd_m2_type input_Kd_m2;

    stanley_constants():
      V_desired(0),
      K(0),
      input_Kp_m1(0),
      input_Ki_m1(0),
      input_Kd_m1(0),
      input_Kp_m2(0),
      input_Ki_m2(0),
      input_Kd_m2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_V_desired;
      u_V_desired.real = this->V_desired;
      *(outbuffer + offset + 0) = (u_V_desired.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_V_desired.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_V_desired.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_V_desired.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->V_desired);
      union {
        float real;
        uint32_t base;
      } u_K;
      u_K.real = this->K;
      *(outbuffer + offset + 0) = (u_K.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_K.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_K.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_K.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->K);
      union {
        float real;
        uint32_t base;
      } u_input_Kp_m1;
      u_input_Kp_m1.real = this->input_Kp_m1;
      *(outbuffer + offset + 0) = (u_input_Kp_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_Kp_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_Kp_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_Kp_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_Kp_m1);
      union {
        float real;
        uint32_t base;
      } u_input_Ki_m1;
      u_input_Ki_m1.real = this->input_Ki_m1;
      *(outbuffer + offset + 0) = (u_input_Ki_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_Ki_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_Ki_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_Ki_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_Ki_m1);
      union {
        float real;
        uint32_t base;
      } u_input_Kd_m1;
      u_input_Kd_m1.real = this->input_Kd_m1;
      *(outbuffer + offset + 0) = (u_input_Kd_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_Kd_m1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_Kd_m1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_Kd_m1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_Kd_m1);
      union {
        float real;
        uint32_t base;
      } u_input_Kp_m2;
      u_input_Kp_m2.real = this->input_Kp_m2;
      *(outbuffer + offset + 0) = (u_input_Kp_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_Kp_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_Kp_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_Kp_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_Kp_m2);
      union {
        float real;
        uint32_t base;
      } u_input_Ki_m2;
      u_input_Ki_m2.real = this->input_Ki_m2;
      *(outbuffer + offset + 0) = (u_input_Ki_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_Ki_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_Ki_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_Ki_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_Ki_m2);
      union {
        float real;
        uint32_t base;
      } u_input_Kd_m2;
      u_input_Kd_m2.real = this->input_Kd_m2;
      *(outbuffer + offset + 0) = (u_input_Kd_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_Kd_m2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_Kd_m2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_Kd_m2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_Kd_m2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_V_desired;
      u_V_desired.base = 0;
      u_V_desired.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_V_desired.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_V_desired.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_V_desired.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->V_desired = u_V_desired.real;
      offset += sizeof(this->V_desired);
      union {
        float real;
        uint32_t base;
      } u_K;
      u_K.base = 0;
      u_K.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_K.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_K.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_K.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->K = u_K.real;
      offset += sizeof(this->K);
      union {
        float real;
        uint32_t base;
      } u_input_Kp_m1;
      u_input_Kp_m1.base = 0;
      u_input_Kp_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_Kp_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_Kp_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_Kp_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_Kp_m1 = u_input_Kp_m1.real;
      offset += sizeof(this->input_Kp_m1);
      union {
        float real;
        uint32_t base;
      } u_input_Ki_m1;
      u_input_Ki_m1.base = 0;
      u_input_Ki_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_Ki_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_Ki_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_Ki_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_Ki_m1 = u_input_Ki_m1.real;
      offset += sizeof(this->input_Ki_m1);
      union {
        float real;
        uint32_t base;
      } u_input_Kd_m1;
      u_input_Kd_m1.base = 0;
      u_input_Kd_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_Kd_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_Kd_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_Kd_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_Kd_m1 = u_input_Kd_m1.real;
      offset += sizeof(this->input_Kd_m1);
      union {
        float real;
        uint32_t base;
      } u_input_Kp_m2;
      u_input_Kp_m2.base = 0;
      u_input_Kp_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_Kp_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_Kp_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_Kp_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_Kp_m2 = u_input_Kp_m2.real;
      offset += sizeof(this->input_Kp_m2);
      union {
        float real;
        uint32_t base;
      } u_input_Ki_m2;
      u_input_Ki_m2.base = 0;
      u_input_Ki_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_Ki_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_Ki_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_Ki_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_Ki_m2 = u_input_Ki_m2.real;
      offset += sizeof(this->input_Ki_m2);
      union {
        float real;
        uint32_t base;
      } u_input_Kd_m2;
      u_input_Kd_m2.base = 0;
      u_input_Kd_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_Kd_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_Kd_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_Kd_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_Kd_m2 = u_input_Kd_m2.real;
      offset += sizeof(this->input_Kd_m2);
     return offset;
    }

    const char * getType(){ return "custom_msg/stanley_constants"; };
    const char * getMD5(){ return "404b6951d30d05b33a57ed5a96fc98d0"; };

  };

}
#endif
