#ifndef _ROS_custom_msg_stanley_outputs_h
#define _ROS_custom_msg_stanley_outputs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msg
{

  class stanley_outputs : public ros::Msg
  {
    public:
      typedef float _theta_d_type;
      _theta_d_type theta_d;
      typedef float _theta_e_type;
      _theta_e_type theta_e;
      typedef float _delta_type;
      _delta_type delta;
      typedef float _e_fa_type;
      _e_fa_type e_fa;
      typedef float _v_linear_type;
      _v_linear_type v_linear;
      typedef float _omega_type;
      _omega_type omega;
      typedef float _steering_angle_type;
      _steering_angle_type steering_angle;
      typedef float _car_yaw_type;
      _car_yaw_type car_yaw;
      typedef float _ref_yaw_type;
      _ref_yaw_type ref_yaw;

    stanley_outputs():
      theta_d(0),
      theta_e(0),
      delta(0),
      e_fa(0),
      v_linear(0),
      omega(0),
      steering_angle(0),
      car_yaw(0),
      ref_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_theta_d;
      u_theta_d.real = this->theta_d;
      *(outbuffer + offset + 0) = (u_theta_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta_d);
      union {
        float real;
        uint32_t base;
      } u_theta_e;
      u_theta_e.real = this->theta_e;
      *(outbuffer + offset + 0) = (u_theta_e.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta_e.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta_e.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta_e.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta_e);
      union {
        float real;
        uint32_t base;
      } u_delta;
      u_delta.real = this->delta;
      *(outbuffer + offset + 0) = (u_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta);
      union {
        float real;
        uint32_t base;
      } u_e_fa;
      u_e_fa.real = this->e_fa;
      *(outbuffer + offset + 0) = (u_e_fa.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_e_fa.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_e_fa.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_e_fa.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e_fa);
      union {
        float real;
        uint32_t base;
      } u_v_linear;
      u_v_linear.real = this->v_linear;
      *(outbuffer + offset + 0) = (u_v_linear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_linear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_linear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_linear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_linear);
      union {
        float real;
        uint32_t base;
      } u_omega;
      u_omega.real = this->omega;
      *(outbuffer + offset + 0) = (u_omega.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_omega.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_omega.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_omega.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->omega);
      union {
        float real;
        uint32_t base;
      } u_steering_angle;
      u_steering_angle.real = this->steering_angle;
      *(outbuffer + offset + 0) = (u_steering_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_angle);
      union {
        float real;
        uint32_t base;
      } u_car_yaw;
      u_car_yaw.real = this->car_yaw;
      *(outbuffer + offset + 0) = (u_car_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_car_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_car_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_car_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->car_yaw);
      union {
        float real;
        uint32_t base;
      } u_ref_yaw;
      u_ref_yaw.real = this->ref_yaw;
      *(outbuffer + offset + 0) = (u_ref_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ref_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ref_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ref_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ref_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_theta_d;
      u_theta_d.base = 0;
      u_theta_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta_d = u_theta_d.real;
      offset += sizeof(this->theta_d);
      union {
        float real;
        uint32_t base;
      } u_theta_e;
      u_theta_e.base = 0;
      u_theta_e.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta_e.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta_e.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta_e.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta_e = u_theta_e.real;
      offset += sizeof(this->theta_e);
      union {
        float real;
        uint32_t base;
      } u_delta;
      u_delta.base = 0;
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta = u_delta.real;
      offset += sizeof(this->delta);
      union {
        float real;
        uint32_t base;
      } u_e_fa;
      u_e_fa.base = 0;
      u_e_fa.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_e_fa.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_e_fa.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_e_fa.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->e_fa = u_e_fa.real;
      offset += sizeof(this->e_fa);
      union {
        float real;
        uint32_t base;
      } u_v_linear;
      u_v_linear.base = 0;
      u_v_linear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_linear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_linear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_linear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_linear = u_v_linear.real;
      offset += sizeof(this->v_linear);
      union {
        float real;
        uint32_t base;
      } u_omega;
      u_omega.base = 0;
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->omega = u_omega.real;
      offset += sizeof(this->omega);
      union {
        float real;
        uint32_t base;
      } u_steering_angle;
      u_steering_angle.base = 0;
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_angle = u_steering_angle.real;
      offset += sizeof(this->steering_angle);
      union {
        float real;
        uint32_t base;
      } u_car_yaw;
      u_car_yaw.base = 0;
      u_car_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_car_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_car_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_car_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->car_yaw = u_car_yaw.real;
      offset += sizeof(this->car_yaw);
      union {
        float real;
        uint32_t base;
      } u_ref_yaw;
      u_ref_yaw.base = 0;
      u_ref_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ref_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ref_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ref_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ref_yaw = u_ref_yaw.real;
      offset += sizeof(this->ref_yaw);
     return offset;
    }

    const char * getType(){ return "custom_msg/stanley_outputs"; };
    const char * getMD5(){ return "7391c579f6428617ed19d457c9a3e19e"; };

  };

}
#endif
