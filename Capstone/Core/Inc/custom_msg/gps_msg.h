#ifndef _ROS_custom_msg_gps_msg_h
#define _ROS_custom_msg_gps_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msg
{

  class gps_msg : public ros::Msg
  {
    public:
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _speed_kmh_type;
      _speed_kmh_type speed_kmh;
      typedef float _northing_type;
      _northing_type northing;
      typedef float _easting_type;
      _easting_type easting;
      typedef float _tracking_angle_type;
      _tracking_angle_type tracking_angle;

    gps_msg():
      latitude(0),
      longitude(0),
      speed_kmh(0),
      northing(0),
      easting(0),
      tracking_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_latitude;
      u_latitude.real = this->latitude;
      *(outbuffer + offset + 0) = (u_latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->latitude);
      union {
        float real;
        uint32_t base;
      } u_longitude;
      u_longitude.real = this->longitude;
      *(outbuffer + offset + 0) = (u_longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->longitude);
      union {
        float real;
        uint32_t base;
      } u_speed_kmh;
      u_speed_kmh.real = this->speed_kmh;
      *(outbuffer + offset + 0) = (u_speed_kmh.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_kmh.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_kmh.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_kmh.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_kmh);
      union {
        float real;
        uint32_t base;
      } u_northing;
      u_northing.real = this->northing;
      *(outbuffer + offset + 0) = (u_northing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_northing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_northing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_northing.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->northing);
      union {
        float real;
        uint32_t base;
      } u_easting;
      u_easting.real = this->easting;
      *(outbuffer + offset + 0) = (u_easting.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_easting.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_easting.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_easting.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->easting);
      union {
        float real;
        uint32_t base;
      } u_tracking_angle;
      u_tracking_angle.real = this->tracking_angle;
      *(outbuffer + offset + 0) = (u_tracking_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tracking_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tracking_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tracking_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_latitude;
      u_latitude.base = 0;
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->latitude = u_latitude.real;
      offset += sizeof(this->latitude);
      union {
        float real;
        uint32_t base;
      } u_longitude;
      u_longitude.base = 0;
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->longitude = u_longitude.real;
      offset += sizeof(this->longitude);
      union {
        float real;
        uint32_t base;
      } u_speed_kmh;
      u_speed_kmh.base = 0;
      u_speed_kmh.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_kmh.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_kmh.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_kmh.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_kmh = u_speed_kmh.real;
      offset += sizeof(this->speed_kmh);
      union {
        float real;
        uint32_t base;
      } u_northing;
      u_northing.base = 0;
      u_northing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_northing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_northing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_northing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->northing = u_northing.real;
      offset += sizeof(this->northing);
      union {
        float real;
        uint32_t base;
      } u_easting;
      u_easting.base = 0;
      u_easting.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_easting.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_easting.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_easting.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->easting = u_easting.real;
      offset += sizeof(this->easting);
      union {
        float real;
        uint32_t base;
      } u_tracking_angle;
      u_tracking_angle.base = 0;
      u_tracking_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tracking_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tracking_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tracking_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tracking_angle = u_tracking_angle.real;
      offset += sizeof(this->tracking_angle);
     return offset;
    }

    const char * getType(){ return "custom_msg/gps_msg"; };
    const char * getMD5(){ return "9a89ca61072a2258a2e6eec048f89329"; };

  };

}
#endif
