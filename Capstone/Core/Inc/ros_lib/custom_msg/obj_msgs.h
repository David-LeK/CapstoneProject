#ifndef _ROS_custom_msg_obj_msgs_h
#define _ROS_custom_msg_obj_msgs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msg
{

  class obj_msgs : public ros::Msg
  {
    public:
      uint32_t distance_length;
      typedef float _distance_type;
      _distance_type st_distance;
      _distance_type * distance;
      uint32_t northing_length;
      typedef float _northing_type;
      _northing_type st_northing;
      _northing_type * northing;
      uint32_t easting_length;
      typedef float _easting_type;
      _easting_type st_easting;
      _easting_type * easting;

    obj_msgs():
      distance_length(0), distance(NULL),
      northing_length(0), northing(NULL),
      easting_length(0), easting(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->distance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->distance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->distance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_length);
      for( uint32_t i = 0; i < distance_length; i++){
      union {
        float real;
        uint32_t base;
      } u_distancei;
      u_distancei.real = this->distance[i];
      *(outbuffer + offset + 0) = (u_distancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance[i]);
      }
      *(outbuffer + offset + 0) = (this->northing_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->northing_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->northing_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->northing_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->northing_length);
      for( uint32_t i = 0; i < northing_length; i++){
      union {
        float real;
        uint32_t base;
      } u_northingi;
      u_northingi.real = this->northing[i];
      *(outbuffer + offset + 0) = (u_northingi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_northingi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_northingi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_northingi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->northing[i]);
      }
      *(outbuffer + offset + 0) = (this->easting_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->easting_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->easting_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->easting_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->easting_length);
      for( uint32_t i = 0; i < easting_length; i++){
      union {
        float real;
        uint32_t base;
      } u_eastingi;
      u_eastingi.real = this->easting[i];
      *(outbuffer + offset + 0) = (u_eastingi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_eastingi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_eastingi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_eastingi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->easting[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t distance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      distance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->distance_length);
      if(distance_lengthT > distance_length)
        this->distance = (float*)realloc(this->distance, distance_lengthT * sizeof(float));
      distance_length = distance_lengthT;
      for( uint32_t i = 0; i < distance_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_distance;
      u_st_distance.base = 0;
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_distance = u_st_distance.real;
      offset += sizeof(this->st_distance);
        memcpy( &(this->distance[i]), &(this->st_distance), sizeof(float));
      }
      uint32_t northing_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      northing_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      northing_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      northing_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->northing_length);
      if(northing_lengthT > northing_length)
        this->northing = (float*)realloc(this->northing, northing_lengthT * sizeof(float));
      northing_length = northing_lengthT;
      for( uint32_t i = 0; i < northing_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_northing;
      u_st_northing.base = 0;
      u_st_northing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_northing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_northing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_northing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_northing = u_st_northing.real;
      offset += sizeof(this->st_northing);
        memcpy( &(this->northing[i]), &(this->st_northing), sizeof(float));
      }
      uint32_t easting_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      easting_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      easting_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      easting_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->easting_length);
      if(easting_lengthT > easting_length)
        this->easting = (float*)realloc(this->easting, easting_lengthT * sizeof(float));
      easting_length = easting_lengthT;
      for( uint32_t i = 0; i < easting_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_easting;
      u_st_easting.base = 0;
      u_st_easting.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_easting.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_easting.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_easting.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_easting = u_st_easting.real;
      offset += sizeof(this->st_easting);
        memcpy( &(this->easting[i]), &(this->st_easting), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "custom_msg/obj_msgs"; };
    const char * getMD5(){ return "2a1bea06901aaaf6ec5c025d3a77a953"; };

  };

}
#endif
