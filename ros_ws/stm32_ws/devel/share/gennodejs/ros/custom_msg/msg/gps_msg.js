// Auto-generated. Do not edit!

// (in-package custom_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gps_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.latitude = null;
      this.longitude = null;
      this.speed_kmh = null;
      this.northing = null;
      this.easting = null;
      this.tracking_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('speed_kmh')) {
        this.speed_kmh = initObj.speed_kmh
      }
      else {
        this.speed_kmh = 0.0;
      }
      if (initObj.hasOwnProperty('northing')) {
        this.northing = initObj.northing
      }
      else {
        this.northing = 0.0;
      }
      if (initObj.hasOwnProperty('easting')) {
        this.easting = initObj.easting
      }
      else {
        this.easting = 0.0;
      }
      if (initObj.hasOwnProperty('tracking_angle')) {
        this.tracking_angle = initObj.tracking_angle
      }
      else {
        this.tracking_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gps_msg
    // Serialize message field [latitude]
    bufferOffset = _serializer.float32(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float32(obj.longitude, buffer, bufferOffset);
    // Serialize message field [speed_kmh]
    bufferOffset = _serializer.float32(obj.speed_kmh, buffer, bufferOffset);
    // Serialize message field [northing]
    bufferOffset = _serializer.float32(obj.northing, buffer, bufferOffset);
    // Serialize message field [easting]
    bufferOffset = _serializer.float32(obj.easting, buffer, bufferOffset);
    // Serialize message field [tracking_angle]
    bufferOffset = _serializer.float32(obj.tracking_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gps_msg
    let len;
    let data = new gps_msg(null);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_kmh]
    data.speed_kmh = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [northing]
    data.northing = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [easting]
    data.easting = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tracking_angle]
    data.tracking_angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg/gps_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a89ca61072a2258a2e6eec048f89329';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 latitude
    float32 longitude
    float32 speed_kmh
    float32 northing
    float32 easting
    float32 tracking_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gps_msg(null);
    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.speed_kmh !== undefined) {
      resolved.speed_kmh = msg.speed_kmh;
    }
    else {
      resolved.speed_kmh = 0.0
    }

    if (msg.northing !== undefined) {
      resolved.northing = msg.northing;
    }
    else {
      resolved.northing = 0.0
    }

    if (msg.easting !== undefined) {
      resolved.easting = msg.easting;
    }
    else {
      resolved.easting = 0.0
    }

    if (msg.tracking_angle !== undefined) {
      resolved.tracking_angle = msg.tracking_angle;
    }
    else {
      resolved.tracking_angle = 0.0
    }

    return resolved;
    }
};

module.exports = gps_msg;
