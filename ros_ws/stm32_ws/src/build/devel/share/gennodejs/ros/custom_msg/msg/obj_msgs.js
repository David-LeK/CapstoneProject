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

class obj_msgs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.distance = null;
      this.northing = null;
      this.easting = null;
    }
    else {
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = [];
      }
      if (initObj.hasOwnProperty('northing')) {
        this.northing = initObj.northing
      }
      else {
        this.northing = [];
      }
      if (initObj.hasOwnProperty('easting')) {
        this.easting = initObj.easting
      }
      else {
        this.easting = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obj_msgs
    // Serialize message field [distance]
    bufferOffset = _arraySerializer.float32(obj.distance, buffer, bufferOffset, null);
    // Serialize message field [northing]
    bufferOffset = _arraySerializer.float32(obj.northing, buffer, bufferOffset, null);
    // Serialize message field [easting]
    bufferOffset = _arraySerializer.float32(obj.easting, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obj_msgs
    let len;
    let data = new obj_msgs(null);
    // Deserialize message field [distance]
    data.distance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [northing]
    data.northing = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [easting]
    data.easting = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.distance.length;
    length += 4 * object.northing.length;
    length += 4 * object.easting.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg/obj_msgs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a1bea06901aaaf6ec5c025d3a77a953';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] distance
    float32[] northing
    float32[] easting
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obj_msgs(null);
    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = []
    }

    if (msg.northing !== undefined) {
      resolved.northing = msg.northing;
    }
    else {
      resolved.northing = []
    }

    if (msg.easting !== undefined) {
      resolved.easting = msg.easting;
    }
    else {
      resolved.easting = []
    }

    return resolved;
    }
};

module.exports = obj_msgs;
