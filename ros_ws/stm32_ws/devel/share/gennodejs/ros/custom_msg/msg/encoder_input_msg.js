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

class encoder_input_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input_setpoint_m1 = null;
      this.input_Kp_m1 = null;
      this.input_Ki_m1 = null;
      this.input_Kd_m1 = null;
      this.input_setpoint_m2 = null;
      this.input_Kp_m2 = null;
      this.input_Ki_m2 = null;
      this.input_Kd_m2 = null;
    }
    else {
      if (initObj.hasOwnProperty('input_setpoint_m1')) {
        this.input_setpoint_m1 = initObj.input_setpoint_m1
      }
      else {
        this.input_setpoint_m1 = 0.0;
      }
      if (initObj.hasOwnProperty('input_Kp_m1')) {
        this.input_Kp_m1 = initObj.input_Kp_m1
      }
      else {
        this.input_Kp_m1 = 0.0;
      }
      if (initObj.hasOwnProperty('input_Ki_m1')) {
        this.input_Ki_m1 = initObj.input_Ki_m1
      }
      else {
        this.input_Ki_m1 = 0.0;
      }
      if (initObj.hasOwnProperty('input_Kd_m1')) {
        this.input_Kd_m1 = initObj.input_Kd_m1
      }
      else {
        this.input_Kd_m1 = 0.0;
      }
      if (initObj.hasOwnProperty('input_setpoint_m2')) {
        this.input_setpoint_m2 = initObj.input_setpoint_m2
      }
      else {
        this.input_setpoint_m2 = 0.0;
      }
      if (initObj.hasOwnProperty('input_Kp_m2')) {
        this.input_Kp_m2 = initObj.input_Kp_m2
      }
      else {
        this.input_Kp_m2 = 0.0;
      }
      if (initObj.hasOwnProperty('input_Ki_m2')) {
        this.input_Ki_m2 = initObj.input_Ki_m2
      }
      else {
        this.input_Ki_m2 = 0.0;
      }
      if (initObj.hasOwnProperty('input_Kd_m2')) {
        this.input_Kd_m2 = initObj.input_Kd_m2
      }
      else {
        this.input_Kd_m2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type encoder_input_msg
    // Serialize message field [input_setpoint_m1]
    bufferOffset = _serializer.float32(obj.input_setpoint_m1, buffer, bufferOffset);
    // Serialize message field [input_Kp_m1]
    bufferOffset = _serializer.float32(obj.input_Kp_m1, buffer, bufferOffset);
    // Serialize message field [input_Ki_m1]
    bufferOffset = _serializer.float32(obj.input_Ki_m1, buffer, bufferOffset);
    // Serialize message field [input_Kd_m1]
    bufferOffset = _serializer.float32(obj.input_Kd_m1, buffer, bufferOffset);
    // Serialize message field [input_setpoint_m2]
    bufferOffset = _serializer.float32(obj.input_setpoint_m2, buffer, bufferOffset);
    // Serialize message field [input_Kp_m2]
    bufferOffset = _serializer.float32(obj.input_Kp_m2, buffer, bufferOffset);
    // Serialize message field [input_Ki_m2]
    bufferOffset = _serializer.float32(obj.input_Ki_m2, buffer, bufferOffset);
    // Serialize message field [input_Kd_m2]
    bufferOffset = _serializer.float32(obj.input_Kd_m2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type encoder_input_msg
    let len;
    let data = new encoder_input_msg(null);
    // Deserialize message field [input_setpoint_m1]
    data.input_setpoint_m1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_Kp_m1]
    data.input_Kp_m1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_Ki_m1]
    data.input_Ki_m1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_Kd_m1]
    data.input_Kd_m1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_setpoint_m2]
    data.input_setpoint_m2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_Kp_m2]
    data.input_Kp_m2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_Ki_m2]
    data.input_Ki_m2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [input_Kd_m2]
    data.input_Kd_m2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg/encoder_input_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db9ba92c90b9ba885220db60c812fbcf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 input_setpoint_m1
    float32 input_Kp_m1
    float32 input_Ki_m1
    float32 input_Kd_m1
    float32 input_setpoint_m2
    float32 input_Kp_m2
    float32 input_Ki_m2
    float32 input_Kd_m2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new encoder_input_msg(null);
    if (msg.input_setpoint_m1 !== undefined) {
      resolved.input_setpoint_m1 = msg.input_setpoint_m1;
    }
    else {
      resolved.input_setpoint_m1 = 0.0
    }

    if (msg.input_Kp_m1 !== undefined) {
      resolved.input_Kp_m1 = msg.input_Kp_m1;
    }
    else {
      resolved.input_Kp_m1 = 0.0
    }

    if (msg.input_Ki_m1 !== undefined) {
      resolved.input_Ki_m1 = msg.input_Ki_m1;
    }
    else {
      resolved.input_Ki_m1 = 0.0
    }

    if (msg.input_Kd_m1 !== undefined) {
      resolved.input_Kd_m1 = msg.input_Kd_m1;
    }
    else {
      resolved.input_Kd_m1 = 0.0
    }

    if (msg.input_setpoint_m2 !== undefined) {
      resolved.input_setpoint_m2 = msg.input_setpoint_m2;
    }
    else {
      resolved.input_setpoint_m2 = 0.0
    }

    if (msg.input_Kp_m2 !== undefined) {
      resolved.input_Kp_m2 = msg.input_Kp_m2;
    }
    else {
      resolved.input_Kp_m2 = 0.0
    }

    if (msg.input_Ki_m2 !== undefined) {
      resolved.input_Ki_m2 = msg.input_Ki_m2;
    }
    else {
      resolved.input_Ki_m2 = 0.0
    }

    if (msg.input_Kd_m2 !== undefined) {
      resolved.input_Kd_m2 = msg.input_Kd_m2;
    }
    else {
      resolved.input_Kd_m2 = 0.0
    }

    return resolved;
    }
};

module.exports = encoder_input_msg;
