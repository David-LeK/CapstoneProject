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

class encoder_output_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output_rpm_m1 = null;
      this.output_controller_m1 = null;
      this.output_rpm_m2 = null;
      this.output_controller_m2 = null;
    }
    else {
      if (initObj.hasOwnProperty('output_rpm_m1')) {
        this.output_rpm_m1 = initObj.output_rpm_m1
      }
      else {
        this.output_rpm_m1 = 0.0;
      }
      if (initObj.hasOwnProperty('output_controller_m1')) {
        this.output_controller_m1 = initObj.output_controller_m1
      }
      else {
        this.output_controller_m1 = 0.0;
      }
      if (initObj.hasOwnProperty('output_rpm_m2')) {
        this.output_rpm_m2 = initObj.output_rpm_m2
      }
      else {
        this.output_rpm_m2 = 0.0;
      }
      if (initObj.hasOwnProperty('output_controller_m2')) {
        this.output_controller_m2 = initObj.output_controller_m2
      }
      else {
        this.output_controller_m2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type encoder_output_msg
    // Serialize message field [output_rpm_m1]
    bufferOffset = _serializer.float32(obj.output_rpm_m1, buffer, bufferOffset);
    // Serialize message field [output_controller_m1]
    bufferOffset = _serializer.float32(obj.output_controller_m1, buffer, bufferOffset);
    // Serialize message field [output_rpm_m2]
    bufferOffset = _serializer.float32(obj.output_rpm_m2, buffer, bufferOffset);
    // Serialize message field [output_controller_m2]
    bufferOffset = _serializer.float32(obj.output_controller_m2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type encoder_output_msg
    let len;
    let data = new encoder_output_msg(null);
    // Deserialize message field [output_rpm_m1]
    data.output_rpm_m1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [output_controller_m1]
    data.output_controller_m1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [output_rpm_m2]
    data.output_rpm_m2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [output_controller_m2]
    data.output_controller_m2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg/encoder_output_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1245866a2cb06bf0708dba963cb3a6ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 output_rpm_m1
    float32 output_controller_m1
    float32 output_rpm_m2
    float32 output_controller_m2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new encoder_output_msg(null);
    if (msg.output_rpm_m1 !== undefined) {
      resolved.output_rpm_m1 = msg.output_rpm_m1;
    }
    else {
      resolved.output_rpm_m1 = 0.0
    }

    if (msg.output_controller_m1 !== undefined) {
      resolved.output_controller_m1 = msg.output_controller_m1;
    }
    else {
      resolved.output_controller_m1 = 0.0
    }

    if (msg.output_rpm_m2 !== undefined) {
      resolved.output_rpm_m2 = msg.output_rpm_m2;
    }
    else {
      resolved.output_rpm_m2 = 0.0
    }

    if (msg.output_controller_m2 !== undefined) {
      resolved.output_controller_m2 = msg.output_controller_m2;
    }
    else {
      resolved.output_controller_m2 = 0.0
    }

    return resolved;
    }
};

module.exports = encoder_output_msg;
