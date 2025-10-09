// Auto-generated. Do not edit!

// (in-package mm11_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetStripFlashRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port = null;
      this.time = null;
      this.period = null;
      this.r_1 = null;
      this.g_1 = null;
      this.b_1 = null;
      this.r_2 = null;
      this.g_2 = null;
      this.b_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0;
      }
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = 0;
      }
      if (initObj.hasOwnProperty('r_1')) {
        this.r_1 = initObj.r_1
      }
      else {
        this.r_1 = 0;
      }
      if (initObj.hasOwnProperty('g_1')) {
        this.g_1 = initObj.g_1
      }
      else {
        this.g_1 = 0;
      }
      if (initObj.hasOwnProperty('b_1')) {
        this.b_1 = initObj.b_1
      }
      else {
        this.b_1 = 0;
      }
      if (initObj.hasOwnProperty('r_2')) {
        this.r_2 = initObj.r_2
      }
      else {
        this.r_2 = 0;
      }
      if (initObj.hasOwnProperty('g_2')) {
        this.g_2 = initObj.g_2
      }
      else {
        this.g_2 = 0;
      }
      if (initObj.hasOwnProperty('b_2')) {
        this.b_2 = initObj.b_2
      }
      else {
        this.b_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetStripFlashRequest
    // Serialize message field [port]
    bufferOffset = _serializer.uint8(obj.port, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.uint16(obj.time, buffer, bufferOffset);
    // Serialize message field [period]
    bufferOffset = _serializer.uint16(obj.period, buffer, bufferOffset);
    // Serialize message field [r_1]
    bufferOffset = _serializer.uint8(obj.r_1, buffer, bufferOffset);
    // Serialize message field [g_1]
    bufferOffset = _serializer.uint8(obj.g_1, buffer, bufferOffset);
    // Serialize message field [b_1]
    bufferOffset = _serializer.uint8(obj.b_1, buffer, bufferOffset);
    // Serialize message field [r_2]
    bufferOffset = _serializer.uint8(obj.r_2, buffer, bufferOffset);
    // Serialize message field [g_2]
    bufferOffset = _serializer.uint8(obj.g_2, buffer, bufferOffset);
    // Serialize message field [b_2]
    bufferOffset = _serializer.uint8(obj.b_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetStripFlashRequest
    let len;
    let data = new SetStripFlashRequest(null);
    // Deserialize message field [port]
    data.port = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [period]
    data.period = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [r_1]
    data.r_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [g_1]
    data.g_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [b_1]
    data.b_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [r_2]
    data.r_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [g_2]
    data.g_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [b_2]
    data.b_2 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 11;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetStripFlashRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d41df4b77ebb64e24b098b717616db6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Flashing for a whole strip
    
    uint8 port
    
    uint16 time
    uint16 period
    
    uint8 r_1
    uint8 g_1
    uint8 b_1
    
    uint8 r_2
    uint8 g_2
    uint8 b_2
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetStripFlashRequest(null);
    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0
    }

    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = 0
    }

    if (msg.r_1 !== undefined) {
      resolved.r_1 = msg.r_1;
    }
    else {
      resolved.r_1 = 0
    }

    if (msg.g_1 !== undefined) {
      resolved.g_1 = msg.g_1;
    }
    else {
      resolved.g_1 = 0
    }

    if (msg.b_1 !== undefined) {
      resolved.b_1 = msg.b_1;
    }
    else {
      resolved.b_1 = 0
    }

    if (msg.r_2 !== undefined) {
      resolved.r_2 = msg.r_2;
    }
    else {
      resolved.r_2 = 0
    }

    if (msg.g_2 !== undefined) {
      resolved.g_2 = msg.g_2;
    }
    else {
      resolved.g_2 = 0
    }

    if (msg.b_2 !== undefined) {
      resolved.b_2 = msg.b_2;
    }
    else {
      resolved.b_2 = 0
    }

    return resolved;
    }
};

class SetStripFlashResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetStripFlashResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetStripFlashResponse
    let len;
    let data = new SetStripFlashResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetStripFlashResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetStripFlashResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetStripFlashRequest,
  Response: SetStripFlashResponse,
  md5sum() { return '4d41df4b77ebb64e24b098b717616db6'; },
  datatype() { return 'mm11_msgs/SetStripFlash'; }
};
