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

class SetStripAnimationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port = null;
      this.animation_id = null;
      this.param_1 = null;
      this.param_2 = null;
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
      if (initObj.hasOwnProperty('animation_id')) {
        this.animation_id = initObj.animation_id
      }
      else {
        this.animation_id = 0;
      }
      if (initObj.hasOwnProperty('param_1')) {
        this.param_1 = initObj.param_1
      }
      else {
        this.param_1 = 0;
      }
      if (initObj.hasOwnProperty('param_2')) {
        this.param_2 = initObj.param_2
      }
      else {
        this.param_2 = 0;
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
    // Serializes a message object of type SetStripAnimationRequest
    // Serialize message field [port]
    bufferOffset = _serializer.uint8(obj.port, buffer, bufferOffset);
    // Serialize message field [animation_id]
    bufferOffset = _serializer.uint8(obj.animation_id, buffer, bufferOffset);
    // Serialize message field [param_1]
    bufferOffset = _serializer.int16(obj.param_1, buffer, bufferOffset);
    // Serialize message field [param_2]
    bufferOffset = _serializer.uint16(obj.param_2, buffer, bufferOffset);
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
    //deserializes a message object of type SetStripAnimationRequest
    let len;
    let data = new SetStripAnimationRequest(null);
    // Deserialize message field [port]
    data.port = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [animation_id]
    data.animation_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [param_1]
    data.param_1 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [param_2]
    data.param_2 = _deserializer.uint16(buffer, bufferOffset);
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
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetStripAnimationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1648b64d1990c75ff0a58f1ddcbf68a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Animate a whole strip
    
    uint8 port
    
    uint8 animation_id
    
    int16 param_1
    uint16 param_2
    
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
    const resolved = new SetStripAnimationRequest(null);
    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.animation_id !== undefined) {
      resolved.animation_id = msg.animation_id;
    }
    else {
      resolved.animation_id = 0
    }

    if (msg.param_1 !== undefined) {
      resolved.param_1 = msg.param_1;
    }
    else {
      resolved.param_1 = 0
    }

    if (msg.param_2 !== undefined) {
      resolved.param_2 = msg.param_2;
    }
    else {
      resolved.param_2 = 0
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

class SetStripAnimationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetStripAnimationResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetStripAnimationResponse
    let len;
    let data = new SetStripAnimationResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetStripAnimationResponse';
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
    const resolved = new SetStripAnimationResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetStripAnimationRequest,
  Response: SetStripAnimationResponse,
  md5sum() { return 'd1648b64d1990c75ff0a58f1ddcbf68a'; },
  datatype() { return 'mm11_msgs/SetStripAnimation'; }
};
