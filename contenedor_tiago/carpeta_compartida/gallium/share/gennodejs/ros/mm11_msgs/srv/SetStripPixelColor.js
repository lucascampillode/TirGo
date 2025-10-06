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

class SetStripPixelColorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port = null;
      this.pixel = null;
      this.r = null;
      this.g = null;
      this.b = null;
    }
    else {
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
      if (initObj.hasOwnProperty('pixel')) {
        this.pixel = initObj.pixel
      }
      else {
        this.pixel = 0;
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0;
      }
      if (initObj.hasOwnProperty('g')) {
        this.g = initObj.g
      }
      else {
        this.g = 0;
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetStripPixelColorRequest
    // Serialize message field [port]
    bufferOffset = _serializer.uint8(obj.port, buffer, bufferOffset);
    // Serialize message field [pixel]
    bufferOffset = _serializer.uint8(obj.pixel, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = _serializer.uint8(obj.r, buffer, bufferOffset);
    // Serialize message field [g]
    bufferOffset = _serializer.uint8(obj.g, buffer, bufferOffset);
    // Serialize message field [b]
    bufferOffset = _serializer.uint8(obj.b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetStripPixelColorRequest
    let len;
    let data = new SetStripPixelColorRequest(null);
    // Deserialize message field [port]
    data.port = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pixel]
    data.pixel = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [g]
    data.g = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [b]
    data.b = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetStripPixelColorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b649b5b940622eb64d13b51266bf8b29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Sets the color of a strip's pixel
    
    uint8 port
    
    uint8 pixel
    
    uint8 r
    uint8 g
    uint8 b
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetStripPixelColorRequest(null);
    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.pixel !== undefined) {
      resolved.pixel = msg.pixel;
    }
    else {
      resolved.pixel = 0
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0
    }

    if (msg.g !== undefined) {
      resolved.g = msg.g;
    }
    else {
      resolved.g = 0
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = 0
    }

    return resolved;
    }
};

class SetStripPixelColorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetStripPixelColorResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetStripPixelColorResponse
    let len;
    let data = new SetStripPixelColorResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetStripPixelColorResponse';
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
    const resolved = new SetStripPixelColorResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetStripPixelColorRequest,
  Response: SetStripPixelColorResponse,
  md5sum() { return 'b649b5b940622eb64d13b51266bf8b29'; },
  datatype() { return 'mm11_msgs/SetStripPixelColor'; }
};
