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

class SetLedsBrightnessRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.brightness = null;
    }
    else {
      if (initObj.hasOwnProperty('brightness')) {
        this.brightness = initObj.brightness
      }
      else {
        this.brightness = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetLedsBrightnessRequest
    // Serialize message field [brightness]
    bufferOffset = _serializer.uint8(obj.brightness, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetLedsBrightnessRequest
    let len;
    let data = new SetLedsBrightnessRequest(null);
    // Deserialize message field [brightness]
    data.brightness = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetLedsBrightnessRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0374114287947f81d241e42c7a34c37e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Set the LEDs brightness
    
    uint8 brightness
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetLedsBrightnessRequest(null);
    if (msg.brightness !== undefined) {
      resolved.brightness = msg.brightness;
    }
    else {
      resolved.brightness = 0
    }

    return resolved;
    }
};

class SetLedsBrightnessResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetLedsBrightnessResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetLedsBrightnessResponse
    let len;
    let data = new SetLedsBrightnessResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetLedsBrightnessResponse';
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
    const resolved = new SetLedsBrightnessResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetLedsBrightnessRequest,
  Response: SetLedsBrightnessResponse,
  md5sum() { return '0374114287947f81d241e42c7a34c37e'; },
  datatype() { return 'mm11_msgs/SetLedsBrightness'; }
};
