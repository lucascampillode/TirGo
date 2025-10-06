// Auto-generated. Do not edit!

// (in-package pal_startup_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class StartupStopRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.app = null;
    }
    else {
      if (initObj.hasOwnProperty('app')) {
        this.app = initObj.app
      }
      else {
        this.app = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartupStopRequest
    // Serialize message field [app]
    bufferOffset = _serializer.string(obj.app, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupStopRequest
    let len;
    let data = new StartupStopRequest(null);
    // Deserialize message field [app]
    data.app = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.app);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_startup_msgs/StartupStopRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95ebb8aaf007b41ba941fcd0718c5693';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string app
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartupStopRequest(null);
    if (msg.app !== undefined) {
      resolved.app = msg.app;
    }
    else {
      resolved.app = ''
    }

    return resolved;
    }
};

class StartupStopResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartupStopResponse
    // Serialize message field [result]
    bufferOffset = _serializer.string(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupStopResponse
    let len;
    let data = new StartupStopResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.result);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_startup_msgs/StartupStopResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c22f2a1ed8654a0b365f1bb3f7ff2c0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartupStopResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: StartupStopRequest,
  Response: StartupStopResponse,
  md5sum() { return 'd0a14343d8976d7512bdd0e94c3d391e'; },
  datatype() { return 'pal_startup_msgs/StartupStop'; }
};
