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

class StartupGetLogRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.app = null;
      this.nlines = null;
    }
    else {
      if (initObj.hasOwnProperty('app')) {
        this.app = initObj.app
      }
      else {
        this.app = '';
      }
      if (initObj.hasOwnProperty('nlines')) {
        this.nlines = initObj.nlines
      }
      else {
        this.nlines = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartupGetLogRequest
    // Serialize message field [app]
    bufferOffset = _serializer.string(obj.app, buffer, bufferOffset);
    // Serialize message field [nlines]
    bufferOffset = _serializer.int32(obj.nlines, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupGetLogRequest
    let len;
    let data = new StartupGetLogRequest(null);
    // Deserialize message field [app]
    data.app = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [nlines]
    data.nlines = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.app);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_startup_msgs/StartupGetLogRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d53a05f3a13b77c91b7924d6279e1ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Returns nlines last lines of app's log
    string app
    int32 nlines
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartupGetLogRequest(null);
    if (msg.app !== undefined) {
      resolved.app = msg.app;
    }
    else {
      resolved.app = ''
    }

    if (msg.nlines !== undefined) {
      resolved.nlines = msg.nlines;
    }
    else {
      resolved.nlines = 0
    }

    return resolved;
    }
};

class StartupGetLogResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.log = null;
    }
    else {
      if (initObj.hasOwnProperty('log')) {
        this.log = initObj.log
      }
      else {
        this.log = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartupGetLogResponse
    // Serialize message field [log]
    bufferOffset = _serializer.string(obj.log, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupGetLogResponse
    let len;
    let data = new StartupGetLogResponse(null);
    // Deserialize message field [log]
    data.log = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.log);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_startup_msgs/StartupGetLogResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c3e2a6f3c8b4a69b39bb08b44fadcdca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string log
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartupGetLogResponse(null);
    if (msg.log !== undefined) {
      resolved.log = msg.log;
    }
    else {
      resolved.log = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: StartupGetLogRequest,
  Response: StartupGetLogResponse,
  md5sum() { return '6bc85f88abf333b9c46f2b5155e5e9f0'; },
  datatype() { return 'pal_startup_msgs/StartupGetLog'; }
};
