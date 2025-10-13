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

class StartupGetLogFileRequest {
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
    // Serializes a message object of type StartupGetLogFileRequest
    // Serialize message field [app]
    bufferOffset = _serializer.string(obj.app, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupGetLogFileRequest
    let len;
    let data = new StartupGetLogFileRequest(null);
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
    return 'pal_startup_msgs/StartupGetLogFileRequest';
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
    const resolved = new StartupGetLogFileRequest(null);
    if (msg.app !== undefined) {
      resolved.app = msg.app;
    }
    else {
      resolved.app = ''
    }

    return resolved;
    }
};

class StartupGetLogFileResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.log_file = null;
    }
    else {
      if (initObj.hasOwnProperty('log_file')) {
        this.log_file = initObj.log_file
      }
      else {
        this.log_file = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartupGetLogFileResponse
    // Serialize message field [log_file]
    bufferOffset = _serializer.string(obj.log_file, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupGetLogFileResponse
    let len;
    let data = new StartupGetLogFileResponse(null);
    // Deserialize message field [log_file]
    data.log_file = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.log_file);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_startup_msgs/StartupGetLogFileResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f0c93d2298afd5dbf53dae9adb7e3486';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string log_file
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartupGetLogFileResponse(null);
    if (msg.log_file !== undefined) {
      resolved.log_file = msg.log_file;
    }
    else {
      resolved.log_file = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: StartupGetLogFileRequest,
  Response: StartupGetLogFileResponse,
  md5sum() { return 'ccb1bd5394209c4a6d7f5a5e0db3dd26'; },
  datatype() { return 'pal_startup_msgs/StartupGetLogFile'; }
};
