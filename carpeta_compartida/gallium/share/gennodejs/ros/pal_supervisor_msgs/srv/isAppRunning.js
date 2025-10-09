// Auto-generated. Do not edit!

// (in-package pal_supervisor_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class isAppRunningRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.appName = null;
    }
    else {
      if (initObj.hasOwnProperty('appName')) {
        this.appName = initObj.appName
      }
      else {
        this.appName = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type isAppRunningRequest
    // Serialize message field [appName]
    bufferOffset = _serializer.string(obj.appName, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type isAppRunningRequest
    let len;
    let data = new isAppRunningRequest(null);
    // Deserialize message field [appName]
    data.appName = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.appName);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/isAppRunningRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f0659613161896351a99d743f4c69bf9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string appName #Name of the application 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new isAppRunningRequest(null);
    if (msg.appName !== undefined) {
      resolved.appName = msg.appName;
    }
    else {
      resolved.appName = ''
    }

    return resolved;
    }
};

class isAppRunningResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.level = null;
    }
    else {
      if (initObj.hasOwnProperty('level')) {
        this.level = initObj.level
      }
      else {
        this.level = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type isAppRunningResponse
    // Serialize message field [level]
    bufferOffset = _serializer.byte(obj.level, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type isAppRunningResponse
    let len;
    let data = new isAppRunningResponse(null);
    // Deserialize message field [level]
    data.level = _deserializer.byte(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/isAppRunningResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb91b3454e0e83a8c6444cd979e78581';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Possible levels of operations
    byte OK=0
    byte WARN=1
    byte ERROR=2
    byte STALE=3 #Application not running
    byte level # level of operation enumerated above 
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new isAppRunningResponse(null);
    if (msg.level !== undefined) {
      resolved.level = msg.level;
    }
    else {
      resolved.level = 0
    }

    return resolved;
    }
};

// Constants for message
isAppRunningResponse.Constants = {
  OK: 0,
  WARN: 1,
  ERROR: 2,
  STALE: 3,
}

module.exports = {
  Request: isAppRunningRequest,
  Response: isAppRunningResponse,
  md5sum() { return 'c82b84a4b8d7b2c6e1185b6a3564ad39'; },
  datatype() { return 'pal_supervisor_msgs/isAppRunning'; }
};
