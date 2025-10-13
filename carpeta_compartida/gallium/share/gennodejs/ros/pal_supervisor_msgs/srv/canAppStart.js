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

class canAppStartRequest {
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
    // Serializes a message object of type canAppStartRequest
    // Serialize message field [appName]
    bufferOffset = _serializer.string(obj.appName, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type canAppStartRequest
    let len;
    let data = new canAppStartRequest(null);
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
    return 'pal_supervisor_msgs/canAppStartRequest';
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
    const resolved = new canAppStartRequest(null);
    if (msg.appName !== undefined) {
      resolved.appName = msg.appName;
    }
    else {
      resolved.appName = ''
    }

    return resolved;
    }
};

class canAppStartResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.canStart = null;
      this.description = null;
    }
    else {
      if (initObj.hasOwnProperty('canStart')) {
        this.canStart = initObj.canStart
      }
      else {
        this.canStart = false;
      }
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type canAppStartResponse
    // Serialize message field [canStart]
    bufferOffset = _serializer.bool(obj.canStart, buffer, bufferOffset);
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type canAppStartResponse
    let len;
    let data = new canAppStartResponse(null);
    // Deserialize message field [canStart]
    data.canStart = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.description);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/canAppStartResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7bff1133ecc80fd0c657d99b5dd8c67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool canStart #True if the application can start
    string description  #Reason why an application can't start. empty if can start
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new canAppStartResponse(null);
    if (msg.canStart !== undefined) {
      resolved.canStart = msg.canStart;
    }
    else {
      resolved.canStart = false
    }

    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: canAppStartRequest,
  Response: canAppStartResponse,
  md5sum() { return '2a4675c414f30fa71f2e7af8b08bf502'; },
  datatype() { return 'pal_supervisor_msgs/canAppStart'; }
};
