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

class shMemIsClosedRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.appName = null;
      this.key = null;
    }
    else {
      if (initObj.hasOwnProperty('appName')) {
        this.appName = initObj.appName
      }
      else {
        this.appName = '';
      }
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shMemIsClosedRequest
    // Serialize message field [appName]
    bufferOffset = _serializer.string(obj.appName, buffer, bufferOffset);
    // Serialize message field [key]
    bufferOffset = _serializer.int32(obj.key, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shMemIsClosedRequest
    let len;
    let data = new shMemIsClosedRequest(null);
    // Deserialize message field [appName]
    data.appName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [key]
    data.key = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.appName);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/shMemIsClosedRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90459c82875202e256f0669058e774a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string appName #Name of the application sending the command
    int32 key #Shared memory Key
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shMemIsClosedRequest(null);
    if (msg.appName !== undefined) {
      resolved.appName = msg.appName;
    }
    else {
      resolved.appName = ''
    }

    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = 0
    }

    return resolved;
    }
};

class shMemIsClosedResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isClosed = null;
    }
    else {
      if (initObj.hasOwnProperty('isClosed')) {
        this.isClosed = initObj.isClosed
      }
      else {
        this.isClosed = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shMemIsClosedResponse
    // Serialize message field [isClosed]
    bufferOffset = _serializer.bool(obj.isClosed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shMemIsClosedResponse
    let len;
    let data = new shMemIsClosedResponse(null);
    // Deserialize message field [isClosed]
    data.isClosed = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/shMemIsClosedResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '47103612e358fb97f8ed773bb47d8e60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isClosed #True if the sharedMemory with the given key has been closed
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shMemIsClosedResponse(null);
    if (msg.isClosed !== undefined) {
      resolved.isClosed = msg.isClosed;
    }
    else {
      resolved.isClosed = false
    }

    return resolved;
    }
};

module.exports = {
  Request: shMemIsClosedRequest,
  Response: shMemIsClosedResponse,
  md5sum() { return '7d5da756f3c73d0365ab7b15abe7a3f4'; },
  datatype() { return 'pal_supervisor_msgs/shMemIsClosed'; }
};
