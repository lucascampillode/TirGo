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

class shMemGetUserIdRequest {
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
    // Serializes a message object of type shMemGetUserIdRequest
    // Serialize message field [appName]
    bufferOffset = _serializer.string(obj.appName, buffer, bufferOffset);
    // Serialize message field [key]
    bufferOffset = _serializer.int32(obj.key, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shMemGetUserIdRequest
    let len;
    let data = new shMemGetUserIdRequest(null);
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
    return 'pal_supervisor_msgs/shMemGetUserIdRequest';
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
    const resolved = new shMemGetUserIdRequest(null);
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

class shMemGetUserIdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.userId = null;
    }
    else {
      if (initObj.hasOwnProperty('userId')) {
        this.userId = initObj.userId
      }
      else {
        this.userId = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shMemGetUserIdResponse
    // Serialize message field [userId]
    bufferOffset = _serializer.string(obj.userId, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shMemGetUserIdResponse
    let len;
    let data = new shMemGetUserIdResponse(null);
    // Deserialize message field [userId]
    data.userId = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.userId);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/shMemGetUserIdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '312d2f7aff7c95ea4de64a64f8265bbd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string userId #Returns the user ID related to the Shared memory
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shMemGetUserIdResponse(null);
    if (msg.userId !== undefined) {
      resolved.userId = msg.userId;
    }
    else {
      resolved.userId = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: shMemGetUserIdRequest,
  Response: shMemGetUserIdResponse,
  md5sum() { return '4c3734a520e124bb4c1f359d31845707'; },
  datatype() { return 'pal_supervisor_msgs/shMemGetUserId'; }
};
