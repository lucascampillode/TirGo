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

let DependencyStatus = require('../msg/DependencyStatus.js');

//-----------------------------------------------------------

class getAppDependenciesRequest {
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
    // Serializes a message object of type getAppDependenciesRequest
    // Serialize message field [appName]
    bufferOffset = _serializer.string(obj.appName, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getAppDependenciesRequest
    let len;
    let data = new getAppDependenciesRequest(null);
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
    return 'pal_supervisor_msgs/getAppDependenciesRequest';
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
    const resolved = new getAppDependenciesRequest(null);
    if (msg.appName !== undefined) {
      resolved.appName = msg.appName;
    }
    else {
      resolved.appName = ''
    }

    return resolved;
    }
};

class getAppDependenciesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dependencies = null;
    }
    else {
      if (initObj.hasOwnProperty('dependencies')) {
        this.dependencies = initObj.dependencies
      }
      else {
        this.dependencies = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getAppDependenciesResponse
    // Serialize message field [dependencies]
    // Serialize the length for message field [dependencies]
    bufferOffset = _serializer.uint32(obj.dependencies.length, buffer, bufferOffset);
    obj.dependencies.forEach((val) => {
      bufferOffset = DependencyStatus.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getAppDependenciesResponse
    let len;
    let data = new getAppDependenciesResponse(null);
    // Deserialize message field [dependencies]
    // Deserialize array length for message field [dependencies]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.dependencies = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.dependencies[i] = DependencyStatus.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.dependencies.forEach((val) => {
      length += DependencyStatus.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/getAppDependenciesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e7de1f5bb22d4fbea8623c3be335477';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    pal_supervisor_msgs/DependencyStatus[] dependencies #Array of dependencies
    
    
    ================================================================================
    MSG: pal_supervisor_msgs/DependencyStatus
    string name # dependency name
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
    const resolved = new getAppDependenciesResponse(null);
    if (msg.dependencies !== undefined) {
      resolved.dependencies = new Array(msg.dependencies.length);
      for (let i = 0; i < resolved.dependencies.length; ++i) {
        resolved.dependencies[i] = DependencyStatus.Resolve(msg.dependencies[i]);
      }
    }
    else {
      resolved.dependencies = []
    }

    return resolved;
    }
};

module.exports = {
  Request: getAppDependenciesRequest,
  Response: getAppDependenciesResponse,
  md5sum() { return 'f7faf0b37d011444aea3a2fa0a051c43'; },
  datatype() { return 'pal_supervisor_msgs/getAppDependencies'; }
};
