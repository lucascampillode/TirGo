// Auto-generated. Do not edit!

// (in-package pal_supervisor_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let StartupStatus = require('../msg/StartupStatus.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class finalStartupStatusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new StartupStatus();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type finalStartupStatusRequest
    // Serialize message field [status]
    bufferOffset = StartupStatus.serialize(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type finalStartupStatusRequest
    let len;
    let data = new finalStartupStatusRequest(null);
    // Deserialize message field [status]
    data.status = StartupStatus.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += StartupStatus.getMessageSize(object.status);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/finalStartupStatusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3fe91db533913eea244a861ae3cea188';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Final status reported by startup launch script. This should be repeated by the supervisor forever
    pal_supervisor_msgs/StartupStatus status
    
    ================================================================================
    MSG: pal_supervisor_msgs/StartupStatus
    string name # application name
    string message # description of the status if required
    # Possible status of startup process
    byte RUNNING=0
    byte WAITING=1
    byte TIMEOUT=2
    byte ABORTED=3 #Application not running
    byte status # status of the startup process for the application. Enumerated above 
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new finalStartupStatusRequest(null);
    if (msg.status !== undefined) {
      resolved.status = StartupStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new StartupStatus()
    }

    return resolved;
    }
};

class finalStartupStatusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type finalStartupStatusResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type finalStartupStatusResponse
    let len;
    let data = new finalStartupStatusResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/finalStartupStatusResponse';
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
    const resolved = new finalStartupStatusResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: finalStartupStatusRequest,
  Response: finalStartupStatusResponse,
  md5sum() { return '3fe91db533913eea244a861ae3cea188'; },
  datatype() { return 'pal_supervisor_msgs/finalStartupStatus'; }
};
