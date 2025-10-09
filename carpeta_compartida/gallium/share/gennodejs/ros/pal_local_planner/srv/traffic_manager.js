// Auto-generated. Do not edit!

// (in-package pal_local_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class traffic_managerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type traffic_managerRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type traffic_managerRequest
    let len;
    let data = new traffic_managerRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_local_planner/traffic_managerRequest';
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
    const resolved = new traffic_managerRequest(null);
    return resolved;
    }
};

class traffic_managerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.situation = null;
    }
    else {
      if (initObj.hasOwnProperty('situation')) {
        this.situation = initObj.situation
      }
      else {
        this.situation = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type traffic_managerResponse
    // Serialize message field [situation]
    bufferOffset = _serializer.int64(obj.situation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type traffic_managerResponse
    let len;
    let data = new traffic_managerResponse(null);
    // Deserialize message field [situation]
    data.situation = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_local_planner/traffic_managerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2687553acfba2efaeee5da06c95db2d1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 situation
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new traffic_managerResponse(null);
    if (msg.situation !== undefined) {
      resolved.situation = msg.situation;
    }
    else {
      resolved.situation = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: traffic_managerRequest,
  Response: traffic_managerResponse,
  md5sum() { return '2687553acfba2efaeee5da06c95db2d1'; },
  datatype() { return 'pal_local_planner/traffic_manager'; }
};
