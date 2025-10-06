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

class numberOfAnomaliesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type numberOfAnomaliesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type numberOfAnomaliesRequest
    let len;
    let data = new numberOfAnomaliesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/numberOfAnomaliesRequest';
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
    const resolved = new numberOfAnomaliesRequest(null);
    return resolved;
    }
};

class numberOfAnomaliesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.numErrors = null;
      this.numWarnings = null;
    }
    else {
      if (initObj.hasOwnProperty('numErrors')) {
        this.numErrors = initObj.numErrors
      }
      else {
        this.numErrors = 0;
      }
      if (initObj.hasOwnProperty('numWarnings')) {
        this.numWarnings = initObj.numWarnings
      }
      else {
        this.numWarnings = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type numberOfAnomaliesResponse
    // Serialize message field [numErrors]
    bufferOffset = _serializer.int32(obj.numErrors, buffer, bufferOffset);
    // Serialize message field [numWarnings]
    bufferOffset = _serializer.int32(obj.numWarnings, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type numberOfAnomaliesResponse
    let len;
    let data = new numberOfAnomaliesResponse(null);
    // Deserialize message field [numErrors]
    data.numErrors = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [numWarnings]
    data.numWarnings = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/numberOfAnomaliesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '256e37138065fcef5d440c4ba2011920';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 numErrors #Number of errors
    int32 numWarnings #Number of warnings
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new numberOfAnomaliesResponse(null);
    if (msg.numErrors !== undefined) {
      resolved.numErrors = msg.numErrors;
    }
    else {
      resolved.numErrors = 0
    }

    if (msg.numWarnings !== undefined) {
      resolved.numWarnings = msg.numWarnings;
    }
    else {
      resolved.numWarnings = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: numberOfAnomaliesRequest,
  Response: numberOfAnomaliesResponse,
  md5sum() { return '256e37138065fcef5d440c4ba2011920'; },
  datatype() { return 'pal_supervisor_msgs/numberOfAnomalies'; }
};
