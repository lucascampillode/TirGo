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

class supervisorIsAvailableRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type supervisorIsAvailableRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type supervisorIsAvailableRequest
    let len;
    let data = new supervisorIsAvailableRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/supervisorIsAvailableRequest';
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
    const resolved = new supervisorIsAvailableRequest(null);
    return resolved;
    }
};

class supervisorIsAvailableResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.available = null;
    }
    else {
      if (initObj.hasOwnProperty('available')) {
        this.available = initObj.available
      }
      else {
        this.available = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type supervisorIsAvailableResponse
    // Serialize message field [available]
    bufferOffset = _serializer.bool(obj.available, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type supervisorIsAvailableResponse
    let len;
    let data = new supervisorIsAvailableResponse(null);
    // Deserialize message field [available]
    data.available = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/supervisorIsAvailableResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'efb9ac7739f299decb8575015d81760c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool available #True if the Supervisor is available, false if it's not
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new supervisorIsAvailableResponse(null);
    if (msg.available !== undefined) {
      resolved.available = msg.available;
    }
    else {
      resolved.available = false
    }

    return resolved;
    }
};

module.exports = {
  Request: supervisorIsAvailableRequest,
  Response: supervisorIsAvailableResponse,
  md5sum() { return 'efb9ac7739f299decb8575015d81760c'; },
  datatype() { return 'pal_supervisor_msgs/supervisorIsAvailable'; }
};
