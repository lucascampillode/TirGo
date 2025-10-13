// Auto-generated. Do not edit!

// (in-package pal_moveit_plugin_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class LoadCapabilitiesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.capabilities = null;
    }
    else {
      if (initObj.hasOwnProperty('capabilities')) {
        this.capabilities = initObj.capabilities
      }
      else {
        this.capabilities = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LoadCapabilitiesRequest
    // Serialize message field [capabilities]
    bufferOffset = _arraySerializer.string(obj.capabilities, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LoadCapabilitiesRequest
    let len;
    let data = new LoadCapabilitiesRequest(null);
    // Deserialize message field [capabilities]
    data.capabilities = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.capabilities.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_moveit_plugin_msgs/LoadCapabilitiesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca5c1647c8abd177cf483d3cb73810ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # list of capabilities
    string[] capabilities
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LoadCapabilitiesRequest(null);
    if (msg.capabilities !== undefined) {
      resolved.capabilities = msg.capabilities;
    }
    else {
      resolved.capabilities = []
    }

    return resolved;
    }
};

class LoadCapabilitiesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LoadCapabilitiesResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.string(obj.error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LoadCapabilitiesResponse
    let len;
    let data = new LoadCapabilitiesResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.error);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_moveit_plugin_msgs/LoadCapabilitiesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45872d25d65c97743cc71afc6d4e884d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string error
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LoadCapabilitiesResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: LoadCapabilitiesRequest,
  Response: LoadCapabilitiesResponse,
  md5sum() { return 'c6f702db26dbc58fc6ddc660a0bd1ac9'; },
  datatype() { return 'pal_moveit_plugin_msgs/LoadCapabilities'; }
};
