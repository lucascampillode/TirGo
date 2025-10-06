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

class tcpIpIsAvailableRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ipAddress = null;
      this.port = null;
    }
    else {
      if (initObj.hasOwnProperty('ipAddress')) {
        this.ipAddress = initObj.ipAddress
      }
      else {
        this.ipAddress = '';
      }
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tcpIpIsAvailableRequest
    // Serialize message field [ipAddress]
    bufferOffset = _serializer.string(obj.ipAddress, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.int32(obj.port, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tcpIpIsAvailableRequest
    let len;
    let data = new tcpIpIsAvailableRequest(null);
    // Deserialize message field [ipAddress]
    data.ipAddress = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.ipAddress);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/tcpIpIsAvailableRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac45a89e4a3889c7f856eca8a375ef01';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string ipAddress #Ip of the TCP Server
    int32 port #Port to use
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tcpIpIsAvailableRequest(null);
    if (msg.ipAddress !== undefined) {
      resolved.ipAddress = msg.ipAddress;
    }
    else {
      resolved.ipAddress = ''
    }

    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    return resolved;
    }
};

class tcpIpIsAvailableResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.available = null;
      this.suggestedSleepTime = null;
    }
    else {
      if (initObj.hasOwnProperty('available')) {
        this.available = initObj.available
      }
      else {
        this.available = false;
      }
      if (initObj.hasOwnProperty('suggestedSleepTime')) {
        this.suggestedSleepTime = initObj.suggestedSleepTime
      }
      else {
        this.suggestedSleepTime = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tcpIpIsAvailableResponse
    // Serialize message field [available]
    bufferOffset = _serializer.bool(obj.available, buffer, bufferOffset);
    // Serialize message field [suggestedSleepTime]
    bufferOffset = _serializer.uint32(obj.suggestedSleepTime, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tcpIpIsAvailableResponse
    let len;
    let data = new tcpIpIsAvailableResponse(null);
    // Deserialize message field [available]
    data.available = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [suggestedSleepTime]
    data.suggestedSleepTime = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_supervisor_msgs/tcpIpIsAvailableResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ddb387405b8e9e09b259cdcaafeb09a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool available #True if the TCP Server is available, false if it's not
    uint32 suggestedSleepTime #Ms suggested to the client to sleep before trying again
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tcpIpIsAvailableResponse(null);
    if (msg.available !== undefined) {
      resolved.available = msg.available;
    }
    else {
      resolved.available = false
    }

    if (msg.suggestedSleepTime !== undefined) {
      resolved.suggestedSleepTime = msg.suggestedSleepTime;
    }
    else {
      resolved.suggestedSleepTime = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: tcpIpIsAvailableRequest,
  Response: tcpIpIsAvailableResponse,
  md5sum() { return 'c87e9a2a51d895bccdb0157125a69381'; },
  datatype() { return 'pal_supervisor_msgs/tcpIpIsAvailable'; }
};
