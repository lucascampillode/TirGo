// Auto-generated. Do not edit!

// (in-package mm11_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetRestartRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_down = null;
    }
    else {
      if (initObj.hasOwnProperty('time_down')) {
        this.time_down = initObj.time_down
      }
      else {
        this.time_down = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRestartRequest
    // Serialize message field [time_down]
    bufferOffset = _serializer.uint8(obj.time_down, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRestartRequest
    let len;
    let data = new SetRestartRequest(null);
    // Deserialize message field [time_down]
    data.time_down = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetRestartRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '312e4924f616d4157dc3bb5aee21a3bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Force a hardware restart of the robot. Keeps the robot down for the indicated seconds.
    
    uint8 time_down
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRestartRequest(null);
    if (msg.time_down !== undefined) {
      resolved.time_down = msg.time_down;
    }
    else {
      resolved.time_down = 0
    }

    return resolved;
    }
};

class SetRestartResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRestartResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRestartResponse
    let len;
    let data = new SetRestartResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetRestartResponse';
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
    const resolved = new SetRestartResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetRestartRequest,
  Response: SetRestartResponse,
  md5sum() { return '312e4924f616d4157dc3bb5aee21a3bb'; },
  datatype() { return 'mm11_msgs/SetRestart'; }
};
