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

class SetWakeupRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_min_down = null;
    }
    else {
      if (initObj.hasOwnProperty('time_min_down')) {
        this.time_min_down = initObj.time_min_down
      }
      else {
        this.time_min_down = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetWakeupRequest
    // Serialize message field [time_min_down]
    bufferOffset = _serializer.uint16(obj.time_min_down, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetWakeupRequest
    let len;
    let data = new SetWakeupRequest(null);
    // Deserialize message field [time_min_down]
    data.time_min_down = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetWakeupRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c161e9706149de203305763332e4462';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Turns off the robot during indicated minutes.
    
    uint16 time_min_down
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetWakeupRequest(null);
    if (msg.time_min_down !== undefined) {
      resolved.time_min_down = msg.time_min_down;
    }
    else {
      resolved.time_min_down = 0
    }

    return resolved;
    }
};

class SetWakeupResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetWakeupResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetWakeupResponse
    let len;
    let data = new SetWakeupResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/SetWakeupResponse';
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
    const resolved = new SetWakeupResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetWakeupRequest,
  Response: SetWakeupResponse,
  md5sum() { return '2c161e9706149de203305763332e4462'; },
  datatype() { return 'mm11_msgs/SetWakeup'; }
};
