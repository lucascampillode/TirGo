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

class GetOutputStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOutputStateRequest
    // Serialize message field [output]
    bufferOffset = _serializer.uint8(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOutputStateRequest
    let len;
    let data = new GetOutputStateRequest(null);
    // Deserialize message field [output]
    data.output = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/GetOutputStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8b26e0bb5c5fc4aee37137f79a26d19';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Get an Output state
    
    uint8 output
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOutputStateRequest(null);
    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = 0
    }

    return resolved;
    }
};

class GetOutputStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOutputStateResponse
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOutputStateResponse
    let len;
    let data = new GetOutputStateResponse(null);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mm11_msgs/GetOutputStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '800f34bc468def1d86e2d42bea5648c0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint8 state
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOutputStateResponse(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetOutputStateRequest,
  Response: GetOutputStateResponse,
  md5sum() { return '30376e3f1875e49fda4a395a76150de1'; },
  datatype() { return 'mm11_msgs/GetOutputState'; }
};
