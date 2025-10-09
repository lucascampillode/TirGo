// Auto-generated. Do not edit!

// (in-package dynamic_footprint.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ChangeParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rate = null;
      this.padding = null;
    }
    else {
      if (initObj.hasOwnProperty('rate')) {
        this.rate = initObj.rate
      }
      else {
        this.rate = 0;
      }
      if (initObj.hasOwnProperty('padding')) {
        this.padding = initObj.padding
      }
      else {
        this.padding = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeParamsRequest
    // Serialize message field [rate]
    bufferOffset = _serializer.int32(obj.rate, buffer, bufferOffset);
    // Serialize message field [padding]
    bufferOffset = _serializer.float32(obj.padding, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeParamsRequest
    let len;
    let data = new ChangeParamsRequest(null);
    // Deserialize message field [rate]
    data.rate = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [padding]
    data.padding = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamic_footprint/ChangeParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54da70ff99cf30bf896a186492a150b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 rate
    float32 padding
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChangeParamsRequest(null);
    if (msg.rate !== undefined) {
      resolved.rate = msg.rate;
    }
    else {
      resolved.rate = 0
    }

    if (msg.padding !== undefined) {
      resolved.padding = msg.padding;
    }
    else {
      resolved.padding = 0.0
    }

    return resolved;
    }
};

class ChangeParamsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeParamsResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeParamsResponse
    let len;
    let data = new ChangeParamsResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamic_footprint/ChangeParamsResponse';
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
    const resolved = new ChangeParamsResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: ChangeParamsRequest,
  Response: ChangeParamsResponse,
  md5sum() { return '54da70ff99cf30bf896a186492a150b3'; },
  datatype() { return 'dynamic_footprint/ChangeParams'; }
};
