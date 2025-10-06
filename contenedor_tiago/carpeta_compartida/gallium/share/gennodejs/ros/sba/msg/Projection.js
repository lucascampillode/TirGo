// Auto-generated. Do not edit!

// (in-package sba.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Projection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camindex = null;
      this.pointindex = null;
      this.u = null;
      this.v = null;
      this.d = null;
      this.stereo = null;
      this.usecovariance = null;
      this.covariance = null;
    }
    else {
      if (initObj.hasOwnProperty('camindex')) {
        this.camindex = initObj.camindex
      }
      else {
        this.camindex = 0;
      }
      if (initObj.hasOwnProperty('pointindex')) {
        this.pointindex = initObj.pointindex
      }
      else {
        this.pointindex = 0;
      }
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = 0.0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0.0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('stereo')) {
        this.stereo = initObj.stereo
      }
      else {
        this.stereo = false;
      }
      if (initObj.hasOwnProperty('usecovariance')) {
        this.usecovariance = initObj.usecovariance
      }
      else {
        this.usecovariance = false;
      }
      if (initObj.hasOwnProperty('covariance')) {
        this.covariance = initObj.covariance
      }
      else {
        this.covariance = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Projection
    // Serialize message field [camindex]
    bufferOffset = _serializer.uint32(obj.camindex, buffer, bufferOffset);
    // Serialize message field [pointindex]
    bufferOffset = _serializer.uint32(obj.pointindex, buffer, bufferOffset);
    // Serialize message field [u]
    bufferOffset = _serializer.float64(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.float64(obj.v, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float64(obj.d, buffer, bufferOffset);
    // Serialize message field [stereo]
    bufferOffset = _serializer.bool(obj.stereo, buffer, bufferOffset);
    // Serialize message field [usecovariance]
    bufferOffset = _serializer.bool(obj.usecovariance, buffer, bufferOffset);
    // Check that the constant length array field [covariance] has the right length
    if (obj.covariance.length !== 9) {
      throw new Error('Unable to serialize array field covariance - length must be 9')
    }
    // Serialize message field [covariance]
    bufferOffset = _arraySerializer.float64(obj.covariance, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Projection
    let len;
    let data = new Projection(null);
    // Deserialize message field [camindex]
    data.camindex = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pointindex]
    data.pointindex = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [u]
    data.u = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [stereo]
    data.stereo = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [usecovariance]
    data.usecovariance = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [covariance]
    data.covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 106;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sba/Projection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d5a2ac666ca0038812f550185cfc756';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Projection
    
    # Camera index
    uint32 camindex
    
    # Point index
    uint32 pointindex
    
    # Projection into the image plane
    float64 u
    float64 v
    float64 d
    
    # Is this a stereo projection? (true if stereo, false if monocular)
    bool stereo
    
    # Use a covariance matrix?
    bool usecovariance
    
    # A 3x3 covariance matrix describing the error
    float64[9] covariance
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Projection(null);
    if (msg.camindex !== undefined) {
      resolved.camindex = msg.camindex;
    }
    else {
      resolved.camindex = 0
    }

    if (msg.pointindex !== undefined) {
      resolved.pointindex = msg.pointindex;
    }
    else {
      resolved.pointindex = 0
    }

    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = 0.0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0.0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.stereo !== undefined) {
      resolved.stereo = msg.stereo;
    }
    else {
      resolved.stereo = false
    }

    if (msg.usecovariance !== undefined) {
      resolved.usecovariance = msg.usecovariance;
    }
    else {
      resolved.usecovariance = false
    }

    if (msg.covariance !== undefined) {
      resolved.covariance = msg.covariance;
    }
    else {
      resolved.covariance = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Projection;
