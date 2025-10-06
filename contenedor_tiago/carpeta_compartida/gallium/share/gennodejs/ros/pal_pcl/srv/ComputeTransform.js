// Auto-generated. Do not edit!

// (in-package pal_pcl.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ComputeTransformRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camera_topic = null;
      this.optical_frame = null;
      this.camera_frame = null;
      this.max_dist = null;
    }
    else {
      if (initObj.hasOwnProperty('camera_topic')) {
        this.camera_topic = initObj.camera_topic
      }
      else {
        this.camera_topic = '';
      }
      if (initObj.hasOwnProperty('optical_frame')) {
        this.optical_frame = initObj.optical_frame
      }
      else {
        this.optical_frame = '';
      }
      if (initObj.hasOwnProperty('camera_frame')) {
        this.camera_frame = initObj.camera_frame
      }
      else {
        this.camera_frame = '';
      }
      if (initObj.hasOwnProperty('max_dist')) {
        this.max_dist = initObj.max_dist
      }
      else {
        this.max_dist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeTransformRequest
    // Serialize message field [camera_topic]
    bufferOffset = _serializer.string(obj.camera_topic, buffer, bufferOffset);
    // Serialize message field [optical_frame]
    bufferOffset = _serializer.string(obj.optical_frame, buffer, bufferOffset);
    // Serialize message field [camera_frame]
    bufferOffset = _serializer.string(obj.camera_frame, buffer, bufferOffset);
    // Serialize message field [max_dist]
    bufferOffset = _serializer.float64(obj.max_dist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeTransformRequest
    let len;
    let data = new ComputeTransformRequest(null);
    // Deserialize message field [camera_topic]
    data.camera_topic = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [optical_frame]
    data.optical_frame = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [camera_frame]
    data.camera_frame = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [max_dist]
    data.max_dist = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.camera_topic);
    length += _getByteLength(object.optical_frame);
    length += _getByteLength(object.camera_frame);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_pcl/ComputeTransformRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81474f94945a292f8b2d7b128ce4473e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string camera_topic
    string optical_frame
    string camera_frame
    float64 max_dist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeTransformRequest(null);
    if (msg.camera_topic !== undefined) {
      resolved.camera_topic = msg.camera_topic;
    }
    else {
      resolved.camera_topic = ''
    }

    if (msg.optical_frame !== undefined) {
      resolved.optical_frame = msg.optical_frame;
    }
    else {
      resolved.optical_frame = ''
    }

    if (msg.camera_frame !== undefined) {
      resolved.camera_frame = msg.camera_frame;
    }
    else {
      resolved.camera_frame = ''
    }

    if (msg.max_dist !== undefined) {
      resolved.max_dist = msg.max_dist;
    }
    else {
      resolved.max_dist = 0.0
    }

    return resolved;
    }
};

class ComputeTransformResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.height = null;
      this.height_error = null;
      this.roll = null;
      this.roll_error = null;
      this.pitch = null;
      this.pitch_error = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('height_error')) {
        this.height_error = initObj.height_error
      }
      else {
        this.height_error = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('roll_error')) {
        this.roll_error = initObj.roll_error
      }
      else {
        this.roll_error = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_error')) {
        this.pitch_error = initObj.pitch_error
      }
      else {
        this.pitch_error = 0.0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeTransformResponse
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [height_error]
    bufferOffset = _serializer.float64(obj.height_error, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [roll_error]
    bufferOffset = _serializer.float64(obj.roll_error, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [pitch_error]
    bufferOffset = _serializer.float64(obj.pitch_error, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.bool(obj.error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeTransformResponse
    let len;
    let data = new ComputeTransformResponse(null);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [height_error]
    data.height_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll_error]
    data.roll_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch_error]
    data.pitch_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_pcl/ComputeTransformResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '77036f6b3c69064f401b263d7460c09d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 height
    float64 height_error
    float64 roll
    float64 roll_error
    float64 pitch
    float64 pitch_error
    bool error
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeTransformResponse(null);
    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.height_error !== undefined) {
      resolved.height_error = msg.height_error;
    }
    else {
      resolved.height_error = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.roll_error !== undefined) {
      resolved.roll_error = msg.roll_error;
    }
    else {
      resolved.roll_error = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.pitch_error !== undefined) {
      resolved.pitch_error = msg.pitch_error;
    }
    else {
      resolved.pitch_error = 0.0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ComputeTransformRequest,
  Response: ComputeTransformResponse,
  md5sum() { return '63c14dea37df6909b0547d1d22800cc9'; },
  datatype() { return 'pal_pcl/ComputeTransform'; }
};
