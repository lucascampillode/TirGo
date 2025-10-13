// Auto-generated. Do not edit!

// (in-package pal_zoi_detector.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

let CurrentZoI = require('../msg/CurrentZoI.js');

//-----------------------------------------------------------

class GetPointZoIRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPointZoIRequest
    // Serialize message field [point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPointZoIRequest
    let len;
    let data = new GetPointZoIRequest(null);
    // Deserialize message field [point]
    data.point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_zoi_detector/GetPointZoIRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7c84ff13976aa04656e56e300124444';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point point
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPointZoIRequest(null);
    if (msg.point !== undefined) {
      resolved.point = geometry_msgs.msg.Point.Resolve(msg.point)
    }
    else {
      resolved.point = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

class GetPointZoIResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.zois = null;
    }
    else {
      if (initObj.hasOwnProperty('zois')) {
        this.zois = initObj.zois
      }
      else {
        this.zois = new CurrentZoI();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPointZoIResponse
    // Serialize message field [zois]
    bufferOffset = CurrentZoI.serialize(obj.zois, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPointZoIResponse
    let len;
    let data = new GetPointZoIResponse(null);
    // Deserialize message field [zois]
    data.zois = CurrentZoI.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CurrentZoI.getMessageSize(object.zois);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pal_zoi_detector/GetPointZoIResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '22208ed97a95f8cd6d2b42d76009c822';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CurrentZoI zois
    
    
    ================================================================================
    MSG: pal_zoi_detector/CurrentZoI
    string[] zois
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPointZoIResponse(null);
    if (msg.zois !== undefined) {
      resolved.zois = CurrentZoI.Resolve(msg.zois)
    }
    else {
      resolved.zois = new CurrentZoI()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPointZoIRequest,
  Response: GetPointZoIResponse,
  md5sum() { return '5b5a8a2cb9fa5d51a60a241ebea72cc6'; },
  datatype() { return 'pal_zoi_detector/GetPointZoI'; }
};
