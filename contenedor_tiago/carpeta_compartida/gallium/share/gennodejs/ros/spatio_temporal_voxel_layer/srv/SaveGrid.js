// Auto-generated. Do not edit!

// (in-package spatio_temporal_voxel_layer.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SaveGridRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.file_name = null;
    }
    else {
      if (initObj.hasOwnProperty('file_name')) {
        this.file_name = initObj.file_name
      }
      else {
        this.file_name = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveGridRequest
    // Serialize message field [file_name]
    bufferOffset = std_msgs.msg.String.serialize(obj.file_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveGridRequest
    let len;
    let data = new SaveGridRequest(null);
    // Deserialize message field [file_name]
    data.file_name = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.file_name);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'spatio_temporal_voxel_layer/SaveGridRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f46a7b91c582f33b0ff0851cd14eed20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String file_name
    
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SaveGridRequest(null);
    if (msg.file_name !== undefined) {
      resolved.file_name = std_msgs.msg.String.Resolve(msg.file_name)
    }
    else {
      resolved.file_name = new std_msgs.msg.String()
    }

    return resolved;
    }
};

class SaveGridResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map_size_bytes = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('map_size_bytes')) {
        this.map_size_bytes = initObj.map_size_bytes
      }
      else {
        this.map_size_bytes = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveGridResponse
    // Serialize message field [map_size_bytes]
    bufferOffset = _serializer.float64(obj.map_size_bytes, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveGridResponse
    let len;
    let data = new SaveGridResponse(null);
    // Deserialize message field [map_size_bytes]
    data.map_size_bytes = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'spatio_temporal_voxel_layer/SaveGridResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e03f48a62494df45bbc478f0477595fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float64 map_size_bytes
    bool status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SaveGridResponse(null);
    if (msg.map_size_bytes !== undefined) {
      resolved.map_size_bytes = msg.map_size_bytes;
    }
    else {
      resolved.map_size_bytes = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SaveGridRequest,
  Response: SaveGridResponse,
  md5sum() { return '793bb7917a99799f9a78324a148a1c17'; },
  datatype() { return 'spatio_temporal_voxel_layer/SaveGrid'; }
};
