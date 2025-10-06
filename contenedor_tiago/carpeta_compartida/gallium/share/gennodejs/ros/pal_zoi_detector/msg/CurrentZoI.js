// Auto-generated. Do not edit!

// (in-package pal_zoi_detector.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CurrentZoI {
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
        this.zois = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CurrentZoI
    // Serialize message field [zois]
    bufferOffset = _arraySerializer.string(obj.zois, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CurrentZoI
    let len;
    let data = new CurrentZoI(null);
    // Deserialize message field [zois]
    data.zois = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.zois.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pal_zoi_detector/CurrentZoI';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '901e944bf726cc1afe5ae5dbf3a91620';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] zois
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CurrentZoI(null);
    if (msg.zois !== undefined) {
      resolved.zois = msg.zois;
    }
    else {
      resolved.zois = []
    }

    return resolved;
    }
};

module.exports = CurrentZoI;
