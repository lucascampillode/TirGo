// Auto-generated. Do not edit!

// (in-package plotjuggler_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DataPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name_index = null;
      this.stamp = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('name_index')) {
        this.name_index = initObj.name_index
      }
      else {
        this.name_index = 0;
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = 0.0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataPoint
    // Serialize message field [name_index]
    bufferOffset = _serializer.uint16(obj.name_index, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.float64(obj.stamp, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float64(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataPoint
    let len;
    let data = new DataPoint(null);
    // Deserialize message field [name_index]
    data.name_index = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plotjuggler_msgs/DataPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '580ca7c40f92b9a6ab4b921c02ebcd28';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 name_index
    float64 stamp
    float64 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataPoint(null);
    if (msg.name_index !== undefined) {
      resolved.name_index = msg.name_index;
    }
    else {
      resolved.name_index = 0
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = 0.0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

module.exports = DataPoint;
