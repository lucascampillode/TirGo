// Auto-generated. Do not edit!

// (in-package plotjuggler_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DataPoint = require('./DataPoint.js');

//-----------------------------------------------------------

class DataPoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dictionary_uuid = null;
      this.samples = null;
    }
    else {
      if (initObj.hasOwnProperty('dictionary_uuid')) {
        this.dictionary_uuid = initObj.dictionary_uuid
      }
      else {
        this.dictionary_uuid = 0;
      }
      if (initObj.hasOwnProperty('samples')) {
        this.samples = initObj.samples
      }
      else {
        this.samples = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataPoints
    // Serialize message field [dictionary_uuid]
    bufferOffset = _serializer.uint32(obj.dictionary_uuid, buffer, bufferOffset);
    // Serialize message field [samples]
    // Serialize the length for message field [samples]
    bufferOffset = _serializer.uint32(obj.samples.length, buffer, bufferOffset);
    obj.samples.forEach((val) => {
      bufferOffset = DataPoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataPoints
    let len;
    let data = new DataPoints(null);
    // Deserialize message field [dictionary_uuid]
    data.dictionary_uuid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [samples]
    // Deserialize array length for message field [samples]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.samples = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.samples[i] = DataPoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 18 * object.samples.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plotjuggler_msgs/DataPoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14e65e7956023a9a11291bc53d5d695a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 dictionary_uuid
    plotjuggler_msgs/DataPoint[] samples
    
    ================================================================================
    MSG: plotjuggler_msgs/DataPoint
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
    const resolved = new DataPoints(null);
    if (msg.dictionary_uuid !== undefined) {
      resolved.dictionary_uuid = msg.dictionary_uuid;
    }
    else {
      resolved.dictionary_uuid = 0
    }

    if (msg.samples !== undefined) {
      resolved.samples = new Array(msg.samples.length);
      for (let i = 0; i < resolved.samples.length; ++i) {
        resolved.samples[i] = DataPoint.Resolve(msg.samples[i]);
      }
    }
    else {
      resolved.samples = []
    }

    return resolved;
    }
};

module.exports = DataPoints;
