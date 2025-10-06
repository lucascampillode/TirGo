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

class Dictionary {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dictionary_uuid = null;
      this.names = null;
    }
    else {
      if (initObj.hasOwnProperty('dictionary_uuid')) {
        this.dictionary_uuid = initObj.dictionary_uuid
      }
      else {
        this.dictionary_uuid = 0;
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Dictionary
    // Serialize message field [dictionary_uuid]
    bufferOffset = _serializer.uint32(obj.dictionary_uuid, buffer, bufferOffset);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Dictionary
    let len;
    let data = new Dictionary(null);
    // Deserialize message field [dictionary_uuid]
    data.dictionary_uuid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plotjuggler_msgs/Dictionary';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '12d13553d8d6a9826829b71cac454ebe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 dictionary_uuid
    string[] names
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Dictionary(null);
    if (msg.dictionary_uuid !== undefined) {
      resolved.dictionary_uuid = msg.dictionary_uuid;
    }
    else {
      resolved.dictionary_uuid = 0
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    return resolved;
    }
};

module.exports = Dictionary;
