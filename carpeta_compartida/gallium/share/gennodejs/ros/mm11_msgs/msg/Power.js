// Auto-generated. Do not edit!

// (in-package mm11_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Power {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.input = null;
      this.charger = null;
      this.dock = null;
      this.pc = null;
      this.charge = null;
      this.is_connected = null;
      this.is_emergency = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = 0.0;
      }
      if (initObj.hasOwnProperty('charger')) {
        this.charger = initObj.charger
      }
      else {
        this.charger = 0.0;
      }
      if (initObj.hasOwnProperty('dock')) {
        this.dock = initObj.dock
      }
      else {
        this.dock = 0.0;
      }
      if (initObj.hasOwnProperty('pc')) {
        this.pc = initObj.pc
      }
      else {
        this.pc = 0.0;
      }
      if (initObj.hasOwnProperty('charge')) {
        this.charge = initObj.charge
      }
      else {
        this.charge = 0;
      }
      if (initObj.hasOwnProperty('is_connected')) {
        this.is_connected = initObj.is_connected
      }
      else {
        this.is_connected = false;
      }
      if (initObj.hasOwnProperty('is_emergency')) {
        this.is_emergency = initObj.is_emergency
      }
      else {
        this.is_emergency = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Power
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [input]
    bufferOffset = _serializer.float64(obj.input, buffer, bufferOffset);
    // Serialize message field [charger]
    bufferOffset = _serializer.float64(obj.charger, buffer, bufferOffset);
    // Serialize message field [dock]
    bufferOffset = _serializer.float64(obj.dock, buffer, bufferOffset);
    // Serialize message field [pc]
    bufferOffset = _serializer.float64(obj.pc, buffer, bufferOffset);
    // Serialize message field [charge]
    bufferOffset = _serializer.uint8(obj.charge, buffer, bufferOffset);
    // Serialize message field [is_connected]
    bufferOffset = _serializer.bool(obj.is_connected, buffer, bufferOffset);
    // Serialize message field [is_emergency]
    bufferOffset = _serializer.bool(obj.is_emergency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Power
    let len;
    let data = new Power(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [input]
    data.input = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [charger]
    data.charger = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dock]
    data.dock = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pc]
    data.pc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [charge]
    data.charge = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [is_connected]
    data.is_connected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_emergency]
    data.is_emergency = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 35;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mm11_msgs/Power';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc034c0a3de8d87cb18c2d9ce6556e15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ROS header
    Header header
    
    # input voltage
    float64 input
    
    # charger voltage
    float64 charger
    
    # dock voltage
    float64 dock
    
    # PC voltage
    float64 pc
    
    # charge
    uint8 charge
    
    # connection to charger
    bool is_connected
    
    # emergency status
    bool is_emergency
    
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Power(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = 0.0
    }

    if (msg.charger !== undefined) {
      resolved.charger = msg.charger;
    }
    else {
      resolved.charger = 0.0
    }

    if (msg.dock !== undefined) {
      resolved.dock = msg.dock;
    }
    else {
      resolved.dock = 0.0
    }

    if (msg.pc !== undefined) {
      resolved.pc = msg.pc;
    }
    else {
      resolved.pc = 0.0
    }

    if (msg.charge !== undefined) {
      resolved.charge = msg.charge;
    }
    else {
      resolved.charge = 0
    }

    if (msg.is_connected !== undefined) {
      resolved.is_connected = msg.is_connected;
    }
    else {
      resolved.is_connected = false
    }

    if (msg.is_emergency !== undefined) {
      resolved.is_emergency = msg.is_emergency;
    }
    else {
      resolved.is_emergency = false
    }

    return resolved;
    }
};

module.exports = Power;
