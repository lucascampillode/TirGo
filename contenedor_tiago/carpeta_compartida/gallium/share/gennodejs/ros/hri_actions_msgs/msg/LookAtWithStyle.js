// Auto-generated. Do not edit!

// (in-package hri_actions_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class LookAtWithStyle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.style = null;
      this.target = null;
    }
    else {
      if (initObj.hasOwnProperty('style')) {
        this.style = initObj.style
      }
      else {
        this.style = 0;
      }
      if (initObj.hasOwnProperty('target')) {
        this.target = initObj.target
      }
      else {
        this.target = new geometry_msgs.msg.PointStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LookAtWithStyle
    // Serialize message field [style]
    bufferOffset = _serializer.int8(obj.style, buffer, bufferOffset);
    // Serialize message field [target]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.target, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LookAtWithStyle
    let len;
    let data = new LookAtWithStyle(null);
    // Deserialize message field [style]
    data.style = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [target]
    data.target = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.target);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hri_actions_msgs/LookAtWithStyle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1997f720101069b0d5f7306d6cb70706';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Specify the target of the robot's gaze as a 3D point. A desired 'style'
    # policy can be specified to control how the gaze motion should be executed
    # (for instance, sudden movement, slow shift, etc.)
    
    # default policy: both eyes and head joints (eg neck) are used, the resulting
    # 'style' is a smooth combination of both
    int8 DEFAULT = 0
    
    # eyes-only policy: only the eyes (if available) should move
    int8 EYES_ONLY = 1
    
    # head-only policy: only the head joints (typically, the neck) should move
    int8 HEAD_ONLY = 2
    
    int8 style
    
    geometry_msgs/PointStamped target
    
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
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
    const resolved = new LookAtWithStyle(null);
    if (msg.style !== undefined) {
      resolved.style = msg.style;
    }
    else {
      resolved.style = 0
    }

    if (msg.target !== undefined) {
      resolved.target = geometry_msgs.msg.PointStamped.Resolve(msg.target)
    }
    else {
      resolved.target = new geometry_msgs.msg.PointStamped()
    }

    return resolved;
    }
};

// Constants for message
LookAtWithStyle.Constants = {
  DEFAULT: 0,
  EYES_ONLY: 1,
  HEAD_ONLY: 2,
}

module.exports = LookAtWithStyle;
