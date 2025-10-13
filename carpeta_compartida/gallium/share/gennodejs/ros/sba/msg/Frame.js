// Auto-generated. Do not edit!

// (in-package sba.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CameraNode = require('./CameraNode.js');
let WorldPoint = require('./WorldPoint.js');
let Projection = require('./Projection.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Frame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.nodes = null;
      this.points = null;
      this.projections = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('nodes')) {
        this.nodes = initObj.nodes
      }
      else {
        this.nodes = [];
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
      if (initObj.hasOwnProperty('projections')) {
        this.projections = initObj.projections
      }
      else {
        this.projections = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Frame
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [nodes]
    // Serialize the length for message field [nodes]
    bufferOffset = _serializer.uint32(obj.nodes.length, buffer, bufferOffset);
    obj.nodes.forEach((val) => {
      bufferOffset = CameraNode.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = WorldPoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [projections]
    // Serialize the length for message field [projections]
    bufferOffset = _serializer.uint32(obj.projections.length, buffer, bufferOffset);
    obj.projections.forEach((val) => {
      bufferOffset = Projection.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Frame
    let len;
    let data = new Frame(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [nodes]
    // Deserialize array length for message field [nodes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodes[i] = CameraNode.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = WorldPoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [projections]
    // Deserialize array length for message field [projections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.projections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.projections[i] = Projection.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 101 * object.nodes.length;
    length += 36 * object.points.length;
    length += 106 * object.projections.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sba/Frame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '418ca143f82258a762de2cff21411737';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # New Frame
    Header header
    
    # New nodes (generally just 1, but want to leave this open)
    CameraNode[] nodes
    
    # New points added from the frame
    WorldPoint[] points
    
    # New projections
    Projection[] projections
    
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
    MSG: sba/CameraNode
    # Node Parameters
    uint32 index
    
    # Contains a translation and rotation
    geometry_msgs/Transform transform
    
    # Camera parameters from the K matrix
    float64 fx
    float64 fy
    float64 cx
    float64 cy
    
    # Only relevant for a stereo camera
    float64 baseline
    
    # Whether the camera is fixed in space: i.e., its position is known
    bool fixed
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: sba/WorldPoint
    # World Point parameters
    
    # Point index
    uint32 index
    
    # Coordinates in the world frame
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: sba/Projection
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
    const resolved = new Frame(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.nodes !== undefined) {
      resolved.nodes = new Array(msg.nodes.length);
      for (let i = 0; i < resolved.nodes.length; ++i) {
        resolved.nodes[i] = CameraNode.Resolve(msg.nodes[i]);
      }
    }
    else {
      resolved.nodes = []
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = WorldPoint.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    if (msg.projections !== undefined) {
      resolved.projections = new Array(msg.projections.length);
      for (let i = 0; i < resolved.projections.length; ++i) {
        resolved.projections[i] = Projection.Resolve(msg.projections[i]);
      }
    }
    else {
      resolved.projections = []
    }

    return resolved;
    }
};

module.exports = Frame;
