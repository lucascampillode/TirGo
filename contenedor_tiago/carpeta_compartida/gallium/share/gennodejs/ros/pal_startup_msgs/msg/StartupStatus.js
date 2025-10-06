// Auto-generated. Do not edit!

// (in-package pal_startup_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class StartupStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.message = null;
      this.status = null;
      this.startup_id = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('startup_id')) {
        this.startup_id = initObj.startup_id
      }
      else {
        this.startup_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartupStatus
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [startup_id]
    bufferOffset = _serializer.string(obj.startup_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartupStatus
    let len;
    let data = new StartupStatus(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [startup_id]
    data.startup_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.message);
    length += _getByteLength(object.status);
    length += _getByteLength(object.startup_id);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pal_startup_msgs/StartupStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca78e8aec80203b6c29ca3e480879775';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name # application name
    string message # description of the status if required
    # Possible status of startup process
    string MISSING=Missing
    string RUNNING=Running
    string PENDING=Pending
    string CRASHED=Crashed
    string STOPPED=Stopped
    string ABORTED=Aborted
    string TIMEOUT=Timeout
    string MANUAL=Manual
    string FINISHED=Finished
    string status # status of the startup process for the application. Enumerated above 
    string startup_id # Id of the node starting the app (ie: control, multimedia)
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartupStatus(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.startup_id !== undefined) {
      resolved.startup_id = msg.startup_id;
    }
    else {
      resolved.startup_id = ''
    }

    return resolved;
    }
};

// Constants for message
StartupStatus.Constants = {
  MISSING: 'Missing',
  RUNNING: 'Running',
  PENDING: 'Pending',
  CRASHED: 'Crashed',
  STOPPED: 'Stopped',
  ABORTED: 'Aborted',
  TIMEOUT: 'Timeout',
  MANUAL: 'Manual',
  FINISHED: 'Finished',
}

module.exports = StartupStatus;
