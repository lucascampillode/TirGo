// Auto-generated. Do not edit!

// (in-package hri_actions_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Intent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.intent = null;
      this.data = null;
      this.source = null;
      this.modality = null;
      this.priority = null;
      this.confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('intent')) {
        this.intent = initObj.intent
      }
      else {
        this.intent = '';
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = '';
      }
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = '';
      }
      if (initObj.hasOwnProperty('modality')) {
        this.modality = initObj.modality
      }
      else {
        this.modality = '';
      }
      if (initObj.hasOwnProperty('priority')) {
        this.priority = initObj.priority
      }
      else {
        this.priority = 0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Intent
    // Serialize message field [intent]
    bufferOffset = _serializer.string(obj.intent, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _serializer.string(obj.data, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.string(obj.source, buffer, bufferOffset);
    // Serialize message field [modality]
    bufferOffset = _serializer.string(obj.modality, buffer, bufferOffset);
    // Serialize message field [priority]
    bufferOffset = _serializer.uint8(obj.priority, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Intent
    let len;
    let data = new Intent(null);
    // Deserialize message field [intent]
    data.intent = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [modality]
    data.modality = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [priority]
    data.priority = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.intent);
    length += _getByteLength(object.data);
    length += _getByteLength(object.source);
    length += _getByteLength(object.modality);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hri_actions_msgs/Intent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd183d4f69bb3932549578b85c6f6c1b0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # An intent is an abstract description of an operation to be performed by the
    # robot.
    #
    # While inspired by the Android intents
    # (https://developer.android.com/reference/android/content/Intent), ROS intents
    # are primarily designed to capture user-initiated intents. For instance, a
    # button click on a touchscreen, the result of a chatbot-based verbal
    # interaction, a command started by a remote user interface.
    #
    # Intents comprise of two mandatory fields: the `intent`, which should be one
    # of the available predefined intents, and the `data` which must be a JSON
    # object containing the data required to fully instantiate the intent.  While
    # the exact content of the JSON object is up to the application, the following
    # standard keys can be used to specify the intent's semantic roles (also called
    # thematic roles or theta roles):
    #
    # - `agent`: the agent expected to perform the intent (if omitted, the robot
    # itself is assumed)
    # - `object` (also named *theme* or *patient* in the linguistics literature): entity
    # undergoing the effect of the intent
    # - `goal`: entity towards which the intent is directed or moves
    # - `recipient`: entity that receives the object (denoted as the `theme`)
    #
    # Examples:
    # 
    # - "I want you to go to the kitchen":
    #   - intent: `move_to`
    #   - data: {"goal":"kitchen_1"}
    #
    # - "Can you take the groceries to Luke in the kitchen?"
    #   - intent: `bring`
    #   - data: {"object": "groceries",
    #            "goal":"kitchen_1",
    #            "recipient": "person_luke"}
    
    ##########################################################################
    #  Intent name
    #
    # A string describing the intent.
    #
    # Where suitable, the intent SHOULD be one of the constant defined below.
    # However, we recognise that the list of intents is possibly large. Therefore,
    # custom strings are also permissible. If you believe your intent name should
    # be standardised and added to the list of pre-defined intents, fill the
    # corresponding entry in the "thematic roles" table below and submit a pull
    # request on this repository.
    string intent
    
    # 'intent' constants
    # see table below for a short description of each.
    
    string ENGAGE_WITH = __intent_engage_with__
    string MOVE_TO = __intent_move_to__
    string GUIDE = __intent_guide__
    string GRAB_OBJECT = __intent_grab_object__
    string BRING_OBJECT = __intent_bring_object__
    string PLACE_OBJECT = __intent_place_object__
    string GREET = __intent_greet__
    string SAY = __intent_say__
    string PRESENT_CONTENT = __intent_present_content__
    string PERFORM_MOTION = __intent_perform_motion__
    string START_ACTIVITY = __intent_start_activity__
    string STOP_ACTIVITY = __intent_stop_activity__
    
    ##########################################################################
    #  Intent data
    #
    # a JSON object containing the data required to fully instantiate the intent.
    # The keys of the object should be one of the thematic role defined above, or
    # the general `other_data`.
    #
    # Each intent defines a specific set of required and optional thematic roles,
    # listed in the following table (note that the `agent` role can be optionally
    # added to all intents, and is omitted from the table for clarity):
    #
    # .. [please keep the RST table syntax below, as it is used to automatically parse the list of intents by some nodes]#
    # .. BEGIN_INTENTS_TABLE
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | **Intent**       | **Description**                                                             | **Required thematic roles**                                 | **Optional thematic roles**                                               |
    # +==================+=============================================================================+=============================================================+===========================================================================+
    # | `ENGAGE_WITH`    | an agent wants to engage with another one                                   | - `recipient`                                               |                                                                           |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `MOVE_TO`        | navigates to a specific location                                            | - `goal`                                                    |                                                                           |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `GUIDE`          | guides someone somewhere                                                    | - `goal`                                                    |                                                                           |
    # |                  |                                                                             | - `recipient`                                               |                                                                           |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `GRAB_OBJECT`    | pick-up a specific object                                                   | - `object`                                                  |                                                                           |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `BRING_OBJECT`   | bring a specific object to a specific place                                 | - `object`                                                  |                                                                           |
    # |                  |                                                                             | - `recipient`                                               |                                                                           |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `PLACE_OBJECT`   | put an object on a support (eg a table)                                     | - `recipient`                                               | - `object` (only required if more that one object could be placed)        |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `GREET`          | greet an agent                                                              | - `recipient`                                               |                                                                           |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `SAY`            | says some text, optionally annotated with gestures or expressions           | - `object` (the text to say)                                | - `recipient`                                                             |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `PRESENT_CONTENT`| present (via a screen, pre-recorded text...) predefined content             | - `object` (the content identifier)                         | - `recipient`                                                             |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `PERFORM_MOTION` | performs a motion (eg, a dance or a specific gesture like pointing, waving) | - `object` (the system-specific name of the motion/gesture) | - `recipient`                                                             |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `START_ACTIVITY` | start a scripted behaviour/activity                                         | - `object` (the name of the activity)                       | - any additional parameter required to start the activity                 |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # | `STOP_ACTIVITY`  | request cancelation of an activity                                          |                                                             | - `object` (the name of the activity. If unset, current main activity)    |
    # +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+
    # .. END_INTENTS_TABLE
    #
    string data
    
    ##########################################################################
    #  Intent source
    #
    # a string describing the source of the intent. This is *not* the node
    # which published the intent, but instead the actual agent who expressed the
    # intent/command/desire.
    # `source` can be either one of the constant below, or the specific id of the
    # person/agent expressing the intent. In a REP-155 compliant system, this ID
    # must be the person ID of the agent.
    string source
    
    # 'source' constants:
    
    # for intents originating from the robot itself
    string ROBOT_ITSELF = __myself__
    # for intents originating from a external robot control system (for instance, a remote control tablet)
    string REMOTE_SUPERVISOR = __remote_supervisor__
    # for intents coming from an agent interacting with the robot, but not uniquely
    # identified
    string UNKNOWN_AGENT = __unknown_agent__
    # for unknown sources
    string UNKNOWN = __unknown__
    
    
    ##########################################################################
    #  Intent modality
    #
    # modality used to originally convey the intent: verbal, via the touchscreen,
    # via a gesture...
    # The special modality MODALITY_INTERNAL must be used for intents coming for the
    # robot's internal processes (eg 'battery low, I need to charge')
    # MUST be one of the MODALITY_ constant below.
    string modality
    
    # 'modality' constants:
    
    string MODALITY_SPEECH = __modality_speech__
    # both gestures (eg waving) and displacement (eg approaching)
    string MODALITY_MOTION = __modality_motion__
    string MODALITY_TOUCHSCREEN = __modality_touchscreen__
    string MODALITY_OTHER = __modality_other__
    string MODALITY_INTERNAL = __modality_internal__
    
    
    ##########################################################################
    #  Intent priority
    #
    # the priority of this intent. This MIGHT be used as a hint by the robot's task
    # scheduler, however is scheduler is *not* forced to respect this priority
    # level. 0 is the lowest priority, 128 is the default priority, 255 is the
    # highest priority.
    uint8 priority
    
    ##########################################################################
    #  Intent confidence
    #
    # a value between 0.0 (no confidence) and 1.0 (full confidence) that the intent
    # was correctly perceived and interpreted.
    #
    # For instance, a 'waving' gesture could be interpreted as an implicit request
    # from a user for the robot to greet back or engage. As this interpretation is
    # not certain, the confidence of the intent may be below 1.0.
    float32 confidence
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Intent(null);
    if (msg.intent !== undefined) {
      resolved.intent = msg.intent;
    }
    else {
      resolved.intent = ''
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = ''
    }

    if (msg.source !== undefined) {
      resolved.source = msg.source;
    }
    else {
      resolved.source = ''
    }

    if (msg.modality !== undefined) {
      resolved.modality = msg.modality;
    }
    else {
      resolved.modality = ''
    }

    if (msg.priority !== undefined) {
      resolved.priority = msg.priority;
    }
    else {
      resolved.priority = 0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    return resolved;
    }
};

// Constants for message
Intent.Constants = {
  ENGAGE_WITH: '__intent_engage_with__',
  MOVE_TO: '__intent_move_to__',
  GUIDE: '__intent_guide__',
  GRAB_OBJECT: '__intent_grab_object__',
  BRING_OBJECT: '__intent_bring_object__',
  PLACE_OBJECT: '__intent_place_object__',
  GREET: '__intent_greet__',
  SAY: '__intent_say__',
  PRESENT_CONTENT: '__intent_present_content__',
  PERFORM_MOTION: '__intent_perform_motion__',
  START_ACTIVITY: '__intent_start_activity__',
  STOP_ACTIVITY: '__intent_stop_activity__',
  ROBOT_ITSELF: '__myself__',
  REMOTE_SUPERVISOR: '__remote_supervisor__',
  UNKNOWN_AGENT: '__unknown_agent__',
  UNKNOWN: '__unknown__',
  MODALITY_SPEECH: '__modality_speech__',
  MODALITY_MOTION: '__modality_motion__',
  MODALITY_TOUCHSCREEN: '__modality_touchscreen__',
  MODALITY_OTHER: '__modality_other__',
  MODALITY_INTERNAL: '__modality_internal__',
}

module.exports = Intent;
