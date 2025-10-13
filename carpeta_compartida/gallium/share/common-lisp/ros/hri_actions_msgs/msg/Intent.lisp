; Auto-generated. Do not edit!


(cl:in-package hri_actions_msgs-msg)


;//! \htmlinclude Intent.msg.html

(cl:defclass <Intent> (roslisp-msg-protocol:ros-message)
  ((intent
    :reader intent
    :initarg :intent
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform "")
   (source
    :reader source
    :initarg :source
    :type cl:string
    :initform "")
   (modality
    :reader modality
    :initarg :modality
    :type cl:string
    :initform "")
   (priority
    :reader priority
    :initarg :priority
    :type cl:fixnum
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass Intent (<Intent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Intent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Intent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hri_actions_msgs-msg:<Intent> is deprecated: use hri_actions_msgs-msg:Intent instead.")))

(cl:ensure-generic-function 'intent-val :lambda-list '(m))
(cl:defmethod intent-val ((m <Intent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:intent-val is deprecated.  Use hri_actions_msgs-msg:intent instead.")
  (intent m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Intent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:data-val is deprecated.  Use hri_actions_msgs-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <Intent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:source-val is deprecated.  Use hri_actions_msgs-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'modality-val :lambda-list '(m))
(cl:defmethod modality-val ((m <Intent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:modality-val is deprecated.  Use hri_actions_msgs-msg:modality instead.")
  (modality m))

(cl:ensure-generic-function 'priority-val :lambda-list '(m))
(cl:defmethod priority-val ((m <Intent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:priority-val is deprecated.  Use hri_actions_msgs-msg:priority instead.")
  (priority m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <Intent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:confidence-val is deprecated.  Use hri_actions_msgs-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Intent>)))
    "Constants for message type '<Intent>"
  '((:ENGAGE_WITH . "__intent_engage_with__")
    (:MOVE_TO . "__intent_move_to__")
    (:GUIDE . "__intent_guide__")
    (:GRAB_OBJECT . "__intent_grab_object__")
    (:BRING_OBJECT . "__intent_bring_object__")
    (:PLACE_OBJECT . "__intent_place_object__")
    (:GREET . "__intent_greet__")
    (:SAY . "__intent_say__")
    (:PRESENT_CONTENT . "__intent_present_content__")
    (:PERFORM_MOTION . "__intent_perform_motion__")
    (:START_ACTIVITY . "__intent_start_activity__")
    (:STOP_ACTIVITY . "__intent_stop_activity__")
    (:ROBOT_ITSELF . "__myself__")
    (:REMOTE_SUPERVISOR . "__remote_supervisor__")
    (:UNKNOWN_AGENT . "__unknown_agent__")
    (:UNKNOWN . "__unknown__")
    (:MODALITY_SPEECH . "__modality_speech__")
    (:MODALITY_MOTION . "__modality_motion__")
    (:MODALITY_TOUCHSCREEN . "__modality_touchscreen__")
    (:MODALITY_OTHER . "__modality_other__")
    (:MODALITY_INTERNAL . "__modality_internal__"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Intent)))
    "Constants for message type 'Intent"
  '((:ENGAGE_WITH . "__intent_engage_with__")
    (:MOVE_TO . "__intent_move_to__")
    (:GUIDE . "__intent_guide__")
    (:GRAB_OBJECT . "__intent_grab_object__")
    (:BRING_OBJECT . "__intent_bring_object__")
    (:PLACE_OBJECT . "__intent_place_object__")
    (:GREET . "__intent_greet__")
    (:SAY . "__intent_say__")
    (:PRESENT_CONTENT . "__intent_present_content__")
    (:PERFORM_MOTION . "__intent_perform_motion__")
    (:START_ACTIVITY . "__intent_start_activity__")
    (:STOP_ACTIVITY . "__intent_stop_activity__")
    (:ROBOT_ITSELF . "__myself__")
    (:REMOTE_SUPERVISOR . "__remote_supervisor__")
    (:UNKNOWN_AGENT . "__unknown_agent__")
    (:UNKNOWN . "__unknown__")
    (:MODALITY_SPEECH . "__modality_speech__")
    (:MODALITY_MOTION . "__modality_motion__")
    (:MODALITY_TOUCHSCREEN . "__modality_touchscreen__")
    (:MODALITY_OTHER . "__modality_other__")
    (:MODALITY_INTERNAL . "__modality_internal__"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Intent>) ostream)
  "Serializes a message object of type '<Intent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'intent))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'intent))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'modality))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'modality))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'priority)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Intent>) istream)
  "Deserializes a message object of type '<Intent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'intent) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'intent) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modality) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'modality) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'priority)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Intent>)))
  "Returns string type for a message object of type '<Intent>"
  "hri_actions_msgs/Intent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Intent)))
  "Returns string type for a message object of type 'Intent"
  "hri_actions_msgs/Intent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Intent>)))
  "Returns md5sum for a message object of type '<Intent>"
  "d183d4f69bb3932549578b85c6f6c1b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Intent)))
  "Returns md5sum for a message object of type 'Intent"
  "d183d4f69bb3932549578b85c6f6c1b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Intent>)))
  "Returns full string definition for message of type '<Intent>"
  (cl:format cl:nil "# An intent is an abstract description of an operation to be performed by the~%# robot.~%#~%# While inspired by the Android intents~%# (https://developer.android.com/reference/android/content/Intent), ROS intents~%# are primarily designed to capture user-initiated intents. For instance, a~%# button click on a touchscreen, the result of a chatbot-based verbal~%# interaction, a command started by a remote user interface.~%#~%# Intents comprise of two mandatory fields: the `intent`, which should be one~%# of the available predefined intents, and the `data` which must be a JSON~%# object containing the data required to fully instantiate the intent.  While~%# the exact content of the JSON object is up to the application, the following~%# standard keys can be used to specify the intent's semantic roles (also called~%# thematic roles or theta roles):~%#~%# - `agent`: the agent expected to perform the intent (if omitted, the robot~%# itself is assumed)~%# - `object` (also named *theme* or *patient* in the linguistics literature): entity~%# undergoing the effect of the intent~%# - `goal`: entity towards which the intent is directed or moves~%# - `recipient`: entity that receives the object (denoted as the `theme`)~%#~%# Examples:~%# ~%# - \"I want you to go to the kitchen\":~%#   - intent: `move_to`~%#   - data: {\"goal\":\"kitchen_1\"}~%#~%# - \"Can you take the groceries to Luke in the kitchen?\"~%#   - intent: `bring`~%#   - data: {\"object\": \"groceries\",~%#            \"goal\":\"kitchen_1\",~%#            \"recipient\": \"person_luke\"}~%~%##########################################################################~%#  Intent name~%#~%# A string describing the intent.~%#~%# Where suitable, the intent SHOULD be one of the constant defined below.~%# However, we recognise that the list of intents is possibly large. Therefore,~%# custom strings are also permissible. If you believe your intent name should~%# be standardised and added to the list of pre-defined intents, fill the~%# corresponding entry in the \"thematic roles\" table below and submit a pull~%# request on this repository.~%string intent~%~%# 'intent' constants~%# see table below for a short description of each.~%~%string ENGAGE_WITH = __intent_engage_with__~%string MOVE_TO = __intent_move_to__~%string GUIDE = __intent_guide__~%string GRAB_OBJECT = __intent_grab_object__~%string BRING_OBJECT = __intent_bring_object__~%string PLACE_OBJECT = __intent_place_object__~%string GREET = __intent_greet__~%string SAY = __intent_say__~%string PRESENT_CONTENT = __intent_present_content__~%string PERFORM_MOTION = __intent_perform_motion__~%string START_ACTIVITY = __intent_start_activity__~%string STOP_ACTIVITY = __intent_stop_activity__~%~%##########################################################################~%#  Intent data~%#~%# a JSON object containing the data required to fully instantiate the intent.~%# The keys of the object should be one of the thematic role defined above, or~%# the general `other_data`.~%#~%# Each intent defines a specific set of required and optional thematic roles,~%# listed in the following table (note that the `agent` role can be optionally~%# added to all intents, and is omitted from the table for clarity):~%#~%# .. [please keep the RST table syntax below, as it is used to automatically parse the list of intents by some nodes]#~%# .. BEGIN_INTENTS_TABLE~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | **Intent**       | **Description**                                                             | **Required thematic roles**                                 | **Optional thematic roles**                                               |~%# +==================+=============================================================================+=============================================================+===========================================================================+~%# | `ENGAGE_WITH`    | an agent wants to engage with another one                                   | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `MOVE_TO`        | navigates to a specific location                                            | - `goal`                                                    |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `GUIDE`          | guides someone somewhere                                                    | - `goal`                                                    |                                                                           |~%# |                  |                                                                             | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `GRAB_OBJECT`    | pick-up a specific object                                                   | - `object`                                                  |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `BRING_OBJECT`   | bring a specific object to a specific place                                 | - `object`                                                  |                                                                           |~%# |                  |                                                                             | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `PLACE_OBJECT`   | put an object on a support (eg a table)                                     | - `recipient`                                               | - `object` (only required if more that one object could be placed)        |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `GREET`          | greet an agent                                                              | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `SAY`            | says some text, optionally annotated with gestures or expressions           | - `object` (the text to say)                                | - `recipient`                                                             |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `PRESENT_CONTENT`| present (via a screen, pre-recorded text...) predefined content             | - `object` (the content identifier)                         | - `recipient`                                                             |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `PERFORM_MOTION` | performs a motion (eg, a dance or a specific gesture like pointing, waving) | - `object` (the system-specific name of the motion/gesture) | - `recipient`                                                             |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `START_ACTIVITY` | start a scripted behaviour/activity                                         | - `object` (the name of the activity)                       | - any additional parameter required to start the activity                 |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `STOP_ACTIVITY`  | request cancelation of an activity                                          |                                                             | - `object` (the name of the activity. If unset, current main activity)    |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# .. END_INTENTS_TABLE~%#~%string data~%~%##########################################################################~%#  Intent source~%#~%# a string describing the source of the intent. This is *not* the node~%# which published the intent, but instead the actual agent who expressed the~%# intent/command/desire.~%# `source` can be either one of the constant below, or the specific id of the~%# person/agent expressing the intent. In a REP-155 compliant system, this ID~%# must be the person ID of the agent.~%string source~%~%# 'source' constants:~%~%# for intents originating from the robot itself~%string ROBOT_ITSELF = __myself__~%# for intents originating from a external robot control system (for instance, a remote control tablet)~%string REMOTE_SUPERVISOR = __remote_supervisor__~%# for intents coming from an agent interacting with the robot, but not uniquely~%# identified~%string UNKNOWN_AGENT = __unknown_agent__~%# for unknown sources~%string UNKNOWN = __unknown__~%~%~%##########################################################################~%#  Intent modality~%#~%# modality used to originally convey the intent: verbal, via the touchscreen,~%# via a gesture...~%# The special modality MODALITY_INTERNAL must be used for intents coming for the~%# robot's internal processes (eg 'battery low, I need to charge')~%# MUST be one of the MODALITY_ constant below.~%string modality~%~%# 'modality' constants:~%~%string MODALITY_SPEECH = __modality_speech__~%# both gestures (eg waving) and displacement (eg approaching)~%string MODALITY_MOTION = __modality_motion__~%string MODALITY_TOUCHSCREEN = __modality_touchscreen__~%string MODALITY_OTHER = __modality_other__~%string MODALITY_INTERNAL = __modality_internal__~%~%~%##########################################################################~%#  Intent priority~%#~%# the priority of this intent. This MIGHT be used as a hint by the robot's task~%# scheduler, however is scheduler is *not* forced to respect this priority~%# level. 0 is the lowest priority, 128 is the default priority, 255 is the~%# highest priority.~%uint8 priority~%~%##########################################################################~%#  Intent confidence~%#~%# a value between 0.0 (no confidence) and 1.0 (full confidence) that the intent~%# was correctly perceived and interpreted.~%#~%# For instance, a 'waving' gesture could be interpreted as an implicit request~%# from a user for the robot to greet back or engage. As this interpretation is~%# not certain, the confidence of the intent may be below 1.0.~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Intent)))
  "Returns full string definition for message of type 'Intent"
  (cl:format cl:nil "# An intent is an abstract description of an operation to be performed by the~%# robot.~%#~%# While inspired by the Android intents~%# (https://developer.android.com/reference/android/content/Intent), ROS intents~%# are primarily designed to capture user-initiated intents. For instance, a~%# button click on a touchscreen, the result of a chatbot-based verbal~%# interaction, a command started by a remote user interface.~%#~%# Intents comprise of two mandatory fields: the `intent`, which should be one~%# of the available predefined intents, and the `data` which must be a JSON~%# object containing the data required to fully instantiate the intent.  While~%# the exact content of the JSON object is up to the application, the following~%# standard keys can be used to specify the intent's semantic roles (also called~%# thematic roles or theta roles):~%#~%# - `agent`: the agent expected to perform the intent (if omitted, the robot~%# itself is assumed)~%# - `object` (also named *theme* or *patient* in the linguistics literature): entity~%# undergoing the effect of the intent~%# - `goal`: entity towards which the intent is directed or moves~%# - `recipient`: entity that receives the object (denoted as the `theme`)~%#~%# Examples:~%# ~%# - \"I want you to go to the kitchen\":~%#   - intent: `move_to`~%#   - data: {\"goal\":\"kitchen_1\"}~%#~%# - \"Can you take the groceries to Luke in the kitchen?\"~%#   - intent: `bring`~%#   - data: {\"object\": \"groceries\",~%#            \"goal\":\"kitchen_1\",~%#            \"recipient\": \"person_luke\"}~%~%##########################################################################~%#  Intent name~%#~%# A string describing the intent.~%#~%# Where suitable, the intent SHOULD be one of the constant defined below.~%# However, we recognise that the list of intents is possibly large. Therefore,~%# custom strings are also permissible. If you believe your intent name should~%# be standardised and added to the list of pre-defined intents, fill the~%# corresponding entry in the \"thematic roles\" table below and submit a pull~%# request on this repository.~%string intent~%~%# 'intent' constants~%# see table below for a short description of each.~%~%string ENGAGE_WITH = __intent_engage_with__~%string MOVE_TO = __intent_move_to__~%string GUIDE = __intent_guide__~%string GRAB_OBJECT = __intent_grab_object__~%string BRING_OBJECT = __intent_bring_object__~%string PLACE_OBJECT = __intent_place_object__~%string GREET = __intent_greet__~%string SAY = __intent_say__~%string PRESENT_CONTENT = __intent_present_content__~%string PERFORM_MOTION = __intent_perform_motion__~%string START_ACTIVITY = __intent_start_activity__~%string STOP_ACTIVITY = __intent_stop_activity__~%~%##########################################################################~%#  Intent data~%#~%# a JSON object containing the data required to fully instantiate the intent.~%# The keys of the object should be one of the thematic role defined above, or~%# the general `other_data`.~%#~%# Each intent defines a specific set of required and optional thematic roles,~%# listed in the following table (note that the `agent` role can be optionally~%# added to all intents, and is omitted from the table for clarity):~%#~%# .. [please keep the RST table syntax below, as it is used to automatically parse the list of intents by some nodes]#~%# .. BEGIN_INTENTS_TABLE~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | **Intent**       | **Description**                                                             | **Required thematic roles**                                 | **Optional thematic roles**                                               |~%# +==================+=============================================================================+=============================================================+===========================================================================+~%# | `ENGAGE_WITH`    | an agent wants to engage with another one                                   | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `MOVE_TO`        | navigates to a specific location                                            | - `goal`                                                    |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `GUIDE`          | guides someone somewhere                                                    | - `goal`                                                    |                                                                           |~%# |                  |                                                                             | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `GRAB_OBJECT`    | pick-up a specific object                                                   | - `object`                                                  |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `BRING_OBJECT`   | bring a specific object to a specific place                                 | - `object`                                                  |                                                                           |~%# |                  |                                                                             | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `PLACE_OBJECT`   | put an object on a support (eg a table)                                     | - `recipient`                                               | - `object` (only required if more that one object could be placed)        |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `GREET`          | greet an agent                                                              | - `recipient`                                               |                                                                           |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `SAY`            | says some text, optionally annotated with gestures or expressions           | - `object` (the text to say)                                | - `recipient`                                                             |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `PRESENT_CONTENT`| present (via a screen, pre-recorded text...) predefined content             | - `object` (the content identifier)                         | - `recipient`                                                             |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `PERFORM_MOTION` | performs a motion (eg, a dance or a specific gesture like pointing, waving) | - `object` (the system-specific name of the motion/gesture) | - `recipient`                                                             |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `START_ACTIVITY` | start a scripted behaviour/activity                                         | - `object` (the name of the activity)                       | - any additional parameter required to start the activity                 |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# | `STOP_ACTIVITY`  | request cancelation of an activity                                          |                                                             | - `object` (the name of the activity. If unset, current main activity)    |~%# +------------------+-----------------------------------------------------------------------------+-------------------------------------------------------------+---------------------------------------------------------------------------+~%# .. END_INTENTS_TABLE~%#~%string data~%~%##########################################################################~%#  Intent source~%#~%# a string describing the source of the intent. This is *not* the node~%# which published the intent, but instead the actual agent who expressed the~%# intent/command/desire.~%# `source` can be either one of the constant below, or the specific id of the~%# person/agent expressing the intent. In a REP-155 compliant system, this ID~%# must be the person ID of the agent.~%string source~%~%# 'source' constants:~%~%# for intents originating from the robot itself~%string ROBOT_ITSELF = __myself__~%# for intents originating from a external robot control system (for instance, a remote control tablet)~%string REMOTE_SUPERVISOR = __remote_supervisor__~%# for intents coming from an agent interacting with the robot, but not uniquely~%# identified~%string UNKNOWN_AGENT = __unknown_agent__~%# for unknown sources~%string UNKNOWN = __unknown__~%~%~%##########################################################################~%#  Intent modality~%#~%# modality used to originally convey the intent: verbal, via the touchscreen,~%# via a gesture...~%# The special modality MODALITY_INTERNAL must be used for intents coming for the~%# robot's internal processes (eg 'battery low, I need to charge')~%# MUST be one of the MODALITY_ constant below.~%string modality~%~%# 'modality' constants:~%~%string MODALITY_SPEECH = __modality_speech__~%# both gestures (eg waving) and displacement (eg approaching)~%string MODALITY_MOTION = __modality_motion__~%string MODALITY_TOUCHSCREEN = __modality_touchscreen__~%string MODALITY_OTHER = __modality_other__~%string MODALITY_INTERNAL = __modality_internal__~%~%~%##########################################################################~%#  Intent priority~%#~%# the priority of this intent. This MIGHT be used as a hint by the robot's task~%# scheduler, however is scheduler is *not* forced to respect this priority~%# level. 0 is the lowest priority, 128 is the default priority, 255 is the~%# highest priority.~%uint8 priority~%~%##########################################################################~%#  Intent confidence~%#~%# a value between 0.0 (no confidence) and 1.0 (full confidence) that the intent~%# was correctly perceived and interpreted.~%#~%# For instance, a 'waving' gesture could be interpreted as an implicit request~%# from a user for the robot to greet back or engage. As this interpretation is~%# not certain, the confidence of the intent may be below 1.0.~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Intent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'intent))
     4 (cl:length (cl:slot-value msg 'data))
     4 (cl:length (cl:slot-value msg 'source))
     4 (cl:length (cl:slot-value msg 'modality))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Intent>))
  "Converts a ROS message object to a list"
  (cl:list 'Intent
    (cl:cons ':intent (intent msg))
    (cl:cons ':data (data msg))
    (cl:cons ':source (source msg))
    (cl:cons ':modality (modality msg))
    (cl:cons ':priority (priority msg))
    (cl:cons ':confidence (confidence msg))
))
