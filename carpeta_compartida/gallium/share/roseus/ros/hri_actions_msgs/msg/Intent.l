;; Auto-generated. Do not edit!


(when (boundp 'hri_actions_msgs::Intent)
  (if (not (find-package "HRI_ACTIONS_MSGS"))
    (make-package "HRI_ACTIONS_MSGS"))
  (shadow 'Intent (find-package "HRI_ACTIONS_MSGS")))
(unless (find-package "HRI_ACTIONS_MSGS::INTENT")
  (make-package "HRI_ACTIONS_MSGS::INTENT"))

(in-package "ROS")
;;//! \htmlinclude Intent.msg.html


(intern "*ENGAGE_WITH*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*ENGAGE_WITH* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*ENGAGE_WITH* "__intent_engage_with__")
(intern "*MOVE_TO*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*MOVE_TO* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*MOVE_TO* "__intent_move_to__")
(intern "*GUIDE*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*GUIDE* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*GUIDE* "__intent_guide__")
(intern "*GRAB_OBJECT*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*GRAB_OBJECT* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*GRAB_OBJECT* "__intent_grab_object__")
(intern "*BRING_OBJECT*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*BRING_OBJECT* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*BRING_OBJECT* "__intent_bring_object__")
(intern "*PLACE_OBJECT*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*PLACE_OBJECT* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*PLACE_OBJECT* "__intent_place_object__")
(intern "*GREET*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*GREET* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*GREET* "__intent_greet__")
(intern "*SAY*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*SAY* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*SAY* "__intent_say__")
(intern "*PRESENT_CONTENT*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*PRESENT_CONTENT* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*PRESENT_CONTENT* "__intent_present_content__")
(intern "*PERFORM_MOTION*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*PERFORM_MOTION* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*PERFORM_MOTION* "__intent_perform_motion__")
(intern "*START_ACTIVITY*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*START_ACTIVITY* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*START_ACTIVITY* "__intent_start_activity__")
(intern "*STOP_ACTIVITY*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*STOP_ACTIVITY* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*STOP_ACTIVITY* "__intent_stop_activity__")
(intern "*ROBOT_ITSELF*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*ROBOT_ITSELF* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*ROBOT_ITSELF* "__myself__")
(intern "*REMOTE_SUPERVISOR*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*REMOTE_SUPERVISOR* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*REMOTE_SUPERVISOR* "__remote_supervisor__")
(intern "*UNKNOWN_AGENT*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*UNKNOWN_AGENT* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*UNKNOWN_AGENT* "__unknown_agent__")
(intern "*UNKNOWN*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*UNKNOWN* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*UNKNOWN* "__unknown__")
(intern "*MODALITY_SPEECH*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*MODALITY_SPEECH* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*MODALITY_SPEECH* "__modality_speech__")
(intern "*MODALITY_MOTION*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*MODALITY_MOTION* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*MODALITY_MOTION* "__modality_motion__")
(intern "*MODALITY_TOUCHSCREEN*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*MODALITY_TOUCHSCREEN* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*MODALITY_TOUCHSCREEN* "__modality_touchscreen__")
(intern "*MODALITY_OTHER*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*MODALITY_OTHER* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*MODALITY_OTHER* "__modality_other__")
(intern "*MODALITY_INTERNAL*" (find-package "HRI_ACTIONS_MSGS::INTENT"))
(shadow '*MODALITY_INTERNAL* (find-package "HRI_ACTIONS_MSGS::INTENT"))
(defconstant hri_actions_msgs::Intent::*MODALITY_INTERNAL* "__modality_internal__")

(defun hri_actions_msgs::Intent-to-symbol (const)
  (cond
        (t nil)))

(defclass hri_actions_msgs::Intent
  :super ros::object
  :slots (_intent _data _source _modality _priority _confidence ))

(defmethod hri_actions_msgs::Intent
  (:init
   (&key
    ((:intent __intent) "")
    ((:data __data) "")
    ((:source __source) "")
    ((:modality __modality) "")
    ((:priority __priority) 0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _intent (string __intent))
   (setq _data (string __data))
   (setq _source (string __source))
   (setq _modality (string __modality))
   (setq _priority (round __priority))
   (setq _confidence (float __confidence))
   self)
  (:intent
   (&optional __intent)
   (if __intent (setq _intent __intent)) _intent)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:modality
   (&optional __modality)
   (if __modality (setq _modality __modality)) _modality)
  (:priority
   (&optional __priority)
   (if __priority (setq _priority __priority)) _priority)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; string _intent
    4 (length _intent)
    ;; string _data
    4 (length _data)
    ;; string _source
    4 (length _source)
    ;; string _modality
    4 (length _modality)
    ;; uint8 _priority
    1
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _intent
       (write-long (length _intent) s) (princ _intent s)
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;; string _source
       (write-long (length _source) s) (princ _source s)
     ;; string _modality
       (write-long (length _modality) s) (princ _modality s)
     ;; uint8 _priority
       (write-byte _priority s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _intent
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _intent (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _modality
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _modality (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _priority
     (setq _priority (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hri_actions_msgs::Intent :md5sum-) "d183d4f69bb3932549578b85c6f6c1b0")
(setf (get hri_actions_msgs::Intent :datatype-) "hri_actions_msgs/Intent")
(setf (get hri_actions_msgs::Intent :definition-)
      "# An intent is an abstract description of an operation to be performed by the
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
# - \"I want you to go to the kitchen\":
#   - intent: `move_to`
#   - data: {\"goal\":\"kitchen_1\"}
#
# - \"Can you take the groceries to Luke in the kitchen?\"
#   - intent: `bring`
#   - data: {\"object\": \"groceries\",
#            \"goal\":\"kitchen_1\",
#            \"recipient\": \"person_luke\"}

##########################################################################
#  Intent name
#
# A string describing the intent.
#
# Where suitable, the intent SHOULD be one of the constant defined below.
# However, we recognise that the list of intents is possibly large. Therefore,
# custom strings are also permissible. If you believe your intent name should
# be standardised and added to the list of pre-defined intents, fill the
# corresponding entry in the \"thematic roles\" table below and submit a pull
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

")



(provide :hri_actions_msgs/Intent "d183d4f69bb3932549578b85c6f6c1b0")


