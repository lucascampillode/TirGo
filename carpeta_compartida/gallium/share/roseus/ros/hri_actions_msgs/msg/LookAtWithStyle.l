;; Auto-generated. Do not edit!


(when (boundp 'hri_actions_msgs::LookAtWithStyle)
  (if (not (find-package "HRI_ACTIONS_MSGS"))
    (make-package "HRI_ACTIONS_MSGS"))
  (shadow 'LookAtWithStyle (find-package "HRI_ACTIONS_MSGS")))
(unless (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE")
  (make-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))

(in-package "ROS")
;;//! \htmlinclude LookAtWithStyle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*DEFAULT*" (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))
(shadow '*DEFAULT* (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))
(defconstant hri_actions_msgs::LookAtWithStyle::*DEFAULT* 0)
(intern "*EYES_ONLY*" (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))
(shadow '*EYES_ONLY* (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))
(defconstant hri_actions_msgs::LookAtWithStyle::*EYES_ONLY* 1)
(intern "*HEAD_ONLY*" (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))
(shadow '*HEAD_ONLY* (find-package "HRI_ACTIONS_MSGS::LOOKATWITHSTYLE"))
(defconstant hri_actions_msgs::LookAtWithStyle::*HEAD_ONLY* 2)

(defun hri_actions_msgs::LookAtWithStyle-to-symbol (const)
  (cond
        ((= const 0) 'hri_actions_msgs::LookAtWithStyle::*DEFAULT*)
        ((= const 1) 'hri_actions_msgs::LookAtWithStyle::*EYES_ONLY*)
        ((= const 2) 'hri_actions_msgs::LookAtWithStyle::*HEAD_ONLY*)
        (t nil)))

(defclass hri_actions_msgs::LookAtWithStyle
  :super ros::object
  :slots (_style _target ))

(defmethod hri_actions_msgs::LookAtWithStyle
  (:init
   (&key
    ((:style __style) 0)
    ((:target __target) (instance geometry_msgs::PointStamped :init))
    )
   (send-super :init)
   (setq _style (round __style))
   (setq _target __target)
   self)
  (:style
   (&optional __style)
   (if __style (setq _style __style)) _style)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:serialization-length
   ()
   (+
    ;; int8 _style
    1
    ;; geometry_msgs/PointStamped _target
    (send _target :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _style
       (write-byte _style s)
     ;; geometry_msgs/PointStamped _target
       (send _target :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _style
     (setq _style (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _style 127) (setq _style (- _style 256)))
   ;; geometry_msgs/PointStamped _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;;
   self)
  )

(setf (get hri_actions_msgs::LookAtWithStyle :md5sum-) "1997f720101069b0d5f7306d6cb70706")
(setf (get hri_actions_msgs::LookAtWithStyle :datatype-) "hri_actions_msgs/LookAtWithStyle")
(setf (get hri_actions_msgs::LookAtWithStyle :definition-)
      "# Specify the target of the robot's gaze as a 3D point. A desired 'style'
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

")



(provide :hri_actions_msgs/LookAtWithStyle "1997f720101069b0d5f7306d6cb70706")


