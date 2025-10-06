;; Auto-generated. Do not edit!


(when (boundp 'corridor_exploration_msgs::GetPose)
  (if (not (find-package "CORRIDOR_EXPLORATION_MSGS"))
    (make-package "CORRIDOR_EXPLORATION_MSGS"))
  (shadow 'GetPose (find-package "CORRIDOR_EXPLORATION_MSGS")))
(unless (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSE")
  (make-package "CORRIDOR_EXPLORATION_MSGS::GETPOSE"))
(unless (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST")
  (make-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(unless (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSERESPONSE")
  (make-package "CORRIDOR_EXPLORATION_MSGS::GETPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GUESS*" (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(shadow '*GUESS* (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(defconstant corridor_exploration_msgs::GetPoseRequest::*GUESS* 1)
(intern "*IDENTITY*" (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(shadow '*IDENTITY* (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(defconstant corridor_exploration_msgs::GetPoseRequest::*IDENTITY* 2)
(intern "*DIRECTION*" (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(shadow '*DIRECTION* (find-package "CORRIDOR_EXPLORATION_MSGS::GETPOSEREQUEST"))
(defconstant corridor_exploration_msgs::GetPoseRequest::*DIRECTION* 3)

(defun corridor_exploration_msgs::GetPoseRequest-to-symbol (const)
  (cond
        ((= const 1) 'corridor_exploration_msgs::GetPoseRequest::*GUESS*)
        ((= const 2) 'corridor_exploration_msgs::GetPoseRequest::*IDENTITY*)
        ((= const 3) 'corridor_exploration_msgs::GetPoseRequest::*DIRECTION*)
        (t nil)))

(defclass corridor_exploration_msgs::GetPoseRequest
  :super ros::object
  :slots (_mode _pose _yaw ))

(defmethod corridor_exploration_msgs::GetPoseRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _pose __pose)
   (setq _yaw (float __yaw))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass corridor_exploration_msgs::GetPoseResponse
  :super ros::object
  :slots (_pose _message _is_open_space _success ))

(defmethod corridor_exploration_msgs::GetPoseResponse
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:message __message) (instance std_msgs::String :init))
    ((:is_open_space __is_open_space) (instance std_msgs::Bool :init))
    ((:success __success) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _message __message)
   (setq _is_open_space __is_open_space)
   (setq _success __success)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:message
   (&rest __message)
   (if (keywordp (car __message))
       (send* _message __message)
     (progn
       (if __message (setq _message (car __message)))
       _message)))
  (:is_open_space
   (&rest __is_open_space)
   (if (keywordp (car __is_open_space))
       (send* _is_open_space __is_open_space)
     (progn
       (if __is_open_space (setq _is_open_space (car __is_open_space)))
       _is_open_space)))
  (:success
   (&rest __success)
   (if (keywordp (car __success))
       (send* _success __success)
     (progn
       (if __success (setq _success (car __success)))
       _success)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; std_msgs/String _message
    (send _message :serialization-length)
    ;; std_msgs/Bool _is_open_space
    (send _is_open_space :serialization-length)
    ;; std_msgs/Bool _success
    (send _success :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; std_msgs/String _message
       (send _message :serialize s)
     ;; std_msgs/Bool _is_open_space
       (send _is_open_space :serialize s)
     ;; std_msgs/Bool _success
       (send _success :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; std_msgs/String _message
     (send _message :deserialize buf ptr-) (incf ptr- (send _message :serialization-length))
   ;; std_msgs/Bool _is_open_space
     (send _is_open_space :deserialize buf ptr-) (incf ptr- (send _is_open_space :serialization-length))
   ;; std_msgs/Bool _success
     (send _success :deserialize buf ptr-) (incf ptr- (send _success :serialization-length))
   ;;
   self)
  )

(defclass corridor_exploration_msgs::GetPose
  :super ros::object
  :slots ())

(setf (get corridor_exploration_msgs::GetPose :md5sum-) "7698893db06037c46b550b3265c90d88")
(setf (get corridor_exploration_msgs::GetPose :datatype-) "corridor_exploration_msgs/GetPose")
(setf (get corridor_exploration_msgs::GetPose :request) corridor_exploration_msgs::GetPoseRequest)
(setf (get corridor_exploration_msgs::GetPose :response) corridor_exploration_msgs::GetPoseResponse)

(defmethod corridor_exploration_msgs::GetPoseRequest
  (:response () (instance corridor_exploration_msgs::GetPoseResponse :init)))

(setf (get corridor_exploration_msgs::GetPoseRequest :md5sum-) "7698893db06037c46b550b3265c90d88")
(setf (get corridor_exploration_msgs::GetPoseRequest :datatype-) "corridor_exploration_msgs/GetPoseRequest")
(setf (get corridor_exploration_msgs::GetPoseRequest :definition-)
      "#Request
#Mode actions
uint8 GUESS = 1
uint8 IDENTITY = 2
uint8 DIRECTION = 3
uint8 mode
geometry_msgs/PoseStamped pose
float64 yaw

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
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
---
#Response

geometry_msgs/PoseStamped pose
std_msgs/String message
std_msgs/Bool is_open_space
std_msgs/Bool success



================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
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
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Bool
")

(setf (get corridor_exploration_msgs::GetPoseResponse :md5sum-) "7698893db06037c46b550b3265c90d88")
(setf (get corridor_exploration_msgs::GetPoseResponse :datatype-) "corridor_exploration_msgs/GetPoseResponse")
(setf (get corridor_exploration_msgs::GetPoseResponse :definition-)
      "#Request
#Mode actions
uint8 GUESS = 1
uint8 IDENTITY = 2
uint8 DIRECTION = 3
uint8 mode
geometry_msgs/PoseStamped pose
float64 yaw

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
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
---
#Response

geometry_msgs/PoseStamped pose
std_msgs/String message
std_msgs/Bool is_open_space
std_msgs/Bool success



================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
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
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Bool
")



(provide :corridor_exploration_msgs/GetPose "7698893db06037c46b550b3265c90d88")


