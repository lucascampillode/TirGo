;; Auto-generated. Do not edit!


(when (boundp 'amcl::SetInitialPose)
  (if (not (find-package "AMCL"))
    (make-package "AMCL"))
  (shadow 'SetInitialPose (find-package "AMCL")))
(unless (find-package "AMCL::SETINITIALPOSE")
  (make-package "AMCL::SETINITIALPOSE"))
(unless (find-package "AMCL::SETINITIALPOSEREQUEST")
  (make-package "AMCL::SETINITIALPOSEREQUEST"))
(unless (find-package "AMCL::SETINITIALPOSERESPONSE")
  (make-package "AMCL::SETINITIALPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass amcl::SetInitialPoseRequest
  :super ros::object
  :slots (_initialpose ))

(defmethod amcl::SetInitialPoseRequest
  (:init
   (&key
    ((:initialpose __initialpose) (instance geometry_msgs::PoseWithCovarianceStamped :init))
    )
   (send-super :init)
   (setq _initialpose __initialpose)
   self)
  (:initialpose
   (&rest __initialpose)
   (if (keywordp (car __initialpose))
       (send* _initialpose __initialpose)
     (progn
       (if __initialpose (setq _initialpose (car __initialpose)))
       _initialpose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseWithCovarianceStamped _initialpose
    (send _initialpose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseWithCovarianceStamped _initialpose
       (send _initialpose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseWithCovarianceStamped _initialpose
     (send _initialpose :deserialize buf ptr-) (incf ptr- (send _initialpose :serialization-length))
   ;;
   self)
  )

(defclass amcl::SetInitialPoseResponse
  :super ros::object
  :slots (_success ))

(defmethod amcl::SetInitialPoseResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass amcl::SetInitialPose
  :super ros::object
  :slots ())

(setf (get amcl::SetInitialPose :md5sum-) "76084d1d94567bfd99de36dd17bd286b")
(setf (get amcl::SetInitialPose :datatype-) "amcl/SetInitialPose")
(setf (get amcl::SetInitialPose :request) amcl::SetInitialPoseRequest)
(setf (get amcl::SetInitialPose :response) amcl::SetInitialPoseResponse)

(defmethod amcl::SetInitialPoseRequest
  (:response () (instance amcl::SetInitialPoseResponse :init)))

(setf (get amcl::SetInitialPoseRequest :md5sum-) "76084d1d94567bfd99de36dd17bd286b")
(setf (get amcl::SetInitialPoseRequest :datatype-) "amcl/SetInitialPoseRequest")
(setf (get amcl::SetInitialPoseRequest :definition-)
      "geometry_msgs/PoseWithCovarianceStamped initialpose

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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
bool success
")

(setf (get amcl::SetInitialPoseResponse :md5sum-) "76084d1d94567bfd99de36dd17bd286b")
(setf (get amcl::SetInitialPoseResponse :datatype-) "amcl/SetInitialPoseResponse")
(setf (get amcl::SetInitialPoseResponse :definition-)
      "geometry_msgs/PoseWithCovarianceStamped initialpose

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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
bool success
")



(provide :amcl/SetInitialPose "76084d1d94567bfd99de36dd17bd286b")


