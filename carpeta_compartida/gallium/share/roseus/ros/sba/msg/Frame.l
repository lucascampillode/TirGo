;; Auto-generated. Do not edit!


(when (boundp 'sba::Frame)
  (if (not (find-package "SBA"))
    (make-package "SBA"))
  (shadow 'Frame (find-package "SBA")))
(unless (find-package "SBA::FRAME")
  (make-package "SBA::FRAME"))

(in-package "ROS")
;;//! \htmlinclude Frame.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sba::Frame
  :super ros::object
  :slots (_header _nodes _points _projections ))

(defmethod sba::Frame
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:nodes __nodes) ())
    ((:points __points) ())
    ((:projections __projections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _nodes __nodes)
   (setq _points __points)
   (setq _projections __projections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:nodes
   (&rest __nodes)
   (if (keywordp (car __nodes))
       (send* _nodes __nodes)
     (progn
       (if __nodes (setq _nodes (car __nodes)))
       _nodes)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:projections
   (&rest __projections)
   (if (keywordp (car __projections))
       (send* _projections __projections)
     (progn
       (if __projections (setq _projections (car __projections)))
       _projections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sba/CameraNode[] _nodes
    (apply #'+ (send-all _nodes :serialization-length)) 4
    ;; sba/WorldPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; sba/Projection[] _projections
    (apply #'+ (send-all _projections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sba/CameraNode[] _nodes
     (write-long (length _nodes) s)
     (dolist (elem _nodes)
       (send elem :serialize s)
       )
     ;; sba/WorldPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; sba/Projection[] _projections
     (write-long (length _projections) s)
     (dolist (elem _projections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sba/CameraNode[] _nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodes (let (r) (dotimes (i n) (push (instance sba::CameraNode :init) r)) r))
     (dolist (elem- _nodes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; sba/WorldPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance sba::WorldPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; sba/Projection[] _projections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _projections (let (r) (dotimes (i n) (push (instance sba::Projection :init) r)) r))
     (dolist (elem- _projections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get sba::Frame :md5sum-) "418ca143f82258a762de2cff21411737")
(setf (get sba::Frame :datatype-) "sba/Frame")
(setf (get sba::Frame :definition-)
      "# New Frame
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


")



(provide :sba/Frame "418ca143f82258a762de2cff21411737")


