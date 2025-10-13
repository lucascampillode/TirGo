;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::lookupTransform)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'lookupTransform (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::LOOKUPTRANSFORM")
  (make-package "PAL_SUPERVISOR_MSGS::LOOKUPTRANSFORM"))
(unless (find-package "PAL_SUPERVISOR_MSGS::LOOKUPTRANSFORMREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::LOOKUPTRANSFORMREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::LOOKUPTRANSFORMRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::LOOKUPTRANSFORMRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pal_supervisor_msgs::lookupTransformRequest
  :super ros::object
  :slots (_target_frame _source_frame _transform_time ))

(defmethod pal_supervisor_msgs::lookupTransformRequest
  (:init
   (&key
    ((:target_frame __target_frame) "")
    ((:source_frame __source_frame) "")
    ((:transform_time __transform_time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _target_frame (string __target_frame))
   (setq _source_frame (string __source_frame))
   (setq _transform_time __transform_time)
   self)
  (:target_frame
   (&optional __target_frame)
   (if __target_frame (setq _target_frame __target_frame)) _target_frame)
  (:source_frame
   (&optional __source_frame)
   (if __source_frame (setq _source_frame __source_frame)) _source_frame)
  (:transform_time
   (&optional __transform_time)
   (if __transform_time (setq _transform_time __transform_time)) _transform_time)
  (:serialization-length
   ()
   (+
    ;; string _target_frame
    4 (length _target_frame)
    ;; string _source_frame
    4 (length _source_frame)
    ;; time _transform_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _target_frame
       (write-long (length _target_frame) s) (princ _target_frame s)
     ;; string _source_frame
       (write-long (length _source_frame) s) (princ _source_frame s)
     ;; time _transform_time
       (write-long (send _transform_time :sec) s) (write-long (send _transform_time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _target_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _source_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _transform_time
     (send _transform_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _transform_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::lookupTransformResponse
  :super ros::object
  :slots (_transform ))

(defmethod pal_supervisor_msgs::lookupTransformResponse
  (:init
   (&key
    ((:transform __transform) (instance geometry_msgs::TransformStamped :init))
    )
   (send-super :init)
   (setq _transform __transform)
   self)
  (:transform
   (&rest __transform)
   (if (keywordp (car __transform))
       (send* _transform __transform)
     (progn
       (if __transform (setq _transform (car __transform)))
       _transform)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/TransformStamped _transform
    (send _transform :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/TransformStamped _transform
       (send _transform :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/TransformStamped _transform
     (send _transform :deserialize buf ptr-) (incf ptr- (send _transform :serialization-length))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::lookupTransform
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::lookupTransform :md5sum-) "02c6b9366a79d71ff07f2dc56c230415")
(setf (get pal_supervisor_msgs::lookupTransform :datatype-) "pal_supervisor_msgs/lookupTransform")
(setf (get pal_supervisor_msgs::lookupTransform :request) pal_supervisor_msgs::lookupTransformRequest)
(setf (get pal_supervisor_msgs::lookupTransform :response) pal_supervisor_msgs::lookupTransformResponse)

(defmethod pal_supervisor_msgs::lookupTransformRequest
  (:response () (instance pal_supervisor_msgs::lookupTransformResponse :init)))

(setf (get pal_supervisor_msgs::lookupTransformRequest :md5sum-) "02c6b9366a79d71ff07f2dc56c230415")
(setf (get pal_supervisor_msgs::lookupTransformRequest :datatype-) "pal_supervisor_msgs/lookupTransformRequest")
(setf (get pal_supervisor_msgs::lookupTransformRequest :definition-)
      "string target_frame #The frame to which data should be transformed
string source_frame #The frame where the data originated
time transform_time
---
geometry_msgs/TransformStamped transform 


================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

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
")

(setf (get pal_supervisor_msgs::lookupTransformResponse :md5sum-) "02c6b9366a79d71ff07f2dc56c230415")
(setf (get pal_supervisor_msgs::lookupTransformResponse :datatype-) "pal_supervisor_msgs/lookupTransformResponse")
(setf (get pal_supervisor_msgs::lookupTransformResponse :definition-)
      "string target_frame #The frame to which data should be transformed
string source_frame #The frame where the data originated
time transform_time
---
geometry_msgs/TransformStamped transform 


================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

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
")



(provide :pal_supervisor_msgs/lookupTransform "02c6b9366a79d71ff07f2dc56c230415")


