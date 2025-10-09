; Auto-generated. Do not edit!


(cl:in-package amcl-srv)


;//! \htmlinclude SetInitialPose-request.msg.html

(cl:defclass <SetInitialPose-request> (roslisp-msg-protocol:ros-message)
  ((initialpose
    :reader initialpose
    :initarg :initialpose
    :type geometry_msgs-msg:PoseWithCovarianceStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovarianceStamped)))
)

(cl:defclass SetInitialPose-request (<SetInitialPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInitialPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInitialPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amcl-srv:<SetInitialPose-request> is deprecated: use amcl-srv:SetInitialPose-request instead.")))

(cl:ensure-generic-function 'initialpose-val :lambda-list '(m))
(cl:defmethod initialpose-val ((m <SetInitialPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amcl-srv:initialpose-val is deprecated.  Use amcl-srv:initialpose instead.")
  (initialpose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInitialPose-request>) ostream)
  "Serializes a message object of type '<SetInitialPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'initialpose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInitialPose-request>) istream)
  "Deserializes a message object of type '<SetInitialPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'initialpose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInitialPose-request>)))
  "Returns string type for a service object of type '<SetInitialPose-request>"
  "amcl/SetInitialPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInitialPose-request)))
  "Returns string type for a service object of type 'SetInitialPose-request"
  "amcl/SetInitialPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInitialPose-request>)))
  "Returns md5sum for a message object of type '<SetInitialPose-request>"
  "76084d1d94567bfd99de36dd17bd286b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInitialPose-request)))
  "Returns md5sum for a message object of type 'SetInitialPose-request"
  "76084d1d94567bfd99de36dd17bd286b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInitialPose-request>)))
  "Returns full string definition for message of type '<SetInitialPose-request>"
  (cl:format cl:nil "geometry_msgs/PoseWithCovarianceStamped initialpose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInitialPose-request)))
  "Returns full string definition for message of type 'SetInitialPose-request"
  (cl:format cl:nil "geometry_msgs/PoseWithCovarianceStamped initialpose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInitialPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'initialpose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInitialPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInitialPose-request
    (cl:cons ':initialpose (initialpose msg))
))
;//! \htmlinclude SetInitialPose-response.msg.html

(cl:defclass <SetInitialPose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetInitialPose-response (<SetInitialPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInitialPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInitialPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amcl-srv:<SetInitialPose-response> is deprecated: use amcl-srv:SetInitialPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetInitialPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amcl-srv:success-val is deprecated.  Use amcl-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInitialPose-response>) ostream)
  "Serializes a message object of type '<SetInitialPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInitialPose-response>) istream)
  "Deserializes a message object of type '<SetInitialPose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInitialPose-response>)))
  "Returns string type for a service object of type '<SetInitialPose-response>"
  "amcl/SetInitialPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInitialPose-response)))
  "Returns string type for a service object of type 'SetInitialPose-response"
  "amcl/SetInitialPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInitialPose-response>)))
  "Returns md5sum for a message object of type '<SetInitialPose-response>"
  "76084d1d94567bfd99de36dd17bd286b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInitialPose-response)))
  "Returns md5sum for a message object of type 'SetInitialPose-response"
  "76084d1d94567bfd99de36dd17bd286b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInitialPose-response>)))
  "Returns full string definition for message of type '<SetInitialPose-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInitialPose-response)))
  "Returns full string definition for message of type 'SetInitialPose-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInitialPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInitialPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInitialPose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetInitialPose)))
  'SetInitialPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetInitialPose)))
  'SetInitialPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInitialPose)))
  "Returns string type for a service object of type '<SetInitialPose>"
  "amcl/SetInitialPose")