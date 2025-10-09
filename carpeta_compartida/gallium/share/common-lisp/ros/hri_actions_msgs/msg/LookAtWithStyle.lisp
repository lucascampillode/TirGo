; Auto-generated. Do not edit!


(cl:in-package hri_actions_msgs-msg)


;//! \htmlinclude LookAtWithStyle.msg.html

(cl:defclass <LookAtWithStyle> (roslisp-msg-protocol:ros-message)
  ((style
    :reader style
    :initarg :style
    :type cl:fixnum
    :initform 0)
   (target
    :reader target
    :initarg :target
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass LookAtWithStyle (<LookAtWithStyle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LookAtWithStyle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LookAtWithStyle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hri_actions_msgs-msg:<LookAtWithStyle> is deprecated: use hri_actions_msgs-msg:LookAtWithStyle instead.")))

(cl:ensure-generic-function 'style-val :lambda-list '(m))
(cl:defmethod style-val ((m <LookAtWithStyle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:style-val is deprecated.  Use hri_actions_msgs-msg:style instead.")
  (style m))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <LookAtWithStyle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:target-val is deprecated.  Use hri_actions_msgs-msg:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<LookAtWithStyle>)))
    "Constants for message type '<LookAtWithStyle>"
  '((:DEFAULT . 0)
    (:EYES_ONLY . 1)
    (:HEAD_ONLY . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'LookAtWithStyle)))
    "Constants for message type 'LookAtWithStyle"
  '((:DEFAULT . 0)
    (:EYES_ONLY . 1)
    (:HEAD_ONLY . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LookAtWithStyle>) ostream)
  "Serializes a message object of type '<LookAtWithStyle>"
  (cl:let* ((signed (cl:slot-value msg 'style)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LookAtWithStyle>) istream)
  "Deserializes a message object of type '<LookAtWithStyle>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'style) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LookAtWithStyle>)))
  "Returns string type for a message object of type '<LookAtWithStyle>"
  "hri_actions_msgs/LookAtWithStyle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LookAtWithStyle)))
  "Returns string type for a message object of type 'LookAtWithStyle"
  "hri_actions_msgs/LookAtWithStyle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LookAtWithStyle>)))
  "Returns md5sum for a message object of type '<LookAtWithStyle>"
  "1997f720101069b0d5f7306d6cb70706")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LookAtWithStyle)))
  "Returns md5sum for a message object of type 'LookAtWithStyle"
  "1997f720101069b0d5f7306d6cb70706")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LookAtWithStyle>)))
  "Returns full string definition for message of type '<LookAtWithStyle>"
  (cl:format cl:nil "# Specify the target of the robot's gaze as a 3D point. A desired 'style'~%# policy can be specified to control how the gaze motion should be executed~%# (for instance, sudden movement, slow shift, etc.)~%~%# default policy: both eyes and head joints (eg neck) are used, the resulting~%# 'style' is a smooth combination of both~%int8 DEFAULT = 0~%~%# eyes-only policy: only the eyes (if available) should move~%int8 EYES_ONLY = 1~%~%# head-only policy: only the head joints (typically, the neck) should move~%int8 HEAD_ONLY = 2~%~%int8 style~%~%geometry_msgs/PointStamped target~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LookAtWithStyle)))
  "Returns full string definition for message of type 'LookAtWithStyle"
  (cl:format cl:nil "# Specify the target of the robot's gaze as a 3D point. A desired 'style'~%# policy can be specified to control how the gaze motion should be executed~%# (for instance, sudden movement, slow shift, etc.)~%~%# default policy: both eyes and head joints (eg neck) are used, the resulting~%# 'style' is a smooth combination of both~%int8 DEFAULT = 0~%~%# eyes-only policy: only the eyes (if available) should move~%int8 EYES_ONLY = 1~%~%# head-only policy: only the head joints (typically, the neck) should move~%int8 HEAD_ONLY = 2~%~%int8 style~%~%geometry_msgs/PointStamped target~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LookAtWithStyle>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LookAtWithStyle>))
  "Converts a ROS message object to a list"
  (cl:list 'LookAtWithStyle
    (cl:cons ':style (style msg))
    (cl:cons ':target (target msg))
))
