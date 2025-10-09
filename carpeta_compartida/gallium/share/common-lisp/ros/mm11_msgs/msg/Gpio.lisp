; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-msg)


;//! \htmlinclude Gpio.msg.html

(cl:defclass <Gpio> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (input_state
    :reader input_state
    :initarg :input_state
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Gpio (<Gpio>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gpio>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gpio)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-msg:<Gpio> is deprecated: use mm11_msgs-msg:Gpio instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Gpio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-msg:header-val is deprecated.  Use mm11_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'input_state-val :lambda-list '(m))
(cl:defmethod input_state-val ((m <Gpio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-msg:input_state-val is deprecated.  Use mm11_msgs-msg:input_state instead.")
  (input_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gpio>) ostream)
  "Serializes a message object of type '<Gpio>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'input_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'input_state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gpio>) istream)
  "Deserializes a message object of type '<Gpio>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'input_state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'input_state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gpio>)))
  "Returns string type for a message object of type '<Gpio>"
  "mm11_msgs/Gpio")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gpio)))
  "Returns string type for a message object of type 'Gpio"
  "mm11_msgs/Gpio")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gpio>)))
  "Returns md5sum for a message object of type '<Gpio>"
  "1d85c88c9ce946bd64d8803ef84f9759")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gpio)))
  "Returns md5sum for a message object of type 'Gpio"
  "1d85c88c9ce946bd64d8803ef84f9759")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gpio>)))
  "Returns full string definition for message of type '<Gpio>"
  (cl:format cl:nil "# ROS header~%Header header~%~%uint8[] input_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gpio)))
  "Returns full string definition for message of type 'Gpio"
  (cl:format cl:nil "# ROS header~%Header header~%~%uint8[] input_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gpio>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'input_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gpio>))
  "Converts a ROS message object to a list"
  (cl:list 'Gpio
    (cl:cons ':header (header msg))
    (cl:cons ':input_state (input_state msg))
))
