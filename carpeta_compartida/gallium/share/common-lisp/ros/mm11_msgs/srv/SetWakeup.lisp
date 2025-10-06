; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetWakeup-request.msg.html

(cl:defclass <SetWakeup-request> (roslisp-msg-protocol:ros-message)
  ((time_min_down
    :reader time_min_down
    :initarg :time_min_down
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetWakeup-request (<SetWakeup-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWakeup-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWakeup-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetWakeup-request> is deprecated: use mm11_msgs-srv:SetWakeup-request instead.")))

(cl:ensure-generic-function 'time_min_down-val :lambda-list '(m))
(cl:defmethod time_min_down-val ((m <SetWakeup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:time_min_down-val is deprecated.  Use mm11_msgs-srv:time_min_down instead.")
  (time_min_down m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWakeup-request>) ostream)
  "Serializes a message object of type '<SetWakeup-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_min_down)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_min_down)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWakeup-request>) istream)
  "Deserializes a message object of type '<SetWakeup-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_min_down)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_min_down)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWakeup-request>)))
  "Returns string type for a service object of type '<SetWakeup-request>"
  "mm11_msgs/SetWakeupRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWakeup-request)))
  "Returns string type for a service object of type 'SetWakeup-request"
  "mm11_msgs/SetWakeupRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWakeup-request>)))
  "Returns md5sum for a message object of type '<SetWakeup-request>"
  "2c161e9706149de203305763332e4462")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWakeup-request)))
  "Returns md5sum for a message object of type 'SetWakeup-request"
  "2c161e9706149de203305763332e4462")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWakeup-request>)))
  "Returns full string definition for message of type '<SetWakeup-request>"
  (cl:format cl:nil "# Turns off the robot during indicated minutes.~%~%uint16 time_min_down~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWakeup-request)))
  "Returns full string definition for message of type 'SetWakeup-request"
  (cl:format cl:nil "# Turns off the robot during indicated minutes.~%~%uint16 time_min_down~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWakeup-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWakeup-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWakeup-request
    (cl:cons ':time_min_down (time_min_down msg))
))
;//! \htmlinclude SetWakeup-response.msg.html

(cl:defclass <SetWakeup-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetWakeup-response (<SetWakeup-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetWakeup-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetWakeup-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetWakeup-response> is deprecated: use mm11_msgs-srv:SetWakeup-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetWakeup-response>) ostream)
  "Serializes a message object of type '<SetWakeup-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetWakeup-response>) istream)
  "Deserializes a message object of type '<SetWakeup-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetWakeup-response>)))
  "Returns string type for a service object of type '<SetWakeup-response>"
  "mm11_msgs/SetWakeupResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWakeup-response)))
  "Returns string type for a service object of type 'SetWakeup-response"
  "mm11_msgs/SetWakeupResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetWakeup-response>)))
  "Returns md5sum for a message object of type '<SetWakeup-response>"
  "2c161e9706149de203305763332e4462")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetWakeup-response)))
  "Returns md5sum for a message object of type 'SetWakeup-response"
  "2c161e9706149de203305763332e4462")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetWakeup-response>)))
  "Returns full string definition for message of type '<SetWakeup-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetWakeup-response)))
  "Returns full string definition for message of type 'SetWakeup-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetWakeup-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetWakeup-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetWakeup-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetWakeup)))
  'SetWakeup-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetWakeup)))
  'SetWakeup-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetWakeup)))
  "Returns string type for a service object of type '<SetWakeup>"
  "mm11_msgs/SetWakeup")