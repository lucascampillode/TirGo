; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetRestart-request.msg.html

(cl:defclass <SetRestart-request> (roslisp-msg-protocol:ros-message)
  ((time_down
    :reader time_down
    :initarg :time_down
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRestart-request (<SetRestart-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRestart-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRestart-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetRestart-request> is deprecated: use mm11_msgs-srv:SetRestart-request instead.")))

(cl:ensure-generic-function 'time_down-val :lambda-list '(m))
(cl:defmethod time_down-val ((m <SetRestart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:time_down-val is deprecated.  Use mm11_msgs-srv:time_down instead.")
  (time_down m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRestart-request>) ostream)
  "Serializes a message object of type '<SetRestart-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_down)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRestart-request>) istream)
  "Deserializes a message object of type '<SetRestart-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_down)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRestart-request>)))
  "Returns string type for a service object of type '<SetRestart-request>"
  "mm11_msgs/SetRestartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRestart-request)))
  "Returns string type for a service object of type 'SetRestart-request"
  "mm11_msgs/SetRestartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRestart-request>)))
  "Returns md5sum for a message object of type '<SetRestart-request>"
  "312e4924f616d4157dc3bb5aee21a3bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRestart-request)))
  "Returns md5sum for a message object of type 'SetRestart-request"
  "312e4924f616d4157dc3bb5aee21a3bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRestart-request>)))
  "Returns full string definition for message of type '<SetRestart-request>"
  (cl:format cl:nil "# Force a hardware restart of the robot. Keeps the robot down for the indicated seconds.~%~%uint8 time_down~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRestart-request)))
  "Returns full string definition for message of type 'SetRestart-request"
  (cl:format cl:nil "# Force a hardware restart of the robot. Keeps the robot down for the indicated seconds.~%~%uint8 time_down~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRestart-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRestart-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRestart-request
    (cl:cons ':time_down (time_down msg))
))
;//! \htmlinclude SetRestart-response.msg.html

(cl:defclass <SetRestart-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetRestart-response (<SetRestart-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRestart-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRestart-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetRestart-response> is deprecated: use mm11_msgs-srv:SetRestart-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRestart-response>) ostream)
  "Serializes a message object of type '<SetRestart-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRestart-response>) istream)
  "Deserializes a message object of type '<SetRestart-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRestart-response>)))
  "Returns string type for a service object of type '<SetRestart-response>"
  "mm11_msgs/SetRestartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRestart-response)))
  "Returns string type for a service object of type 'SetRestart-response"
  "mm11_msgs/SetRestartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRestart-response>)))
  "Returns md5sum for a message object of type '<SetRestart-response>"
  "312e4924f616d4157dc3bb5aee21a3bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRestart-response)))
  "Returns md5sum for a message object of type 'SetRestart-response"
  "312e4924f616d4157dc3bb5aee21a3bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRestart-response>)))
  "Returns full string definition for message of type '<SetRestart-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRestart-response)))
  "Returns full string definition for message of type 'SetRestart-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRestart-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRestart-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRestart-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRestart)))
  'SetRestart-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRestart)))
  'SetRestart-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRestart)))
  "Returns string type for a service object of type '<SetRestart>"
  "mm11_msgs/SetRestart")