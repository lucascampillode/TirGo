; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude GetInputState-request.msg.html

(cl:defclass <GetInputState-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetInputState-request (<GetInputState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetInputState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetInputState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<GetInputState-request> is deprecated: use mm11_msgs-srv:GetInputState-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetInputState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:input-val is deprecated.  Use mm11_msgs-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetInputState-request>) ostream)
  "Serializes a message object of type '<GetInputState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'input)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetInputState-request>) istream)
  "Deserializes a message object of type '<GetInputState-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'input)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetInputState-request>)))
  "Returns string type for a service object of type '<GetInputState-request>"
  "mm11_msgs/GetInputStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetInputState-request)))
  "Returns string type for a service object of type 'GetInputState-request"
  "mm11_msgs/GetInputStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetInputState-request>)))
  "Returns md5sum for a message object of type '<GetInputState-request>"
  "11be78a327f2fee556995234c4d74c2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetInputState-request)))
  "Returns md5sum for a message object of type 'GetInputState-request"
  "11be78a327f2fee556995234c4d74c2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetInputState-request>)))
  "Returns full string definition for message of type '<GetInputState-request>"
  (cl:format cl:nil "# Get an Input state~%~%uint8 input~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetInputState-request)))
  "Returns full string definition for message of type 'GetInputState-request"
  (cl:format cl:nil "# Get an Input state~%~%uint8 input~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetInputState-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetInputState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetInputState-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetInputState-response.msg.html

(cl:defclass <GetInputState-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetInputState-response (<GetInputState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetInputState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetInputState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<GetInputState-response> is deprecated: use mm11_msgs-srv:GetInputState-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetInputState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:state-val is deprecated.  Use mm11_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetInputState-response>) ostream)
  "Serializes a message object of type '<GetInputState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetInputState-response>) istream)
  "Deserializes a message object of type '<GetInputState-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetInputState-response>)))
  "Returns string type for a service object of type '<GetInputState-response>"
  "mm11_msgs/GetInputStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetInputState-response)))
  "Returns string type for a service object of type 'GetInputState-response"
  "mm11_msgs/GetInputStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetInputState-response>)))
  "Returns md5sum for a message object of type '<GetInputState-response>"
  "11be78a327f2fee556995234c4d74c2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetInputState-response)))
  "Returns md5sum for a message object of type 'GetInputState-response"
  "11be78a327f2fee556995234c4d74c2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetInputState-response>)))
  "Returns full string definition for message of type '<GetInputState-response>"
  (cl:format cl:nil "~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetInputState-response)))
  "Returns full string definition for message of type 'GetInputState-response"
  (cl:format cl:nil "~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetInputState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetInputState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetInputState-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetInputState)))
  'GetInputState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetInputState)))
  'GetInputState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetInputState)))
  "Returns string type for a service object of type '<GetInputState>"
  "mm11_msgs/GetInputState")