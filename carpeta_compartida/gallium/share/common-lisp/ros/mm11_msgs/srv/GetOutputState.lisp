; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude GetOutputState-request.msg.html

(cl:defclass <GetOutputState-request> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetOutputState-request (<GetOutputState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOutputState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOutputState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<GetOutputState-request> is deprecated: use mm11_msgs-srv:GetOutputState-request instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetOutputState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:output-val is deprecated.  Use mm11_msgs-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOutputState-request>) ostream)
  "Serializes a message object of type '<GetOutputState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'output)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOutputState-request>) istream)
  "Deserializes a message object of type '<GetOutputState-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'output)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOutputState-request>)))
  "Returns string type for a service object of type '<GetOutputState-request>"
  "mm11_msgs/GetOutputStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOutputState-request)))
  "Returns string type for a service object of type 'GetOutputState-request"
  "mm11_msgs/GetOutputStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOutputState-request>)))
  "Returns md5sum for a message object of type '<GetOutputState-request>"
  "30376e3f1875e49fda4a395a76150de1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOutputState-request)))
  "Returns md5sum for a message object of type 'GetOutputState-request"
  "30376e3f1875e49fda4a395a76150de1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOutputState-request>)))
  "Returns full string definition for message of type '<GetOutputState-request>"
  (cl:format cl:nil "# Get an Output state~%~%uint8 output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOutputState-request)))
  "Returns full string definition for message of type 'GetOutputState-request"
  (cl:format cl:nil "# Get an Output state~%~%uint8 output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOutputState-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOutputState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOutputState-request
    (cl:cons ':output (output msg))
))
;//! \htmlinclude GetOutputState-response.msg.html

(cl:defclass <GetOutputState-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetOutputState-response (<GetOutputState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOutputState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOutputState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<GetOutputState-response> is deprecated: use mm11_msgs-srv:GetOutputState-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GetOutputState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:state-val is deprecated.  Use mm11_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOutputState-response>) ostream)
  "Serializes a message object of type '<GetOutputState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOutputState-response>) istream)
  "Deserializes a message object of type '<GetOutputState-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOutputState-response>)))
  "Returns string type for a service object of type '<GetOutputState-response>"
  "mm11_msgs/GetOutputStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOutputState-response)))
  "Returns string type for a service object of type 'GetOutputState-response"
  "mm11_msgs/GetOutputStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOutputState-response>)))
  "Returns md5sum for a message object of type '<GetOutputState-response>"
  "30376e3f1875e49fda4a395a76150de1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOutputState-response)))
  "Returns md5sum for a message object of type 'GetOutputState-response"
  "30376e3f1875e49fda4a395a76150de1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOutputState-response>)))
  "Returns full string definition for message of type '<GetOutputState-response>"
  (cl:format cl:nil "~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOutputState-response)))
  "Returns full string definition for message of type 'GetOutputState-response"
  (cl:format cl:nil "~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOutputState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOutputState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOutputState-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOutputState)))
  'GetOutputState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOutputState)))
  'GetOutputState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOutputState)))
  "Returns string type for a service object of type '<GetOutputState>"
  "mm11_msgs/GetOutputState")