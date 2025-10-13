; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetOutputState-request.msg.html

(cl:defclass <SetOutputState-request> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetOutputState-request (<SetOutputState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOutputState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOutputState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetOutputState-request> is deprecated: use mm11_msgs-srv:SetOutputState-request instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetOutputState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:output-val is deprecated.  Use mm11_msgs-srv:output instead.")
  (output m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetOutputState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:state-val is deprecated.  Use mm11_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOutputState-request>) ostream)
  "Serializes a message object of type '<SetOutputState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'output)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOutputState-request>) istream)
  "Deserializes a message object of type '<SetOutputState-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'output)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOutputState-request>)))
  "Returns string type for a service object of type '<SetOutputState-request>"
  "mm11_msgs/SetOutputStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOutputState-request)))
  "Returns string type for a service object of type 'SetOutputState-request"
  "mm11_msgs/SetOutputStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOutputState-request>)))
  "Returns md5sum for a message object of type '<SetOutputState-request>"
  "db96c205088a50d0cf4188b4e981907d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOutputState-request)))
  "Returns md5sum for a message object of type 'SetOutputState-request"
  "db96c205088a50d0cf4188b4e981907d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOutputState-request>)))
  "Returns full string definition for message of type '<SetOutputState-request>"
  (cl:format cl:nil "# Set an Output state~%~%uint8 output~%~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOutputState-request)))
  "Returns full string definition for message of type 'SetOutputState-request"
  (cl:format cl:nil "# Set an Output state~%~%uint8 output~%~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOutputState-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOutputState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOutputState-request
    (cl:cons ':output (output msg))
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetOutputState-response.msg.html

(cl:defclass <SetOutputState-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetOutputState-response (<SetOutputState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOutputState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOutputState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetOutputState-response> is deprecated: use mm11_msgs-srv:SetOutputState-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOutputState-response>) ostream)
  "Serializes a message object of type '<SetOutputState-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOutputState-response>) istream)
  "Deserializes a message object of type '<SetOutputState-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOutputState-response>)))
  "Returns string type for a service object of type '<SetOutputState-response>"
  "mm11_msgs/SetOutputStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOutputState-response)))
  "Returns string type for a service object of type 'SetOutputState-response"
  "mm11_msgs/SetOutputStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOutputState-response>)))
  "Returns md5sum for a message object of type '<SetOutputState-response>"
  "db96c205088a50d0cf4188b4e981907d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOutputState-response)))
  "Returns md5sum for a message object of type 'SetOutputState-response"
  "db96c205088a50d0cf4188b4e981907d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOutputState-response>)))
  "Returns full string definition for message of type '<SetOutputState-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOutputState-response)))
  "Returns full string definition for message of type 'SetOutputState-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOutputState-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOutputState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOutputState-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOutputState)))
  'SetOutputState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOutputState)))
  'SetOutputState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOutputState)))
  "Returns string type for a service object of type '<SetOutputState>"
  "mm11_msgs/SetOutputState")