; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetLedState-request.msg.html

(cl:defclass <SetLedState-request> (roslisp-msg-protocol:ros-message)
  ((led
    :reader led
    :initarg :led
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetLedState-request (<SetLedState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLedState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLedState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetLedState-request> is deprecated: use mm11_msgs-srv:SetLedState-request instead.")))

(cl:ensure-generic-function 'led-val :lambda-list '(m))
(cl:defmethod led-val ((m <SetLedState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:led-val is deprecated.  Use mm11_msgs-srv:led instead.")
  (led m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetLedState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:state-val is deprecated.  Use mm11_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLedState-request>) ostream)
  "Serializes a message object of type '<SetLedState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'led)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLedState-request>) istream)
  "Deserializes a message object of type '<SetLedState-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'led)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLedState-request>)))
  "Returns string type for a service object of type '<SetLedState-request>"
  "mm11_msgs/SetLedStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLedState-request)))
  "Returns string type for a service object of type 'SetLedState-request"
  "mm11_msgs/SetLedStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLedState-request>)))
  "Returns md5sum for a message object of type '<SetLedState-request>"
  "97b59df318c0b0c26d854b0a9da207f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLedState-request)))
  "Returns md5sum for a message object of type 'SetLedState-request"
  "97b59df318c0b0c26d854b0a9da207f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLedState-request>)))
  "Returns full string definition for message of type '<SetLedState-request>"
  (cl:format cl:nil "# Set a LED state~%~%uint8 led~%~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLedState-request)))
  "Returns full string definition for message of type 'SetLedState-request"
  (cl:format cl:nil "# Set a LED state~%~%uint8 led~%~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLedState-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLedState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLedState-request
    (cl:cons ':led (led msg))
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetLedState-response.msg.html

(cl:defclass <SetLedState-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetLedState-response (<SetLedState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLedState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLedState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetLedState-response> is deprecated: use mm11_msgs-srv:SetLedState-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLedState-response>) ostream)
  "Serializes a message object of type '<SetLedState-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLedState-response>) istream)
  "Deserializes a message object of type '<SetLedState-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLedState-response>)))
  "Returns string type for a service object of type '<SetLedState-response>"
  "mm11_msgs/SetLedStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLedState-response)))
  "Returns string type for a service object of type 'SetLedState-response"
  "mm11_msgs/SetLedStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLedState-response>)))
  "Returns md5sum for a message object of type '<SetLedState-response>"
  "97b59df318c0b0c26d854b0a9da207f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLedState-response)))
  "Returns md5sum for a message object of type 'SetLedState-response"
  "97b59df318c0b0c26d854b0a9da207f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLedState-response>)))
  "Returns full string definition for message of type '<SetLedState-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLedState-response)))
  "Returns full string definition for message of type 'SetLedState-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLedState-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLedState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLedState-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetLedState)))
  'SetLedState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetLedState)))
  'SetLedState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLedState)))
  "Returns string type for a service object of type '<SetLedState>"
  "mm11_msgs/SetLedState")