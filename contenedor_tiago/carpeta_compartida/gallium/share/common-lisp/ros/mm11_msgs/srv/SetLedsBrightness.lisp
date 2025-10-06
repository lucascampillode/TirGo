; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetLedsBrightness-request.msg.html

(cl:defclass <SetLedsBrightness-request> (roslisp-msg-protocol:ros-message)
  ((brightness
    :reader brightness
    :initarg :brightness
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetLedsBrightness-request (<SetLedsBrightness-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLedsBrightness-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLedsBrightness-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetLedsBrightness-request> is deprecated: use mm11_msgs-srv:SetLedsBrightness-request instead.")))

(cl:ensure-generic-function 'brightness-val :lambda-list '(m))
(cl:defmethod brightness-val ((m <SetLedsBrightness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:brightness-val is deprecated.  Use mm11_msgs-srv:brightness instead.")
  (brightness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLedsBrightness-request>) ostream)
  "Serializes a message object of type '<SetLedsBrightness-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brightness)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLedsBrightness-request>) istream)
  "Deserializes a message object of type '<SetLedsBrightness-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brightness)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLedsBrightness-request>)))
  "Returns string type for a service object of type '<SetLedsBrightness-request>"
  "mm11_msgs/SetLedsBrightnessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLedsBrightness-request)))
  "Returns string type for a service object of type 'SetLedsBrightness-request"
  "mm11_msgs/SetLedsBrightnessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLedsBrightness-request>)))
  "Returns md5sum for a message object of type '<SetLedsBrightness-request>"
  "0374114287947f81d241e42c7a34c37e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLedsBrightness-request)))
  "Returns md5sum for a message object of type 'SetLedsBrightness-request"
  "0374114287947f81d241e42c7a34c37e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLedsBrightness-request>)))
  "Returns full string definition for message of type '<SetLedsBrightness-request>"
  (cl:format cl:nil "# Set the LEDs brightness~%~%uint8 brightness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLedsBrightness-request)))
  "Returns full string definition for message of type 'SetLedsBrightness-request"
  (cl:format cl:nil "# Set the LEDs brightness~%~%uint8 brightness~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLedsBrightness-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLedsBrightness-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLedsBrightness-request
    (cl:cons ':brightness (brightness msg))
))
;//! \htmlinclude SetLedsBrightness-response.msg.html

(cl:defclass <SetLedsBrightness-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetLedsBrightness-response (<SetLedsBrightness-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLedsBrightness-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLedsBrightness-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetLedsBrightness-response> is deprecated: use mm11_msgs-srv:SetLedsBrightness-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLedsBrightness-response>) ostream)
  "Serializes a message object of type '<SetLedsBrightness-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLedsBrightness-response>) istream)
  "Deserializes a message object of type '<SetLedsBrightness-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLedsBrightness-response>)))
  "Returns string type for a service object of type '<SetLedsBrightness-response>"
  "mm11_msgs/SetLedsBrightnessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLedsBrightness-response)))
  "Returns string type for a service object of type 'SetLedsBrightness-response"
  "mm11_msgs/SetLedsBrightnessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLedsBrightness-response>)))
  "Returns md5sum for a message object of type '<SetLedsBrightness-response>"
  "0374114287947f81d241e42c7a34c37e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLedsBrightness-response)))
  "Returns md5sum for a message object of type 'SetLedsBrightness-response"
  "0374114287947f81d241e42c7a34c37e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLedsBrightness-response>)))
  "Returns full string definition for message of type '<SetLedsBrightness-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLedsBrightness-response)))
  "Returns full string definition for message of type 'SetLedsBrightness-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLedsBrightness-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLedsBrightness-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLedsBrightness-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetLedsBrightness)))
  'SetLedsBrightness-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetLedsBrightness)))
  'SetLedsBrightness-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLedsBrightness)))
  "Returns string type for a service object of type '<SetLedsBrightness>"
  "mm11_msgs/SetLedsBrightness")