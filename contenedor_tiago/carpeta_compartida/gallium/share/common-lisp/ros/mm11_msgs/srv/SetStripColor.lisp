; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetStripColor-request.msg.html

(cl:defclass <SetStripColor-request> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0)
   (r
    :reader r
    :initarg :r
    :type cl:fixnum
    :initform 0)
   (g
    :reader g
    :initarg :g
    :type cl:fixnum
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetStripColor-request (<SetStripColor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripColor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripColor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripColor-request> is deprecated: use mm11_msgs-srv:SetStripColor-request instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <SetStripColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:port-val is deprecated.  Use mm11_msgs-srv:port instead.")
  (port m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <SetStripColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:r-val is deprecated.  Use mm11_msgs-srv:r instead.")
  (r m))

(cl:ensure-generic-function 'g-val :lambda-list '(m))
(cl:defmethod g-val ((m <SetStripColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:g-val is deprecated.  Use mm11_msgs-srv:g instead.")
  (g m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <SetStripColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:b-val is deprecated.  Use mm11_msgs-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripColor-request>) ostream)
  "Serializes a message object of type '<SetStripColor-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripColor-request>) istream)
  "Deserializes a message object of type '<SetStripColor-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripColor-request>)))
  "Returns string type for a service object of type '<SetStripColor-request>"
  "mm11_msgs/SetStripColorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripColor-request)))
  "Returns string type for a service object of type 'SetStripColor-request"
  "mm11_msgs/SetStripColorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripColor-request>)))
  "Returns md5sum for a message object of type '<SetStripColor-request>"
  "96052dec11380a267d245b4ea069fa2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripColor-request)))
  "Returns md5sum for a message object of type 'SetStripColor-request"
  "96052dec11380a267d245b4ea069fa2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripColor-request>)))
  "Returns full string definition for message of type '<SetStripColor-request>"
  (cl:format cl:nil "# Sets the color of a whole strip~%~%uint8 port~%~%uint8 r~%uint8 g~%uint8 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripColor-request)))
  "Returns full string definition for message of type 'SetStripColor-request"
  (cl:format cl:nil "# Sets the color of a whole strip~%~%uint8 port~%~%uint8 r~%uint8 g~%uint8 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripColor-request>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripColor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripColor-request
    (cl:cons ':port (port msg))
    (cl:cons ':r (r msg))
    (cl:cons ':g (g msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude SetStripColor-response.msg.html

(cl:defclass <SetStripColor-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetStripColor-response (<SetStripColor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripColor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripColor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripColor-response> is deprecated: use mm11_msgs-srv:SetStripColor-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripColor-response>) ostream)
  "Serializes a message object of type '<SetStripColor-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripColor-response>) istream)
  "Deserializes a message object of type '<SetStripColor-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripColor-response>)))
  "Returns string type for a service object of type '<SetStripColor-response>"
  "mm11_msgs/SetStripColorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripColor-response)))
  "Returns string type for a service object of type 'SetStripColor-response"
  "mm11_msgs/SetStripColorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripColor-response>)))
  "Returns md5sum for a message object of type '<SetStripColor-response>"
  "96052dec11380a267d245b4ea069fa2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripColor-response)))
  "Returns md5sum for a message object of type 'SetStripColor-response"
  "96052dec11380a267d245b4ea069fa2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripColor-response>)))
  "Returns full string definition for message of type '<SetStripColor-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripColor-response)))
  "Returns full string definition for message of type 'SetStripColor-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripColor-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripColor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripColor-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStripColor)))
  'SetStripColor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStripColor)))
  'SetStripColor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripColor)))
  "Returns string type for a service object of type '<SetStripColor>"
  "mm11_msgs/SetStripColor")