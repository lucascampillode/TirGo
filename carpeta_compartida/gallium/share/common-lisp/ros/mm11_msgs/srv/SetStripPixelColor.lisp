; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetStripPixelColor-request.msg.html

(cl:defclass <SetStripPixelColor-request> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0)
   (pixel
    :reader pixel
    :initarg :pixel
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

(cl:defclass SetStripPixelColor-request (<SetStripPixelColor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripPixelColor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripPixelColor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripPixelColor-request> is deprecated: use mm11_msgs-srv:SetStripPixelColor-request instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <SetStripPixelColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:port-val is deprecated.  Use mm11_msgs-srv:port instead.")
  (port m))

(cl:ensure-generic-function 'pixel-val :lambda-list '(m))
(cl:defmethod pixel-val ((m <SetStripPixelColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:pixel-val is deprecated.  Use mm11_msgs-srv:pixel instead.")
  (pixel m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <SetStripPixelColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:r-val is deprecated.  Use mm11_msgs-srv:r instead.")
  (r m))

(cl:ensure-generic-function 'g-val :lambda-list '(m))
(cl:defmethod g-val ((m <SetStripPixelColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:g-val is deprecated.  Use mm11_msgs-srv:g instead.")
  (g m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <SetStripPixelColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:b-val is deprecated.  Use mm11_msgs-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripPixelColor-request>) ostream)
  "Serializes a message object of type '<SetStripPixelColor-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pixel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripPixelColor-request>) istream)
  "Deserializes a message object of type '<SetStripPixelColor-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pixel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripPixelColor-request>)))
  "Returns string type for a service object of type '<SetStripPixelColor-request>"
  "mm11_msgs/SetStripPixelColorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripPixelColor-request)))
  "Returns string type for a service object of type 'SetStripPixelColor-request"
  "mm11_msgs/SetStripPixelColorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripPixelColor-request>)))
  "Returns md5sum for a message object of type '<SetStripPixelColor-request>"
  "b649b5b940622eb64d13b51266bf8b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripPixelColor-request)))
  "Returns md5sum for a message object of type 'SetStripPixelColor-request"
  "b649b5b940622eb64d13b51266bf8b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripPixelColor-request>)))
  "Returns full string definition for message of type '<SetStripPixelColor-request>"
  (cl:format cl:nil "# Sets the color of a strip's pixel~%~%uint8 port~%~%uint8 pixel~%~%uint8 r~%uint8 g~%uint8 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripPixelColor-request)))
  "Returns full string definition for message of type 'SetStripPixelColor-request"
  (cl:format cl:nil "# Sets the color of a strip's pixel~%~%uint8 port~%~%uint8 pixel~%~%uint8 r~%uint8 g~%uint8 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripPixelColor-request>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripPixelColor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripPixelColor-request
    (cl:cons ':port (port msg))
    (cl:cons ':pixel (pixel msg))
    (cl:cons ':r (r msg))
    (cl:cons ':g (g msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude SetStripPixelColor-response.msg.html

(cl:defclass <SetStripPixelColor-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetStripPixelColor-response (<SetStripPixelColor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripPixelColor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripPixelColor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripPixelColor-response> is deprecated: use mm11_msgs-srv:SetStripPixelColor-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripPixelColor-response>) ostream)
  "Serializes a message object of type '<SetStripPixelColor-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripPixelColor-response>) istream)
  "Deserializes a message object of type '<SetStripPixelColor-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripPixelColor-response>)))
  "Returns string type for a service object of type '<SetStripPixelColor-response>"
  "mm11_msgs/SetStripPixelColorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripPixelColor-response)))
  "Returns string type for a service object of type 'SetStripPixelColor-response"
  "mm11_msgs/SetStripPixelColorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripPixelColor-response>)))
  "Returns md5sum for a message object of type '<SetStripPixelColor-response>"
  "b649b5b940622eb64d13b51266bf8b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripPixelColor-response)))
  "Returns md5sum for a message object of type 'SetStripPixelColor-response"
  "b649b5b940622eb64d13b51266bf8b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripPixelColor-response>)))
  "Returns full string definition for message of type '<SetStripPixelColor-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripPixelColor-response)))
  "Returns full string definition for message of type 'SetStripPixelColor-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripPixelColor-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripPixelColor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripPixelColor-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStripPixelColor)))
  'SetStripPixelColor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStripPixelColor)))
  'SetStripPixelColor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripPixelColor)))
  "Returns string type for a service object of type '<SetStripPixelColor>"
  "mm11_msgs/SetStripPixelColor")