; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetStripAnimation-request.msg.html

(cl:defclass <SetStripAnimation-request> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0)
   (animation_id
    :reader animation_id
    :initarg :animation_id
    :type cl:fixnum
    :initform 0)
   (param_1
    :reader param_1
    :initarg :param_1
    :type cl:fixnum
    :initform 0)
   (param_2
    :reader param_2
    :initarg :param_2
    :type cl:fixnum
    :initform 0)
   (r_1
    :reader r_1
    :initarg :r_1
    :type cl:fixnum
    :initform 0)
   (g_1
    :reader g_1
    :initarg :g_1
    :type cl:fixnum
    :initform 0)
   (b_1
    :reader b_1
    :initarg :b_1
    :type cl:fixnum
    :initform 0)
   (r_2
    :reader r_2
    :initarg :r_2
    :type cl:fixnum
    :initform 0)
   (g_2
    :reader g_2
    :initarg :g_2
    :type cl:fixnum
    :initform 0)
   (b_2
    :reader b_2
    :initarg :b_2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetStripAnimation-request (<SetStripAnimation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripAnimation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripAnimation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripAnimation-request> is deprecated: use mm11_msgs-srv:SetStripAnimation-request instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:port-val is deprecated.  Use mm11_msgs-srv:port instead.")
  (port m))

(cl:ensure-generic-function 'animation_id-val :lambda-list '(m))
(cl:defmethod animation_id-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:animation_id-val is deprecated.  Use mm11_msgs-srv:animation_id instead.")
  (animation_id m))

(cl:ensure-generic-function 'param_1-val :lambda-list '(m))
(cl:defmethod param_1-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:param_1-val is deprecated.  Use mm11_msgs-srv:param_1 instead.")
  (param_1 m))

(cl:ensure-generic-function 'param_2-val :lambda-list '(m))
(cl:defmethod param_2-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:param_2-val is deprecated.  Use mm11_msgs-srv:param_2 instead.")
  (param_2 m))

(cl:ensure-generic-function 'r_1-val :lambda-list '(m))
(cl:defmethod r_1-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:r_1-val is deprecated.  Use mm11_msgs-srv:r_1 instead.")
  (r_1 m))

(cl:ensure-generic-function 'g_1-val :lambda-list '(m))
(cl:defmethod g_1-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:g_1-val is deprecated.  Use mm11_msgs-srv:g_1 instead.")
  (g_1 m))

(cl:ensure-generic-function 'b_1-val :lambda-list '(m))
(cl:defmethod b_1-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:b_1-val is deprecated.  Use mm11_msgs-srv:b_1 instead.")
  (b_1 m))

(cl:ensure-generic-function 'r_2-val :lambda-list '(m))
(cl:defmethod r_2-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:r_2-val is deprecated.  Use mm11_msgs-srv:r_2 instead.")
  (r_2 m))

(cl:ensure-generic-function 'g_2-val :lambda-list '(m))
(cl:defmethod g_2-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:g_2-val is deprecated.  Use mm11_msgs-srv:g_2 instead.")
  (g_2 m))

(cl:ensure-generic-function 'b_2-val :lambda-list '(m))
(cl:defmethod b_2-val ((m <SetStripAnimation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:b_2-val is deprecated.  Use mm11_msgs-srv:b_2 instead.")
  (b_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripAnimation-request>) ostream)
  "Serializes a message object of type '<SetStripAnimation-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'animation_id)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'param_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'param_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripAnimation-request>) istream)
  "Deserializes a message object of type '<SetStripAnimation-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'animation_id)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param_1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'param_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripAnimation-request>)))
  "Returns string type for a service object of type '<SetStripAnimation-request>"
  "mm11_msgs/SetStripAnimationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripAnimation-request)))
  "Returns string type for a service object of type 'SetStripAnimation-request"
  "mm11_msgs/SetStripAnimationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripAnimation-request>)))
  "Returns md5sum for a message object of type '<SetStripAnimation-request>"
  "d1648b64d1990c75ff0a58f1ddcbf68a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripAnimation-request)))
  "Returns md5sum for a message object of type 'SetStripAnimation-request"
  "d1648b64d1990c75ff0a58f1ddcbf68a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripAnimation-request>)))
  "Returns full string definition for message of type '<SetStripAnimation-request>"
  (cl:format cl:nil "# Animate a whole strip~%~%uint8 port~%~%uint8 animation_id~%~%int16 param_1~%uint16 param_2~%~%uint8 r_1~%uint8 g_1~%uint8 b_1~%~%uint8 r_2~%uint8 g_2~%uint8 b_2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripAnimation-request)))
  "Returns full string definition for message of type 'SetStripAnimation-request"
  (cl:format cl:nil "# Animate a whole strip~%~%uint8 port~%~%uint8 animation_id~%~%int16 param_1~%uint16 param_2~%~%uint8 r_1~%uint8 g_1~%uint8 b_1~%~%uint8 r_2~%uint8 g_2~%uint8 b_2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripAnimation-request>))
  (cl:+ 0
     1
     1
     2
     2
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripAnimation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripAnimation-request
    (cl:cons ':port (port msg))
    (cl:cons ':animation_id (animation_id msg))
    (cl:cons ':param_1 (param_1 msg))
    (cl:cons ':param_2 (param_2 msg))
    (cl:cons ':r_1 (r_1 msg))
    (cl:cons ':g_1 (g_1 msg))
    (cl:cons ':b_1 (b_1 msg))
    (cl:cons ':r_2 (r_2 msg))
    (cl:cons ':g_2 (g_2 msg))
    (cl:cons ':b_2 (b_2 msg))
))
;//! \htmlinclude SetStripAnimation-response.msg.html

(cl:defclass <SetStripAnimation-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetStripAnimation-response (<SetStripAnimation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripAnimation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripAnimation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripAnimation-response> is deprecated: use mm11_msgs-srv:SetStripAnimation-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripAnimation-response>) ostream)
  "Serializes a message object of type '<SetStripAnimation-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripAnimation-response>) istream)
  "Deserializes a message object of type '<SetStripAnimation-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripAnimation-response>)))
  "Returns string type for a service object of type '<SetStripAnimation-response>"
  "mm11_msgs/SetStripAnimationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripAnimation-response)))
  "Returns string type for a service object of type 'SetStripAnimation-response"
  "mm11_msgs/SetStripAnimationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripAnimation-response>)))
  "Returns md5sum for a message object of type '<SetStripAnimation-response>"
  "d1648b64d1990c75ff0a58f1ddcbf68a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripAnimation-response)))
  "Returns md5sum for a message object of type 'SetStripAnimation-response"
  "d1648b64d1990c75ff0a58f1ddcbf68a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripAnimation-response>)))
  "Returns full string definition for message of type '<SetStripAnimation-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripAnimation-response)))
  "Returns full string definition for message of type 'SetStripAnimation-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripAnimation-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripAnimation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripAnimation-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStripAnimation)))
  'SetStripAnimation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStripAnimation)))
  'SetStripAnimation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripAnimation)))
  "Returns string type for a service object of type '<SetStripAnimation>"
  "mm11_msgs/SetStripAnimation")