; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-srv)


;//! \htmlinclude SetStripFlash-request.msg.html

(cl:defclass <SetStripFlash-request> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:fixnum
    :initform 0)
   (period
    :reader period
    :initarg :period
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

(cl:defclass SetStripFlash-request (<SetStripFlash-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripFlash-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripFlash-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripFlash-request> is deprecated: use mm11_msgs-srv:SetStripFlash-request instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:port-val is deprecated.  Use mm11_msgs-srv:port instead.")
  (port m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:time-val is deprecated.  Use mm11_msgs-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:period-val is deprecated.  Use mm11_msgs-srv:period instead.")
  (period m))

(cl:ensure-generic-function 'r_1-val :lambda-list '(m))
(cl:defmethod r_1-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:r_1-val is deprecated.  Use mm11_msgs-srv:r_1 instead.")
  (r_1 m))

(cl:ensure-generic-function 'g_1-val :lambda-list '(m))
(cl:defmethod g_1-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:g_1-val is deprecated.  Use mm11_msgs-srv:g_1 instead.")
  (g_1 m))

(cl:ensure-generic-function 'b_1-val :lambda-list '(m))
(cl:defmethod b_1-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:b_1-val is deprecated.  Use mm11_msgs-srv:b_1 instead.")
  (b_1 m))

(cl:ensure-generic-function 'r_2-val :lambda-list '(m))
(cl:defmethod r_2-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:r_2-val is deprecated.  Use mm11_msgs-srv:r_2 instead.")
  (r_2 m))

(cl:ensure-generic-function 'g_2-val :lambda-list '(m))
(cl:defmethod g_2-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:g_2-val is deprecated.  Use mm11_msgs-srv:g_2 instead.")
  (g_2 m))

(cl:ensure-generic-function 'b_2-val :lambda-list '(m))
(cl:defmethod b_2-val ((m <SetStripFlash-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-srv:b_2-val is deprecated.  Use mm11_msgs-srv:b_2 instead.")
  (b_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripFlash-request>) ostream)
  "Serializes a message object of type '<SetStripFlash-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripFlash-request>) istream)
  "Deserializes a message object of type '<SetStripFlash-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'r_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'g_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'b_2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripFlash-request>)))
  "Returns string type for a service object of type '<SetStripFlash-request>"
  "mm11_msgs/SetStripFlashRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripFlash-request)))
  "Returns string type for a service object of type 'SetStripFlash-request"
  "mm11_msgs/SetStripFlashRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripFlash-request>)))
  "Returns md5sum for a message object of type '<SetStripFlash-request>"
  "4d41df4b77ebb64e24b098b717616db6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripFlash-request)))
  "Returns md5sum for a message object of type 'SetStripFlash-request"
  "4d41df4b77ebb64e24b098b717616db6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripFlash-request>)))
  "Returns full string definition for message of type '<SetStripFlash-request>"
  (cl:format cl:nil "# Flashing for a whole strip~%~%uint8 port~%~%uint16 time~%uint16 period~%~%uint8 r_1~%uint8 g_1~%uint8 b_1~%~%uint8 r_2~%uint8 g_2~%uint8 b_2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripFlash-request)))
  "Returns full string definition for message of type 'SetStripFlash-request"
  (cl:format cl:nil "# Flashing for a whole strip~%~%uint8 port~%~%uint16 time~%uint16 period~%~%uint8 r_1~%uint8 g_1~%uint8 b_1~%~%uint8 r_2~%uint8 g_2~%uint8 b_2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripFlash-request>))
  (cl:+ 0
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripFlash-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripFlash-request
    (cl:cons ':port (port msg))
    (cl:cons ':time (time msg))
    (cl:cons ':period (period msg))
    (cl:cons ':r_1 (r_1 msg))
    (cl:cons ':g_1 (g_1 msg))
    (cl:cons ':b_1 (b_1 msg))
    (cl:cons ':r_2 (r_2 msg))
    (cl:cons ':g_2 (g_2 msg))
    (cl:cons ':b_2 (b_2 msg))
))
;//! \htmlinclude SetStripFlash-response.msg.html

(cl:defclass <SetStripFlash-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetStripFlash-response (<SetStripFlash-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStripFlash-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStripFlash-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-srv:<SetStripFlash-response> is deprecated: use mm11_msgs-srv:SetStripFlash-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStripFlash-response>) ostream)
  "Serializes a message object of type '<SetStripFlash-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStripFlash-response>) istream)
  "Deserializes a message object of type '<SetStripFlash-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStripFlash-response>)))
  "Returns string type for a service object of type '<SetStripFlash-response>"
  "mm11_msgs/SetStripFlashResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripFlash-response)))
  "Returns string type for a service object of type 'SetStripFlash-response"
  "mm11_msgs/SetStripFlashResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStripFlash-response>)))
  "Returns md5sum for a message object of type '<SetStripFlash-response>"
  "4d41df4b77ebb64e24b098b717616db6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStripFlash-response)))
  "Returns md5sum for a message object of type 'SetStripFlash-response"
  "4d41df4b77ebb64e24b098b717616db6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStripFlash-response>)))
  "Returns full string definition for message of type '<SetStripFlash-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStripFlash-response)))
  "Returns full string definition for message of type 'SetStripFlash-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStripFlash-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStripFlash-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStripFlash-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStripFlash)))
  'SetStripFlash-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStripFlash)))
  'SetStripFlash-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStripFlash)))
  "Returns string type for a service object of type '<SetStripFlash>"
  "mm11_msgs/SetStripFlash")