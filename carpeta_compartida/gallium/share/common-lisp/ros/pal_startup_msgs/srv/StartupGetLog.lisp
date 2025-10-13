; Auto-generated. Do not edit!


(cl:in-package pal_startup_msgs-srv)


;//! \htmlinclude StartupGetLog-request.msg.html

(cl:defclass <StartupGetLog-request> (roslisp-msg-protocol:ros-message)
  ((app
    :reader app
    :initarg :app
    :type cl:string
    :initform "")
   (nlines
    :reader nlines
    :initarg :nlines
    :type cl:integer
    :initform 0))
)

(cl:defclass StartupGetLog-request (<StartupGetLog-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupGetLog-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupGetLog-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupGetLog-request> is deprecated: use pal_startup_msgs-srv:StartupGetLog-request instead.")))

(cl:ensure-generic-function 'app-val :lambda-list '(m))
(cl:defmethod app-val ((m <StartupGetLog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:app-val is deprecated.  Use pal_startup_msgs-srv:app instead.")
  (app m))

(cl:ensure-generic-function 'nlines-val :lambda-list '(m))
(cl:defmethod nlines-val ((m <StartupGetLog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:nlines-val is deprecated.  Use pal_startup_msgs-srv:nlines instead.")
  (nlines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupGetLog-request>) ostream)
  "Serializes a message object of type '<StartupGetLog-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'app))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'app))
  (cl:let* ((signed (cl:slot-value msg 'nlines)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupGetLog-request>) istream)
  "Deserializes a message object of type '<StartupGetLog-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'app) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'app) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nlines) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupGetLog-request>)))
  "Returns string type for a service object of type '<StartupGetLog-request>"
  "pal_startup_msgs/StartupGetLogRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupGetLog-request)))
  "Returns string type for a service object of type 'StartupGetLog-request"
  "pal_startup_msgs/StartupGetLogRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupGetLog-request>)))
  "Returns md5sum for a message object of type '<StartupGetLog-request>"
  "6bc85f88abf333b9c46f2b5155e5e9f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupGetLog-request)))
  "Returns md5sum for a message object of type 'StartupGetLog-request"
  "6bc85f88abf333b9c46f2b5155e5e9f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupGetLog-request>)))
  "Returns full string definition for message of type '<StartupGetLog-request>"
  (cl:format cl:nil "# Returns nlines last lines of app's log~%string app~%int32 nlines~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupGetLog-request)))
  "Returns full string definition for message of type 'StartupGetLog-request"
  (cl:format cl:nil "# Returns nlines last lines of app's log~%string app~%int32 nlines~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupGetLog-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'app))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupGetLog-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupGetLog-request
    (cl:cons ':app (app msg))
    (cl:cons ':nlines (nlines msg))
))
;//! \htmlinclude StartupGetLog-response.msg.html

(cl:defclass <StartupGetLog-response> (roslisp-msg-protocol:ros-message)
  ((log
    :reader log
    :initarg :log
    :type cl:string
    :initform ""))
)

(cl:defclass StartupGetLog-response (<StartupGetLog-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupGetLog-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupGetLog-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupGetLog-response> is deprecated: use pal_startup_msgs-srv:StartupGetLog-response instead.")))

(cl:ensure-generic-function 'log-val :lambda-list '(m))
(cl:defmethod log-val ((m <StartupGetLog-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:log-val is deprecated.  Use pal_startup_msgs-srv:log instead.")
  (log m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupGetLog-response>) ostream)
  "Serializes a message object of type '<StartupGetLog-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'log))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'log))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupGetLog-response>) istream)
  "Deserializes a message object of type '<StartupGetLog-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'log) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'log) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupGetLog-response>)))
  "Returns string type for a service object of type '<StartupGetLog-response>"
  "pal_startup_msgs/StartupGetLogResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupGetLog-response)))
  "Returns string type for a service object of type 'StartupGetLog-response"
  "pal_startup_msgs/StartupGetLogResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupGetLog-response>)))
  "Returns md5sum for a message object of type '<StartupGetLog-response>"
  "6bc85f88abf333b9c46f2b5155e5e9f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupGetLog-response)))
  "Returns md5sum for a message object of type 'StartupGetLog-response"
  "6bc85f88abf333b9c46f2b5155e5e9f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupGetLog-response>)))
  "Returns full string definition for message of type '<StartupGetLog-response>"
  (cl:format cl:nil "string log~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupGetLog-response)))
  "Returns full string definition for message of type 'StartupGetLog-response"
  (cl:format cl:nil "string log~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupGetLog-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'log))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupGetLog-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupGetLog-response
    (cl:cons ':log (log msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartupGetLog)))
  'StartupGetLog-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartupGetLog)))
  'StartupGetLog-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupGetLog)))
  "Returns string type for a service object of type '<StartupGetLog>"
  "pal_startup_msgs/StartupGetLog")