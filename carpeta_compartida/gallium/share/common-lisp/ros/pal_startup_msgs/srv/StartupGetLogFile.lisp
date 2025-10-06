; Auto-generated. Do not edit!


(cl:in-package pal_startup_msgs-srv)


;//! \htmlinclude StartupGetLogFile-request.msg.html

(cl:defclass <StartupGetLogFile-request> (roslisp-msg-protocol:ros-message)
  ((app
    :reader app
    :initarg :app
    :type cl:string
    :initform ""))
)

(cl:defclass StartupGetLogFile-request (<StartupGetLogFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupGetLogFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupGetLogFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupGetLogFile-request> is deprecated: use pal_startup_msgs-srv:StartupGetLogFile-request instead.")))

(cl:ensure-generic-function 'app-val :lambda-list '(m))
(cl:defmethod app-val ((m <StartupGetLogFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:app-val is deprecated.  Use pal_startup_msgs-srv:app instead.")
  (app m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupGetLogFile-request>) ostream)
  "Serializes a message object of type '<StartupGetLogFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'app))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'app))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupGetLogFile-request>) istream)
  "Deserializes a message object of type '<StartupGetLogFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'app) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'app) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupGetLogFile-request>)))
  "Returns string type for a service object of type '<StartupGetLogFile-request>"
  "pal_startup_msgs/StartupGetLogFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupGetLogFile-request)))
  "Returns string type for a service object of type 'StartupGetLogFile-request"
  "pal_startup_msgs/StartupGetLogFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupGetLogFile-request>)))
  "Returns md5sum for a message object of type '<StartupGetLogFile-request>"
  "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupGetLogFile-request)))
  "Returns md5sum for a message object of type 'StartupGetLogFile-request"
  "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupGetLogFile-request>)))
  "Returns full string definition for message of type '<StartupGetLogFile-request>"
  (cl:format cl:nil "string app~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupGetLogFile-request)))
  "Returns full string definition for message of type 'StartupGetLogFile-request"
  (cl:format cl:nil "string app~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupGetLogFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'app))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupGetLogFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupGetLogFile-request
    (cl:cons ':app (app msg))
))
;//! \htmlinclude StartupGetLogFile-response.msg.html

(cl:defclass <StartupGetLogFile-response> (roslisp-msg-protocol:ros-message)
  ((log_file
    :reader log_file
    :initarg :log_file
    :type cl:string
    :initform ""))
)

(cl:defclass StartupGetLogFile-response (<StartupGetLogFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupGetLogFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupGetLogFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupGetLogFile-response> is deprecated: use pal_startup_msgs-srv:StartupGetLogFile-response instead.")))

(cl:ensure-generic-function 'log_file-val :lambda-list '(m))
(cl:defmethod log_file-val ((m <StartupGetLogFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:log_file-val is deprecated.  Use pal_startup_msgs-srv:log_file instead.")
  (log_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupGetLogFile-response>) ostream)
  "Serializes a message object of type '<StartupGetLogFile-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'log_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'log_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupGetLogFile-response>) istream)
  "Deserializes a message object of type '<StartupGetLogFile-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'log_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'log_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupGetLogFile-response>)))
  "Returns string type for a service object of type '<StartupGetLogFile-response>"
  "pal_startup_msgs/StartupGetLogFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupGetLogFile-response)))
  "Returns string type for a service object of type 'StartupGetLogFile-response"
  "pal_startup_msgs/StartupGetLogFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupGetLogFile-response>)))
  "Returns md5sum for a message object of type '<StartupGetLogFile-response>"
  "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupGetLogFile-response)))
  "Returns md5sum for a message object of type 'StartupGetLogFile-response"
  "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupGetLogFile-response>)))
  "Returns full string definition for message of type '<StartupGetLogFile-response>"
  (cl:format cl:nil "string log_file~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupGetLogFile-response)))
  "Returns full string definition for message of type 'StartupGetLogFile-response"
  (cl:format cl:nil "string log_file~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupGetLogFile-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'log_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupGetLogFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupGetLogFile-response
    (cl:cons ':log_file (log_file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartupGetLogFile)))
  'StartupGetLogFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartupGetLogFile)))
  'StartupGetLogFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupGetLogFile)))
  "Returns string type for a service object of type '<StartupGetLogFile>"
  "pal_startup_msgs/StartupGetLogFile")