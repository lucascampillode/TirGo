; Auto-generated. Do not edit!


(cl:in-package pal_startup_msgs-srv)


;//! \htmlinclude StartupStop-request.msg.html

(cl:defclass <StartupStop-request> (roslisp-msg-protocol:ros-message)
  ((app
    :reader app
    :initarg :app
    :type cl:string
    :initform ""))
)

(cl:defclass StartupStop-request (<StartupStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupStop-request> is deprecated: use pal_startup_msgs-srv:StartupStop-request instead.")))

(cl:ensure-generic-function 'app-val :lambda-list '(m))
(cl:defmethod app-val ((m <StartupStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:app-val is deprecated.  Use pal_startup_msgs-srv:app instead.")
  (app m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupStop-request>) ostream)
  "Serializes a message object of type '<StartupStop-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'app))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'app))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupStop-request>) istream)
  "Deserializes a message object of type '<StartupStop-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupStop-request>)))
  "Returns string type for a service object of type '<StartupStop-request>"
  "pal_startup_msgs/StartupStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStop-request)))
  "Returns string type for a service object of type 'StartupStop-request"
  "pal_startup_msgs/StartupStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupStop-request>)))
  "Returns md5sum for a message object of type '<StartupStop-request>"
  "d0a14343d8976d7512bdd0e94c3d391e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupStop-request)))
  "Returns md5sum for a message object of type 'StartupStop-request"
  "d0a14343d8976d7512bdd0e94c3d391e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupStop-request>)))
  "Returns full string definition for message of type '<StartupStop-request>"
  (cl:format cl:nil "string app~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupStop-request)))
  "Returns full string definition for message of type 'StartupStop-request"
  (cl:format cl:nil "string app~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupStop-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'app))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupStop-request
    (cl:cons ':app (app msg))
))
;//! \htmlinclude StartupStop-response.msg.html

(cl:defclass <StartupStop-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass StartupStop-response (<StartupStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupStop-response> is deprecated: use pal_startup_msgs-srv:StartupStop-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <StartupStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:result-val is deprecated.  Use pal_startup_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupStop-response>) ostream)
  "Serializes a message object of type '<StartupStop-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupStop-response>) istream)
  "Deserializes a message object of type '<StartupStop-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupStop-response>)))
  "Returns string type for a service object of type '<StartupStop-response>"
  "pal_startup_msgs/StartupStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStop-response)))
  "Returns string type for a service object of type 'StartupStop-response"
  "pal_startup_msgs/StartupStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupStop-response>)))
  "Returns md5sum for a message object of type '<StartupStop-response>"
  "d0a14343d8976d7512bdd0e94c3d391e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupStop-response)))
  "Returns md5sum for a message object of type 'StartupStop-response"
  "d0a14343d8976d7512bdd0e94c3d391e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupStop-response>)))
  "Returns full string definition for message of type '<StartupStop-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupStop-response)))
  "Returns full string definition for message of type 'StartupStop-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupStop-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupStop-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartupStop)))
  'StartupStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartupStop)))
  'StartupStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStop)))
  "Returns string type for a service object of type '<StartupStop>"
  "pal_startup_msgs/StartupStop")