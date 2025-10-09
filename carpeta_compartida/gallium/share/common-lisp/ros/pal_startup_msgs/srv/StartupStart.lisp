; Auto-generated. Do not edit!


(cl:in-package pal_startup_msgs-srv)


;//! \htmlinclude StartupStart-request.msg.html

(cl:defclass <StartupStart-request> (roslisp-msg-protocol:ros-message)
  ((app
    :reader app
    :initarg :app
    :type cl:string
    :initform "")
   (args
    :reader args
    :initarg :args
    :type cl:string
    :initform ""))
)

(cl:defclass StartupStart-request (<StartupStart-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupStart-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupStart-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupStart-request> is deprecated: use pal_startup_msgs-srv:StartupStart-request instead.")))

(cl:ensure-generic-function 'app-val :lambda-list '(m))
(cl:defmethod app-val ((m <StartupStart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:app-val is deprecated.  Use pal_startup_msgs-srv:app instead.")
  (app m))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <StartupStart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:args-val is deprecated.  Use pal_startup_msgs-srv:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupStart-request>) ostream)
  "Serializes a message object of type '<StartupStart-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'app))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'app))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'args))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupStart-request>) istream)
  "Deserializes a message object of type '<StartupStart-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'app) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'app) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'args) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'args) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupStart-request>)))
  "Returns string type for a service object of type '<StartupStart-request>"
  "pal_startup_msgs/StartupStartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStart-request)))
  "Returns string type for a service object of type 'StartupStart-request"
  "pal_startup_msgs/StartupStartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupStart-request>)))
  "Returns md5sum for a message object of type '<StartupStart-request>"
  "9b78b04bdfab704be898cd6637524866")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupStart-request)))
  "Returns md5sum for a message object of type 'StartupStart-request"
  "9b78b04bdfab704be898cd6637524866")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupStart-request>)))
  "Returns full string definition for message of type '<StartupStart-request>"
  (cl:format cl:nil "string app~%string args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupStart-request)))
  "Returns full string definition for message of type 'StartupStart-request"
  (cl:format cl:nil "string app~%string args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupStart-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'app))
     4 (cl:length (cl:slot-value msg 'args))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupStart-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupStart-request
    (cl:cons ':app (app msg))
    (cl:cons ':args (args msg))
))
;//! \htmlinclude StartupStart-response.msg.html

(cl:defclass <StartupStart-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass StartupStart-response (<StartupStart-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupStart-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupStart-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-srv:<StartupStart-response> is deprecated: use pal_startup_msgs-srv:StartupStart-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <StartupStart-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-srv:result-val is deprecated.  Use pal_startup_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupStart-response>) ostream)
  "Serializes a message object of type '<StartupStart-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupStart-response>) istream)
  "Deserializes a message object of type '<StartupStart-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupStart-response>)))
  "Returns string type for a service object of type '<StartupStart-response>"
  "pal_startup_msgs/StartupStartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStart-response)))
  "Returns string type for a service object of type 'StartupStart-response"
  "pal_startup_msgs/StartupStartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupStart-response>)))
  "Returns md5sum for a message object of type '<StartupStart-response>"
  "9b78b04bdfab704be898cd6637524866")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupStart-response)))
  "Returns md5sum for a message object of type 'StartupStart-response"
  "9b78b04bdfab704be898cd6637524866")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupStart-response>)))
  "Returns full string definition for message of type '<StartupStart-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupStart-response)))
  "Returns full string definition for message of type 'StartupStart-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupStart-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupStart-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupStart-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartupStart)))
  'StartupStart-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartupStart)))
  'StartupStart-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStart)))
  "Returns string type for a service object of type '<StartupStart>"
  "pal_startup_msgs/StartupStart")