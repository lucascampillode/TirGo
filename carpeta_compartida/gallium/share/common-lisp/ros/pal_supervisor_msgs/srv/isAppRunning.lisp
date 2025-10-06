; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude isAppRunning-request.msg.html

(cl:defclass <isAppRunning-request> (roslisp-msg-protocol:ros-message)
  ((appName
    :reader appName
    :initarg :appName
    :type cl:string
    :initform ""))
)

(cl:defclass isAppRunning-request (<isAppRunning-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <isAppRunning-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'isAppRunning-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<isAppRunning-request> is deprecated: use pal_supervisor_msgs-srv:isAppRunning-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <isAppRunning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <isAppRunning-request>) ostream)
  "Serializes a message object of type '<isAppRunning-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'appName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'appName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <isAppRunning-request>) istream)
  "Deserializes a message object of type '<isAppRunning-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'appName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'appName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<isAppRunning-request>)))
  "Returns string type for a service object of type '<isAppRunning-request>"
  "pal_supervisor_msgs/isAppRunningRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'isAppRunning-request)))
  "Returns string type for a service object of type 'isAppRunning-request"
  "pal_supervisor_msgs/isAppRunningRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<isAppRunning-request>)))
  "Returns md5sum for a message object of type '<isAppRunning-request>"
  "c82b84a4b8d7b2c6e1185b6a3564ad39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'isAppRunning-request)))
  "Returns md5sum for a message object of type 'isAppRunning-request"
  "c82b84a4b8d7b2c6e1185b6a3564ad39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<isAppRunning-request>)))
  "Returns full string definition for message of type '<isAppRunning-request>"
  (cl:format cl:nil "string appName #Name of the application ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'isAppRunning-request)))
  "Returns full string definition for message of type 'isAppRunning-request"
  (cl:format cl:nil "string appName #Name of the application ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <isAppRunning-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <isAppRunning-request>))
  "Converts a ROS message object to a list"
  (cl:list 'isAppRunning-request
    (cl:cons ':appName (appName msg))
))
;//! \htmlinclude isAppRunning-response.msg.html

(cl:defclass <isAppRunning-response> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:integer
    :initform 0))
)

(cl:defclass isAppRunning-response (<isAppRunning-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <isAppRunning-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'isAppRunning-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<isAppRunning-response> is deprecated: use pal_supervisor_msgs-srv:isAppRunning-response instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <isAppRunning-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:level-val is deprecated.  Use pal_supervisor_msgs-srv:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<isAppRunning-response>)))
    "Constants for message type '<isAppRunning-response>"
  '((:OK . 0)
    (:WARN . 1)
    (:ERROR . 2)
    (:STALE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'isAppRunning-response)))
    "Constants for message type 'isAppRunning-response"
  '((:OK . 0)
    (:WARN . 1)
    (:ERROR . 2)
    (:STALE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <isAppRunning-response>) ostream)
  "Serializes a message object of type '<isAppRunning-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'level)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <isAppRunning-response>) istream)
  "Deserializes a message object of type '<isAppRunning-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'level)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<isAppRunning-response>)))
  "Returns string type for a service object of type '<isAppRunning-response>"
  "pal_supervisor_msgs/isAppRunningResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'isAppRunning-response)))
  "Returns string type for a service object of type 'isAppRunning-response"
  "pal_supervisor_msgs/isAppRunningResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<isAppRunning-response>)))
  "Returns md5sum for a message object of type '<isAppRunning-response>"
  "c82b84a4b8d7b2c6e1185b6a3564ad39")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'isAppRunning-response)))
  "Returns md5sum for a message object of type 'isAppRunning-response"
  "c82b84a4b8d7b2c6e1185b6a3564ad39")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<isAppRunning-response>)))
  "Returns full string definition for message of type '<isAppRunning-response>"
  (cl:format cl:nil "# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3 #Application not running~%byte level # level of operation enumerated above ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'isAppRunning-response)))
  "Returns full string definition for message of type 'isAppRunning-response"
  (cl:format cl:nil "# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3 #Application not running~%byte level # level of operation enumerated above ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <isAppRunning-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <isAppRunning-response>))
  "Converts a ROS message object to a list"
  (cl:list 'isAppRunning-response
    (cl:cons ':level (level msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'isAppRunning)))
  'isAppRunning-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'isAppRunning)))
  'isAppRunning-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'isAppRunning)))
  "Returns string type for a service object of type '<isAppRunning>"
  "pal_supervisor_msgs/isAppRunning")