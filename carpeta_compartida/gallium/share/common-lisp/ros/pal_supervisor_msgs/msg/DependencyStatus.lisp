; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-msg)


;//! \htmlinclude DependencyStatus.msg.html

(cl:defclass <DependencyStatus> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (level
    :reader level
    :initarg :level
    :type cl:integer
    :initform 0))
)

(cl:defclass DependencyStatus (<DependencyStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DependencyStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DependencyStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-msg:<DependencyStatus> is deprecated: use pal_supervisor_msgs-msg:DependencyStatus instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <DependencyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-msg:name-val is deprecated.  Use pal_supervisor_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <DependencyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-msg:level-val is deprecated.  Use pal_supervisor_msgs-msg:level instead.")
  (level m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DependencyStatus>)))
    "Constants for message type '<DependencyStatus>"
  '((:OK . 0)
    (:WARN . 1)
    (:ERROR . 2)
    (:STALE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DependencyStatus)))
    "Constants for message type 'DependencyStatus"
  '((:OK . 0)
    (:WARN . 1)
    (:ERROR . 2)
    (:STALE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DependencyStatus>) ostream)
  "Serializes a message object of type '<DependencyStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'level)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DependencyStatus>) istream)
  "Deserializes a message object of type '<DependencyStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'level)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DependencyStatus>)))
  "Returns string type for a message object of type '<DependencyStatus>"
  "pal_supervisor_msgs/DependencyStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DependencyStatus)))
  "Returns string type for a message object of type 'DependencyStatus"
  "pal_supervisor_msgs/DependencyStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DependencyStatus>)))
  "Returns md5sum for a message object of type '<DependencyStatus>"
  "d472e0bb1e75f743e86f41e4cedcf8bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DependencyStatus)))
  "Returns md5sum for a message object of type 'DependencyStatus"
  "d472e0bb1e75f743e86f41e4cedcf8bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DependencyStatus>)))
  "Returns full string definition for message of type '<DependencyStatus>"
  (cl:format cl:nil "string name # dependency name~%# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3 #Application not running~%byte level # level of operation enumerated above ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DependencyStatus)))
  "Returns full string definition for message of type 'DependencyStatus"
  (cl:format cl:nil "string name # dependency name~%# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3 #Application not running~%byte level # level of operation enumerated above ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DependencyStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DependencyStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'DependencyStatus
    (cl:cons ':name (name msg))
    (cl:cons ':level (level msg))
))
