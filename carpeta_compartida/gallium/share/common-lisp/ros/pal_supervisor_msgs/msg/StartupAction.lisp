; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-msg)


;//! \htmlinclude StartupAction.msg.html

(cl:defclass <StartupAction> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0))
)

(cl:defclass StartupAction (<StartupAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-msg:<StartupAction> is deprecated: use pal_supervisor_msgs-msg:StartupAction instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <StartupAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-msg:name-val is deprecated.  Use pal_supervisor_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <StartupAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-msg:action-val is deprecated.  Use pal_supervisor_msgs-msg:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StartupAction>)))
    "Constants for message type '<StartupAction>"
  '((:START . 0)
    (:ABORT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StartupAction)))
    "Constants for message type 'StartupAction"
  '((:START . 0)
    (:ABORT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupAction>) ostream)
  "Serializes a message object of type '<StartupAction>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupAction>) istream)
  "Deserializes a message object of type '<StartupAction>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupAction>)))
  "Returns string type for a message object of type '<StartupAction>"
  "pal_supervisor_msgs/StartupAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupAction)))
  "Returns string type for a message object of type 'StartupAction"
  "pal_supervisor_msgs/StartupAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupAction>)))
  "Returns md5sum for a message object of type '<StartupAction>"
  "c3b072b09f12c731c73300aff3335bc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupAction)))
  "Returns md5sum for a message object of type 'StartupAction"
  "c3b072b09f12c731c73300aff3335bc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupAction>)))
  "Returns full string definition for message of type '<StartupAction>"
  (cl:format cl:nil "string name # application name~%# Possible status of startup process~%byte START=0~%byte ABORT=1~%byte action # action to be done by startup process for the application. Enumerated above ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupAction)))
  "Returns full string definition for message of type 'StartupAction"
  (cl:format cl:nil "string name # application name~%# Possible status of startup process~%byte START=0~%byte ABORT=1~%byte action # action to be done by startup process for the application. Enumerated above ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupAction>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupAction>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupAction
    (cl:cons ':name (name msg))
    (cl:cons ':action (action msg))
))
