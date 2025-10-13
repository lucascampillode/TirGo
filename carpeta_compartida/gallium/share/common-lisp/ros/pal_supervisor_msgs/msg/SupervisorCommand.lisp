; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-msg)


;//! \htmlinclude SupervisorCommand.msg.html

(cl:defclass <SupervisorCommand> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (params
    :reader params
    :initarg :params
    :type (cl:vector pal_supervisor_msgs-msg:Parameter)
   :initform (cl:make-array 0 :element-type 'pal_supervisor_msgs-msg:Parameter :initial-element (cl:make-instance 'pal_supervisor_msgs-msg:Parameter))))
)

(cl:defclass SupervisorCommand (<SupervisorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SupervisorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SupervisorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-msg:<SupervisorCommand> is deprecated: use pal_supervisor_msgs-msg:SupervisorCommand instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SupervisorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-msg:name-val is deprecated.  Use pal_supervisor_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'params-val :lambda-list '(m))
(cl:defmethod params-val ((m <SupervisorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-msg:params-val is deprecated.  Use pal_supervisor_msgs-msg:params instead.")
  (params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SupervisorCommand>) ostream)
  "Serializes a message object of type '<SupervisorCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'params))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'params))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SupervisorCommand>) istream)
  "Deserializes a message object of type '<SupervisorCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'params) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'params)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pal_supervisor_msgs-msg:Parameter))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SupervisorCommand>)))
  "Returns string type for a message object of type '<SupervisorCommand>"
  "pal_supervisor_msgs/SupervisorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SupervisorCommand)))
  "Returns string type for a message object of type 'SupervisorCommand"
  "pal_supervisor_msgs/SupervisorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SupervisorCommand>)))
  "Returns md5sum for a message object of type '<SupervisorCommand>"
  "9389508dd5191c710316e34d805a4c6c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SupervisorCommand)))
  "Returns md5sum for a message object of type 'SupervisorCommand"
  "9389508dd5191c710316e34d805a4c6c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SupervisorCommand>)))
  "Returns full string definition for message of type '<SupervisorCommand>"
  (cl:format cl:nil "string name #Command name~%pal_supervisor_msgs/Parameter[] params #Pairs of name and value, include all the parameters for the command~%~%================================================================================~%MSG: pal_supervisor_msgs/Parameter~%string name # parameter name~%string value # value of the parameter, all non string types will be \"stringized\" if possible~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SupervisorCommand)))
  "Returns full string definition for message of type 'SupervisorCommand"
  (cl:format cl:nil "string name #Command name~%pal_supervisor_msgs/Parameter[] params #Pairs of name and value, include all the parameters for the command~%~%================================================================================~%MSG: pal_supervisor_msgs/Parameter~%string name # parameter name~%string value # value of the parameter, all non string types will be \"stringized\" if possible~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SupervisorCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'params) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SupervisorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'SupervisorCommand
    (cl:cons ':name (name msg))
    (cl:cons ':params (params msg))
))
