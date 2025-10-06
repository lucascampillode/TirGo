; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude shMemIsAvailable-request.msg.html

(cl:defclass <shMemIsAvailable-request> (roslisp-msg-protocol:ros-message)
  ((appName
    :reader appName
    :initarg :appName
    :type cl:string
    :initform "")
   (key
    :reader key
    :initarg :key
    :type cl:integer
    :initform 0))
)

(cl:defclass shMemIsAvailable-request (<shMemIsAvailable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemIsAvailable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemIsAvailable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemIsAvailable-request> is deprecated: use pal_supervisor_msgs-srv:shMemIsAvailable-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <shMemIsAvailable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <shMemIsAvailable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:key-val is deprecated.  Use pal_supervisor_msgs-srv:key instead.")
  (key m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemIsAvailable-request>) ostream)
  "Serializes a message object of type '<shMemIsAvailable-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'appName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'appName))
  (cl:let* ((signed (cl:slot-value msg 'key)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemIsAvailable-request>) istream)
  "Deserializes a message object of type '<shMemIsAvailable-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'appName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'appName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemIsAvailable-request>)))
  "Returns string type for a service object of type '<shMemIsAvailable-request>"
  "pal_supervisor_msgs/shMemIsAvailableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsAvailable-request)))
  "Returns string type for a service object of type 'shMemIsAvailable-request"
  "pal_supervisor_msgs/shMemIsAvailableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemIsAvailable-request>)))
  "Returns md5sum for a message object of type '<shMemIsAvailable-request>"
  "ecf0a07e30d52a9c701ab569c846bf9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemIsAvailable-request)))
  "Returns md5sum for a message object of type 'shMemIsAvailable-request"
  "ecf0a07e30d52a9c701ab569c846bf9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemIsAvailable-request>)))
  "Returns full string definition for message of type '<shMemIsAvailable-request>"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemIsAvailable-request)))
  "Returns full string definition for message of type 'shMemIsAvailable-request"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemIsAvailable-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemIsAvailable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemIsAvailable-request
    (cl:cons ':appName (appName msg))
    (cl:cons ':key (key msg))
))
;//! \htmlinclude shMemIsAvailable-response.msg.html

(cl:defclass <shMemIsAvailable-response> (roslisp-msg-protocol:ros-message)
  ((available
    :reader available
    :initarg :available
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass shMemIsAvailable-response (<shMemIsAvailable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemIsAvailable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemIsAvailable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemIsAvailable-response> is deprecated: use pal_supervisor_msgs-srv:shMemIsAvailable-response instead.")))

(cl:ensure-generic-function 'available-val :lambda-list '(m))
(cl:defmethod available-val ((m <shMemIsAvailable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:available-val is deprecated.  Use pal_supervisor_msgs-srv:available instead.")
  (available m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemIsAvailable-response>) ostream)
  "Serializes a message object of type '<shMemIsAvailable-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'available) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemIsAvailable-response>) istream)
  "Deserializes a message object of type '<shMemIsAvailable-response>"
    (cl:setf (cl:slot-value msg 'available) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemIsAvailable-response>)))
  "Returns string type for a service object of type '<shMemIsAvailable-response>"
  "pal_supervisor_msgs/shMemIsAvailableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsAvailable-response)))
  "Returns string type for a service object of type 'shMemIsAvailable-response"
  "pal_supervisor_msgs/shMemIsAvailableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemIsAvailable-response>)))
  "Returns md5sum for a message object of type '<shMemIsAvailable-response>"
  "ecf0a07e30d52a9c701ab569c846bf9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemIsAvailable-response)))
  "Returns md5sum for a message object of type 'shMemIsAvailable-response"
  "ecf0a07e30d52a9c701ab569c846bf9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemIsAvailable-response>)))
  "Returns full string definition for message of type '<shMemIsAvailable-response>"
  (cl:format cl:nil "bool available #True if the sharedMemory with the given key is active and opened by it's owner.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemIsAvailable-response)))
  "Returns full string definition for message of type 'shMemIsAvailable-response"
  (cl:format cl:nil "bool available #True if the sharedMemory with the given key is active and opened by it's owner.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemIsAvailable-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemIsAvailable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemIsAvailable-response
    (cl:cons ':available (available msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'shMemIsAvailable)))
  'shMemIsAvailable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'shMemIsAvailable)))
  'shMemIsAvailable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsAvailable)))
  "Returns string type for a service object of type '<shMemIsAvailable>"
  "pal_supervisor_msgs/shMemIsAvailable")