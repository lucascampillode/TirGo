; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude shMemGetUserId-request.msg.html

(cl:defclass <shMemGetUserId-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass shMemGetUserId-request (<shMemGetUserId-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemGetUserId-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemGetUserId-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemGetUserId-request> is deprecated: use pal_supervisor_msgs-srv:shMemGetUserId-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <shMemGetUserId-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <shMemGetUserId-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:key-val is deprecated.  Use pal_supervisor_msgs-srv:key instead.")
  (key m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemGetUserId-request>) ostream)
  "Serializes a message object of type '<shMemGetUserId-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemGetUserId-request>) istream)
  "Deserializes a message object of type '<shMemGetUserId-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemGetUserId-request>)))
  "Returns string type for a service object of type '<shMemGetUserId-request>"
  "pal_supervisor_msgs/shMemGetUserIdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemGetUserId-request)))
  "Returns string type for a service object of type 'shMemGetUserId-request"
  "pal_supervisor_msgs/shMemGetUserIdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemGetUserId-request>)))
  "Returns md5sum for a message object of type '<shMemGetUserId-request>"
  "4c3734a520e124bb4c1f359d31845707")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemGetUserId-request)))
  "Returns md5sum for a message object of type 'shMemGetUserId-request"
  "4c3734a520e124bb4c1f359d31845707")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemGetUserId-request>)))
  "Returns full string definition for message of type '<shMemGetUserId-request>"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemGetUserId-request)))
  "Returns full string definition for message of type 'shMemGetUserId-request"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemGetUserId-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemGetUserId-request>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemGetUserId-request
    (cl:cons ':appName (appName msg))
    (cl:cons ':key (key msg))
))
;//! \htmlinclude shMemGetUserId-response.msg.html

(cl:defclass <shMemGetUserId-response> (roslisp-msg-protocol:ros-message)
  ((userId
    :reader userId
    :initarg :userId
    :type cl:string
    :initform ""))
)

(cl:defclass shMemGetUserId-response (<shMemGetUserId-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemGetUserId-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemGetUserId-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemGetUserId-response> is deprecated: use pal_supervisor_msgs-srv:shMemGetUserId-response instead.")))

(cl:ensure-generic-function 'userId-val :lambda-list '(m))
(cl:defmethod userId-val ((m <shMemGetUserId-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:userId-val is deprecated.  Use pal_supervisor_msgs-srv:userId instead.")
  (userId m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemGetUserId-response>) ostream)
  "Serializes a message object of type '<shMemGetUserId-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'userId))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'userId))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemGetUserId-response>) istream)
  "Deserializes a message object of type '<shMemGetUserId-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'userId) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'userId) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemGetUserId-response>)))
  "Returns string type for a service object of type '<shMemGetUserId-response>"
  "pal_supervisor_msgs/shMemGetUserIdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemGetUserId-response)))
  "Returns string type for a service object of type 'shMemGetUserId-response"
  "pal_supervisor_msgs/shMemGetUserIdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemGetUserId-response>)))
  "Returns md5sum for a message object of type '<shMemGetUserId-response>"
  "4c3734a520e124bb4c1f359d31845707")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemGetUserId-response)))
  "Returns md5sum for a message object of type 'shMemGetUserId-response"
  "4c3734a520e124bb4c1f359d31845707")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemGetUserId-response>)))
  "Returns full string definition for message of type '<shMemGetUserId-response>"
  (cl:format cl:nil "string userId #Returns the user ID related to the Shared memory~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemGetUserId-response)))
  "Returns full string definition for message of type 'shMemGetUserId-response"
  (cl:format cl:nil "string userId #Returns the user ID related to the Shared memory~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemGetUserId-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'userId))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemGetUserId-response>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemGetUserId-response
    (cl:cons ':userId (userId msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'shMemGetUserId)))
  'shMemGetUserId-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'shMemGetUserId)))
  'shMemGetUserId-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemGetUserId)))
  "Returns string type for a service object of type '<shMemGetUserId>"
  "pal_supervisor_msgs/shMemGetUserId")