; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude shMemIsTimestampValid-request.msg.html

(cl:defclass <shMemIsTimestampValid-request> (roslisp-msg-protocol:ros-message)
  ((appName
    :reader appName
    :initarg :appName
    :type cl:string
    :initform "")
   (key
    :reader key
    :initarg :key
    :type cl:integer
    :initform 0)
   (offset
    :reader offset
    :initarg :offset
    :type cl:integer
    :initform 0))
)

(cl:defclass shMemIsTimestampValid-request (<shMemIsTimestampValid-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemIsTimestampValid-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemIsTimestampValid-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemIsTimestampValid-request> is deprecated: use pal_supervisor_msgs-srv:shMemIsTimestampValid-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <shMemIsTimestampValid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <shMemIsTimestampValid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:key-val is deprecated.  Use pal_supervisor_msgs-srv:key instead.")
  (key m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <shMemIsTimestampValid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:offset-val is deprecated.  Use pal_supervisor_msgs-srv:offset instead.")
  (offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemIsTimestampValid-request>) ostream)
  "Serializes a message object of type '<shMemIsTimestampValid-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'offset)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'offset)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'offset)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'offset)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemIsTimestampValid-request>) istream)
  "Deserializes a message object of type '<shMemIsTimestampValid-request>"
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'offset)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'offset)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'offset)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'offset)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemIsTimestampValid-request>)))
  "Returns string type for a service object of type '<shMemIsTimestampValid-request>"
  "pal_supervisor_msgs/shMemIsTimestampValidRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsTimestampValid-request)))
  "Returns string type for a service object of type 'shMemIsTimestampValid-request"
  "pal_supervisor_msgs/shMemIsTimestampValidRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemIsTimestampValid-request>)))
  "Returns md5sum for a message object of type '<shMemIsTimestampValid-request>"
  "8fd93cf90468f6698ba910ec3df45dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemIsTimestampValid-request)))
  "Returns md5sum for a message object of type 'shMemIsTimestampValid-request"
  "8fd93cf90468f6698ba910ec3df45dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemIsTimestampValid-request>)))
  "Returns full string definition for message of type '<shMemIsTimestampValid-request>"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%uint32 offset #Offset inside the ShMem of the timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemIsTimestampValid-request)))
  "Returns full string definition for message of type 'shMemIsTimestampValid-request"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%uint32 offset #Offset inside the ShMem of the timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemIsTimestampValid-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemIsTimestampValid-request>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemIsTimestampValid-request
    (cl:cons ':appName (appName msg))
    (cl:cons ':key (key msg))
    (cl:cons ':offset (offset msg))
))
;//! \htmlinclude shMemIsTimestampValid-response.msg.html

(cl:defclass <shMemIsTimestampValid-response> (roslisp-msg-protocol:ros-message)
  ((isValid
    :reader isValid
    :initarg :isValid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass shMemIsTimestampValid-response (<shMemIsTimestampValid-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemIsTimestampValid-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemIsTimestampValid-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemIsTimestampValid-response> is deprecated: use pal_supervisor_msgs-srv:shMemIsTimestampValid-response instead.")))

(cl:ensure-generic-function 'isValid-val :lambda-list '(m))
(cl:defmethod isValid-val ((m <shMemIsTimestampValid-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:isValid-val is deprecated.  Use pal_supervisor_msgs-srv:isValid instead.")
  (isValid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemIsTimestampValid-response>) ostream)
  "Serializes a message object of type '<shMemIsTimestampValid-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isValid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemIsTimestampValid-response>) istream)
  "Deserializes a message object of type '<shMemIsTimestampValid-response>"
    (cl:setf (cl:slot-value msg 'isValid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemIsTimestampValid-response>)))
  "Returns string type for a service object of type '<shMemIsTimestampValid-response>"
  "pal_supervisor_msgs/shMemIsTimestampValidResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsTimestampValid-response)))
  "Returns string type for a service object of type 'shMemIsTimestampValid-response"
  "pal_supervisor_msgs/shMemIsTimestampValidResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemIsTimestampValid-response>)))
  "Returns md5sum for a message object of type '<shMemIsTimestampValid-response>"
  "8fd93cf90468f6698ba910ec3df45dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemIsTimestampValid-response)))
  "Returns md5sum for a message object of type 'shMemIsTimestampValid-response"
  "8fd93cf90468f6698ba910ec3df45dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemIsTimestampValid-response>)))
  "Returns full string definition for message of type '<shMemIsTimestampValid-response>"
  (cl:format cl:nil "bool isValid #Returns if the timeout check is valid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemIsTimestampValid-response)))
  "Returns full string definition for message of type 'shMemIsTimestampValid-response"
  (cl:format cl:nil "bool isValid #Returns if the timeout check is valid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemIsTimestampValid-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemIsTimestampValid-response>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemIsTimestampValid-response
    (cl:cons ':isValid (isValid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'shMemIsTimestampValid)))
  'shMemIsTimestampValid-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'shMemIsTimestampValid)))
  'shMemIsTimestampValid-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsTimestampValid)))
  "Returns string type for a service object of type '<shMemIsTimestampValid>"
  "pal_supervisor_msgs/shMemIsTimestampValid")