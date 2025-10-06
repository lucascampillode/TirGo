; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude shMemIsClosed-request.msg.html

(cl:defclass <shMemIsClosed-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass shMemIsClosed-request (<shMemIsClosed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemIsClosed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemIsClosed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemIsClosed-request> is deprecated: use pal_supervisor_msgs-srv:shMemIsClosed-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <shMemIsClosed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <shMemIsClosed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:key-val is deprecated.  Use pal_supervisor_msgs-srv:key instead.")
  (key m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemIsClosed-request>) ostream)
  "Serializes a message object of type '<shMemIsClosed-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemIsClosed-request>) istream)
  "Deserializes a message object of type '<shMemIsClosed-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemIsClosed-request>)))
  "Returns string type for a service object of type '<shMemIsClosed-request>"
  "pal_supervisor_msgs/shMemIsClosedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsClosed-request)))
  "Returns string type for a service object of type 'shMemIsClosed-request"
  "pal_supervisor_msgs/shMemIsClosedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemIsClosed-request>)))
  "Returns md5sum for a message object of type '<shMemIsClosed-request>"
  "7d5da756f3c73d0365ab7b15abe7a3f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemIsClosed-request)))
  "Returns md5sum for a message object of type 'shMemIsClosed-request"
  "7d5da756f3c73d0365ab7b15abe7a3f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemIsClosed-request>)))
  "Returns full string definition for message of type '<shMemIsClosed-request>"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemIsClosed-request)))
  "Returns full string definition for message of type 'shMemIsClosed-request"
  (cl:format cl:nil "string appName #Name of the application sending the command~%int32 key #Shared memory Key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemIsClosed-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemIsClosed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemIsClosed-request
    (cl:cons ':appName (appName msg))
    (cl:cons ':key (key msg))
))
;//! \htmlinclude shMemIsClosed-response.msg.html

(cl:defclass <shMemIsClosed-response> (roslisp-msg-protocol:ros-message)
  ((isClosed
    :reader isClosed
    :initarg :isClosed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass shMemIsClosed-response (<shMemIsClosed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shMemIsClosed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shMemIsClosed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<shMemIsClosed-response> is deprecated: use pal_supervisor_msgs-srv:shMemIsClosed-response instead.")))

(cl:ensure-generic-function 'isClosed-val :lambda-list '(m))
(cl:defmethod isClosed-val ((m <shMemIsClosed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:isClosed-val is deprecated.  Use pal_supervisor_msgs-srv:isClosed instead.")
  (isClosed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shMemIsClosed-response>) ostream)
  "Serializes a message object of type '<shMemIsClosed-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isClosed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shMemIsClosed-response>) istream)
  "Deserializes a message object of type '<shMemIsClosed-response>"
    (cl:setf (cl:slot-value msg 'isClosed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shMemIsClosed-response>)))
  "Returns string type for a service object of type '<shMemIsClosed-response>"
  "pal_supervisor_msgs/shMemIsClosedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsClosed-response)))
  "Returns string type for a service object of type 'shMemIsClosed-response"
  "pal_supervisor_msgs/shMemIsClosedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shMemIsClosed-response>)))
  "Returns md5sum for a message object of type '<shMemIsClosed-response>"
  "7d5da756f3c73d0365ab7b15abe7a3f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shMemIsClosed-response)))
  "Returns md5sum for a message object of type 'shMemIsClosed-response"
  "7d5da756f3c73d0365ab7b15abe7a3f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shMemIsClosed-response>)))
  "Returns full string definition for message of type '<shMemIsClosed-response>"
  (cl:format cl:nil "bool isClosed #True if the sharedMemory with the given key has been closed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shMemIsClosed-response)))
  "Returns full string definition for message of type 'shMemIsClosed-response"
  (cl:format cl:nil "bool isClosed #True if the sharedMemory with the given key has been closed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shMemIsClosed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shMemIsClosed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'shMemIsClosed-response
    (cl:cons ':isClosed (isClosed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'shMemIsClosed)))
  'shMemIsClosed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'shMemIsClosed)))
  'shMemIsClosed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shMemIsClosed)))
  "Returns string type for a service object of type '<shMemIsClosed>"
  "pal_supervisor_msgs/shMemIsClosed")