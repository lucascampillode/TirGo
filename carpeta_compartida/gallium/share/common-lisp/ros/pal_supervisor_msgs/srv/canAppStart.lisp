; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude canAppStart-request.msg.html

(cl:defclass <canAppStart-request> (roslisp-msg-protocol:ros-message)
  ((appName
    :reader appName
    :initarg :appName
    :type cl:string
    :initform ""))
)

(cl:defclass canAppStart-request (<canAppStart-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <canAppStart-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'canAppStart-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<canAppStart-request> is deprecated: use pal_supervisor_msgs-srv:canAppStart-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <canAppStart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <canAppStart-request>) ostream)
  "Serializes a message object of type '<canAppStart-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'appName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'appName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <canAppStart-request>) istream)
  "Deserializes a message object of type '<canAppStart-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<canAppStart-request>)))
  "Returns string type for a service object of type '<canAppStart-request>"
  "pal_supervisor_msgs/canAppStartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'canAppStart-request)))
  "Returns string type for a service object of type 'canAppStart-request"
  "pal_supervisor_msgs/canAppStartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<canAppStart-request>)))
  "Returns md5sum for a message object of type '<canAppStart-request>"
  "2a4675c414f30fa71f2e7af8b08bf502")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'canAppStart-request)))
  "Returns md5sum for a message object of type 'canAppStart-request"
  "2a4675c414f30fa71f2e7af8b08bf502")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<canAppStart-request>)))
  "Returns full string definition for message of type '<canAppStart-request>"
  (cl:format cl:nil "string appName #Name of the application ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'canAppStart-request)))
  "Returns full string definition for message of type 'canAppStart-request"
  (cl:format cl:nil "string appName #Name of the application ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <canAppStart-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <canAppStart-request>))
  "Converts a ROS message object to a list"
  (cl:list 'canAppStart-request
    (cl:cons ':appName (appName msg))
))
;//! \htmlinclude canAppStart-response.msg.html

(cl:defclass <canAppStart-response> (roslisp-msg-protocol:ros-message)
  ((canStart
    :reader canStart
    :initarg :canStart
    :type cl:boolean
    :initform cl:nil)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass canAppStart-response (<canAppStart-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <canAppStart-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'canAppStart-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<canAppStart-response> is deprecated: use pal_supervisor_msgs-srv:canAppStart-response instead.")))

(cl:ensure-generic-function 'canStart-val :lambda-list '(m))
(cl:defmethod canStart-val ((m <canAppStart-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:canStart-val is deprecated.  Use pal_supervisor_msgs-srv:canStart instead.")
  (canStart m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <canAppStart-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:description-val is deprecated.  Use pal_supervisor_msgs-srv:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <canAppStart-response>) ostream)
  "Serializes a message object of type '<canAppStart-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'canStart) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <canAppStart-response>) istream)
  "Deserializes a message object of type '<canAppStart-response>"
    (cl:setf (cl:slot-value msg 'canStart) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<canAppStart-response>)))
  "Returns string type for a service object of type '<canAppStart-response>"
  "pal_supervisor_msgs/canAppStartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'canAppStart-response)))
  "Returns string type for a service object of type 'canAppStart-response"
  "pal_supervisor_msgs/canAppStartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<canAppStart-response>)))
  "Returns md5sum for a message object of type '<canAppStart-response>"
  "2a4675c414f30fa71f2e7af8b08bf502")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'canAppStart-response)))
  "Returns md5sum for a message object of type 'canAppStart-response"
  "2a4675c414f30fa71f2e7af8b08bf502")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<canAppStart-response>)))
  "Returns full string definition for message of type '<canAppStart-response>"
  (cl:format cl:nil "bool canStart #True if the application can start~%string description  #Reason why an application can't start. empty if can start~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'canAppStart-response)))
  "Returns full string definition for message of type 'canAppStart-response"
  (cl:format cl:nil "bool canStart #True if the application can start~%string description  #Reason why an application can't start. empty if can start~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <canAppStart-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <canAppStart-response>))
  "Converts a ROS message object to a list"
  (cl:list 'canAppStart-response
    (cl:cons ':canStart (canStart msg))
    (cl:cons ':description (description msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'canAppStart)))
  'canAppStart-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'canAppStart)))
  'canAppStart-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'canAppStart)))
  "Returns string type for a service object of type '<canAppStart>"
  "pal_supervisor_msgs/canAppStart")