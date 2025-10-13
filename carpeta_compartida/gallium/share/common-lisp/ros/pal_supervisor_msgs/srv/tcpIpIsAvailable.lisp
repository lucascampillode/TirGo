; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude tcpIpIsAvailable-request.msg.html

(cl:defclass <tcpIpIsAvailable-request> (roslisp-msg-protocol:ros-message)
  ((ipAddress
    :reader ipAddress
    :initarg :ipAddress
    :type cl:string
    :initform "")
   (port
    :reader port
    :initarg :port
    :type cl:integer
    :initform 0))
)

(cl:defclass tcpIpIsAvailable-request (<tcpIpIsAvailable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tcpIpIsAvailable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tcpIpIsAvailable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<tcpIpIsAvailable-request> is deprecated: use pal_supervisor_msgs-srv:tcpIpIsAvailable-request instead.")))

(cl:ensure-generic-function 'ipAddress-val :lambda-list '(m))
(cl:defmethod ipAddress-val ((m <tcpIpIsAvailable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:ipAddress-val is deprecated.  Use pal_supervisor_msgs-srv:ipAddress instead.")
  (ipAddress m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <tcpIpIsAvailable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:port-val is deprecated.  Use pal_supervisor_msgs-srv:port instead.")
  (port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tcpIpIsAvailable-request>) ostream)
  "Serializes a message object of type '<tcpIpIsAvailable-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ipAddress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ipAddress))
  (cl:let* ((signed (cl:slot-value msg 'port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tcpIpIsAvailable-request>) istream)
  "Deserializes a message object of type '<tcpIpIsAvailable-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ipAddress) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ipAddress) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tcpIpIsAvailable-request>)))
  "Returns string type for a service object of type '<tcpIpIsAvailable-request>"
  "pal_supervisor_msgs/tcpIpIsAvailableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tcpIpIsAvailable-request)))
  "Returns string type for a service object of type 'tcpIpIsAvailable-request"
  "pal_supervisor_msgs/tcpIpIsAvailableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tcpIpIsAvailable-request>)))
  "Returns md5sum for a message object of type '<tcpIpIsAvailable-request>"
  "c87e9a2a51d895bccdb0157125a69381")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tcpIpIsAvailable-request)))
  "Returns md5sum for a message object of type 'tcpIpIsAvailable-request"
  "c87e9a2a51d895bccdb0157125a69381")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tcpIpIsAvailable-request>)))
  "Returns full string definition for message of type '<tcpIpIsAvailable-request>"
  (cl:format cl:nil "string ipAddress #Ip of the TCP Server~%int32 port #Port to use~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tcpIpIsAvailable-request)))
  "Returns full string definition for message of type 'tcpIpIsAvailable-request"
  (cl:format cl:nil "string ipAddress #Ip of the TCP Server~%int32 port #Port to use~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tcpIpIsAvailable-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ipAddress))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tcpIpIsAvailable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'tcpIpIsAvailable-request
    (cl:cons ':ipAddress (ipAddress msg))
    (cl:cons ':port (port msg))
))
;//! \htmlinclude tcpIpIsAvailable-response.msg.html

(cl:defclass <tcpIpIsAvailable-response> (roslisp-msg-protocol:ros-message)
  ((available
    :reader available
    :initarg :available
    :type cl:boolean
    :initform cl:nil)
   (suggestedSleepTime
    :reader suggestedSleepTime
    :initarg :suggestedSleepTime
    :type cl:integer
    :initform 0))
)

(cl:defclass tcpIpIsAvailable-response (<tcpIpIsAvailable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tcpIpIsAvailable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tcpIpIsAvailable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<tcpIpIsAvailable-response> is deprecated: use pal_supervisor_msgs-srv:tcpIpIsAvailable-response instead.")))

(cl:ensure-generic-function 'available-val :lambda-list '(m))
(cl:defmethod available-val ((m <tcpIpIsAvailable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:available-val is deprecated.  Use pal_supervisor_msgs-srv:available instead.")
  (available m))

(cl:ensure-generic-function 'suggestedSleepTime-val :lambda-list '(m))
(cl:defmethod suggestedSleepTime-val ((m <tcpIpIsAvailable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:suggestedSleepTime-val is deprecated.  Use pal_supervisor_msgs-srv:suggestedSleepTime instead.")
  (suggestedSleepTime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tcpIpIsAvailable-response>) ostream)
  "Serializes a message object of type '<tcpIpIsAvailable-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'available) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'suggestedSleepTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'suggestedSleepTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'suggestedSleepTime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'suggestedSleepTime)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tcpIpIsAvailable-response>) istream)
  "Deserializes a message object of type '<tcpIpIsAvailable-response>"
    (cl:setf (cl:slot-value msg 'available) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'suggestedSleepTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'suggestedSleepTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'suggestedSleepTime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'suggestedSleepTime)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tcpIpIsAvailable-response>)))
  "Returns string type for a service object of type '<tcpIpIsAvailable-response>"
  "pal_supervisor_msgs/tcpIpIsAvailableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tcpIpIsAvailable-response)))
  "Returns string type for a service object of type 'tcpIpIsAvailable-response"
  "pal_supervisor_msgs/tcpIpIsAvailableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tcpIpIsAvailable-response>)))
  "Returns md5sum for a message object of type '<tcpIpIsAvailable-response>"
  "c87e9a2a51d895bccdb0157125a69381")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tcpIpIsAvailable-response)))
  "Returns md5sum for a message object of type 'tcpIpIsAvailable-response"
  "c87e9a2a51d895bccdb0157125a69381")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tcpIpIsAvailable-response>)))
  "Returns full string definition for message of type '<tcpIpIsAvailable-response>"
  (cl:format cl:nil "bool available #True if the TCP Server is available, false if it's not~%uint32 suggestedSleepTime #Ms suggested to the client to sleep before trying again~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tcpIpIsAvailable-response)))
  "Returns full string definition for message of type 'tcpIpIsAvailable-response"
  (cl:format cl:nil "bool available #True if the TCP Server is available, false if it's not~%uint32 suggestedSleepTime #Ms suggested to the client to sleep before trying again~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tcpIpIsAvailable-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tcpIpIsAvailable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'tcpIpIsAvailable-response
    (cl:cons ':available (available msg))
    (cl:cons ':suggestedSleepTime (suggestedSleepTime msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'tcpIpIsAvailable)))
  'tcpIpIsAvailable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'tcpIpIsAvailable)))
  'tcpIpIsAvailable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tcpIpIsAvailable)))
  "Returns string type for a service object of type '<tcpIpIsAvailable>"
  "pal_supervisor_msgs/tcpIpIsAvailable")