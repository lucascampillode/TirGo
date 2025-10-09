; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude numberOfAnomalies-request.msg.html

(cl:defclass <numberOfAnomalies-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass numberOfAnomalies-request (<numberOfAnomalies-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <numberOfAnomalies-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'numberOfAnomalies-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<numberOfAnomalies-request> is deprecated: use pal_supervisor_msgs-srv:numberOfAnomalies-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <numberOfAnomalies-request>) ostream)
  "Serializes a message object of type '<numberOfAnomalies-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <numberOfAnomalies-request>) istream)
  "Deserializes a message object of type '<numberOfAnomalies-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<numberOfAnomalies-request>)))
  "Returns string type for a service object of type '<numberOfAnomalies-request>"
  "pal_supervisor_msgs/numberOfAnomaliesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'numberOfAnomalies-request)))
  "Returns string type for a service object of type 'numberOfAnomalies-request"
  "pal_supervisor_msgs/numberOfAnomaliesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<numberOfAnomalies-request>)))
  "Returns md5sum for a message object of type '<numberOfAnomalies-request>"
  "256e37138065fcef5d440c4ba2011920")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'numberOfAnomalies-request)))
  "Returns md5sum for a message object of type 'numberOfAnomalies-request"
  "256e37138065fcef5d440c4ba2011920")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<numberOfAnomalies-request>)))
  "Returns full string definition for message of type '<numberOfAnomalies-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'numberOfAnomalies-request)))
  "Returns full string definition for message of type 'numberOfAnomalies-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <numberOfAnomalies-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <numberOfAnomalies-request>))
  "Converts a ROS message object to a list"
  (cl:list 'numberOfAnomalies-request
))
;//! \htmlinclude numberOfAnomalies-response.msg.html

(cl:defclass <numberOfAnomalies-response> (roslisp-msg-protocol:ros-message)
  ((numErrors
    :reader numErrors
    :initarg :numErrors
    :type cl:integer
    :initform 0)
   (numWarnings
    :reader numWarnings
    :initarg :numWarnings
    :type cl:integer
    :initform 0))
)

(cl:defclass numberOfAnomalies-response (<numberOfAnomalies-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <numberOfAnomalies-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'numberOfAnomalies-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<numberOfAnomalies-response> is deprecated: use pal_supervisor_msgs-srv:numberOfAnomalies-response instead.")))

(cl:ensure-generic-function 'numErrors-val :lambda-list '(m))
(cl:defmethod numErrors-val ((m <numberOfAnomalies-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:numErrors-val is deprecated.  Use pal_supervisor_msgs-srv:numErrors instead.")
  (numErrors m))

(cl:ensure-generic-function 'numWarnings-val :lambda-list '(m))
(cl:defmethod numWarnings-val ((m <numberOfAnomalies-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:numWarnings-val is deprecated.  Use pal_supervisor_msgs-srv:numWarnings instead.")
  (numWarnings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <numberOfAnomalies-response>) ostream)
  "Serializes a message object of type '<numberOfAnomalies-response>"
  (cl:let* ((signed (cl:slot-value msg 'numErrors)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'numWarnings)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <numberOfAnomalies-response>) istream)
  "Deserializes a message object of type '<numberOfAnomalies-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numErrors) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numWarnings) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<numberOfAnomalies-response>)))
  "Returns string type for a service object of type '<numberOfAnomalies-response>"
  "pal_supervisor_msgs/numberOfAnomaliesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'numberOfAnomalies-response)))
  "Returns string type for a service object of type 'numberOfAnomalies-response"
  "pal_supervisor_msgs/numberOfAnomaliesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<numberOfAnomalies-response>)))
  "Returns md5sum for a message object of type '<numberOfAnomalies-response>"
  "256e37138065fcef5d440c4ba2011920")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'numberOfAnomalies-response)))
  "Returns md5sum for a message object of type 'numberOfAnomalies-response"
  "256e37138065fcef5d440c4ba2011920")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<numberOfAnomalies-response>)))
  "Returns full string definition for message of type '<numberOfAnomalies-response>"
  (cl:format cl:nil "int32 numErrors #Number of errors~%int32 numWarnings #Number of warnings~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'numberOfAnomalies-response)))
  "Returns full string definition for message of type 'numberOfAnomalies-response"
  (cl:format cl:nil "int32 numErrors #Number of errors~%int32 numWarnings #Number of warnings~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <numberOfAnomalies-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <numberOfAnomalies-response>))
  "Converts a ROS message object to a list"
  (cl:list 'numberOfAnomalies-response
    (cl:cons ':numErrors (numErrors msg))
    (cl:cons ':numWarnings (numWarnings msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'numberOfAnomalies)))
  'numberOfAnomalies-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'numberOfAnomalies)))
  'numberOfAnomalies-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'numberOfAnomalies)))
  "Returns string type for a service object of type '<numberOfAnomalies>"
  "pal_supervisor_msgs/numberOfAnomalies")