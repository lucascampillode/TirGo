; Auto-generated. Do not edit!


(cl:in-package pal_local_planner-srv)


;//! \htmlinclude traffic_manager-request.msg.html

(cl:defclass <traffic_manager-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass traffic_manager-request (<traffic_manager-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traffic_manager-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traffic_manager-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_local_planner-srv:<traffic_manager-request> is deprecated: use pal_local_planner-srv:traffic_manager-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traffic_manager-request>) ostream)
  "Serializes a message object of type '<traffic_manager-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traffic_manager-request>) istream)
  "Deserializes a message object of type '<traffic_manager-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traffic_manager-request>)))
  "Returns string type for a service object of type '<traffic_manager-request>"
  "pal_local_planner/traffic_managerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traffic_manager-request)))
  "Returns string type for a service object of type 'traffic_manager-request"
  "pal_local_planner/traffic_managerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traffic_manager-request>)))
  "Returns md5sum for a message object of type '<traffic_manager-request>"
  "2687553acfba2efaeee5da06c95db2d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traffic_manager-request)))
  "Returns md5sum for a message object of type 'traffic_manager-request"
  "2687553acfba2efaeee5da06c95db2d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traffic_manager-request>)))
  "Returns full string definition for message of type '<traffic_manager-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traffic_manager-request)))
  "Returns full string definition for message of type 'traffic_manager-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traffic_manager-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traffic_manager-request>))
  "Converts a ROS message object to a list"
  (cl:list 'traffic_manager-request
))
;//! \htmlinclude traffic_manager-response.msg.html

(cl:defclass <traffic_manager-response> (roslisp-msg-protocol:ros-message)
  ((situation
    :reader situation
    :initarg :situation
    :type cl:integer
    :initform 0))
)

(cl:defclass traffic_manager-response (<traffic_manager-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traffic_manager-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traffic_manager-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_local_planner-srv:<traffic_manager-response> is deprecated: use pal_local_planner-srv:traffic_manager-response instead.")))

(cl:ensure-generic-function 'situation-val :lambda-list '(m))
(cl:defmethod situation-val ((m <traffic_manager-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_local_planner-srv:situation-val is deprecated.  Use pal_local_planner-srv:situation instead.")
  (situation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traffic_manager-response>) ostream)
  "Serializes a message object of type '<traffic_manager-response>"
  (cl:let* ((signed (cl:slot-value msg 'situation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traffic_manager-response>) istream)
  "Deserializes a message object of type '<traffic_manager-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'situation) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traffic_manager-response>)))
  "Returns string type for a service object of type '<traffic_manager-response>"
  "pal_local_planner/traffic_managerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traffic_manager-response)))
  "Returns string type for a service object of type 'traffic_manager-response"
  "pal_local_planner/traffic_managerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traffic_manager-response>)))
  "Returns md5sum for a message object of type '<traffic_manager-response>"
  "2687553acfba2efaeee5da06c95db2d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traffic_manager-response)))
  "Returns md5sum for a message object of type 'traffic_manager-response"
  "2687553acfba2efaeee5da06c95db2d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traffic_manager-response>)))
  "Returns full string definition for message of type '<traffic_manager-response>"
  (cl:format cl:nil "int64 situation~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traffic_manager-response)))
  "Returns full string definition for message of type 'traffic_manager-response"
  (cl:format cl:nil "int64 situation~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traffic_manager-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traffic_manager-response>))
  "Converts a ROS message object to a list"
  (cl:list 'traffic_manager-response
    (cl:cons ':situation (situation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'traffic_manager)))
  'traffic_manager-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'traffic_manager)))
  'traffic_manager-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traffic_manager)))
  "Returns string type for a service object of type '<traffic_manager>"
  "pal_local_planner/traffic_manager")