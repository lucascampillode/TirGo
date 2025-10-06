; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude finalStartupStatus-request.msg.html

(cl:defclass <finalStartupStatus-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type pal_supervisor_msgs-msg:StartupStatus
    :initform (cl:make-instance 'pal_supervisor_msgs-msg:StartupStatus)))
)

(cl:defclass finalStartupStatus-request (<finalStartupStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <finalStartupStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'finalStartupStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<finalStartupStatus-request> is deprecated: use pal_supervisor_msgs-srv:finalStartupStatus-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <finalStartupStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:status-val is deprecated.  Use pal_supervisor_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <finalStartupStatus-request>) ostream)
  "Serializes a message object of type '<finalStartupStatus-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <finalStartupStatus-request>) istream)
  "Deserializes a message object of type '<finalStartupStatus-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<finalStartupStatus-request>)))
  "Returns string type for a service object of type '<finalStartupStatus-request>"
  "pal_supervisor_msgs/finalStartupStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'finalStartupStatus-request)))
  "Returns string type for a service object of type 'finalStartupStatus-request"
  "pal_supervisor_msgs/finalStartupStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<finalStartupStatus-request>)))
  "Returns md5sum for a message object of type '<finalStartupStatus-request>"
  "3fe91db533913eea244a861ae3cea188")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'finalStartupStatus-request)))
  "Returns md5sum for a message object of type 'finalStartupStatus-request"
  "3fe91db533913eea244a861ae3cea188")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<finalStartupStatus-request>)))
  "Returns full string definition for message of type '<finalStartupStatus-request>"
  (cl:format cl:nil "#Final status reported by startup launch script. This should be repeated by the supervisor forever~%pal_supervisor_msgs/StartupStatus status~%~%================================================================================~%MSG: pal_supervisor_msgs/StartupStatus~%string name # application name~%string message # description of the status if required~%# Possible status of startup process~%byte RUNNING=0~%byte WAITING=1~%byte TIMEOUT=2~%byte ABORTED=3 #Application not running~%byte status # status of the startup process for the application. Enumerated above ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'finalStartupStatus-request)))
  "Returns full string definition for message of type 'finalStartupStatus-request"
  (cl:format cl:nil "#Final status reported by startup launch script. This should be repeated by the supervisor forever~%pal_supervisor_msgs/StartupStatus status~%~%================================================================================~%MSG: pal_supervisor_msgs/StartupStatus~%string name # application name~%string message # description of the status if required~%# Possible status of startup process~%byte RUNNING=0~%byte WAITING=1~%byte TIMEOUT=2~%byte ABORTED=3 #Application not running~%byte status # status of the startup process for the application. Enumerated above ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <finalStartupStatus-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <finalStartupStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'finalStartupStatus-request
    (cl:cons ':status (status msg))
))
;//! \htmlinclude finalStartupStatus-response.msg.html

(cl:defclass <finalStartupStatus-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass finalStartupStatus-response (<finalStartupStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <finalStartupStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'finalStartupStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<finalStartupStatus-response> is deprecated: use pal_supervisor_msgs-srv:finalStartupStatus-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <finalStartupStatus-response>) ostream)
  "Serializes a message object of type '<finalStartupStatus-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <finalStartupStatus-response>) istream)
  "Deserializes a message object of type '<finalStartupStatus-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<finalStartupStatus-response>)))
  "Returns string type for a service object of type '<finalStartupStatus-response>"
  "pal_supervisor_msgs/finalStartupStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'finalStartupStatus-response)))
  "Returns string type for a service object of type 'finalStartupStatus-response"
  "pal_supervisor_msgs/finalStartupStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<finalStartupStatus-response>)))
  "Returns md5sum for a message object of type '<finalStartupStatus-response>"
  "3fe91db533913eea244a861ae3cea188")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'finalStartupStatus-response)))
  "Returns md5sum for a message object of type 'finalStartupStatus-response"
  "3fe91db533913eea244a861ae3cea188")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<finalStartupStatus-response>)))
  "Returns full string definition for message of type '<finalStartupStatus-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'finalStartupStatus-response)))
  "Returns full string definition for message of type 'finalStartupStatus-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <finalStartupStatus-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <finalStartupStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'finalStartupStatus-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'finalStartupStatus)))
  'finalStartupStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'finalStartupStatus)))
  'finalStartupStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'finalStartupStatus)))
  "Returns string type for a service object of type '<finalStartupStatus>"
  "pal_supervisor_msgs/finalStartupStatus")