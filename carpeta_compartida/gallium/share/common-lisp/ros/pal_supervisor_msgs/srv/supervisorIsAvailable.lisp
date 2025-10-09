; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude supervisorIsAvailable-request.msg.html

(cl:defclass <supervisorIsAvailable-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass supervisorIsAvailable-request (<supervisorIsAvailable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <supervisorIsAvailable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'supervisorIsAvailable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<supervisorIsAvailable-request> is deprecated: use pal_supervisor_msgs-srv:supervisorIsAvailable-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <supervisorIsAvailable-request>) ostream)
  "Serializes a message object of type '<supervisorIsAvailable-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <supervisorIsAvailable-request>) istream)
  "Deserializes a message object of type '<supervisorIsAvailable-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<supervisorIsAvailable-request>)))
  "Returns string type for a service object of type '<supervisorIsAvailable-request>"
  "pal_supervisor_msgs/supervisorIsAvailableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'supervisorIsAvailable-request)))
  "Returns string type for a service object of type 'supervisorIsAvailable-request"
  "pal_supervisor_msgs/supervisorIsAvailableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<supervisorIsAvailable-request>)))
  "Returns md5sum for a message object of type '<supervisorIsAvailable-request>"
  "efb9ac7739f299decb8575015d81760c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'supervisorIsAvailable-request)))
  "Returns md5sum for a message object of type 'supervisorIsAvailable-request"
  "efb9ac7739f299decb8575015d81760c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<supervisorIsAvailable-request>)))
  "Returns full string definition for message of type '<supervisorIsAvailable-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'supervisorIsAvailable-request)))
  "Returns full string definition for message of type 'supervisorIsAvailable-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <supervisorIsAvailable-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <supervisorIsAvailable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'supervisorIsAvailable-request
))
;//! \htmlinclude supervisorIsAvailable-response.msg.html

(cl:defclass <supervisorIsAvailable-response> (roslisp-msg-protocol:ros-message)
  ((available
    :reader available
    :initarg :available
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass supervisorIsAvailable-response (<supervisorIsAvailable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <supervisorIsAvailable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'supervisorIsAvailable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<supervisorIsAvailable-response> is deprecated: use pal_supervisor_msgs-srv:supervisorIsAvailable-response instead.")))

(cl:ensure-generic-function 'available-val :lambda-list '(m))
(cl:defmethod available-val ((m <supervisorIsAvailable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:available-val is deprecated.  Use pal_supervisor_msgs-srv:available instead.")
  (available m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <supervisorIsAvailable-response>) ostream)
  "Serializes a message object of type '<supervisorIsAvailable-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'available) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <supervisorIsAvailable-response>) istream)
  "Deserializes a message object of type '<supervisorIsAvailable-response>"
    (cl:setf (cl:slot-value msg 'available) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<supervisorIsAvailable-response>)))
  "Returns string type for a service object of type '<supervisorIsAvailable-response>"
  "pal_supervisor_msgs/supervisorIsAvailableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'supervisorIsAvailable-response)))
  "Returns string type for a service object of type 'supervisorIsAvailable-response"
  "pal_supervisor_msgs/supervisorIsAvailableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<supervisorIsAvailable-response>)))
  "Returns md5sum for a message object of type '<supervisorIsAvailable-response>"
  "efb9ac7739f299decb8575015d81760c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'supervisorIsAvailable-response)))
  "Returns md5sum for a message object of type 'supervisorIsAvailable-response"
  "efb9ac7739f299decb8575015d81760c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<supervisorIsAvailable-response>)))
  "Returns full string definition for message of type '<supervisorIsAvailable-response>"
  (cl:format cl:nil "bool available #True if the Supervisor is available, false if it's not~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'supervisorIsAvailable-response)))
  "Returns full string definition for message of type 'supervisorIsAvailable-response"
  (cl:format cl:nil "bool available #True if the Supervisor is available, false if it's not~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <supervisorIsAvailable-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <supervisorIsAvailable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'supervisorIsAvailable-response
    (cl:cons ':available (available msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'supervisorIsAvailable)))
  'supervisorIsAvailable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'supervisorIsAvailable)))
  'supervisorIsAvailable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'supervisorIsAvailable)))
  "Returns string type for a service object of type '<supervisorIsAvailable>"
  "pal_supervisor_msgs/supervisorIsAvailable")