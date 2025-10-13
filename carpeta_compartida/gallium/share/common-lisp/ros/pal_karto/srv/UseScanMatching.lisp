; Auto-generated. Do not edit!


(cl:in-package pal_karto-srv)


;//! \htmlinclude UseScanMatching-request.msg.html

(cl:defclass <UseScanMatching-request> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UseScanMatching-request (<UseScanMatching-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UseScanMatching-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UseScanMatching-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_karto-srv:<UseScanMatching-request> is deprecated: use pal_karto-srv:UseScanMatching-request instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <UseScanMatching-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_karto-srv:value-val is deprecated.  Use pal_karto-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UseScanMatching-request>) ostream)
  "Serializes a message object of type '<UseScanMatching-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UseScanMatching-request>) istream)
  "Deserializes a message object of type '<UseScanMatching-request>"
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UseScanMatching-request>)))
  "Returns string type for a service object of type '<UseScanMatching-request>"
  "pal_karto/UseScanMatchingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UseScanMatching-request)))
  "Returns string type for a service object of type 'UseScanMatching-request"
  "pal_karto/UseScanMatchingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UseScanMatching-request>)))
  "Returns md5sum for a message object of type '<UseScanMatching-request>"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UseScanMatching-request)))
  "Returns md5sum for a message object of type 'UseScanMatching-request"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UseScanMatching-request>)))
  "Returns full string definition for message of type '<UseScanMatching-request>"
  (cl:format cl:nil "bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UseScanMatching-request)))
  "Returns full string definition for message of type 'UseScanMatching-request"
  (cl:format cl:nil "bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UseScanMatching-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UseScanMatching-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UseScanMatching-request
    (cl:cons ':value (value msg))
))
;//! \htmlinclude UseScanMatching-response.msg.html

(cl:defclass <UseScanMatching-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass UseScanMatching-response (<UseScanMatching-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UseScanMatching-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UseScanMatching-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_karto-srv:<UseScanMatching-response> is deprecated: use pal_karto-srv:UseScanMatching-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UseScanMatching-response>) ostream)
  "Serializes a message object of type '<UseScanMatching-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UseScanMatching-response>) istream)
  "Deserializes a message object of type '<UseScanMatching-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UseScanMatching-response>)))
  "Returns string type for a service object of type '<UseScanMatching-response>"
  "pal_karto/UseScanMatchingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UseScanMatching-response)))
  "Returns string type for a service object of type 'UseScanMatching-response"
  "pal_karto/UseScanMatchingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UseScanMatching-response>)))
  "Returns md5sum for a message object of type '<UseScanMatching-response>"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UseScanMatching-response)))
  "Returns md5sum for a message object of type 'UseScanMatching-response"
  "e431d687bf4b2c65fbd94b12ae0cb5d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UseScanMatching-response>)))
  "Returns full string definition for message of type '<UseScanMatching-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UseScanMatching-response)))
  "Returns full string definition for message of type 'UseScanMatching-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UseScanMatching-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UseScanMatching-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UseScanMatching-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UseScanMatching)))
  'UseScanMatching-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UseScanMatching)))
  'UseScanMatching-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UseScanMatching)))
  "Returns string type for a service object of type '<UseScanMatching>"
  "pal_karto/UseScanMatching")