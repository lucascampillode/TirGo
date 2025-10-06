; Auto-generated. Do not edit!


(cl:in-package dynamic_footprint-srv)


;//! \htmlinclude ChangeParams-request.msg.html

(cl:defclass <ChangeParams-request> (roslisp-msg-protocol:ros-message)
  ((rate
    :reader rate
    :initarg :rate
    :type cl:integer
    :initform 0)
   (padding
    :reader padding
    :initarg :padding
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChangeParams-request (<ChangeParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic_footprint-srv:<ChangeParams-request> is deprecated: use dynamic_footprint-srv:ChangeParams-request instead.")))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <ChangeParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_footprint-srv:rate-val is deprecated.  Use dynamic_footprint-srv:rate instead.")
  (rate m))

(cl:ensure-generic-function 'padding-val :lambda-list '(m))
(cl:defmethod padding-val ((m <ChangeParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamic_footprint-srv:padding-val is deprecated.  Use dynamic_footprint-srv:padding instead.")
  (padding m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeParams-request>) ostream)
  "Serializes a message object of type '<ChangeParams-request>"
  (cl:let* ((signed (cl:slot-value msg 'rate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'padding))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeParams-request>) istream)
  "Deserializes a message object of type '<ChangeParams-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rate) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'padding) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeParams-request>)))
  "Returns string type for a service object of type '<ChangeParams-request>"
  "dynamic_footprint/ChangeParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeParams-request)))
  "Returns string type for a service object of type 'ChangeParams-request"
  "dynamic_footprint/ChangeParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeParams-request>)))
  "Returns md5sum for a message object of type '<ChangeParams-request>"
  "54da70ff99cf30bf896a186492a150b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeParams-request)))
  "Returns md5sum for a message object of type 'ChangeParams-request"
  "54da70ff99cf30bf896a186492a150b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeParams-request>)))
  "Returns full string definition for message of type '<ChangeParams-request>"
  (cl:format cl:nil "int32 rate~%float32 padding~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeParams-request)))
  "Returns full string definition for message of type 'ChangeParams-request"
  (cl:format cl:nil "int32 rate~%float32 padding~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeParams-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeParams-request
    (cl:cons ':rate (rate msg))
    (cl:cons ':padding (padding msg))
))
;//! \htmlinclude ChangeParams-response.msg.html

(cl:defclass <ChangeParams-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ChangeParams-response (<ChangeParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamic_footprint-srv:<ChangeParams-response> is deprecated: use dynamic_footprint-srv:ChangeParams-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeParams-response>) ostream)
  "Serializes a message object of type '<ChangeParams-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeParams-response>) istream)
  "Deserializes a message object of type '<ChangeParams-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeParams-response>)))
  "Returns string type for a service object of type '<ChangeParams-response>"
  "dynamic_footprint/ChangeParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeParams-response)))
  "Returns string type for a service object of type 'ChangeParams-response"
  "dynamic_footprint/ChangeParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeParams-response>)))
  "Returns md5sum for a message object of type '<ChangeParams-response>"
  "54da70ff99cf30bf896a186492a150b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeParams-response)))
  "Returns md5sum for a message object of type 'ChangeParams-response"
  "54da70ff99cf30bf896a186492a150b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeParams-response>)))
  "Returns full string definition for message of type '<ChangeParams-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeParams-response)))
  "Returns full string definition for message of type 'ChangeParams-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeParams-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeParams-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangeParams)))
  'ChangeParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangeParams)))
  'ChangeParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeParams)))
  "Returns string type for a service object of type '<ChangeParams>"
  "dynamic_footprint/ChangeParams")