; Auto-generated. Do not edit!


(cl:in-package amcl-srv)


;//! \htmlinclude Randomize-request.msg.html

(cl:defclass <Randomize-request> (roslisp-msg-protocol:ros-message)
  ((radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass Randomize-request (<Randomize-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Randomize-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Randomize-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amcl-srv:<Randomize-request> is deprecated: use amcl-srv:Randomize-request instead.")))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <Randomize-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amcl-srv:radius-val is deprecated.  Use amcl-srv:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Randomize-request>) ostream)
  "Serializes a message object of type '<Randomize-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Randomize-request>) istream)
  "Deserializes a message object of type '<Randomize-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Randomize-request>)))
  "Returns string type for a service object of type '<Randomize-request>"
  "amcl/RandomizeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Randomize-request)))
  "Returns string type for a service object of type 'Randomize-request"
  "amcl/RandomizeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Randomize-request>)))
  "Returns md5sum for a message object of type '<Randomize-request>"
  "0b690044065f2210effb664cc7a28125")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Randomize-request)))
  "Returns md5sum for a message object of type 'Randomize-request"
  "0b690044065f2210effb664cc7a28125")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Randomize-request>)))
  "Returns full string definition for message of type '<Randomize-request>"
  (cl:format cl:nil "float32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Randomize-request)))
  "Returns full string definition for message of type 'Randomize-request"
  (cl:format cl:nil "float32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Randomize-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Randomize-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Randomize-request
    (cl:cons ':radius (radius msg))
))
;//! \htmlinclude Randomize-response.msg.html

(cl:defclass <Randomize-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Randomize-response (<Randomize-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Randomize-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Randomize-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amcl-srv:<Randomize-response> is deprecated: use amcl-srv:Randomize-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Randomize-response>) ostream)
  "Serializes a message object of type '<Randomize-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Randomize-response>) istream)
  "Deserializes a message object of type '<Randomize-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Randomize-response>)))
  "Returns string type for a service object of type '<Randomize-response>"
  "amcl/RandomizeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Randomize-response)))
  "Returns string type for a service object of type 'Randomize-response"
  "amcl/RandomizeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Randomize-response>)))
  "Returns md5sum for a message object of type '<Randomize-response>"
  "0b690044065f2210effb664cc7a28125")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Randomize-response)))
  "Returns md5sum for a message object of type 'Randomize-response"
  "0b690044065f2210effb664cc7a28125")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Randomize-response>)))
  "Returns full string definition for message of type '<Randomize-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Randomize-response)))
  "Returns full string definition for message of type 'Randomize-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Randomize-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Randomize-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Randomize-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Randomize)))
  'Randomize-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Randomize)))
  'Randomize-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Randomize)))
  "Returns string type for a service object of type '<Randomize>"
  "amcl/Randomize")