; Auto-generated. Do not edit!


(cl:in-package pal_zoi_detector-srv)


;//! \htmlinclude GetPointZoI-request.msg.html

(cl:defclass <GetPointZoI-request> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass GetPointZoI-request (<GetPointZoI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPointZoI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPointZoI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_zoi_detector-srv:<GetPointZoI-request> is deprecated: use pal_zoi_detector-srv:GetPointZoI-request instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <GetPointZoI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_zoi_detector-srv:point-val is deprecated.  Use pal_zoi_detector-srv:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPointZoI-request>) ostream)
  "Serializes a message object of type '<GetPointZoI-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPointZoI-request>) istream)
  "Deserializes a message object of type '<GetPointZoI-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPointZoI-request>)))
  "Returns string type for a service object of type '<GetPointZoI-request>"
  "pal_zoi_detector/GetPointZoIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPointZoI-request)))
  "Returns string type for a service object of type 'GetPointZoI-request"
  "pal_zoi_detector/GetPointZoIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPointZoI-request>)))
  "Returns md5sum for a message object of type '<GetPointZoI-request>"
  "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPointZoI-request)))
  "Returns md5sum for a message object of type 'GetPointZoI-request"
  "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPointZoI-request>)))
  "Returns full string definition for message of type '<GetPointZoI-request>"
  (cl:format cl:nil "geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPointZoI-request)))
  "Returns full string definition for message of type 'GetPointZoI-request"
  (cl:format cl:nil "geometry_msgs/Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPointZoI-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPointZoI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPointZoI-request
    (cl:cons ':point (point msg))
))
;//! \htmlinclude GetPointZoI-response.msg.html

(cl:defclass <GetPointZoI-response> (roslisp-msg-protocol:ros-message)
  ((zois
    :reader zois
    :initarg :zois
    :type pal_zoi_detector-msg:CurrentZoI
    :initform (cl:make-instance 'pal_zoi_detector-msg:CurrentZoI)))
)

(cl:defclass GetPointZoI-response (<GetPointZoI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPointZoI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPointZoI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_zoi_detector-srv:<GetPointZoI-response> is deprecated: use pal_zoi_detector-srv:GetPointZoI-response instead.")))

(cl:ensure-generic-function 'zois-val :lambda-list '(m))
(cl:defmethod zois-val ((m <GetPointZoI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_zoi_detector-srv:zois-val is deprecated.  Use pal_zoi_detector-srv:zois instead.")
  (zois m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPointZoI-response>) ostream)
  "Serializes a message object of type '<GetPointZoI-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'zois) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPointZoI-response>) istream)
  "Deserializes a message object of type '<GetPointZoI-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'zois) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPointZoI-response>)))
  "Returns string type for a service object of type '<GetPointZoI-response>"
  "pal_zoi_detector/GetPointZoIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPointZoI-response)))
  "Returns string type for a service object of type 'GetPointZoI-response"
  "pal_zoi_detector/GetPointZoIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPointZoI-response>)))
  "Returns md5sum for a message object of type '<GetPointZoI-response>"
  "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPointZoI-response)))
  "Returns md5sum for a message object of type 'GetPointZoI-response"
  "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPointZoI-response>)))
  "Returns full string definition for message of type '<GetPointZoI-response>"
  (cl:format cl:nil "CurrentZoI zois~%~%~%================================================================================~%MSG: pal_zoi_detector/CurrentZoI~%string[] zois~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPointZoI-response)))
  "Returns full string definition for message of type 'GetPointZoI-response"
  (cl:format cl:nil "CurrentZoI zois~%~%~%================================================================================~%MSG: pal_zoi_detector/CurrentZoI~%string[] zois~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPointZoI-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'zois))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPointZoI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPointZoI-response
    (cl:cons ':zois (zois msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPointZoI)))
  'GetPointZoI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPointZoI)))
  'GetPointZoI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPointZoI)))
  "Returns string type for a service object of type '<GetPointZoI>"
  "pal_zoi_detector/GetPointZoI")