; Auto-generated. Do not edit!


(cl:in-package pal_pcl-srv)


;//! \htmlinclude save-request.msg.html

(cl:defclass <save-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass save-request (<save-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <save-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'save-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_pcl-srv:<save-request> is deprecated: use pal_pcl-srv:save-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <save-request>) ostream)
  "Serializes a message object of type '<save-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <save-request>) istream)
  "Deserializes a message object of type '<save-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<save-request>)))
  "Returns string type for a service object of type '<save-request>"
  "pal_pcl/saveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'save-request)))
  "Returns string type for a service object of type 'save-request"
  "pal_pcl/saveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<save-request>)))
  "Returns md5sum for a message object of type '<save-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'save-request)))
  "Returns md5sum for a message object of type 'save-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<save-request>)))
  "Returns full string definition for message of type '<save-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'save-request)))
  "Returns full string definition for message of type 'save-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <save-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <save-request>))
  "Converts a ROS message object to a list"
  (cl:list 'save-request
))
;//! \htmlinclude save-response.msg.html

(cl:defclass <save-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass save-response (<save-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <save-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'save-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_pcl-srv:<save-response> is deprecated: use pal_pcl-srv:save-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <save-response>) ostream)
  "Serializes a message object of type '<save-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <save-response>) istream)
  "Deserializes a message object of type '<save-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<save-response>)))
  "Returns string type for a service object of type '<save-response>"
  "pal_pcl/saveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'save-response)))
  "Returns string type for a service object of type 'save-response"
  "pal_pcl/saveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<save-response>)))
  "Returns md5sum for a message object of type '<save-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'save-response)))
  "Returns md5sum for a message object of type 'save-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<save-response>)))
  "Returns full string definition for message of type '<save-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'save-response)))
  "Returns full string definition for message of type 'save-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <save-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <save-response>))
  "Converts a ROS message object to a list"
  (cl:list 'save-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'save)))
  'save-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'save)))
  'save-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'save)))
  "Returns string type for a service object of type '<save>"
  "pal_pcl/save")