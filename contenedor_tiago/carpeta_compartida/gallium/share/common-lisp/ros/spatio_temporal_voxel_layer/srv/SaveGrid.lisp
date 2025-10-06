; Auto-generated. Do not edit!


(cl:in-package spatio_temporal_voxel_layer-srv)


;//! \htmlinclude SaveGrid-request.msg.html

(cl:defclass <SaveGrid-request> (roslisp-msg-protocol:ros-message)
  ((file_name
    :reader file_name
    :initarg :file_name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass SaveGrid-request (<SaveGrid-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveGrid-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveGrid-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spatio_temporal_voxel_layer-srv:<SaveGrid-request> is deprecated: use spatio_temporal_voxel_layer-srv:SaveGrid-request instead.")))

(cl:ensure-generic-function 'file_name-val :lambda-list '(m))
(cl:defmethod file_name-val ((m <SaveGrid-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spatio_temporal_voxel_layer-srv:file_name-val is deprecated.  Use spatio_temporal_voxel_layer-srv:file_name instead.")
  (file_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveGrid-request>) ostream)
  "Serializes a message object of type '<SaveGrid-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'file_name) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveGrid-request>) istream)
  "Deserializes a message object of type '<SaveGrid-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'file_name) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveGrid-request>)))
  "Returns string type for a service object of type '<SaveGrid-request>"
  "spatio_temporal_voxel_layer/SaveGridRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveGrid-request)))
  "Returns string type for a service object of type 'SaveGrid-request"
  "spatio_temporal_voxel_layer/SaveGridRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveGrid-request>)))
  "Returns md5sum for a message object of type '<SaveGrid-request>"
  "793bb7917a99799f9a78324a148a1c17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveGrid-request)))
  "Returns md5sum for a message object of type 'SaveGrid-request"
  "793bb7917a99799f9a78324a148a1c17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveGrid-request>)))
  "Returns full string definition for message of type '<SaveGrid-request>"
  (cl:format cl:nil "std_msgs/String file_name~%~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveGrid-request)))
  "Returns full string definition for message of type 'SaveGrid-request"
  (cl:format cl:nil "std_msgs/String file_name~%~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveGrid-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'file_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveGrid-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveGrid-request
    (cl:cons ':file_name (file_name msg))
))
;//! \htmlinclude SaveGrid-response.msg.html

(cl:defclass <SaveGrid-response> (roslisp-msg-protocol:ros-message)
  ((map_size_bytes
    :reader map_size_bytes
    :initarg :map_size_bytes
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SaveGrid-response (<SaveGrid-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveGrid-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveGrid-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spatio_temporal_voxel_layer-srv:<SaveGrid-response> is deprecated: use spatio_temporal_voxel_layer-srv:SaveGrid-response instead.")))

(cl:ensure-generic-function 'map_size_bytes-val :lambda-list '(m))
(cl:defmethod map_size_bytes-val ((m <SaveGrid-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spatio_temporal_voxel_layer-srv:map_size_bytes-val is deprecated.  Use spatio_temporal_voxel_layer-srv:map_size_bytes instead.")
  (map_size_bytes m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SaveGrid-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spatio_temporal_voxel_layer-srv:status-val is deprecated.  Use spatio_temporal_voxel_layer-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveGrid-response>) ostream)
  "Serializes a message object of type '<SaveGrid-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'map_size_bytes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveGrid-response>) istream)
  "Deserializes a message object of type '<SaveGrid-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'map_size_bytes) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveGrid-response>)))
  "Returns string type for a service object of type '<SaveGrid-response>"
  "spatio_temporal_voxel_layer/SaveGridResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveGrid-response)))
  "Returns string type for a service object of type 'SaveGrid-response"
  "spatio_temporal_voxel_layer/SaveGridResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveGrid-response>)))
  "Returns md5sum for a message object of type '<SaveGrid-response>"
  "793bb7917a99799f9a78324a148a1c17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveGrid-response)))
  "Returns md5sum for a message object of type 'SaveGrid-response"
  "793bb7917a99799f9a78324a148a1c17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveGrid-response>)))
  "Returns full string definition for message of type '<SaveGrid-response>"
  (cl:format cl:nil "~%float64 map_size_bytes~%bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveGrid-response)))
  "Returns full string definition for message of type 'SaveGrid-response"
  (cl:format cl:nil "~%float64 map_size_bytes~%bool status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveGrid-response>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveGrid-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveGrid-response
    (cl:cons ':map_size_bytes (map_size_bytes msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SaveGrid)))
  'SaveGrid-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SaveGrid)))
  'SaveGrid-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveGrid)))
  "Returns string type for a service object of type '<SaveGrid>"
  "spatio_temporal_voxel_layer/SaveGrid")