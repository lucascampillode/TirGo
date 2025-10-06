; Auto-generated. Do not edit!


(cl:in-package plotjuggler_msgs-msg)


;//! \htmlinclude DataPoint.msg.html

(cl:defclass <DataPoint> (roslisp-msg-protocol:ros-message)
  ((name_index
    :reader name_index
    :initarg :name_index
    :type cl:fixnum
    :initform 0)
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:float
    :initform 0.0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass DataPoint (<DataPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plotjuggler_msgs-msg:<DataPoint> is deprecated: use plotjuggler_msgs-msg:DataPoint instead.")))

(cl:ensure-generic-function 'name_index-val :lambda-list '(m))
(cl:defmethod name_index-val ((m <DataPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:name_index-val is deprecated.  Use plotjuggler_msgs-msg:name_index instead.")
  (name_index m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <DataPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:stamp-val is deprecated.  Use plotjuggler_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <DataPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:value-val is deprecated.  Use plotjuggler_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataPoint>) ostream)
  "Serializes a message object of type '<DataPoint>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'name_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'name_index)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'stamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataPoint>) istream)
  "Deserializes a message object of type '<DataPoint>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'name_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'name_index)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stamp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataPoint>)))
  "Returns string type for a message object of type '<DataPoint>"
  "plotjuggler_msgs/DataPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataPoint)))
  "Returns string type for a message object of type 'DataPoint"
  "plotjuggler_msgs/DataPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataPoint>)))
  "Returns md5sum for a message object of type '<DataPoint>"
  "580ca7c40f92b9a6ab4b921c02ebcd28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataPoint)))
  "Returns md5sum for a message object of type 'DataPoint"
  "580ca7c40f92b9a6ab4b921c02ebcd28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataPoint>)))
  "Returns full string definition for message of type '<DataPoint>"
  (cl:format cl:nil "uint16 name_index~%float64 stamp~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataPoint)))
  "Returns full string definition for message of type 'DataPoint"
  (cl:format cl:nil "uint16 name_index~%float64 stamp~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataPoint>))
  (cl:+ 0
     2
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'DataPoint
    (cl:cons ':name_index (name_index msg))
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':value (value msg))
))
