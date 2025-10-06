; Auto-generated. Do not edit!


(cl:in-package plotjuggler_msgs-msg)


;//! \htmlinclude DataPoints.msg.html

(cl:defclass <DataPoints> (roslisp-msg-protocol:ros-message)
  ((dictionary_uuid
    :reader dictionary_uuid
    :initarg :dictionary_uuid
    :type cl:integer
    :initform 0)
   (samples
    :reader samples
    :initarg :samples
    :type (cl:vector plotjuggler_msgs-msg:DataPoint)
   :initform (cl:make-array 0 :element-type 'plotjuggler_msgs-msg:DataPoint :initial-element (cl:make-instance 'plotjuggler_msgs-msg:DataPoint))))
)

(cl:defclass DataPoints (<DataPoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataPoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataPoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plotjuggler_msgs-msg:<DataPoints> is deprecated: use plotjuggler_msgs-msg:DataPoints instead.")))

(cl:ensure-generic-function 'dictionary_uuid-val :lambda-list '(m))
(cl:defmethod dictionary_uuid-val ((m <DataPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:dictionary_uuid-val is deprecated.  Use plotjuggler_msgs-msg:dictionary_uuid instead.")
  (dictionary_uuid m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <DataPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:samples-val is deprecated.  Use plotjuggler_msgs-msg:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataPoints>) ostream)
  "Serializes a message object of type '<DataPoints>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'samples))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'samples))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataPoints>) istream)
  "Deserializes a message object of type '<DataPoints>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'samples) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'samples)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'plotjuggler_msgs-msg:DataPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataPoints>)))
  "Returns string type for a message object of type '<DataPoints>"
  "plotjuggler_msgs/DataPoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataPoints)))
  "Returns string type for a message object of type 'DataPoints"
  "plotjuggler_msgs/DataPoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataPoints>)))
  "Returns md5sum for a message object of type '<DataPoints>"
  "14e65e7956023a9a11291bc53d5d695a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataPoints)))
  "Returns md5sum for a message object of type 'DataPoints"
  "14e65e7956023a9a11291bc53d5d695a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataPoints>)))
  "Returns full string definition for message of type '<DataPoints>"
  (cl:format cl:nil "uint32 dictionary_uuid~%plotjuggler_msgs/DataPoint[] samples~%~%================================================================================~%MSG: plotjuggler_msgs/DataPoint~%uint16 name_index~%float64 stamp~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataPoints)))
  "Returns full string definition for message of type 'DataPoints"
  (cl:format cl:nil "uint32 dictionary_uuid~%plotjuggler_msgs/DataPoint[] samples~%~%================================================================================~%MSG: plotjuggler_msgs/DataPoint~%uint16 name_index~%float64 stamp~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataPoints>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataPoints>))
  "Converts a ROS message object to a list"
  (cl:list 'DataPoints
    (cl:cons ':dictionary_uuid (dictionary_uuid msg))
    (cl:cons ':samples (samples msg))
))
