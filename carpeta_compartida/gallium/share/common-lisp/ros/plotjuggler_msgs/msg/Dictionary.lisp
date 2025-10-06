; Auto-generated. Do not edit!


(cl:in-package plotjuggler_msgs-msg)


;//! \htmlinclude Dictionary.msg.html

(cl:defclass <Dictionary> (roslisp-msg-protocol:ros-message)
  ((dictionary_uuid
    :reader dictionary_uuid
    :initarg :dictionary_uuid
    :type cl:integer
    :initform 0)
   (names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Dictionary (<Dictionary>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dictionary>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dictionary)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name plotjuggler_msgs-msg:<Dictionary> is deprecated: use plotjuggler_msgs-msg:Dictionary instead.")))

(cl:ensure-generic-function 'dictionary_uuid-val :lambda-list '(m))
(cl:defmethod dictionary_uuid-val ((m <Dictionary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:dictionary_uuid-val is deprecated.  Use plotjuggler_msgs-msg:dictionary_uuid instead.")
  (dictionary_uuid m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <Dictionary>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader plotjuggler_msgs-msg:names-val is deprecated.  Use plotjuggler_msgs-msg:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dictionary>) ostream)
  "Serializes a message object of type '<Dictionary>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dictionary_uuid)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dictionary>) istream)
  "Deserializes a message object of type '<Dictionary>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dictionary_uuid)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dictionary>)))
  "Returns string type for a message object of type '<Dictionary>"
  "plotjuggler_msgs/Dictionary")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dictionary)))
  "Returns string type for a message object of type 'Dictionary"
  "plotjuggler_msgs/Dictionary")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dictionary>)))
  "Returns md5sum for a message object of type '<Dictionary>"
  "12d13553d8d6a9826829b71cac454ebe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dictionary)))
  "Returns md5sum for a message object of type 'Dictionary"
  "12d13553d8d6a9826829b71cac454ebe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dictionary>)))
  "Returns full string definition for message of type '<Dictionary>"
  (cl:format cl:nil "uint32 dictionary_uuid~%string[] names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dictionary)))
  "Returns full string definition for message of type 'Dictionary"
  (cl:format cl:nil "uint32 dictionary_uuid~%string[] names~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dictionary>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dictionary>))
  "Converts a ROS message object to a list"
  (cl:list 'Dictionary
    (cl:cons ':dictionary_uuid (dictionary_uuid msg))
    (cl:cons ':names (names msg))
))
