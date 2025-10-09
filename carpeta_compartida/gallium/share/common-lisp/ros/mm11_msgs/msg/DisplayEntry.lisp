; Auto-generated. Do not edit!


(cl:in-package mm11_msgs-msg)


;//! \htmlinclude DisplayEntry.msg.html

(cl:defclass <DisplayEntry> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass DisplayEntry (<DisplayEntry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisplayEntry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisplayEntry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mm11_msgs-msg:<DisplayEntry> is deprecated: use mm11_msgs-msg:DisplayEntry instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <DisplayEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-msg:type-val is deprecated.  Use mm11_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <DisplayEntry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mm11_msgs-msg:text-val is deprecated.  Use mm11_msgs-msg:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DisplayEntry>)))
    "Constants for message type '<DisplayEntry>"
  '((:BUTTON . 0)
    (:LABEL . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DisplayEntry)))
    "Constants for message type 'DisplayEntry"
  '((:BUTTON . 0)
    (:LABEL . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisplayEntry>) ostream)
  "Serializes a message object of type '<DisplayEntry>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisplayEntry>) istream)
  "Deserializes a message object of type '<DisplayEntry>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisplayEntry>)))
  "Returns string type for a message object of type '<DisplayEntry>"
  "mm11_msgs/DisplayEntry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisplayEntry)))
  "Returns string type for a message object of type 'DisplayEntry"
  "mm11_msgs/DisplayEntry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisplayEntry>)))
  "Returns md5sum for a message object of type '<DisplayEntry>"
  "e3e7126b9c0f9fa8a31e38b081f54f81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisplayEntry)))
  "Returns md5sum for a message object of type 'DisplayEntry"
  "e3e7126b9c0f9fa8a31e38b081f54f81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisplayEntry>)))
  "Returns full string definition for message of type '<DisplayEntry>"
  (cl:format cl:nil "uint8 BUTTON=0~%uint8 LABEL=1~%~%# LABEL or BUTTON~%uint8 type~%# label or button text~%string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisplayEntry)))
  "Returns full string definition for message of type 'DisplayEntry"
  (cl:format cl:nil "uint8 BUTTON=0~%uint8 LABEL=1~%~%# LABEL or BUTTON~%uint8 type~%# label or button text~%string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisplayEntry>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisplayEntry>))
  "Converts a ROS message object to a list"
  (cl:list 'DisplayEntry
    (cl:cons ':type (type msg))
    (cl:cons ':text (text msg))
))
