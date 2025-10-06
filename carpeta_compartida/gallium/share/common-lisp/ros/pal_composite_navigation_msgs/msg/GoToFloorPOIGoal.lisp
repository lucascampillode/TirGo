; Auto-generated. Do not edit!


(cl:in-package pal_composite_navigation_msgs-msg)


;//! \htmlinclude GoToFloorPOIGoal.msg.html

(cl:defclass <GoToFloorPOIGoal> (roslisp-msg-protocol:ros-message)
  ((floor
    :reader floor
    :initarg :floor
    :type cl:string
    :initform "")
   (poi
    :reader poi
    :initarg :poi
    :type cl:string
    :initform "")
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:real
    :initform 0))
)

(cl:defclass GoToFloorPOIGoal (<GoToFloorPOIGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToFloorPOIGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToFloorPOIGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_composite_navigation_msgs-msg:<GoToFloorPOIGoal> is deprecated: use pal_composite_navigation_msgs-msg:GoToFloorPOIGoal instead.")))

(cl:ensure-generic-function 'floor-val :lambda-list '(m))
(cl:defmethod floor-val ((m <GoToFloorPOIGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_composite_navigation_msgs-msg:floor-val is deprecated.  Use pal_composite_navigation_msgs-msg:floor instead.")
  (floor m))

(cl:ensure-generic-function 'poi-val :lambda-list '(m))
(cl:defmethod poi-val ((m <GoToFloorPOIGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_composite_navigation_msgs-msg:poi-val is deprecated.  Use pal_composite_navigation_msgs-msg:poi instead.")
  (poi m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <GoToFloorPOIGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_composite_navigation_msgs-msg:timeout-val is deprecated.  Use pal_composite_navigation_msgs-msg:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToFloorPOIGoal>) ostream)
  "Serializes a message object of type '<GoToFloorPOIGoal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'floor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'floor))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'poi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'poi))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timeout)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timeout) (cl:floor (cl:slot-value msg 'timeout)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToFloorPOIGoal>) istream)
  "Deserializes a message object of type '<GoToFloorPOIGoal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'floor) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'floor) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'poi) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'poi) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timeout) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToFloorPOIGoal>)))
  "Returns string type for a message object of type '<GoToFloorPOIGoal>"
  "pal_composite_navigation_msgs/GoToFloorPOIGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToFloorPOIGoal)))
  "Returns string type for a message object of type 'GoToFloorPOIGoal"
  "pal_composite_navigation_msgs/GoToFloorPOIGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToFloorPOIGoal>)))
  "Returns md5sum for a message object of type '<GoToFloorPOIGoal>"
  "0a32ca4cb10e6201941dab34784fa2b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToFloorPOIGoal)))
  "Returns md5sum for a message object of type 'GoToFloorPOIGoal"
  "0a32ca4cb10e6201941dab34784fa2b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToFloorPOIGoal>)))
  "Returns full string definition for message of type '<GoToFloorPOIGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string floor~%string poi~%duration timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToFloorPOIGoal)))
  "Returns full string definition for message of type 'GoToFloorPOIGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string floor~%string poi~%duration timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToFloorPOIGoal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'floor))
     4 (cl:length (cl:slot-value msg 'poi))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToFloorPOIGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToFloorPOIGoal
    (cl:cons ':floor (floor msg))
    (cl:cons ':poi (poi msg))
    (cl:cons ':timeout (timeout msg))
))
