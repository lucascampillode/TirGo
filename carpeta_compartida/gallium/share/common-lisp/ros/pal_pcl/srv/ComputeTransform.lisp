; Auto-generated. Do not edit!


(cl:in-package pal_pcl-srv)


;//! \htmlinclude ComputeTransform-request.msg.html

(cl:defclass <ComputeTransform-request> (roslisp-msg-protocol:ros-message)
  ((camera_topic
    :reader camera_topic
    :initarg :camera_topic
    :type cl:string
    :initform "")
   (optical_frame
    :reader optical_frame
    :initarg :optical_frame
    :type cl:string
    :initform "")
   (camera_frame
    :reader camera_frame
    :initarg :camera_frame
    :type cl:string
    :initform "")
   (max_dist
    :reader max_dist
    :initarg :max_dist
    :type cl:float
    :initform 0.0))
)

(cl:defclass ComputeTransform-request (<ComputeTransform-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeTransform-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeTransform-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_pcl-srv:<ComputeTransform-request> is deprecated: use pal_pcl-srv:ComputeTransform-request instead.")))

(cl:ensure-generic-function 'camera_topic-val :lambda-list '(m))
(cl:defmethod camera_topic-val ((m <ComputeTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:camera_topic-val is deprecated.  Use pal_pcl-srv:camera_topic instead.")
  (camera_topic m))

(cl:ensure-generic-function 'optical_frame-val :lambda-list '(m))
(cl:defmethod optical_frame-val ((m <ComputeTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:optical_frame-val is deprecated.  Use pal_pcl-srv:optical_frame instead.")
  (optical_frame m))

(cl:ensure-generic-function 'camera_frame-val :lambda-list '(m))
(cl:defmethod camera_frame-val ((m <ComputeTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:camera_frame-val is deprecated.  Use pal_pcl-srv:camera_frame instead.")
  (camera_frame m))

(cl:ensure-generic-function 'max_dist-val :lambda-list '(m))
(cl:defmethod max_dist-val ((m <ComputeTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:max_dist-val is deprecated.  Use pal_pcl-srv:max_dist instead.")
  (max_dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeTransform-request>) ostream)
  "Serializes a message object of type '<ComputeTransform-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'camera_topic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'camera_topic))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'optical_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'optical_frame))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'camera_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'camera_frame))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeTransform-request>) istream)
  "Deserializes a message object of type '<ComputeTransform-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera_topic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'camera_topic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'optical_frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'optical_frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera_frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'camera_frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_dist) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeTransform-request>)))
  "Returns string type for a service object of type '<ComputeTransform-request>"
  "pal_pcl/ComputeTransformRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeTransform-request)))
  "Returns string type for a service object of type 'ComputeTransform-request"
  "pal_pcl/ComputeTransformRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeTransform-request>)))
  "Returns md5sum for a message object of type '<ComputeTransform-request>"
  "63c14dea37df6909b0547d1d22800cc9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeTransform-request)))
  "Returns md5sum for a message object of type 'ComputeTransform-request"
  "63c14dea37df6909b0547d1d22800cc9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeTransform-request>)))
  "Returns full string definition for message of type '<ComputeTransform-request>"
  (cl:format cl:nil "string camera_topic~%string optical_frame~%string camera_frame~%float64 max_dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeTransform-request)))
  "Returns full string definition for message of type 'ComputeTransform-request"
  (cl:format cl:nil "string camera_topic~%string optical_frame~%string camera_frame~%float64 max_dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeTransform-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'camera_topic))
     4 (cl:length (cl:slot-value msg 'optical_frame))
     4 (cl:length (cl:slot-value msg 'camera_frame))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeTransform-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeTransform-request
    (cl:cons ':camera_topic (camera_topic msg))
    (cl:cons ':optical_frame (optical_frame msg))
    (cl:cons ':camera_frame (camera_frame msg))
    (cl:cons ':max_dist (max_dist msg))
))
;//! \htmlinclude ComputeTransform-response.msg.html

(cl:defclass <ComputeTransform-response> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (height_error
    :reader height_error
    :initarg :height_error
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (roll_error
    :reader roll_error
    :initarg :roll_error
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (pitch_error
    :reader pitch_error
    :initarg :pitch_error
    :type cl:float
    :initform 0.0)
   (error
    :reader error
    :initarg :error
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ComputeTransform-response (<ComputeTransform-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeTransform-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeTransform-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_pcl-srv:<ComputeTransform-response> is deprecated: use pal_pcl-srv:ComputeTransform-response instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:height-val is deprecated.  Use pal_pcl-srv:height instead.")
  (height m))

(cl:ensure-generic-function 'height_error-val :lambda-list '(m))
(cl:defmethod height_error-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:height_error-val is deprecated.  Use pal_pcl-srv:height_error instead.")
  (height_error m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:roll-val is deprecated.  Use pal_pcl-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'roll_error-val :lambda-list '(m))
(cl:defmethod roll_error-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:roll_error-val is deprecated.  Use pal_pcl-srv:roll_error instead.")
  (roll_error m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:pitch-val is deprecated.  Use pal_pcl-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'pitch_error-val :lambda-list '(m))
(cl:defmethod pitch_error-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:pitch_error-val is deprecated.  Use pal_pcl-srv:pitch_error instead.")
  (pitch_error m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <ComputeTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_pcl-srv:error-val is deprecated.  Use pal_pcl-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeTransform-response>) ostream)
  "Serializes a message object of type '<ComputeTransform-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'error) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeTransform-response>) istream)
  "Deserializes a message object of type '<ComputeTransform-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'error) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeTransform-response>)))
  "Returns string type for a service object of type '<ComputeTransform-response>"
  "pal_pcl/ComputeTransformResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeTransform-response)))
  "Returns string type for a service object of type 'ComputeTransform-response"
  "pal_pcl/ComputeTransformResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeTransform-response>)))
  "Returns md5sum for a message object of type '<ComputeTransform-response>"
  "63c14dea37df6909b0547d1d22800cc9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeTransform-response)))
  "Returns md5sum for a message object of type 'ComputeTransform-response"
  "63c14dea37df6909b0547d1d22800cc9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeTransform-response>)))
  "Returns full string definition for message of type '<ComputeTransform-response>"
  (cl:format cl:nil "float64 height~%float64 height_error~%float64 roll~%float64 roll_error~%float64 pitch~%float64 pitch_error~%bool error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeTransform-response)))
  "Returns full string definition for message of type 'ComputeTransform-response"
  (cl:format cl:nil "float64 height~%float64 height_error~%float64 roll~%float64 roll_error~%float64 pitch~%float64 pitch_error~%bool error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeTransform-response>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeTransform-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeTransform-response
    (cl:cons ':height (height msg))
    (cl:cons ':height_error (height_error msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':roll_error (roll_error msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':pitch_error (pitch_error msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ComputeTransform)))
  'ComputeTransform-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ComputeTransform)))
  'ComputeTransform-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeTransform)))
  "Returns string type for a service object of type '<ComputeTransform>"
  "pal_pcl/ComputeTransform")