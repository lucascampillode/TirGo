; Auto-generated. Do not edit!


(cl:in-package sba-msg)


;//! \htmlinclude Projection.msg.html

(cl:defclass <Projection> (roslisp-msg-protocol:ros-message)
  ((camindex
    :reader camindex
    :initarg :camindex
    :type cl:integer
    :initform 0)
   (pointindex
    :reader pointindex
    :initarg :pointindex
    :type cl:integer
    :initform 0)
   (u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0)
   (stereo
    :reader stereo
    :initarg :stereo
    :type cl:boolean
    :initform cl:nil)
   (usecovariance
    :reader usecovariance
    :initarg :usecovariance
    :type cl:boolean
    :initform cl:nil)
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Projection (<Projection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Projection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Projection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sba-msg:<Projection> is deprecated: use sba-msg:Projection instead.")))

(cl:ensure-generic-function 'camindex-val :lambda-list '(m))
(cl:defmethod camindex-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:camindex-val is deprecated.  Use sba-msg:camindex instead.")
  (camindex m))

(cl:ensure-generic-function 'pointindex-val :lambda-list '(m))
(cl:defmethod pointindex-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:pointindex-val is deprecated.  Use sba-msg:pointindex instead.")
  (pointindex m))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:u-val is deprecated.  Use sba-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:v-val is deprecated.  Use sba-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:d-val is deprecated.  Use sba-msg:d instead.")
  (d m))

(cl:ensure-generic-function 'stereo-val :lambda-list '(m))
(cl:defmethod stereo-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:stereo-val is deprecated.  Use sba-msg:stereo instead.")
  (stereo m))

(cl:ensure-generic-function 'usecovariance-val :lambda-list '(m))
(cl:defmethod usecovariance-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:usecovariance-val is deprecated.  Use sba-msg:usecovariance instead.")
  (usecovariance m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <Projection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sba-msg:covariance-val is deprecated.  Use sba-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Projection>) ostream)
  "Serializes a message object of type '<Projection>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'camindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'camindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'camindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pointindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pointindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pointindex)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pointindex)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stereo) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'usecovariance) 1 0)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Projection>) istream)
  "Deserializes a message object of type '<Projection>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'camindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'camindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'camindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pointindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pointindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pointindex)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pointindex)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'stereo) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'usecovariance) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Projection>)))
  "Returns string type for a message object of type '<Projection>"
  "sba/Projection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Projection)))
  "Returns string type for a message object of type 'Projection"
  "sba/Projection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Projection>)))
  "Returns md5sum for a message object of type '<Projection>"
  "3d5a2ac666ca0038812f550185cfc756")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Projection)))
  "Returns md5sum for a message object of type 'Projection"
  "3d5a2ac666ca0038812f550185cfc756")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Projection>)))
  "Returns full string definition for message of type '<Projection>"
  (cl:format cl:nil "# Projection~%~%# Camera index~%uint32 camindex~%~%# Point index~%uint32 pointindex~%~%# Projection into the image plane~%float64 u~%float64 v~%float64 d~%~%# Is this a stereo projection? (true if stereo, false if monocular)~%bool stereo~%~%# Use a covariance matrix?~%bool usecovariance~%~%# A 3x3 covariance matrix describing the error~%float64[9] covariance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Projection)))
  "Returns full string definition for message of type 'Projection"
  (cl:format cl:nil "# Projection~%~%# Camera index~%uint32 camindex~%~%# Point index~%uint32 pointindex~%~%# Projection into the image plane~%float64 u~%float64 v~%float64 d~%~%# Is this a stereo projection? (true if stereo, false if monocular)~%bool stereo~%~%# Use a covariance matrix?~%bool usecovariance~%~%# A 3x3 covariance matrix describing the error~%float64[9] covariance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Projection>))
  (cl:+ 0
     4
     4
     8
     8
     8
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Projection>))
  "Converts a ROS message object to a list"
  (cl:list 'Projection
    (cl:cons ':camindex (camindex msg))
    (cl:cons ':pointindex (pointindex msg))
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
    (cl:cons ':d (d msg))
    (cl:cons ':stereo (stereo msg))
    (cl:cons ':usecovariance (usecovariance msg))
    (cl:cons ':covariance (covariance msg))
))
