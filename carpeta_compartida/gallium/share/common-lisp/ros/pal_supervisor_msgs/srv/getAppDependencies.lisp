; Auto-generated. Do not edit!


(cl:in-package pal_supervisor_msgs-srv)


;//! \htmlinclude getAppDependencies-request.msg.html

(cl:defclass <getAppDependencies-request> (roslisp-msg-protocol:ros-message)
  ((appName
    :reader appName
    :initarg :appName
    :type cl:string
    :initform ""))
)

(cl:defclass getAppDependencies-request (<getAppDependencies-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAppDependencies-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAppDependencies-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<getAppDependencies-request> is deprecated: use pal_supervisor_msgs-srv:getAppDependencies-request instead.")))

(cl:ensure-generic-function 'appName-val :lambda-list '(m))
(cl:defmethod appName-val ((m <getAppDependencies-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:appName-val is deprecated.  Use pal_supervisor_msgs-srv:appName instead.")
  (appName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAppDependencies-request>) ostream)
  "Serializes a message object of type '<getAppDependencies-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'appName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'appName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAppDependencies-request>) istream)
  "Deserializes a message object of type '<getAppDependencies-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'appName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'appName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAppDependencies-request>)))
  "Returns string type for a service object of type '<getAppDependencies-request>"
  "pal_supervisor_msgs/getAppDependenciesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAppDependencies-request)))
  "Returns string type for a service object of type 'getAppDependencies-request"
  "pal_supervisor_msgs/getAppDependenciesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAppDependencies-request>)))
  "Returns md5sum for a message object of type '<getAppDependencies-request>"
  "f7faf0b37d011444aea3a2fa0a051c43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAppDependencies-request)))
  "Returns md5sum for a message object of type 'getAppDependencies-request"
  "f7faf0b37d011444aea3a2fa0a051c43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAppDependencies-request>)))
  "Returns full string definition for message of type '<getAppDependencies-request>"
  (cl:format cl:nil "string appName #Name of the application ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAppDependencies-request)))
  "Returns full string definition for message of type 'getAppDependencies-request"
  (cl:format cl:nil "string appName #Name of the application ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAppDependencies-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'appName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAppDependencies-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getAppDependencies-request
    (cl:cons ':appName (appName msg))
))
;//! \htmlinclude getAppDependencies-response.msg.html

(cl:defclass <getAppDependencies-response> (roslisp-msg-protocol:ros-message)
  ((dependencies
    :reader dependencies
    :initarg :dependencies
    :type (cl:vector pal_supervisor_msgs-msg:DependencyStatus)
   :initform (cl:make-array 0 :element-type 'pal_supervisor_msgs-msg:DependencyStatus :initial-element (cl:make-instance 'pal_supervisor_msgs-msg:DependencyStatus))))
)

(cl:defclass getAppDependencies-response (<getAppDependencies-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAppDependencies-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAppDependencies-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_supervisor_msgs-srv:<getAppDependencies-response> is deprecated: use pal_supervisor_msgs-srv:getAppDependencies-response instead.")))

(cl:ensure-generic-function 'dependencies-val :lambda-list '(m))
(cl:defmethod dependencies-val ((m <getAppDependencies-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_supervisor_msgs-srv:dependencies-val is deprecated.  Use pal_supervisor_msgs-srv:dependencies instead.")
  (dependencies m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAppDependencies-response>) ostream)
  "Serializes a message object of type '<getAppDependencies-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dependencies))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'dependencies))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAppDependencies-response>) istream)
  "Deserializes a message object of type '<getAppDependencies-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dependencies) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dependencies)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pal_supervisor_msgs-msg:DependencyStatus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAppDependencies-response>)))
  "Returns string type for a service object of type '<getAppDependencies-response>"
  "pal_supervisor_msgs/getAppDependenciesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAppDependencies-response)))
  "Returns string type for a service object of type 'getAppDependencies-response"
  "pal_supervisor_msgs/getAppDependenciesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAppDependencies-response>)))
  "Returns md5sum for a message object of type '<getAppDependencies-response>"
  "f7faf0b37d011444aea3a2fa0a051c43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAppDependencies-response)))
  "Returns md5sum for a message object of type 'getAppDependencies-response"
  "f7faf0b37d011444aea3a2fa0a051c43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAppDependencies-response>)))
  "Returns full string definition for message of type '<getAppDependencies-response>"
  (cl:format cl:nil "pal_supervisor_msgs/DependencyStatus[] dependencies #Array of dependencies~%~%~%================================================================================~%MSG: pal_supervisor_msgs/DependencyStatus~%string name # dependency name~%# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3 #Application not running~%byte level # level of operation enumerated above ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAppDependencies-response)))
  "Returns full string definition for message of type 'getAppDependencies-response"
  (cl:format cl:nil "pal_supervisor_msgs/DependencyStatus[] dependencies #Array of dependencies~%~%~%================================================================================~%MSG: pal_supervisor_msgs/DependencyStatus~%string name # dependency name~%# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3 #Application not running~%byte level # level of operation enumerated above ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAppDependencies-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dependencies) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAppDependencies-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getAppDependencies-response
    (cl:cons ':dependencies (dependencies msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getAppDependencies)))
  'getAppDependencies-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getAppDependencies)))
  'getAppDependencies-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAppDependencies)))
  "Returns string type for a service object of type '<getAppDependencies>"
  "pal_supervisor_msgs/getAppDependencies")