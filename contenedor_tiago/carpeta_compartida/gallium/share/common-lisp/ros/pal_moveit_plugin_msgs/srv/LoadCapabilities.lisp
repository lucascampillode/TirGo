; Auto-generated. Do not edit!


(cl:in-package pal_moveit_plugin_msgs-srv)


;//! \htmlinclude LoadCapabilities-request.msg.html

(cl:defclass <LoadCapabilities-request> (roslisp-msg-protocol:ros-message)
  ((capabilities
    :reader capabilities
    :initarg :capabilities
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass LoadCapabilities-request (<LoadCapabilities-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadCapabilities-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadCapabilities-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_moveit_plugin_msgs-srv:<LoadCapabilities-request> is deprecated: use pal_moveit_plugin_msgs-srv:LoadCapabilities-request instead.")))

(cl:ensure-generic-function 'capabilities-val :lambda-list '(m))
(cl:defmethod capabilities-val ((m <LoadCapabilities-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_moveit_plugin_msgs-srv:capabilities-val is deprecated.  Use pal_moveit_plugin_msgs-srv:capabilities instead.")
  (capabilities m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadCapabilities-request>) ostream)
  "Serializes a message object of type '<LoadCapabilities-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'capabilities))))
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
   (cl:slot-value msg 'capabilities))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadCapabilities-request>) istream)
  "Deserializes a message object of type '<LoadCapabilities-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'capabilities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'capabilities)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadCapabilities-request>)))
  "Returns string type for a service object of type '<LoadCapabilities-request>"
  "pal_moveit_plugin_msgs/LoadCapabilitiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadCapabilities-request)))
  "Returns string type for a service object of type 'LoadCapabilities-request"
  "pal_moveit_plugin_msgs/LoadCapabilitiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadCapabilities-request>)))
  "Returns md5sum for a message object of type '<LoadCapabilities-request>"
  "c6f702db26dbc58fc6ddc660a0bd1ac9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadCapabilities-request)))
  "Returns md5sum for a message object of type 'LoadCapabilities-request"
  "c6f702db26dbc58fc6ddc660a0bd1ac9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadCapabilities-request>)))
  "Returns full string definition for message of type '<LoadCapabilities-request>"
  (cl:format cl:nil "# list of capabilities~%string[] capabilities~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadCapabilities-request)))
  "Returns full string definition for message of type 'LoadCapabilities-request"
  (cl:format cl:nil "# list of capabilities~%string[] capabilities~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadCapabilities-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'capabilities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadCapabilities-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadCapabilities-request
    (cl:cons ':capabilities (capabilities msg))
))
;//! \htmlinclude LoadCapabilities-response.msg.html

(cl:defclass <LoadCapabilities-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (error
    :reader error
    :initarg :error
    :type cl:string
    :initform ""))
)

(cl:defclass LoadCapabilities-response (<LoadCapabilities-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadCapabilities-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadCapabilities-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_moveit_plugin_msgs-srv:<LoadCapabilities-response> is deprecated: use pal_moveit_plugin_msgs-srv:LoadCapabilities-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LoadCapabilities-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_moveit_plugin_msgs-srv:success-val is deprecated.  Use pal_moveit_plugin_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <LoadCapabilities-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_moveit_plugin_msgs-srv:error-val is deprecated.  Use pal_moveit_plugin_msgs-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadCapabilities-response>) ostream)
  "Serializes a message object of type '<LoadCapabilities-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadCapabilities-response>) istream)
  "Deserializes a message object of type '<LoadCapabilities-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadCapabilities-response>)))
  "Returns string type for a service object of type '<LoadCapabilities-response>"
  "pal_moveit_plugin_msgs/LoadCapabilitiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadCapabilities-response)))
  "Returns string type for a service object of type 'LoadCapabilities-response"
  "pal_moveit_plugin_msgs/LoadCapabilitiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadCapabilities-response>)))
  "Returns md5sum for a message object of type '<LoadCapabilities-response>"
  "c6f702db26dbc58fc6ddc660a0bd1ac9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadCapabilities-response)))
  "Returns md5sum for a message object of type 'LoadCapabilities-response"
  "c6f702db26dbc58fc6ddc660a0bd1ac9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadCapabilities-response>)))
  "Returns full string definition for message of type '<LoadCapabilities-response>"
  (cl:format cl:nil "bool success~%string error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadCapabilities-response)))
  "Returns full string definition for message of type 'LoadCapabilities-response"
  (cl:format cl:nil "bool success~%string error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadCapabilities-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'error))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadCapabilities-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadCapabilities-response
    (cl:cons ':success (success msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadCapabilities)))
  'LoadCapabilities-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadCapabilities)))
  'LoadCapabilities-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadCapabilities)))
  "Returns string type for a service object of type '<LoadCapabilities>"
  "pal_moveit_plugin_msgs/LoadCapabilities")