; Auto-generated. Do not edit!


(cl:in-package pal_startup_msgs-msg)


;//! \htmlinclude StartupStatus.msg.html

(cl:defclass <StartupStatus> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform "")
   (startup_id
    :reader startup_id
    :initarg :startup_id
    :type cl:string
    :initform ""))
)

(cl:defclass StartupStatus (<StartupStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartupStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartupStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_startup_msgs-msg:<StartupStatus> is deprecated: use pal_startup_msgs-msg:StartupStatus instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <StartupStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-msg:name-val is deprecated.  Use pal_startup_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <StartupStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-msg:message-val is deprecated.  Use pal_startup_msgs-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <StartupStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-msg:status-val is deprecated.  Use pal_startup_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'startup_id-val :lambda-list '(m))
(cl:defmethod startup_id-val ((m <StartupStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_startup_msgs-msg:startup_id-val is deprecated.  Use pal_startup_msgs-msg:startup_id instead.")
  (startup_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StartupStatus>)))
    "Constants for message type '<StartupStatus>"
  '((:MISSING . "Missing")
    (:RUNNING . "Running")
    (:PENDING . "Pending")
    (:CRASHED . "Crashed")
    (:STOPPED . "Stopped")
    (:ABORTED . "Aborted")
    (:TIMEOUT . "Timeout")
    (:MANUAL . "Manual")
    (:FINISHED . "Finished"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StartupStatus)))
    "Constants for message type 'StartupStatus"
  '((:MISSING . "Missing")
    (:RUNNING . "Running")
    (:PENDING . "Pending")
    (:CRASHED . "Crashed")
    (:STOPPED . "Stopped")
    (:ABORTED . "Aborted")
    (:TIMEOUT . "Timeout")
    (:MANUAL . "Manual")
    (:FINISHED . "Finished"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartupStatus>) ostream)
  "Serializes a message object of type '<StartupStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'startup_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'startup_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartupStatus>) istream)
  "Deserializes a message object of type '<StartupStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'startup_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'startup_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartupStatus>)))
  "Returns string type for a message object of type '<StartupStatus>"
  "pal_startup_msgs/StartupStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartupStatus)))
  "Returns string type for a message object of type 'StartupStatus"
  "pal_startup_msgs/StartupStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartupStatus>)))
  "Returns md5sum for a message object of type '<StartupStatus>"
  "ca78e8aec80203b6c29ca3e480879775")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartupStatus)))
  "Returns md5sum for a message object of type 'StartupStatus"
  "ca78e8aec80203b6c29ca3e480879775")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartupStatus>)))
  "Returns full string definition for message of type '<StartupStatus>"
  (cl:format cl:nil "string name # application name~%string message # description of the status if required~%# Possible status of startup process~%string MISSING=Missing~%string RUNNING=Running~%string PENDING=Pending~%string CRASHED=Crashed~%string STOPPED=Stopped~%string ABORTED=Aborted~%string TIMEOUT=Timeout~%string MANUAL=Manual~%string FINISHED=Finished~%string status # status of the startup process for the application. Enumerated above ~%string startup_id # Id of the node starting the app (ie: control, multimedia)~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartupStatus)))
  "Returns full string definition for message of type 'StartupStatus"
  (cl:format cl:nil "string name # application name~%string message # description of the status if required~%# Possible status of startup process~%string MISSING=Missing~%string RUNNING=Running~%string PENDING=Pending~%string CRASHED=Crashed~%string STOPPED=Stopped~%string ABORTED=Aborted~%string TIMEOUT=Timeout~%string MANUAL=Manual~%string FINISHED=Finished~%string status # status of the startup process for the application. Enumerated above ~%string startup_id # Id of the node starting the app (ie: control, multimedia)~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartupStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:length (cl:slot-value msg 'status))
     4 (cl:length (cl:slot-value msg 'startup_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartupStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'StartupStatus
    (cl:cons ':name (name msg))
    (cl:cons ':message (message msg))
    (cl:cons ':status (status msg))
    (cl:cons ':startup_id (startup_id msg))
))
