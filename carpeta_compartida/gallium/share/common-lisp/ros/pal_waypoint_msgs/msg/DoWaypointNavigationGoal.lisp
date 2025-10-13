; Auto-generated. Do not edit!


(cl:in-package pal_waypoint_msgs-msg)


;//! \htmlinclude DoWaypointNavigationGoal.msg.html

(cl:defclass <DoWaypointNavigationGoal> (roslisp-msg-protocol:ros-message)
  ((group
    :reader group
    :initarg :group
    :type cl:string
    :initform "")
   (first_wp
    :reader first_wp
    :initarg :first_wp
    :type cl:integer
    :initform 0)
   (num_wp
    :reader num_wp
    :initarg :num_wp
    :type cl:integer
    :initform 0)
   (continue_on_error
    :reader continue_on_error
    :initarg :continue_on_error
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DoWaypointNavigationGoal (<DoWaypointNavigationGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DoWaypointNavigationGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DoWaypointNavigationGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_waypoint_msgs-msg:<DoWaypointNavigationGoal> is deprecated: use pal_waypoint_msgs-msg:DoWaypointNavigationGoal instead.")))

(cl:ensure-generic-function 'group-val :lambda-list '(m))
(cl:defmethod group-val ((m <DoWaypointNavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_waypoint_msgs-msg:group-val is deprecated.  Use pal_waypoint_msgs-msg:group instead.")
  (group m))

(cl:ensure-generic-function 'first_wp-val :lambda-list '(m))
(cl:defmethod first_wp-val ((m <DoWaypointNavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_waypoint_msgs-msg:first_wp-val is deprecated.  Use pal_waypoint_msgs-msg:first_wp instead.")
  (first_wp m))

(cl:ensure-generic-function 'num_wp-val :lambda-list '(m))
(cl:defmethod num_wp-val ((m <DoWaypointNavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_waypoint_msgs-msg:num_wp-val is deprecated.  Use pal_waypoint_msgs-msg:num_wp instead.")
  (num_wp m))

(cl:ensure-generic-function 'continue_on_error-val :lambda-list '(m))
(cl:defmethod continue_on_error-val ((m <DoWaypointNavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_waypoint_msgs-msg:continue_on_error-val is deprecated.  Use pal_waypoint_msgs-msg:continue_on_error instead.")
  (continue_on_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DoWaypointNavigationGoal>) ostream)
  "Serializes a message object of type '<DoWaypointNavigationGoal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'group))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'group))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_wp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'first_wp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'first_wp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'first_wp)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_wp)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'continue_on_error) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DoWaypointNavigationGoal>) istream)
  "Deserializes a message object of type '<DoWaypointNavigationGoal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'group) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'group) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_wp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'first_wp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'first_wp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'first_wp)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_wp) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'continue_on_error) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DoWaypointNavigationGoal>)))
  "Returns string type for a message object of type '<DoWaypointNavigationGoal>"
  "pal_waypoint_msgs/DoWaypointNavigationGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoWaypointNavigationGoal)))
  "Returns string type for a message object of type 'DoWaypointNavigationGoal"
  "pal_waypoint_msgs/DoWaypointNavigationGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DoWaypointNavigationGoal>)))
  "Returns md5sum for a message object of type '<DoWaypointNavigationGoal>"
  "61b2209700297362573013f901409437")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DoWaypointNavigationGoal)))
  "Returns md5sum for a message object of type 'DoWaypointNavigationGoal"
  "61b2209700297362573013f901409437")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DoWaypointNavigationGoal>)))
  "Returns full string definition for message of type '<DoWaypointNavigationGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Group of the waypoints~%string group~%~%# Start navigation from first_wp~%uint32 first_wp~%~%# Number of waypoints to visit, 0 means all ~%int32 num_wp~%~%#If a point in the waypoint group fails, ignore and continue to next one~%bool continue_on_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DoWaypointNavigationGoal)))
  "Returns full string definition for message of type 'DoWaypointNavigationGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Group of the waypoints~%string group~%~%# Start navigation from first_wp~%uint32 first_wp~%~%# Number of waypoints to visit, 0 means all ~%int32 num_wp~%~%#If a point in the waypoint group fails, ignore and continue to next one~%bool continue_on_error~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DoWaypointNavigationGoal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'group))
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DoWaypointNavigationGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'DoWaypointNavigationGoal
    (cl:cons ':group (group msg))
    (cl:cons ':first_wp (first_wp msg))
    (cl:cons ':num_wp (num_wp msg))
    (cl:cons ':continue_on_error (continue_on_error msg))
))
