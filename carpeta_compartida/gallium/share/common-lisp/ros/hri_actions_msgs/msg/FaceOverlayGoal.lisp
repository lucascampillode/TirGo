; Auto-generated. Do not edit!


(cl:in-package hri_actions_msgs-msg)


;//! \htmlinclude FaceOverlayGoal.msg.html

(cl:defclass <FaceOverlayGoal> (roslisp-msg-protocol:ros-message)
  ((media_url
    :reader media_url
    :initarg :media_url
    :type cl:string
    :initform "")
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0)
   (layer
    :reader layer
    :initarg :layer
    :type cl:fixnum
    :initform 0)
   (layout
    :reader layout
    :initarg :layout
    :type cl:fixnum
    :initform 0)
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0)
   (offset_x
    :reader offset_x
    :initarg :offset_x
    :type cl:float
    :initform 0.0)
   (offset_y
    :reader offset_y
    :initarg :offset_y
    :type cl:float
    :initform 0.0)
   (fade
    :reader fade
    :initarg :fade
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FaceOverlayGoal (<FaceOverlayGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaceOverlayGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaceOverlayGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hri_actions_msgs-msg:<FaceOverlayGoal> is deprecated: use hri_actions_msgs-msg:FaceOverlayGoal instead.")))

(cl:ensure-generic-function 'media_url-val :lambda-list '(m))
(cl:defmethod media_url-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:media_url-val is deprecated.  Use hri_actions_msgs-msg:media_url instead.")
  (media_url m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:duration-val is deprecated.  Use hri_actions_msgs-msg:duration instead.")
  (duration m))

(cl:ensure-generic-function 'layer-val :lambda-list '(m))
(cl:defmethod layer-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:layer-val is deprecated.  Use hri_actions_msgs-msg:layer instead.")
  (layer m))

(cl:ensure-generic-function 'layout-val :lambda-list '(m))
(cl:defmethod layout-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:layout-val is deprecated.  Use hri_actions_msgs-msg:layout instead.")
  (layout m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:scale-val is deprecated.  Use hri_actions_msgs-msg:scale instead.")
  (scale m))

(cl:ensure-generic-function 'offset_x-val :lambda-list '(m))
(cl:defmethod offset_x-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:offset_x-val is deprecated.  Use hri_actions_msgs-msg:offset_x instead.")
  (offset_x m))

(cl:ensure-generic-function 'offset_y-val :lambda-list '(m))
(cl:defmethod offset_y-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:offset_y-val is deprecated.  Use hri_actions_msgs-msg:offset_y instead.")
  (offset_y m))

(cl:ensure-generic-function 'fade-val :lambda-list '(m))
(cl:defmethod fade-val ((m <FaceOverlayGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hri_actions_msgs-msg:fade-val is deprecated.  Use hri_actions_msgs-msg:fade instead.")
  (fade m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FaceOverlayGoal>)))
    "Constants for message type '<FaceOverlayGoal>"
  '((:BACKGROUND . 0)
    (:FOREGROUND . 1)
    (:EXTEND . 0)
    (:COPY . 1)
    (:MIRROR . 2)
    (:LEFT_EYE_ONLY . 3)
    (:RIGHT_EYE_ONLY . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FaceOverlayGoal)))
    "Constants for message type 'FaceOverlayGoal"
  '((:BACKGROUND . 0)
    (:FOREGROUND . 1)
    (:EXTEND . 0)
    (:COPY . 1)
    (:MIRROR . 2)
    (:LEFT_EYE_ONLY . 3)
    (:RIGHT_EYE_ONLY . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaceOverlayGoal>) ostream)
  "Serializes a message object of type '<FaceOverlayGoal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'media_url))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'media_url))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'layer)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'layout)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'offset_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'offset_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fade) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaceOverlayGoal>) istream)
  "Deserializes a message object of type '<FaceOverlayGoal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'media_url) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'media_url) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'layer)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'layout)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'fade) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaceOverlayGoal>)))
  "Returns string type for a message object of type '<FaceOverlayGoal>"
  "hri_actions_msgs/FaceOverlayGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaceOverlayGoal)))
  "Returns string type for a message object of type 'FaceOverlayGoal"
  "hri_actions_msgs/FaceOverlayGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaceOverlayGoal>)))
  "Returns md5sum for a message object of type '<FaceOverlayGoal>"
  "953d334319802dfd447bdb6867044fd8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaceOverlayGoal)))
  "Returns md5sum for a message object of type 'FaceOverlayGoal"
  "953d334319802dfd447bdb6867044fd8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaceOverlayGoal>)))
  "Returns full string definition for message of type '<FaceOverlayGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%# URL of the picture/animation to overlay on the face; currently only supports~%# absolute local path~%string media_url~%~%# duration, in seconds, of the overlay visibility~%# - if the duration is larger than the duration of the animation, the animation~%# will loop.~%# - a duration of 0 means 'forever'. In this case, the action *will never~%# complete* on its own. Cancelling the action will remove the overlay from the~%# face.~%float32 duration~%~%~%###############################################~%## LAYER~%~%# display the image in the background (the eye's iris/pupil will be drawn on~%# top of the overlay~%uint8 BACKGROUND=0~%# display the image on top of the eye (the eye's iris/pupil will be occluded by~%# the image, except where transparent)~%uint8 FOREGROUND=1~%~%# one of BACKGROUND or FOREGROUND~%uint8 layer~%~%###############################################~%## LAYOUT~%~%# extend the provided image to cover both eyes.~%# Note: this typically is the only sensible option for robot with a single face panel~%uint8 EXTEND=0~%# on robots with 2 panels (one per eye), copy the provided image onto each eye~%uint8 COPY=1~%#  on robots with 2 panels (one per eye), mirror the provided image onto each eyes (symmetry over the vertical axis)~%uint8 MIRROR=2~%# on robots with 2 panels (one per eye), only display the image on the left eye~%uint8 LEFT_EYE_ONLY=3~%# on robots with 2 panels (one per eye), only display the image on the right eye~%uint8 RIGHT_EYE_ONLY=4~%~%# one of the layout constant defined above~%uint8 layout~%~%~%# scale of the image: 1.0 (default) means full width of the face/eye (depending~%# on the chosen layout); 0.1 means 10% of the width.~%# Note that any value <= 0.0 is ignored (replaced by 1.0)~%float32 scale~%~%# position offset, expressed as a proportion of the face or eye's width/height. Values~%# in [-1.0, 1.0] range.~%#~%# (0.0, 0.0) means that the image is centered with the face/eye~%# (-1.0, 1.0) means that the image's center would be located in the top left~%# corner of the face/eye panel.~%float32 offset_x~%float32 offset_y~%~%###############################################~%## EFFECTS~%~%# if true, the overlay will fade in/fade out~%bool fade~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaceOverlayGoal)))
  "Returns full string definition for message of type 'FaceOverlayGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%# URL of the picture/animation to overlay on the face; currently only supports~%# absolute local path~%string media_url~%~%# duration, in seconds, of the overlay visibility~%# - if the duration is larger than the duration of the animation, the animation~%# will loop.~%# - a duration of 0 means 'forever'. In this case, the action *will never~%# complete* on its own. Cancelling the action will remove the overlay from the~%# face.~%float32 duration~%~%~%###############################################~%## LAYER~%~%# display the image in the background (the eye's iris/pupil will be drawn on~%# top of the overlay~%uint8 BACKGROUND=0~%# display the image on top of the eye (the eye's iris/pupil will be occluded by~%# the image, except where transparent)~%uint8 FOREGROUND=1~%~%# one of BACKGROUND or FOREGROUND~%uint8 layer~%~%###############################################~%## LAYOUT~%~%# extend the provided image to cover both eyes.~%# Note: this typically is the only sensible option for robot with a single face panel~%uint8 EXTEND=0~%# on robots with 2 panels (one per eye), copy the provided image onto each eye~%uint8 COPY=1~%#  on robots with 2 panels (one per eye), mirror the provided image onto each eyes (symmetry over the vertical axis)~%uint8 MIRROR=2~%# on robots with 2 panels (one per eye), only display the image on the left eye~%uint8 LEFT_EYE_ONLY=3~%# on robots with 2 panels (one per eye), only display the image on the right eye~%uint8 RIGHT_EYE_ONLY=4~%~%# one of the layout constant defined above~%uint8 layout~%~%~%# scale of the image: 1.0 (default) means full width of the face/eye (depending~%# on the chosen layout); 0.1 means 10% of the width.~%# Note that any value <= 0.0 is ignored (replaced by 1.0)~%float32 scale~%~%# position offset, expressed as a proportion of the face or eye's width/height. Values~%# in [-1.0, 1.0] range.~%#~%# (0.0, 0.0) means that the image is centered with the face/eye~%# (-1.0, 1.0) means that the image's center would be located in the top left~%# corner of the face/eye panel.~%float32 offset_x~%float32 offset_y~%~%###############################################~%## EFFECTS~%~%# if true, the overlay will fade in/fade out~%bool fade~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaceOverlayGoal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'media_url))
     4
     1
     1
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaceOverlayGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'FaceOverlayGoal
    (cl:cons ':media_url (media_url msg))
    (cl:cons ':duration (duration msg))
    (cl:cons ':layer (layer msg))
    (cl:cons ':layout (layout msg))
    (cl:cons ':scale (scale msg))
    (cl:cons ':offset_x (offset_x msg))
    (cl:cons ':offset_y (offset_y msg))
    (cl:cons ':fade (fade msg))
))
