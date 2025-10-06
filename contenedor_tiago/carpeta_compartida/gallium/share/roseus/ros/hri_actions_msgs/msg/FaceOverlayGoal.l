;; Auto-generated. Do not edit!


(when (boundp 'hri_actions_msgs::FaceOverlayGoal)
  (if (not (find-package "HRI_ACTIONS_MSGS"))
    (make-package "HRI_ACTIONS_MSGS"))
  (shadow 'FaceOverlayGoal (find-package "HRI_ACTIONS_MSGS")))
(unless (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL")
  (make-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))

(in-package "ROS")
;;//! \htmlinclude FaceOverlayGoal.msg.html


(intern "*BACKGROUND*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*BACKGROUND* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*BACKGROUND* 0)
(intern "*FOREGROUND*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*FOREGROUND* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*FOREGROUND* 1)
(intern "*EXTEND*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*EXTEND* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*EXTEND* 0)
(intern "*COPY*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*COPY* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*COPY* 1)
(intern "*MIRROR*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*MIRROR* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*MIRROR* 2)
(intern "*LEFT_EYE_ONLY*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*LEFT_EYE_ONLY* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*LEFT_EYE_ONLY* 3)
(intern "*RIGHT_EYE_ONLY*" (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(shadow '*RIGHT_EYE_ONLY* (find-package "HRI_ACTIONS_MSGS::FACEOVERLAYGOAL"))
(defconstant hri_actions_msgs::FaceOverlayGoal::*RIGHT_EYE_ONLY* 4)

(defun hri_actions_msgs::FaceOverlayGoal-to-symbol (const)
  (cond
        ((= const 0) 'hri_actions_msgs::FaceOverlayGoal::*BACKGROUND*)
        ((= const 1) 'hri_actions_msgs::FaceOverlayGoal::*FOREGROUND*)
        ((= const 0) 'hri_actions_msgs::FaceOverlayGoal::*EXTEND*)
        ((= const 1) 'hri_actions_msgs::FaceOverlayGoal::*COPY*)
        ((= const 2) 'hri_actions_msgs::FaceOverlayGoal::*MIRROR*)
        ((= const 3) 'hri_actions_msgs::FaceOverlayGoal::*LEFT_EYE_ONLY*)
        ((= const 4) 'hri_actions_msgs::FaceOverlayGoal::*RIGHT_EYE_ONLY*)
        (t nil)))

(defclass hri_actions_msgs::FaceOverlayGoal
  :super ros::object
  :slots (_media_url _duration _layer _layout _scale _offset_x _offset_y _fade ))

(defmethod hri_actions_msgs::FaceOverlayGoal
  (:init
   (&key
    ((:media_url __media_url) "")
    ((:duration __duration) 0.0)
    ((:layer __layer) 0)
    ((:layout __layout) 0)
    ((:scale __scale) 0.0)
    ((:offset_x __offset_x) 0.0)
    ((:offset_y __offset_y) 0.0)
    ((:fade __fade) nil)
    )
   (send-super :init)
   (setq _media_url (string __media_url))
   (setq _duration (float __duration))
   (setq _layer (round __layer))
   (setq _layout (round __layout))
   (setq _scale (float __scale))
   (setq _offset_x (float __offset_x))
   (setq _offset_y (float __offset_y))
   (setq _fade __fade)
   self)
  (:media_url
   (&optional __media_url)
   (if __media_url (setq _media_url __media_url)) _media_url)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:layer
   (&optional __layer)
   (if __layer (setq _layer __layer)) _layer)
  (:layout
   (&optional __layout)
   (if __layout (setq _layout __layout)) _layout)
  (:scale
   (&optional __scale)
   (if __scale (setq _scale __scale)) _scale)
  (:offset_x
   (&optional __offset_x)
   (if __offset_x (setq _offset_x __offset_x)) _offset_x)
  (:offset_y
   (&optional __offset_y)
   (if __offset_y (setq _offset_y __offset_y)) _offset_y)
  (:fade
   (&optional (__fade :null))
   (if (not (eq __fade :null)) (setq _fade __fade)) _fade)
  (:serialization-length
   ()
   (+
    ;; string _media_url
    4 (length _media_url)
    ;; float32 _duration
    4
    ;; uint8 _layer
    1
    ;; uint8 _layout
    1
    ;; float32 _scale
    4
    ;; float32 _offset_x
    4
    ;; float32 _offset_y
    4
    ;; bool _fade
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _media_url
       (write-long (length _media_url) s) (princ _media_url s)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _layer
       (write-byte _layer s)
     ;; uint8 _layout
       (write-byte _layout s)
     ;; float32 _scale
       (sys::poke _scale (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _offset_x
       (sys::poke _offset_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _offset_y
       (sys::poke _offset_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _fade
       (if _fade (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _media_url
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _media_url (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _layer
     (setq _layer (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _layout
     (setq _layout (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _scale
     (setq _scale (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _offset_x
     (setq _offset_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _offset_y
     (setq _offset_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _fade
     (setq _fade (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get hri_actions_msgs::FaceOverlayGoal :md5sum-) "953d334319802dfd447bdb6867044fd8")
(setf (get hri_actions_msgs::FaceOverlayGoal :datatype-) "hri_actions_msgs/FaceOverlayGoal")
(setf (get hri_actions_msgs::FaceOverlayGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======


# URL of the picture/animation to overlay on the face; currently only supports
# absolute local path
string media_url

# duration, in seconds, of the overlay visibility
# - if the duration is larger than the duration of the animation, the animation
# will loop.
# - a duration of 0 means 'forever'. In this case, the action *will never
# complete* on its own. Cancelling the action will remove the overlay from the
# face.
float32 duration


###############################################
## LAYER

# display the image in the background (the eye's iris/pupil will be drawn on
# top of the overlay
uint8 BACKGROUND=0
# display the image on top of the eye (the eye's iris/pupil will be occluded by
# the image, except where transparent)
uint8 FOREGROUND=1

# one of BACKGROUND or FOREGROUND
uint8 layer

###############################################
## LAYOUT

# extend the provided image to cover both eyes.
# Note: this typically is the only sensible option for robot with a single face panel
uint8 EXTEND=0
# on robots with 2 panels (one per eye), copy the provided image onto each eye
uint8 COPY=1
#  on robots with 2 panels (one per eye), mirror the provided image onto each eyes (symmetry over the vertical axis)
uint8 MIRROR=2
# on robots with 2 panels (one per eye), only display the image on the left eye
uint8 LEFT_EYE_ONLY=3
# on robots with 2 panels (one per eye), only display the image on the right eye
uint8 RIGHT_EYE_ONLY=4

# one of the layout constant defined above
uint8 layout


# scale of the image: 1.0 (default) means full width of the face/eye (depending
# on the chosen layout); 0.1 means 10% of the width.
# Note that any value <= 0.0 is ignored (replaced by 1.0)
float32 scale

# position offset, expressed as a proportion of the face or eye's width/height. Values
# in [-1.0, 1.0] range.
#
# (0.0, 0.0) means that the image is centered with the face/eye
# (-1.0, 1.0) means that the image's center would be located in the top left
# corner of the face/eye panel.
float32 offset_x
float32 offset_y

###############################################
## EFFECTS

# if true, the overlay will fade in/fade out
bool fade


")



(provide :hri_actions_msgs/FaceOverlayGoal "953d334319802dfd447bdb6867044fd8")


