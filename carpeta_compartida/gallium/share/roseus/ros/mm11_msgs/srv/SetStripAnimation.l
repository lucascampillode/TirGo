;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetStripAnimation)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetStripAnimation (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETSTRIPANIMATION")
  (make-package "MM11_MSGS::SETSTRIPANIMATION"))
(unless (find-package "MM11_MSGS::SETSTRIPANIMATIONREQUEST")
  (make-package "MM11_MSGS::SETSTRIPANIMATIONREQUEST"))
(unless (find-package "MM11_MSGS::SETSTRIPANIMATIONRESPONSE")
  (make-package "MM11_MSGS::SETSTRIPANIMATIONRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetStripAnimationRequest
  :super ros::object
  :slots (_port _animation_id _param_1 _param_2 _r_1 _g_1 _b_1 _r_2 _g_2 _b_2 ))

(defmethod mm11_msgs::SetStripAnimationRequest
  (:init
   (&key
    ((:port __port) 0)
    ((:animation_id __animation_id) 0)
    ((:param_1 __param_1) 0)
    ((:param_2 __param_2) 0)
    ((:r_1 __r_1) 0)
    ((:g_1 __g_1) 0)
    ((:b_1 __b_1) 0)
    ((:r_2 __r_2) 0)
    ((:g_2 __g_2) 0)
    ((:b_2 __b_2) 0)
    )
   (send-super :init)
   (setq _port (round __port))
   (setq _animation_id (round __animation_id))
   (setq _param_1 (round __param_1))
   (setq _param_2 (round __param_2))
   (setq _r_1 (round __r_1))
   (setq _g_1 (round __g_1))
   (setq _b_1 (round __b_1))
   (setq _r_2 (round __r_2))
   (setq _g_2 (round __g_2))
   (setq _b_2 (round __b_2))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:animation_id
   (&optional __animation_id)
   (if __animation_id (setq _animation_id __animation_id)) _animation_id)
  (:param_1
   (&optional __param_1)
   (if __param_1 (setq _param_1 __param_1)) _param_1)
  (:param_2
   (&optional __param_2)
   (if __param_2 (setq _param_2 __param_2)) _param_2)
  (:r_1
   (&optional __r_1)
   (if __r_1 (setq _r_1 __r_1)) _r_1)
  (:g_1
   (&optional __g_1)
   (if __g_1 (setq _g_1 __g_1)) _g_1)
  (:b_1
   (&optional __b_1)
   (if __b_1 (setq _b_1 __b_1)) _b_1)
  (:r_2
   (&optional __r_2)
   (if __r_2 (setq _r_2 __r_2)) _r_2)
  (:g_2
   (&optional __g_2)
   (if __g_2 (setq _g_2 __g_2)) _g_2)
  (:b_2
   (&optional __b_2)
   (if __b_2 (setq _b_2 __b_2)) _b_2)
  (:serialization-length
   ()
   (+
    ;; uint8 _port
    1
    ;; uint8 _animation_id
    1
    ;; int16 _param_1
    2
    ;; uint16 _param_2
    2
    ;; uint8 _r_1
    1
    ;; uint8 _g_1
    1
    ;; uint8 _b_1
    1
    ;; uint8 _r_2
    1
    ;; uint8 _g_2
    1
    ;; uint8 _b_2
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _port
       (write-byte _port s)
     ;; uint8 _animation_id
       (write-byte _animation_id s)
     ;; int16 _param_1
       (write-word _param_1 s)
     ;; uint16 _param_2
       (write-word _param_2 s)
     ;; uint8 _r_1
       (write-byte _r_1 s)
     ;; uint8 _g_1
       (write-byte _g_1 s)
     ;; uint8 _b_1
       (write-byte _b_1 s)
     ;; uint8 _r_2
       (write-byte _r_2 s)
     ;; uint8 _g_2
       (write-byte _g_2 s)
     ;; uint8 _b_2
       (write-byte _b_2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _port
     (setq _port (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _animation_id
     (setq _animation_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _param_1
     (setq _param_1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _param_2
     (setq _param_2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _r_1
     (setq _r_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g_1
     (setq _g_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b_1
     (setq _b_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _r_2
     (setq _r_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g_2
     (setq _g_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b_2
     (setq _b_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetStripAnimationResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetStripAnimationResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass mm11_msgs::SetStripAnimation
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetStripAnimation :md5sum-) "d1648b64d1990c75ff0a58f1ddcbf68a")
(setf (get mm11_msgs::SetStripAnimation :datatype-) "mm11_msgs/SetStripAnimation")
(setf (get mm11_msgs::SetStripAnimation :request) mm11_msgs::SetStripAnimationRequest)
(setf (get mm11_msgs::SetStripAnimation :response) mm11_msgs::SetStripAnimationResponse)

(defmethod mm11_msgs::SetStripAnimationRequest
  (:response () (instance mm11_msgs::SetStripAnimationResponse :init)))

(setf (get mm11_msgs::SetStripAnimationRequest :md5sum-) "d1648b64d1990c75ff0a58f1ddcbf68a")
(setf (get mm11_msgs::SetStripAnimationRequest :datatype-) "mm11_msgs/SetStripAnimationRequest")
(setf (get mm11_msgs::SetStripAnimationRequest :definition-)
      "# Animate a whole strip

uint8 port

uint8 animation_id

int16 param_1
uint16 param_2

uint8 r_1
uint8 g_1
uint8 b_1

uint8 r_2
uint8 g_2
uint8 b_2

---
")

(setf (get mm11_msgs::SetStripAnimationResponse :md5sum-) "d1648b64d1990c75ff0a58f1ddcbf68a")
(setf (get mm11_msgs::SetStripAnimationResponse :datatype-) "mm11_msgs/SetStripAnimationResponse")
(setf (get mm11_msgs::SetStripAnimationResponse :definition-)
      "# Animate a whole strip

uint8 port

uint8 animation_id

int16 param_1
uint16 param_2

uint8 r_1
uint8 g_1
uint8 b_1

uint8 r_2
uint8 g_2
uint8 b_2

---
")



(provide :mm11_msgs/SetStripAnimation "d1648b64d1990c75ff0a58f1ddcbf68a")


