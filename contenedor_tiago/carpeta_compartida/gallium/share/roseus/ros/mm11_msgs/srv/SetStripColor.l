;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetStripColor)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetStripColor (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETSTRIPCOLOR")
  (make-package "MM11_MSGS::SETSTRIPCOLOR"))
(unless (find-package "MM11_MSGS::SETSTRIPCOLORREQUEST")
  (make-package "MM11_MSGS::SETSTRIPCOLORREQUEST"))
(unless (find-package "MM11_MSGS::SETSTRIPCOLORRESPONSE")
  (make-package "MM11_MSGS::SETSTRIPCOLORRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetStripColorRequest
  :super ros::object
  :slots (_port _r _g _b ))

(defmethod mm11_msgs::SetStripColorRequest
  (:init
   (&key
    ((:port __port) 0)
    ((:r __r) 0)
    ((:g __g) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _port (round __port))
   (setq _r (round __r))
   (setq _g (round __g))
   (setq _b (round __b))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:g
   (&optional __g)
   (if __g (setq _g __g)) _g)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; uint8 _port
    1
    ;; uint8 _r
    1
    ;; uint8 _g
    1
    ;; uint8 _b
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _port
       (write-byte _port s)
     ;; uint8 _r
       (write-byte _r s)
     ;; uint8 _g
       (write-byte _g s)
     ;; uint8 _b
       (write-byte _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _port
     (setq _port (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _r
     (setq _r (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g
     (setq _g (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b
     (setq _b (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetStripColorResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetStripColorResponse
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

(defclass mm11_msgs::SetStripColor
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetStripColor :md5sum-) "96052dec11380a267d245b4ea069fa2d")
(setf (get mm11_msgs::SetStripColor :datatype-) "mm11_msgs/SetStripColor")
(setf (get mm11_msgs::SetStripColor :request) mm11_msgs::SetStripColorRequest)
(setf (get mm11_msgs::SetStripColor :response) mm11_msgs::SetStripColorResponse)

(defmethod mm11_msgs::SetStripColorRequest
  (:response () (instance mm11_msgs::SetStripColorResponse :init)))

(setf (get mm11_msgs::SetStripColorRequest :md5sum-) "96052dec11380a267d245b4ea069fa2d")
(setf (get mm11_msgs::SetStripColorRequest :datatype-) "mm11_msgs/SetStripColorRequest")
(setf (get mm11_msgs::SetStripColorRequest :definition-)
      "# Sets the color of a whole strip

uint8 port

uint8 r
uint8 g
uint8 b

---
")

(setf (get mm11_msgs::SetStripColorResponse :md5sum-) "96052dec11380a267d245b4ea069fa2d")
(setf (get mm11_msgs::SetStripColorResponse :datatype-) "mm11_msgs/SetStripColorResponse")
(setf (get mm11_msgs::SetStripColorResponse :definition-)
      "# Sets the color of a whole strip

uint8 port

uint8 r
uint8 g
uint8 b

---
")



(provide :mm11_msgs/SetStripColor "96052dec11380a267d245b4ea069fa2d")


