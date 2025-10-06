;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetStripPixelColor)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetStripPixelColor (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETSTRIPPIXELCOLOR")
  (make-package "MM11_MSGS::SETSTRIPPIXELCOLOR"))
(unless (find-package "MM11_MSGS::SETSTRIPPIXELCOLORREQUEST")
  (make-package "MM11_MSGS::SETSTRIPPIXELCOLORREQUEST"))
(unless (find-package "MM11_MSGS::SETSTRIPPIXELCOLORRESPONSE")
  (make-package "MM11_MSGS::SETSTRIPPIXELCOLORRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetStripPixelColorRequest
  :super ros::object
  :slots (_port _pixel _r _g _b ))

(defmethod mm11_msgs::SetStripPixelColorRequest
  (:init
   (&key
    ((:port __port) 0)
    ((:pixel __pixel) 0)
    ((:r __r) 0)
    ((:g __g) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _port (round __port))
   (setq _pixel (round __pixel))
   (setq _r (round __r))
   (setq _g (round __g))
   (setq _b (round __b))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:pixel
   (&optional __pixel)
   (if __pixel (setq _pixel __pixel)) _pixel)
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
    ;; uint8 _pixel
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
     ;; uint8 _pixel
       (write-byte _pixel s)
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
   ;; uint8 _pixel
     (setq _pixel (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _r
     (setq _r (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g
     (setq _g (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b
     (setq _b (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetStripPixelColorResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetStripPixelColorResponse
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

(defclass mm11_msgs::SetStripPixelColor
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetStripPixelColor :md5sum-) "b649b5b940622eb64d13b51266bf8b29")
(setf (get mm11_msgs::SetStripPixelColor :datatype-) "mm11_msgs/SetStripPixelColor")
(setf (get mm11_msgs::SetStripPixelColor :request) mm11_msgs::SetStripPixelColorRequest)
(setf (get mm11_msgs::SetStripPixelColor :response) mm11_msgs::SetStripPixelColorResponse)

(defmethod mm11_msgs::SetStripPixelColorRequest
  (:response () (instance mm11_msgs::SetStripPixelColorResponse :init)))

(setf (get mm11_msgs::SetStripPixelColorRequest :md5sum-) "b649b5b940622eb64d13b51266bf8b29")
(setf (get mm11_msgs::SetStripPixelColorRequest :datatype-) "mm11_msgs/SetStripPixelColorRequest")
(setf (get mm11_msgs::SetStripPixelColorRequest :definition-)
      "# Sets the color of a strip's pixel

uint8 port

uint8 pixel

uint8 r
uint8 g
uint8 b

---
")

(setf (get mm11_msgs::SetStripPixelColorResponse :md5sum-) "b649b5b940622eb64d13b51266bf8b29")
(setf (get mm11_msgs::SetStripPixelColorResponse :datatype-) "mm11_msgs/SetStripPixelColorResponse")
(setf (get mm11_msgs::SetStripPixelColorResponse :definition-)
      "# Sets the color of a strip's pixel

uint8 port

uint8 pixel

uint8 r
uint8 g
uint8 b

---
")



(provide :mm11_msgs/SetStripPixelColor "b649b5b940622eb64d13b51266bf8b29")


