;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetLedsBrightness)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetLedsBrightness (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETLEDSBRIGHTNESS")
  (make-package "MM11_MSGS::SETLEDSBRIGHTNESS"))
(unless (find-package "MM11_MSGS::SETLEDSBRIGHTNESSREQUEST")
  (make-package "MM11_MSGS::SETLEDSBRIGHTNESSREQUEST"))
(unless (find-package "MM11_MSGS::SETLEDSBRIGHTNESSRESPONSE")
  (make-package "MM11_MSGS::SETLEDSBRIGHTNESSRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetLedsBrightnessRequest
  :super ros::object
  :slots (_brightness ))

(defmethod mm11_msgs::SetLedsBrightnessRequest
  (:init
   (&key
    ((:brightness __brightness) 0)
    )
   (send-super :init)
   (setq _brightness (round __brightness))
   self)
  (:brightness
   (&optional __brightness)
   (if __brightness (setq _brightness __brightness)) _brightness)
  (:serialization-length
   ()
   (+
    ;; uint8 _brightness
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _brightness
       (write-byte _brightness s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _brightness
     (setq _brightness (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetLedsBrightnessResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetLedsBrightnessResponse
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

(defclass mm11_msgs::SetLedsBrightness
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetLedsBrightness :md5sum-) "0374114287947f81d241e42c7a34c37e")
(setf (get mm11_msgs::SetLedsBrightness :datatype-) "mm11_msgs/SetLedsBrightness")
(setf (get mm11_msgs::SetLedsBrightness :request) mm11_msgs::SetLedsBrightnessRequest)
(setf (get mm11_msgs::SetLedsBrightness :response) mm11_msgs::SetLedsBrightnessResponse)

(defmethod mm11_msgs::SetLedsBrightnessRequest
  (:response () (instance mm11_msgs::SetLedsBrightnessResponse :init)))

(setf (get mm11_msgs::SetLedsBrightnessRequest :md5sum-) "0374114287947f81d241e42c7a34c37e")
(setf (get mm11_msgs::SetLedsBrightnessRequest :datatype-) "mm11_msgs/SetLedsBrightnessRequest")
(setf (get mm11_msgs::SetLedsBrightnessRequest :definition-)
      "# Set the LEDs brightness

uint8 brightness

---
")

(setf (get mm11_msgs::SetLedsBrightnessResponse :md5sum-) "0374114287947f81d241e42c7a34c37e")
(setf (get mm11_msgs::SetLedsBrightnessResponse :datatype-) "mm11_msgs/SetLedsBrightnessResponse")
(setf (get mm11_msgs::SetLedsBrightnessResponse :definition-)
      "# Set the LEDs brightness

uint8 brightness

---
")



(provide :mm11_msgs/SetLedsBrightness "0374114287947f81d241e42c7a34c37e")


