;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetLedState)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetLedState (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETLEDSTATE")
  (make-package "MM11_MSGS::SETLEDSTATE"))
(unless (find-package "MM11_MSGS::SETLEDSTATEREQUEST")
  (make-package "MM11_MSGS::SETLEDSTATEREQUEST"))
(unless (find-package "MM11_MSGS::SETLEDSTATERESPONSE")
  (make-package "MM11_MSGS::SETLEDSTATERESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetLedStateRequest
  :super ros::object
  :slots (_led _state ))

(defmethod mm11_msgs::SetLedStateRequest
  (:init
   (&key
    ((:led __led) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _led (round __led))
   (setq _state (round __state))
   self)
  (:led
   (&optional __led)
   (if __led (setq _led __led)) _led)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _led
    1
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _led
       (write-byte _led s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _led
     (setq _led (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetLedStateResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetLedStateResponse
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

(defclass mm11_msgs::SetLedState
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetLedState :md5sum-) "97b59df318c0b0c26d854b0a9da207f8")
(setf (get mm11_msgs::SetLedState :datatype-) "mm11_msgs/SetLedState")
(setf (get mm11_msgs::SetLedState :request) mm11_msgs::SetLedStateRequest)
(setf (get mm11_msgs::SetLedState :response) mm11_msgs::SetLedStateResponse)

(defmethod mm11_msgs::SetLedStateRequest
  (:response () (instance mm11_msgs::SetLedStateResponse :init)))

(setf (get mm11_msgs::SetLedStateRequest :md5sum-) "97b59df318c0b0c26d854b0a9da207f8")
(setf (get mm11_msgs::SetLedStateRequest :datatype-) "mm11_msgs/SetLedStateRequest")
(setf (get mm11_msgs::SetLedStateRequest :definition-)
      "# Set a LED state

uint8 led

uint8 state

---
")

(setf (get mm11_msgs::SetLedStateResponse :md5sum-) "97b59df318c0b0c26d854b0a9da207f8")
(setf (get mm11_msgs::SetLedStateResponse :datatype-) "mm11_msgs/SetLedStateResponse")
(setf (get mm11_msgs::SetLedStateResponse :definition-)
      "# Set a LED state

uint8 led

uint8 state

---
")



(provide :mm11_msgs/SetLedState "97b59df318c0b0c26d854b0a9da207f8")


