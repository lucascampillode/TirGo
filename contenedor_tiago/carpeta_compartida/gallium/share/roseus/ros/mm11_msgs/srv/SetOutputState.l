;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetOutputState)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetOutputState (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETOUTPUTSTATE")
  (make-package "MM11_MSGS::SETOUTPUTSTATE"))
(unless (find-package "MM11_MSGS::SETOUTPUTSTATEREQUEST")
  (make-package "MM11_MSGS::SETOUTPUTSTATEREQUEST"))
(unless (find-package "MM11_MSGS::SETOUTPUTSTATERESPONSE")
  (make-package "MM11_MSGS::SETOUTPUTSTATERESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetOutputStateRequest
  :super ros::object
  :slots (_output _state ))

(defmethod mm11_msgs::SetOutputStateRequest
  (:init
   (&key
    ((:output __output) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _output (round __output))
   (setq _state (round __state))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _output
    1
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _output
       (write-byte _output s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _output
     (setq _output (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetOutputStateResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetOutputStateResponse
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

(defclass mm11_msgs::SetOutputState
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetOutputState :md5sum-) "db96c205088a50d0cf4188b4e981907d")
(setf (get mm11_msgs::SetOutputState :datatype-) "mm11_msgs/SetOutputState")
(setf (get mm11_msgs::SetOutputState :request) mm11_msgs::SetOutputStateRequest)
(setf (get mm11_msgs::SetOutputState :response) mm11_msgs::SetOutputStateResponse)

(defmethod mm11_msgs::SetOutputStateRequest
  (:response () (instance mm11_msgs::SetOutputStateResponse :init)))

(setf (get mm11_msgs::SetOutputStateRequest :md5sum-) "db96c205088a50d0cf4188b4e981907d")
(setf (get mm11_msgs::SetOutputStateRequest :datatype-) "mm11_msgs/SetOutputStateRequest")
(setf (get mm11_msgs::SetOutputStateRequest :definition-)
      "# Set an Output state

uint8 output

uint8 state

---
")

(setf (get mm11_msgs::SetOutputStateResponse :md5sum-) "db96c205088a50d0cf4188b4e981907d")
(setf (get mm11_msgs::SetOutputStateResponse :datatype-) "mm11_msgs/SetOutputStateResponse")
(setf (get mm11_msgs::SetOutputStateResponse :definition-)
      "# Set an Output state

uint8 output

uint8 state

---
")



(provide :mm11_msgs/SetOutputState "db96c205088a50d0cf4188b4e981907d")


