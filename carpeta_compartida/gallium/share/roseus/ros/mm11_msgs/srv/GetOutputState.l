;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::GetOutputState)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'GetOutputState (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::GETOUTPUTSTATE")
  (make-package "MM11_MSGS::GETOUTPUTSTATE"))
(unless (find-package "MM11_MSGS::GETOUTPUTSTATEREQUEST")
  (make-package "MM11_MSGS::GETOUTPUTSTATEREQUEST"))
(unless (find-package "MM11_MSGS::GETOUTPUTSTATERESPONSE")
  (make-package "MM11_MSGS::GETOUTPUTSTATERESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::GetOutputStateRequest
  :super ros::object
  :slots (_output ))

(defmethod mm11_msgs::GetOutputStateRequest
  (:init
   (&key
    ((:output __output) 0)
    )
   (send-super :init)
   (setq _output (round __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; uint8 _output
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _output
       (write-byte _output s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _output
     (setq _output (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::GetOutputStateResponse
  :super ros::object
  :slots (_state ))

(defmethod mm11_msgs::GetOutputStateResponse
  (:init
   (&key
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::GetOutputState
  :super ros::object
  :slots ())

(setf (get mm11_msgs::GetOutputState :md5sum-) "30376e3f1875e49fda4a395a76150de1")
(setf (get mm11_msgs::GetOutputState :datatype-) "mm11_msgs/GetOutputState")
(setf (get mm11_msgs::GetOutputState :request) mm11_msgs::GetOutputStateRequest)
(setf (get mm11_msgs::GetOutputState :response) mm11_msgs::GetOutputStateResponse)

(defmethod mm11_msgs::GetOutputStateRequest
  (:response () (instance mm11_msgs::GetOutputStateResponse :init)))

(setf (get mm11_msgs::GetOutputStateRequest :md5sum-) "30376e3f1875e49fda4a395a76150de1")
(setf (get mm11_msgs::GetOutputStateRequest :datatype-) "mm11_msgs/GetOutputStateRequest")
(setf (get mm11_msgs::GetOutputStateRequest :definition-)
      "# Get an Output state

uint8 output

---

uint8 state

")

(setf (get mm11_msgs::GetOutputStateResponse :md5sum-) "30376e3f1875e49fda4a395a76150de1")
(setf (get mm11_msgs::GetOutputStateResponse :datatype-) "mm11_msgs/GetOutputStateResponse")
(setf (get mm11_msgs::GetOutputStateResponse :definition-)
      "# Get an Output state

uint8 output

---

uint8 state

")



(provide :mm11_msgs/GetOutputState "30376e3f1875e49fda4a395a76150de1")


