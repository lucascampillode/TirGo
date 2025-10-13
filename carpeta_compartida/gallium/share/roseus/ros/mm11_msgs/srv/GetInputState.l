;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::GetInputState)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'GetInputState (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::GETINPUTSTATE")
  (make-package "MM11_MSGS::GETINPUTSTATE"))
(unless (find-package "MM11_MSGS::GETINPUTSTATEREQUEST")
  (make-package "MM11_MSGS::GETINPUTSTATEREQUEST"))
(unless (find-package "MM11_MSGS::GETINPUTSTATERESPONSE")
  (make-package "MM11_MSGS::GETINPUTSTATERESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::GetInputStateRequest
  :super ros::object
  :slots (_input ))

(defmethod mm11_msgs::GetInputStateRequest
  (:init
   (&key
    ((:input __input) 0)
    )
   (send-super :init)
   (setq _input (round __input))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; uint8 _input
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _input
       (write-byte _input s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _input
     (setq _input (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::GetInputStateResponse
  :super ros::object
  :slots (_state ))

(defmethod mm11_msgs::GetInputStateResponse
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

(defclass mm11_msgs::GetInputState
  :super ros::object
  :slots ())

(setf (get mm11_msgs::GetInputState :md5sum-) "11be78a327f2fee556995234c4d74c2d")
(setf (get mm11_msgs::GetInputState :datatype-) "mm11_msgs/GetInputState")
(setf (get mm11_msgs::GetInputState :request) mm11_msgs::GetInputStateRequest)
(setf (get mm11_msgs::GetInputState :response) mm11_msgs::GetInputStateResponse)

(defmethod mm11_msgs::GetInputStateRequest
  (:response () (instance mm11_msgs::GetInputStateResponse :init)))

(setf (get mm11_msgs::GetInputStateRequest :md5sum-) "11be78a327f2fee556995234c4d74c2d")
(setf (get mm11_msgs::GetInputStateRequest :datatype-) "mm11_msgs/GetInputStateRequest")
(setf (get mm11_msgs::GetInputStateRequest :definition-)
      "# Get an Input state

uint8 input

---

uint8 state

")

(setf (get mm11_msgs::GetInputStateResponse :md5sum-) "11be78a327f2fee556995234c4d74c2d")
(setf (get mm11_msgs::GetInputStateResponse :datatype-) "mm11_msgs/GetInputStateResponse")
(setf (get mm11_msgs::GetInputStateResponse :definition-)
      "# Get an Input state

uint8 input

---

uint8 state

")



(provide :mm11_msgs/GetInputState "11be78a327f2fee556995234c4d74c2d")


