;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetRestart)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetRestart (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETRESTART")
  (make-package "MM11_MSGS::SETRESTART"))
(unless (find-package "MM11_MSGS::SETRESTARTREQUEST")
  (make-package "MM11_MSGS::SETRESTARTREQUEST"))
(unless (find-package "MM11_MSGS::SETRESTARTRESPONSE")
  (make-package "MM11_MSGS::SETRESTARTRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetRestartRequest
  :super ros::object
  :slots (_time_down ))

(defmethod mm11_msgs::SetRestartRequest
  (:init
   (&key
    ((:time_down __time_down) 0)
    )
   (send-super :init)
   (setq _time_down (round __time_down))
   self)
  (:time_down
   (&optional __time_down)
   (if __time_down (setq _time_down __time_down)) _time_down)
  (:serialization-length
   ()
   (+
    ;; uint8 _time_down
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _time_down
       (write-byte _time_down s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _time_down
     (setq _time_down (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetRestartResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetRestartResponse
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

(defclass mm11_msgs::SetRestart
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetRestart :md5sum-) "312e4924f616d4157dc3bb5aee21a3bb")
(setf (get mm11_msgs::SetRestart :datatype-) "mm11_msgs/SetRestart")
(setf (get mm11_msgs::SetRestart :request) mm11_msgs::SetRestartRequest)
(setf (get mm11_msgs::SetRestart :response) mm11_msgs::SetRestartResponse)

(defmethod mm11_msgs::SetRestartRequest
  (:response () (instance mm11_msgs::SetRestartResponse :init)))

(setf (get mm11_msgs::SetRestartRequest :md5sum-) "312e4924f616d4157dc3bb5aee21a3bb")
(setf (get mm11_msgs::SetRestartRequest :datatype-) "mm11_msgs/SetRestartRequest")
(setf (get mm11_msgs::SetRestartRequest :definition-)
      "# Force a hardware restart of the robot. Keeps the robot down for the indicated seconds.

uint8 time_down

---
")

(setf (get mm11_msgs::SetRestartResponse :md5sum-) "312e4924f616d4157dc3bb5aee21a3bb")
(setf (get mm11_msgs::SetRestartResponse :datatype-) "mm11_msgs/SetRestartResponse")
(setf (get mm11_msgs::SetRestartResponse :definition-)
      "# Force a hardware restart of the robot. Keeps the robot down for the indicated seconds.

uint8 time_down

---
")



(provide :mm11_msgs/SetRestart "312e4924f616d4157dc3bb5aee21a3bb")


