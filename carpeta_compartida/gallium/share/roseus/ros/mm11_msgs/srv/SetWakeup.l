;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetWakeup)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetWakeup (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETWAKEUP")
  (make-package "MM11_MSGS::SETWAKEUP"))
(unless (find-package "MM11_MSGS::SETWAKEUPREQUEST")
  (make-package "MM11_MSGS::SETWAKEUPREQUEST"))
(unless (find-package "MM11_MSGS::SETWAKEUPRESPONSE")
  (make-package "MM11_MSGS::SETWAKEUPRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetWakeupRequest
  :super ros::object
  :slots (_time_min_down ))

(defmethod mm11_msgs::SetWakeupRequest
  (:init
   (&key
    ((:time_min_down __time_min_down) 0)
    )
   (send-super :init)
   (setq _time_min_down (round __time_min_down))
   self)
  (:time_min_down
   (&optional __time_min_down)
   (if __time_min_down (setq _time_min_down __time_min_down)) _time_min_down)
  (:serialization-length
   ()
   (+
    ;; uint16 _time_min_down
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _time_min_down
       (write-word _time_min_down s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _time_min_down
     (setq _time_min_down (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass mm11_msgs::SetWakeupResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetWakeupResponse
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

(defclass mm11_msgs::SetWakeup
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetWakeup :md5sum-) "2c161e9706149de203305763332e4462")
(setf (get mm11_msgs::SetWakeup :datatype-) "mm11_msgs/SetWakeup")
(setf (get mm11_msgs::SetWakeup :request) mm11_msgs::SetWakeupRequest)
(setf (get mm11_msgs::SetWakeup :response) mm11_msgs::SetWakeupResponse)

(defmethod mm11_msgs::SetWakeupRequest
  (:response () (instance mm11_msgs::SetWakeupResponse :init)))

(setf (get mm11_msgs::SetWakeupRequest :md5sum-) "2c161e9706149de203305763332e4462")
(setf (get mm11_msgs::SetWakeupRequest :datatype-) "mm11_msgs/SetWakeupRequest")
(setf (get mm11_msgs::SetWakeupRequest :definition-)
      "# Turns off the robot during indicated minutes.

uint16 time_min_down

---
")

(setf (get mm11_msgs::SetWakeupResponse :md5sum-) "2c161e9706149de203305763332e4462")
(setf (get mm11_msgs::SetWakeupResponse :datatype-) "mm11_msgs/SetWakeupResponse")
(setf (get mm11_msgs::SetWakeupResponse :definition-)
      "# Turns off the robot during indicated minutes.

uint16 time_min_down

---
")



(provide :mm11_msgs/SetWakeup "2c161e9706149de203305763332e4462")


