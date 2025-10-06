;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::Gpio)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'Gpio (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::GPIO")
  (make-package "MM11_MSGS::GPIO"))

(in-package "ROS")
;;//! \htmlinclude Gpio.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mm11_msgs::Gpio
  :super ros::object
  :slots (_header _input_state ))

(defmethod mm11_msgs::Gpio
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:input_state __input_state) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _input_state __input_state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:input_state
   (&optional __input_state)
   (if __input_state (setq _input_state __input_state)) _input_state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8[] _input_state
    (* 1    (length _input_state)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8[] _input_state
     (write-long (length _input_state) s)
     (princ _input_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8[] _input_state
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _input_state (make-array n :element-type :char))
     (replace _input_state buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get mm11_msgs::Gpio :md5sum-) "1d85c88c9ce946bd64d8803ef84f9759")
(setf (get mm11_msgs::Gpio :datatype-) "mm11_msgs/Gpio")
(setf (get mm11_msgs::Gpio :definition-)
      "# ROS header
Header header

uint8[] input_state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :mm11_msgs/Gpio "1d85c88c9ce946bd64d8803ef84f9759")


