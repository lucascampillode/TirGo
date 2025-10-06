;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::Power)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'Power (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::POWER")
  (make-package "MM11_MSGS::POWER"))

(in-package "ROS")
;;//! \htmlinclude Power.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mm11_msgs::Power
  :super ros::object
  :slots (_header _input _charger _dock _pc _charge _is_connected _is_emergency ))

(defmethod mm11_msgs::Power
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:input __input) 0.0)
    ((:charger __charger) 0.0)
    ((:dock __dock) 0.0)
    ((:pc __pc) 0.0)
    ((:charge __charge) 0)
    ((:is_connected __is_connected) nil)
    ((:is_emergency __is_emergency) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _input (float __input))
   (setq _charger (float __charger))
   (setq _dock (float __dock))
   (setq _pc (float __pc))
   (setq _charge (round __charge))
   (setq _is_connected __is_connected)
   (setq _is_emergency __is_emergency)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:charger
   (&optional __charger)
   (if __charger (setq _charger __charger)) _charger)
  (:dock
   (&optional __dock)
   (if __dock (setq _dock __dock)) _dock)
  (:pc
   (&optional __pc)
   (if __pc (setq _pc __pc)) _pc)
  (:charge
   (&optional __charge)
   (if __charge (setq _charge __charge)) _charge)
  (:is_connected
   (&optional (__is_connected :null))
   (if (not (eq __is_connected :null)) (setq _is_connected __is_connected)) _is_connected)
  (:is_emergency
   (&optional (__is_emergency :null))
   (if (not (eq __is_emergency :null)) (setq _is_emergency __is_emergency)) _is_emergency)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _input
    8
    ;; float64 _charger
    8
    ;; float64 _dock
    8
    ;; float64 _pc
    8
    ;; uint8 _charge
    1
    ;; bool _is_connected
    1
    ;; bool _is_emergency
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _input
       (sys::poke _input (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _charger
       (sys::poke _charger (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dock
       (sys::poke _dock (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pc
       (sys::poke _pc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _charge
       (write-byte _charge s)
     ;; bool _is_connected
       (if _is_connected (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_emergency
       (if _is_emergency (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _input
     (setq _input (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _charger
     (setq _charger (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dock
     (setq _dock (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pc
     (setq _pc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _charge
     (setq _charge (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _is_connected
     (setq _is_connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_emergency
     (setq _is_emergency (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mm11_msgs::Power :md5sum-) "fc034c0a3de8d87cb18c2d9ce6556e15")
(setf (get mm11_msgs::Power :datatype-) "mm11_msgs/Power")
(setf (get mm11_msgs::Power :definition-)
      "# ROS header
Header header

# input voltage
float64 input

# charger voltage
float64 charger

# dock voltage
float64 dock

# PC voltage
float64 pc

# charge
uint8 charge

# connection to charger
bool is_connected

# emergency status
bool is_emergency



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



(provide :mm11_msgs/Power "fc034c0a3de8d87cb18c2d9ce6556e15")


