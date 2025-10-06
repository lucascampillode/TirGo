;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::SetStripFlash)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'SetStripFlash (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::SETSTRIPFLASH")
  (make-package "MM11_MSGS::SETSTRIPFLASH"))
(unless (find-package "MM11_MSGS::SETSTRIPFLASHREQUEST")
  (make-package "MM11_MSGS::SETSTRIPFLASHREQUEST"))
(unless (find-package "MM11_MSGS::SETSTRIPFLASHRESPONSE")
  (make-package "MM11_MSGS::SETSTRIPFLASHRESPONSE"))

(in-package "ROS")





(defclass mm11_msgs::SetStripFlashRequest
  :super ros::object
  :slots (_port _time _period _r_1 _g_1 _b_1 _r_2 _g_2 _b_2 ))

(defmethod mm11_msgs::SetStripFlashRequest
  (:init
   (&key
    ((:port __port) 0)
    ((:time __time) 0)
    ((:period __period) 0)
    ((:r_1 __r_1) 0)
    ((:g_1 __g_1) 0)
    ((:b_1 __b_1) 0)
    ((:r_2 __r_2) 0)
    ((:g_2 __g_2) 0)
    ((:b_2 __b_2) 0)
    )
   (send-super :init)
   (setq _port (round __port))
   (setq _time (round __time))
   (setq _period (round __period))
   (setq _r_1 (round __r_1))
   (setq _g_1 (round __g_1))
   (setq _b_1 (round __b_1))
   (setq _r_2 (round __r_2))
   (setq _g_2 (round __g_2))
   (setq _b_2 (round __b_2))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:r_1
   (&optional __r_1)
   (if __r_1 (setq _r_1 __r_1)) _r_1)
  (:g_1
   (&optional __g_1)
   (if __g_1 (setq _g_1 __g_1)) _g_1)
  (:b_1
   (&optional __b_1)
   (if __b_1 (setq _b_1 __b_1)) _b_1)
  (:r_2
   (&optional __r_2)
   (if __r_2 (setq _r_2 __r_2)) _r_2)
  (:g_2
   (&optional __g_2)
   (if __g_2 (setq _g_2 __g_2)) _g_2)
  (:b_2
   (&optional __b_2)
   (if __b_2 (setq _b_2 __b_2)) _b_2)
  (:serialization-length
   ()
   (+
    ;; uint8 _port
    1
    ;; uint16 _time
    2
    ;; uint16 _period
    2
    ;; uint8 _r_1
    1
    ;; uint8 _g_1
    1
    ;; uint8 _b_1
    1
    ;; uint8 _r_2
    1
    ;; uint8 _g_2
    1
    ;; uint8 _b_2
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _port
       (write-byte _port s)
     ;; uint16 _time
       (write-word _time s)
     ;; uint16 _period
       (write-word _period s)
     ;; uint8 _r_1
       (write-byte _r_1 s)
     ;; uint8 _g_1
       (write-byte _g_1 s)
     ;; uint8 _b_1
       (write-byte _b_1 s)
     ;; uint8 _r_2
       (write-byte _r_2 s)
     ;; uint8 _g_2
       (write-byte _g_2 s)
     ;; uint8 _b_2
       (write-byte _b_2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _port
     (setq _port (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _time
     (setq _time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _period
     (setq _period (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _r_1
     (setq _r_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g_1
     (setq _g_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b_1
     (setq _b_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _r_2
     (setq _r_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g_2
     (setq _g_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b_2
     (setq _b_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mm11_msgs::SetStripFlashResponse
  :super ros::object
  :slots ())

(defmethod mm11_msgs::SetStripFlashResponse
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

(defclass mm11_msgs::SetStripFlash
  :super ros::object
  :slots ())

(setf (get mm11_msgs::SetStripFlash :md5sum-) "4d41df4b77ebb64e24b098b717616db6")
(setf (get mm11_msgs::SetStripFlash :datatype-) "mm11_msgs/SetStripFlash")
(setf (get mm11_msgs::SetStripFlash :request) mm11_msgs::SetStripFlashRequest)
(setf (get mm11_msgs::SetStripFlash :response) mm11_msgs::SetStripFlashResponse)

(defmethod mm11_msgs::SetStripFlashRequest
  (:response () (instance mm11_msgs::SetStripFlashResponse :init)))

(setf (get mm11_msgs::SetStripFlashRequest :md5sum-) "4d41df4b77ebb64e24b098b717616db6")
(setf (get mm11_msgs::SetStripFlashRequest :datatype-) "mm11_msgs/SetStripFlashRequest")
(setf (get mm11_msgs::SetStripFlashRequest :definition-)
      "# Flashing for a whole strip

uint8 port

uint16 time
uint16 period

uint8 r_1
uint8 g_1
uint8 b_1

uint8 r_2
uint8 g_2
uint8 b_2

---
")

(setf (get mm11_msgs::SetStripFlashResponse :md5sum-) "4d41df4b77ebb64e24b098b717616db6")
(setf (get mm11_msgs::SetStripFlashResponse :datatype-) "mm11_msgs/SetStripFlashResponse")
(setf (get mm11_msgs::SetStripFlashResponse :definition-)
      "# Flashing for a whole strip

uint8 port

uint16 time
uint16 period

uint8 r_1
uint8 g_1
uint8 b_1

uint8 r_2
uint8 g_2
uint8 b_2

---
")



(provide :mm11_msgs/SetStripFlash "4d41df4b77ebb64e24b098b717616db6")


