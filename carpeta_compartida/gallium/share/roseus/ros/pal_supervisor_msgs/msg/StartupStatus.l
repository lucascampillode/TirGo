;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::StartupStatus)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'StartupStatus (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS")
  (make-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))

(in-package "ROS")
;;//! \htmlinclude StartupStatus.msg.html


(intern "*RUNNING*" (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(shadow '*RUNNING* (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(defconstant pal_supervisor_msgs::StartupStatus::*RUNNING* 0)
(intern "*WAITING*" (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(shadow '*WAITING* (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(defconstant pal_supervisor_msgs::StartupStatus::*WAITING* 1)
(intern "*TIMEOUT*" (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(shadow '*TIMEOUT* (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(defconstant pal_supervisor_msgs::StartupStatus::*TIMEOUT* 2)
(intern "*ABORTED*" (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(shadow '*ABORTED* (find-package "PAL_SUPERVISOR_MSGS::STARTUPSTATUS"))
(defconstant pal_supervisor_msgs::StartupStatus::*ABORTED* 3)

(defun pal_supervisor_msgs::StartupStatus-to-symbol (const)
  (cond
        ((= const 0) 'pal_supervisor_msgs::StartupStatus::*RUNNING*)
        ((= const 1) 'pal_supervisor_msgs::StartupStatus::*WAITING*)
        ((= const 2) 'pal_supervisor_msgs::StartupStatus::*TIMEOUT*)
        ((= const 3) 'pal_supervisor_msgs::StartupStatus::*ABORTED*)
        (t nil)))

(defclass pal_supervisor_msgs::StartupStatus
  :super ros::object
  :slots (_name _message _status ))

(defmethod pal_supervisor_msgs::StartupStatus
  (:init
   (&key
    ((:name __name) "")
    ((:message __message) "")
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _message (string __message))
   (setq _status (round __status))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _message
    4 (length _message)
    ;; byte _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; byte _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; byte _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;;
   self)
  )

(setf (get pal_supervisor_msgs::StartupStatus :md5sum-) "6c21656eb2da6dbc6c6eca6aff588168")
(setf (get pal_supervisor_msgs::StartupStatus :datatype-) "pal_supervisor_msgs/StartupStatus")
(setf (get pal_supervisor_msgs::StartupStatus :definition-)
      "string name # application name
string message # description of the status if required
# Possible status of startup process
byte RUNNING=0
byte WAITING=1
byte TIMEOUT=2
byte ABORTED=3 #Application not running
byte status # status of the startup process for the application. Enumerated above 


")



(provide :pal_supervisor_msgs/StartupStatus "6c21656eb2da6dbc6c6eca6aff588168")


