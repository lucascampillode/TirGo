;; Auto-generated. Do not edit!


(when (boundp 'pal_startup_msgs::StartupStatus)
  (if (not (find-package "PAL_STARTUP_MSGS"))
    (make-package "PAL_STARTUP_MSGS"))
  (shadow 'StartupStatus (find-package "PAL_STARTUP_MSGS")))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))

(in-package "ROS")
;;//! \htmlinclude StartupStatus.msg.html


(intern "*MISSING*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*MISSING* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*MISSING* "Missing")
(intern "*RUNNING*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*RUNNING* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*RUNNING* "Running")
(intern "*PENDING*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*PENDING* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*PENDING* "Pending")
(intern "*CRASHED*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*CRASHED* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*CRASHED* "Crashed")
(intern "*STOPPED*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*STOPPED* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*STOPPED* "Stopped")
(intern "*ABORTED*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*ABORTED* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*ABORTED* "Aborted")
(intern "*TIMEOUT*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*TIMEOUT* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*TIMEOUT* "Timeout")
(intern "*MANUAL*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*MANUAL* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*MANUAL* "Manual")
(intern "*FINISHED*" (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(shadow '*FINISHED* (find-package "PAL_STARTUP_MSGS::STARTUPSTATUS"))
(defconstant pal_startup_msgs::StartupStatus::*FINISHED* "Finished")

(defun pal_startup_msgs::StartupStatus-to-symbol (const)
  (cond
        (t nil)))

(defclass pal_startup_msgs::StartupStatus
  :super ros::object
  :slots (_name _message _status _startup_id ))

(defmethod pal_startup_msgs::StartupStatus
  (:init
   (&key
    ((:name __name) "")
    ((:message __message) "")
    ((:status __status) "")
    ((:startup_id __startup_id) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _message (string __message))
   (setq _status (string __status))
   (setq _startup_id (string __startup_id))
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
  (:startup_id
   (&optional __startup_id)
   (if __startup_id (setq _startup_id __startup_id)) _startup_id)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _message
    4 (length _message)
    ;; string _status
    4 (length _status)
    ;; string _startup_id
    4 (length _startup_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; string _startup_id
       (write-long (length _startup_id) s) (princ _startup_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _startup_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _startup_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pal_startup_msgs::StartupStatus :md5sum-) "ca78e8aec80203b6c29ca3e480879775")
(setf (get pal_startup_msgs::StartupStatus :datatype-) "pal_startup_msgs/StartupStatus")
(setf (get pal_startup_msgs::StartupStatus :definition-)
      "string name # application name
string message # description of the status if required
# Possible status of startup process
string MISSING=Missing
string RUNNING=Running
string PENDING=Pending
string CRASHED=Crashed
string STOPPED=Stopped
string ABORTED=Aborted
string TIMEOUT=Timeout
string MANUAL=Manual
string FINISHED=Finished
string status # status of the startup process for the application. Enumerated above 
string startup_id # Id of the node starting the app (ie: control, multimedia)


")



(provide :pal_startup_msgs/StartupStatus "ca78e8aec80203b6c29ca3e480879775")


