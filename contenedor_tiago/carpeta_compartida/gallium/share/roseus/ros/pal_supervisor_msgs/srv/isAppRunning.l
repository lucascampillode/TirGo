;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::isAppRunning)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'isAppRunning (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNING")
  (make-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNING"))
(unless (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::isAppRunningRequest
  :super ros::object
  :slots (_appName ))

(defmethod pal_supervisor_msgs::isAppRunningRequest
  (:init
   (&key
    ((:appName __appName) "")
    )
   (send-super :init)
   (setq _appName (string __appName))
   self)
  (:appName
   (&optional __appName)
   (if __appName (setq _appName __appName)) _appName)
  (:serialization-length
   ()
   (+
    ;; string _appName
    4 (length _appName)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _appName
       (write-long (length _appName) s) (princ _appName s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _appName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _appName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(intern "*OK*" (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(shadow '*OK* (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(defconstant pal_supervisor_msgs::isAppRunningResponse::*OK* 0)
(intern "*WARN*" (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(shadow '*WARN* (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(defconstant pal_supervisor_msgs::isAppRunningResponse::*WARN* 1)
(intern "*ERROR*" (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(shadow '*ERROR* (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(defconstant pal_supervisor_msgs::isAppRunningResponse::*ERROR* 2)
(intern "*STALE*" (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(shadow '*STALE* (find-package "PAL_SUPERVISOR_MSGS::ISAPPRUNNINGRESPONSE"))
(defconstant pal_supervisor_msgs::isAppRunningResponse::*STALE* 3)

(defun pal_supervisor_msgs::isAppRunningResponse-to-symbol (const)
  (cond
        ((= const 0) 'pal_supervisor_msgs::isAppRunningResponse::*OK*)
        ((= const 1) 'pal_supervisor_msgs::isAppRunningResponse::*WARN*)
        ((= const 2) 'pal_supervisor_msgs::isAppRunningResponse::*ERROR*)
        ((= const 3) 'pal_supervisor_msgs::isAppRunningResponse::*STALE*)
        (t nil)))

(defclass pal_supervisor_msgs::isAppRunningResponse
  :super ros::object
  :slots (_level ))

(defmethod pal_supervisor_msgs::isAppRunningResponse
  (:init
   (&key
    ((:level __level) 0)
    )
   (send-super :init)
   (setq _level (round __level))
   self)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:serialization-length
   ()
   (+
    ;; byte _level
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; byte _level
       (write-byte _level s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; byte _level
     (setq _level (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _level 127) (setq _level (- _level 256)))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::isAppRunning
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::isAppRunning :md5sum-) "c82b84a4b8d7b2c6e1185b6a3564ad39")
(setf (get pal_supervisor_msgs::isAppRunning :datatype-) "pal_supervisor_msgs/isAppRunning")
(setf (get pal_supervisor_msgs::isAppRunning :request) pal_supervisor_msgs::isAppRunningRequest)
(setf (get pal_supervisor_msgs::isAppRunning :response) pal_supervisor_msgs::isAppRunningResponse)

(defmethod pal_supervisor_msgs::isAppRunningRequest
  (:response () (instance pal_supervisor_msgs::isAppRunningResponse :init)))

(setf (get pal_supervisor_msgs::isAppRunningRequest :md5sum-) "c82b84a4b8d7b2c6e1185b6a3564ad39")
(setf (get pal_supervisor_msgs::isAppRunningRequest :datatype-) "pal_supervisor_msgs/isAppRunningRequest")
(setf (get pal_supervisor_msgs::isAppRunningRequest :definition-)
      "string appName #Name of the application 
---
# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3 #Application not running
byte level # level of operation enumerated above 

")

(setf (get pal_supervisor_msgs::isAppRunningResponse :md5sum-) "c82b84a4b8d7b2c6e1185b6a3564ad39")
(setf (get pal_supervisor_msgs::isAppRunningResponse :datatype-) "pal_supervisor_msgs/isAppRunningResponse")
(setf (get pal_supervisor_msgs::isAppRunningResponse :definition-)
      "string appName #Name of the application 
---
# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3 #Application not running
byte level # level of operation enumerated above 

")



(provide :pal_supervisor_msgs/isAppRunning "c82b84a4b8d7b2c6e1185b6a3564ad39")


