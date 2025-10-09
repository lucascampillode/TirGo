;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::DependencyStatus)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'DependencyStatus (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS")
  (make-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))

(in-package "ROS")
;;//! \htmlinclude DependencyStatus.msg.html


(intern "*OK*" (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(shadow '*OK* (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(defconstant pal_supervisor_msgs::DependencyStatus::*OK* 0)
(intern "*WARN*" (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(shadow '*WARN* (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(defconstant pal_supervisor_msgs::DependencyStatus::*WARN* 1)
(intern "*ERROR*" (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(shadow '*ERROR* (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(defconstant pal_supervisor_msgs::DependencyStatus::*ERROR* 2)
(intern "*STALE*" (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(shadow '*STALE* (find-package "PAL_SUPERVISOR_MSGS::DEPENDENCYSTATUS"))
(defconstant pal_supervisor_msgs::DependencyStatus::*STALE* 3)

(defun pal_supervisor_msgs::DependencyStatus-to-symbol (const)
  (cond
        ((= const 0) 'pal_supervisor_msgs::DependencyStatus::*OK*)
        ((= const 1) 'pal_supervisor_msgs::DependencyStatus::*WARN*)
        ((= const 2) 'pal_supervisor_msgs::DependencyStatus::*ERROR*)
        ((= const 3) 'pal_supervisor_msgs::DependencyStatus::*STALE*)
        (t nil)))

(defclass pal_supervisor_msgs::DependencyStatus
  :super ros::object
  :slots (_name _level ))

(defmethod pal_supervisor_msgs::DependencyStatus
  (:init
   (&key
    ((:name __name) "")
    ((:level __level) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _level (round __level))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; byte _level
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; byte _level
       (write-byte _level s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; byte _level
     (setq _level (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _level 127) (setq _level (- _level 256)))
   ;;
   self)
  )

(setf (get pal_supervisor_msgs::DependencyStatus :md5sum-) "d472e0bb1e75f743e86f41e4cedcf8bd")
(setf (get pal_supervisor_msgs::DependencyStatus :datatype-) "pal_supervisor_msgs/DependencyStatus")
(setf (get pal_supervisor_msgs::DependencyStatus :definition-)
      "string name # dependency name
# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3 #Application not running
byte level # level of operation enumerated above 

")



(provide :pal_supervisor_msgs/DependencyStatus "d472e0bb1e75f743e86f41e4cedcf8bd")


