;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::StartupAction)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'StartupAction (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::STARTUPACTION")
  (make-package "PAL_SUPERVISOR_MSGS::STARTUPACTION"))

(in-package "ROS")
;;//! \htmlinclude StartupAction.msg.html


(intern "*START*" (find-package "PAL_SUPERVISOR_MSGS::STARTUPACTION"))
(shadow '*START* (find-package "PAL_SUPERVISOR_MSGS::STARTUPACTION"))
(defconstant pal_supervisor_msgs::StartupAction::*START* 0)
(intern "*ABORT*" (find-package "PAL_SUPERVISOR_MSGS::STARTUPACTION"))
(shadow '*ABORT* (find-package "PAL_SUPERVISOR_MSGS::STARTUPACTION"))
(defconstant pal_supervisor_msgs::StartupAction::*ABORT* 1)

(defun pal_supervisor_msgs::StartupAction-to-symbol (const)
  (cond
        ((= const 0) 'pal_supervisor_msgs::StartupAction::*START*)
        ((= const 1) 'pal_supervisor_msgs::StartupAction::*ABORT*)
        (t nil)))

(defclass pal_supervisor_msgs::StartupAction
  :super ros::object
  :slots (_name _action ))

(defmethod pal_supervisor_msgs::StartupAction
  (:init
   (&key
    ((:name __name) "")
    ((:action __action) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _action (round __action))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; byte _action
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; byte _action
       (write-byte _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; byte _action
     (setq _action (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _action 127) (setq _action (- _action 256)))
   ;;
   self)
  )

(setf (get pal_supervisor_msgs::StartupAction :md5sum-) "c3b072b09f12c731c73300aff3335bc4")
(setf (get pal_supervisor_msgs::StartupAction :datatype-) "pal_supervisor_msgs/StartupAction")
(setf (get pal_supervisor_msgs::StartupAction :definition-)
      "string name # application name
# Possible status of startup process
byte START=0
byte ABORT=1
byte action # action to be done by startup process for the application. Enumerated above 

")



(provide :pal_supervisor_msgs/StartupAction "c3b072b09f12c731c73300aff3335bc4")


