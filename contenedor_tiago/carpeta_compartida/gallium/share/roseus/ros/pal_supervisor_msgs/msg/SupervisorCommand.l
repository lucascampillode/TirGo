;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::SupervisorCommand)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'SupervisorCommand (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::SUPERVISORCOMMAND")
  (make-package "PAL_SUPERVISOR_MSGS::SUPERVISORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude SupervisorCommand.msg.html


(defclass pal_supervisor_msgs::SupervisorCommand
  :super ros::object
  :slots (_name _params ))

(defmethod pal_supervisor_msgs::SupervisorCommand
  (:init
   (&key
    ((:name __name) "")
    ((:params __params) ())
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _params __params)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:params
   (&rest __params)
   (if (keywordp (car __params))
       (send* _params __params)
     (progn
       (if __params (setq _params (car __params)))
       _params)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; pal_supervisor_msgs/Parameter[] _params
    (apply #'+ (send-all _params :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; pal_supervisor_msgs/Parameter[] _params
     (write-long (length _params) s)
     (dolist (elem _params)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; pal_supervisor_msgs/Parameter[] _params
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _params (let (r) (dotimes (i n) (push (instance pal_supervisor_msgs::Parameter :init) r)) r))
     (dolist (elem- _params)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pal_supervisor_msgs::SupervisorCommand :md5sum-) "9389508dd5191c710316e34d805a4c6c")
(setf (get pal_supervisor_msgs::SupervisorCommand :datatype-) "pal_supervisor_msgs/SupervisorCommand")
(setf (get pal_supervisor_msgs::SupervisorCommand :definition-)
      "string name #Command name
pal_supervisor_msgs/Parameter[] params #Pairs of name and value, include all the parameters for the command

================================================================================
MSG: pal_supervisor_msgs/Parameter
string name # parameter name
string value # value of the parameter, all non string types will be \"stringized\" if possible

")



(provide :pal_supervisor_msgs/SupervisorCommand "9389508dd5191c710316e34d805a4c6c")


