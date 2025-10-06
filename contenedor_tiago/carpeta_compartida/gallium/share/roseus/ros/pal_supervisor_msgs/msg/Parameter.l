;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::Parameter)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'Parameter (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::PARAMETER")
  (make-package "PAL_SUPERVISOR_MSGS::PARAMETER"))

(in-package "ROS")
;;//! \htmlinclude Parameter.msg.html


(defclass pal_supervisor_msgs::Parameter
  :super ros::object
  :slots (_name _value ))

(defmethod pal_supervisor_msgs::Parameter
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value (string __value))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pal_supervisor_msgs::Parameter :md5sum-) "bc6ccc4a57f61779c8eaae61e9f422e0")
(setf (get pal_supervisor_msgs::Parameter :datatype-) "pal_supervisor_msgs/Parameter")
(setf (get pal_supervisor_msgs::Parameter :definition-)
      "string name # parameter name
string value # value of the parameter, all non string types will be \"stringized\" if possible

")



(provide :pal_supervisor_msgs/Parameter "bc6ccc4a57f61779c8eaae61e9f422e0")


