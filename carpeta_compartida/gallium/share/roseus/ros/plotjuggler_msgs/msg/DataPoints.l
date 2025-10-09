;; Auto-generated. Do not edit!


(when (boundp 'plotjuggler_msgs::DataPoints)
  (if (not (find-package "PLOTJUGGLER_MSGS"))
    (make-package "PLOTJUGGLER_MSGS"))
  (shadow 'DataPoints (find-package "PLOTJUGGLER_MSGS")))
(unless (find-package "PLOTJUGGLER_MSGS::DATAPOINTS")
  (make-package "PLOTJUGGLER_MSGS::DATAPOINTS"))

(in-package "ROS")
;;//! \htmlinclude DataPoints.msg.html


(defclass plotjuggler_msgs::DataPoints
  :super ros::object
  :slots (_dictionary_uuid _samples ))

(defmethod plotjuggler_msgs::DataPoints
  (:init
   (&key
    ((:dictionary_uuid __dictionary_uuid) 0)
    ((:samples __samples) ())
    )
   (send-super :init)
   (setq _dictionary_uuid (round __dictionary_uuid))
   (setq _samples __samples)
   self)
  (:dictionary_uuid
   (&optional __dictionary_uuid)
   (if __dictionary_uuid (setq _dictionary_uuid __dictionary_uuid)) _dictionary_uuid)
  (:samples
   (&rest __samples)
   (if (keywordp (car __samples))
       (send* _samples __samples)
     (progn
       (if __samples (setq _samples (car __samples)))
       _samples)))
  (:serialization-length
   ()
   (+
    ;; uint32 _dictionary_uuid
    4
    ;; plotjuggler_msgs/DataPoint[] _samples
    (apply #'+ (send-all _samples :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _dictionary_uuid
       (write-long _dictionary_uuid s)
     ;; plotjuggler_msgs/DataPoint[] _samples
     (write-long (length _samples) s)
     (dolist (elem _samples)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _dictionary_uuid
     (setq _dictionary_uuid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; plotjuggler_msgs/DataPoint[] _samples
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _samples (let (r) (dotimes (i n) (push (instance plotjuggler_msgs::DataPoint :init) r)) r))
     (dolist (elem- _samples)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get plotjuggler_msgs::DataPoints :md5sum-) "14e65e7956023a9a11291bc53d5d695a")
(setf (get plotjuggler_msgs::DataPoints :datatype-) "plotjuggler_msgs/DataPoints")
(setf (get plotjuggler_msgs::DataPoints :definition-)
      "uint32 dictionary_uuid
plotjuggler_msgs/DataPoint[] samples

================================================================================
MSG: plotjuggler_msgs/DataPoint
uint16 name_index
float64 stamp
float64 value

")



(provide :plotjuggler_msgs/DataPoints "14e65e7956023a9a11291bc53d5d695a")


