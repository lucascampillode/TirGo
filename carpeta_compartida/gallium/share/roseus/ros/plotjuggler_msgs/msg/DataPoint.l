;; Auto-generated. Do not edit!


(when (boundp 'plotjuggler_msgs::DataPoint)
  (if (not (find-package "PLOTJUGGLER_MSGS"))
    (make-package "PLOTJUGGLER_MSGS"))
  (shadow 'DataPoint (find-package "PLOTJUGGLER_MSGS")))
(unless (find-package "PLOTJUGGLER_MSGS::DATAPOINT")
  (make-package "PLOTJUGGLER_MSGS::DATAPOINT"))

(in-package "ROS")
;;//! \htmlinclude DataPoint.msg.html


(defclass plotjuggler_msgs::DataPoint
  :super ros::object
  :slots (_name_index _stamp _value ))

(defmethod plotjuggler_msgs::DataPoint
  (:init
   (&key
    ((:name_index __name_index) 0)
    ((:stamp __stamp) 0.0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _name_index (round __name_index))
   (setq _stamp (float __stamp))
   (setq _value (float __value))
   self)
  (:name_index
   (&optional __name_index)
   (if __name_index (setq _name_index __name_index)) _name_index)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; uint16 _name_index
    2
    ;; float64 _stamp
    8
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _name_index
       (write-word _name_index s)
     ;; float64 _stamp
       (sys::poke _stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _name_index
     (setq _name_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _stamp
     (setq _stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get plotjuggler_msgs::DataPoint :md5sum-) "580ca7c40f92b9a6ab4b921c02ebcd28")
(setf (get plotjuggler_msgs::DataPoint :datatype-) "plotjuggler_msgs/DataPoint")
(setf (get plotjuggler_msgs::DataPoint :definition-)
      "uint16 name_index
float64 stamp
float64 value

")



(provide :plotjuggler_msgs/DataPoint "580ca7c40f92b9a6ab4b921c02ebcd28")


