;; Auto-generated. Do not edit!


(when (boundp 'plotjuggler_msgs::Dictionary)
  (if (not (find-package "PLOTJUGGLER_MSGS"))
    (make-package "PLOTJUGGLER_MSGS"))
  (shadow 'Dictionary (find-package "PLOTJUGGLER_MSGS")))
(unless (find-package "PLOTJUGGLER_MSGS::DICTIONARY")
  (make-package "PLOTJUGGLER_MSGS::DICTIONARY"))

(in-package "ROS")
;;//! \htmlinclude Dictionary.msg.html


(defclass plotjuggler_msgs::Dictionary
  :super ros::object
  :slots (_dictionary_uuid _names ))

(defmethod plotjuggler_msgs::Dictionary
  (:init
   (&key
    ((:dictionary_uuid __dictionary_uuid) 0)
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _dictionary_uuid (round __dictionary_uuid))
   (setq _names __names)
   self)
  (:dictionary_uuid
   (&optional __dictionary_uuid)
   (if __dictionary_uuid (setq _dictionary_uuid __dictionary_uuid)) _dictionary_uuid)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:serialization-length
   ()
   (+
    ;; uint32 _dictionary_uuid
    4
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _dictionary_uuid
       (write-long _dictionary_uuid s)
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _dictionary_uuid
     (setq _dictionary_uuid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get plotjuggler_msgs::Dictionary :md5sum-) "12d13553d8d6a9826829b71cac454ebe")
(setf (get plotjuggler_msgs::Dictionary :datatype-) "plotjuggler_msgs/Dictionary")
(setf (get plotjuggler_msgs::Dictionary :definition-)
      "uint32 dictionary_uuid
string[] names

")



(provide :plotjuggler_msgs/Dictionary "12d13553d8d6a9826829b71cac454ebe")


