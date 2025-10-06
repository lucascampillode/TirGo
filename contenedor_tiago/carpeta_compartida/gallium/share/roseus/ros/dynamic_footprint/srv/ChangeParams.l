;; Auto-generated. Do not edit!


(when (boundp 'dynamic_footprint::ChangeParams)
  (if (not (find-package "DYNAMIC_FOOTPRINT"))
    (make-package "DYNAMIC_FOOTPRINT"))
  (shadow 'ChangeParams (find-package "DYNAMIC_FOOTPRINT")))
(unless (find-package "DYNAMIC_FOOTPRINT::CHANGEPARAMS")
  (make-package "DYNAMIC_FOOTPRINT::CHANGEPARAMS"))
(unless (find-package "DYNAMIC_FOOTPRINT::CHANGEPARAMSREQUEST")
  (make-package "DYNAMIC_FOOTPRINT::CHANGEPARAMSREQUEST"))
(unless (find-package "DYNAMIC_FOOTPRINT::CHANGEPARAMSRESPONSE")
  (make-package "DYNAMIC_FOOTPRINT::CHANGEPARAMSRESPONSE"))

(in-package "ROS")





(defclass dynamic_footprint::ChangeParamsRequest
  :super ros::object
  :slots (_rate _padding ))

(defmethod dynamic_footprint::ChangeParamsRequest
  (:init
   (&key
    ((:rate __rate) 0)
    ((:padding __padding) 0.0)
    )
   (send-super :init)
   (setq _rate (round __rate))
   (setq _padding (float __padding))
   self)
  (:rate
   (&optional __rate)
   (if __rate (setq _rate __rate)) _rate)
  (:padding
   (&optional __padding)
   (if __padding (setq _padding __padding)) _padding)
  (:serialization-length
   ()
   (+
    ;; int32 _rate
    4
    ;; float32 _padding
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _rate
       (write-long _rate s)
     ;; float32 _padding
       (sys::poke _padding (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _rate
     (setq _rate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _padding
     (setq _padding (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dynamic_footprint::ChangeParamsResponse
  :super ros::object
  :slots ())

(defmethod dynamic_footprint::ChangeParamsResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dynamic_footprint::ChangeParams
  :super ros::object
  :slots ())

(setf (get dynamic_footprint::ChangeParams :md5sum-) "54da70ff99cf30bf896a186492a150b3")
(setf (get dynamic_footprint::ChangeParams :datatype-) "dynamic_footprint/ChangeParams")
(setf (get dynamic_footprint::ChangeParams :request) dynamic_footprint::ChangeParamsRequest)
(setf (get dynamic_footprint::ChangeParams :response) dynamic_footprint::ChangeParamsResponse)

(defmethod dynamic_footprint::ChangeParamsRequest
  (:response () (instance dynamic_footprint::ChangeParamsResponse :init)))

(setf (get dynamic_footprint::ChangeParamsRequest :md5sum-) "54da70ff99cf30bf896a186492a150b3")
(setf (get dynamic_footprint::ChangeParamsRequest :datatype-) "dynamic_footprint/ChangeParamsRequest")
(setf (get dynamic_footprint::ChangeParamsRequest :definition-)
      "int32 rate
float32 padding
---

")

(setf (get dynamic_footprint::ChangeParamsResponse :md5sum-) "54da70ff99cf30bf896a186492a150b3")
(setf (get dynamic_footprint::ChangeParamsResponse :datatype-) "dynamic_footprint/ChangeParamsResponse")
(setf (get dynamic_footprint::ChangeParamsResponse :definition-)
      "int32 rate
float32 padding
---

")



(provide :dynamic_footprint/ChangeParams "54da70ff99cf30bf896a186492a150b3")


