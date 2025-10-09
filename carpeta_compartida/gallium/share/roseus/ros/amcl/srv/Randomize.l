;; Auto-generated. Do not edit!


(when (boundp 'amcl::Randomize)
  (if (not (find-package "AMCL"))
    (make-package "AMCL"))
  (shadow 'Randomize (find-package "AMCL")))
(unless (find-package "AMCL::RANDOMIZE")
  (make-package "AMCL::RANDOMIZE"))
(unless (find-package "AMCL::RANDOMIZEREQUEST")
  (make-package "AMCL::RANDOMIZEREQUEST"))
(unless (find-package "AMCL::RANDOMIZERESPONSE")
  (make-package "AMCL::RANDOMIZERESPONSE"))

(in-package "ROS")





(defclass amcl::RandomizeRequest
  :super ros::object
  :slots (_radius ))

(defmethod amcl::RandomizeRequest
  (:init
   (&key
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _radius (float __radius))
   self)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; float32 _radius
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass amcl::RandomizeResponse
  :super ros::object
  :slots ())

(defmethod amcl::RandomizeResponse
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

(defclass amcl::Randomize
  :super ros::object
  :slots ())

(setf (get amcl::Randomize :md5sum-) "0b690044065f2210effb664cc7a28125")
(setf (get amcl::Randomize :datatype-) "amcl/Randomize")
(setf (get amcl::Randomize :request) amcl::RandomizeRequest)
(setf (get amcl::Randomize :response) amcl::RandomizeResponse)

(defmethod amcl::RandomizeRequest
  (:response () (instance amcl::RandomizeResponse :init)))

(setf (get amcl::RandomizeRequest :md5sum-) "0b690044065f2210effb664cc7a28125")
(setf (get amcl::RandomizeRequest :datatype-) "amcl/RandomizeRequest")
(setf (get amcl::RandomizeRequest :definition-)
      "float32 radius
---

")

(setf (get amcl::RandomizeResponse :md5sum-) "0b690044065f2210effb664cc7a28125")
(setf (get amcl::RandomizeResponse :datatype-) "amcl/RandomizeResponse")
(setf (get amcl::RandomizeResponse :definition-)
      "float32 radius
---

")



(provide :amcl/Randomize "0b690044065f2210effb664cc7a28125")


