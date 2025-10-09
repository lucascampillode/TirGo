;; Auto-generated. Do not edit!


(when (boundp 'sba::WorldPoint)
  (if (not (find-package "SBA"))
    (make-package "SBA"))
  (shadow 'WorldPoint (find-package "SBA")))
(unless (find-package "SBA::WORLDPOINT")
  (make-package "SBA::WORLDPOINT"))

(in-package "ROS")
;;//! \htmlinclude WorldPoint.msg.html


(defclass sba::WorldPoint
  :super ros::object
  :slots (_index _x _y _z _w ))

(defmethod sba::WorldPoint
  (:init
   (&key
    ((:index __index) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:w __w) 0.0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _w (float __w))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:serialization-length
   ()
   (+
    ;; uint32 _index
    4
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _w
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _index
       (write-long _index s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _w
       (sys::poke _w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _w
     (setq _w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get sba::WorldPoint :md5sum-) "39d176a0b9c218409063b0b8e9be53a7")
(setf (get sba::WorldPoint :datatype-) "sba/WorldPoint")
(setf (get sba::WorldPoint :definition-)
      "# World Point parameters

# Point index
uint32 index

# Coordinates in the world frame
float64 x
float64 y
float64 z
float64 w

")



(provide :sba/WorldPoint "39d176a0b9c218409063b0b8e9be53a7")


