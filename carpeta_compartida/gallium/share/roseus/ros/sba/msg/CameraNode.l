;; Auto-generated. Do not edit!


(when (boundp 'sba::CameraNode)
  (if (not (find-package "SBA"))
    (make-package "SBA"))
  (shadow 'CameraNode (find-package "SBA")))
(unless (find-package "SBA::CAMERANODE")
  (make-package "SBA::CAMERANODE"))

(in-package "ROS")
;;//! \htmlinclude CameraNode.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass sba::CameraNode
  :super ros::object
  :slots (_index _transform _fx _fy _cx _cy _baseline _fixed ))

(defmethod sba::CameraNode
  (:init
   (&key
    ((:index __index) 0)
    ((:transform __transform) (instance geometry_msgs::Transform :init))
    ((:fx __fx) 0.0)
    ((:fy __fy) 0.0)
    ((:cx __cx) 0.0)
    ((:cy __cy) 0.0)
    ((:baseline __baseline) 0.0)
    ((:fixed __fixed) nil)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _transform __transform)
   (setq _fx (float __fx))
   (setq _fy (float __fy))
   (setq _cx (float __cx))
   (setq _cy (float __cy))
   (setq _baseline (float __baseline))
   (setq _fixed __fixed)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:transform
   (&rest __transform)
   (if (keywordp (car __transform))
       (send* _transform __transform)
     (progn
       (if __transform (setq _transform (car __transform)))
       _transform)))
  (:fx
   (&optional __fx)
   (if __fx (setq _fx __fx)) _fx)
  (:fy
   (&optional __fy)
   (if __fy (setq _fy __fy)) _fy)
  (:cx
   (&optional __cx)
   (if __cx (setq _cx __cx)) _cx)
  (:cy
   (&optional __cy)
   (if __cy (setq _cy __cy)) _cy)
  (:baseline
   (&optional __baseline)
   (if __baseline (setq _baseline __baseline)) _baseline)
  (:fixed
   (&optional (__fixed :null))
   (if (not (eq __fixed :null)) (setq _fixed __fixed)) _fixed)
  (:serialization-length
   ()
   (+
    ;; uint32 _index
    4
    ;; geometry_msgs/Transform _transform
    (send _transform :serialization-length)
    ;; float64 _fx
    8
    ;; float64 _fy
    8
    ;; float64 _cx
    8
    ;; float64 _cy
    8
    ;; float64 _baseline
    8
    ;; bool _fixed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _index
       (write-long _index s)
     ;; geometry_msgs/Transform _transform
       (send _transform :serialize s)
     ;; float64 _fx
       (sys::poke _fx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _fy
       (sys::poke _fy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cx
       (sys::poke _cx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cy
       (sys::poke _cy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _baseline
       (sys::poke _baseline (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _fixed
       (if _fixed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Transform _transform
     (send _transform :deserialize buf ptr-) (incf ptr- (send _transform :serialization-length))
   ;; float64 _fx
     (setq _fx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _fy
     (setq _fy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cx
     (setq _cx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cy
     (setq _cy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _baseline
     (setq _baseline (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _fixed
     (setq _fixed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sba::CameraNode :md5sum-) "0605774cb8ba22085a77f85c7154baa7")
(setf (get sba::CameraNode :datatype-) "sba/CameraNode")
(setf (get sba::CameraNode :definition-)
      "# Node Parameters
uint32 index

# Contains a translation and rotation
geometry_msgs/Transform transform

# Camera parameters from the K matrix
float64 fx
float64 fy
float64 cx
float64 cy

# Only relevant for a stereo camera
float64 baseline

# Whether the camera is fixed in space: i.e., its position is known
bool fixed

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :sba/CameraNode "0605774cb8ba22085a77f85c7154baa7")


