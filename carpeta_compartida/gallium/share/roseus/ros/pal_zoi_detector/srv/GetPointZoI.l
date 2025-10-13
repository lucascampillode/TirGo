;; Auto-generated. Do not edit!


(when (boundp 'pal_zoi_detector::GetPointZoI)
  (if (not (find-package "PAL_ZOI_DETECTOR"))
    (make-package "PAL_ZOI_DETECTOR"))
  (shadow 'GetPointZoI (find-package "PAL_ZOI_DETECTOR")))
(unless (find-package "PAL_ZOI_DETECTOR::GETPOINTZOI")
  (make-package "PAL_ZOI_DETECTOR::GETPOINTZOI"))
(unless (find-package "PAL_ZOI_DETECTOR::GETPOINTZOIREQUEST")
  (make-package "PAL_ZOI_DETECTOR::GETPOINTZOIREQUEST"))
(unless (find-package "PAL_ZOI_DETECTOR::GETPOINTZOIRESPONSE")
  (make-package "PAL_ZOI_DETECTOR::GETPOINTZOIRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass pal_zoi_detector::GetPointZoIRequest
  :super ros::object
  :slots (_point ))

(defmethod pal_zoi_detector::GetPointZoIRequest
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _point __point)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(defclass pal_zoi_detector::GetPointZoIResponse
  :super ros::object
  :slots (_zois ))

(defmethod pal_zoi_detector::GetPointZoIResponse
  (:init
   (&key
    ((:zois __zois) (instance pal_zoi_detector::CurrentZoI :init))
    )
   (send-super :init)
   (setq _zois __zois)
   self)
  (:zois
   (&rest __zois)
   (if (keywordp (car __zois))
       (send* _zois __zois)
     (progn
       (if __zois (setq _zois (car __zois)))
       _zois)))
  (:serialization-length
   ()
   (+
    ;; pal_zoi_detector/CurrentZoI _zois
    (send _zois :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pal_zoi_detector/CurrentZoI _zois
       (send _zois :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pal_zoi_detector/CurrentZoI _zois
     (send _zois :deserialize buf ptr-) (incf ptr- (send _zois :serialization-length))
   ;;
   self)
  )

(defclass pal_zoi_detector::GetPointZoI
  :super ros::object
  :slots ())

(setf (get pal_zoi_detector::GetPointZoI :md5sum-) "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(setf (get pal_zoi_detector::GetPointZoI :datatype-) "pal_zoi_detector/GetPointZoI")
(setf (get pal_zoi_detector::GetPointZoI :request) pal_zoi_detector::GetPointZoIRequest)
(setf (get pal_zoi_detector::GetPointZoI :response) pal_zoi_detector::GetPointZoIResponse)

(defmethod pal_zoi_detector::GetPointZoIRequest
  (:response () (instance pal_zoi_detector::GetPointZoIResponse :init)))

(setf (get pal_zoi_detector::GetPointZoIRequest :md5sum-) "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(setf (get pal_zoi_detector::GetPointZoIRequest :datatype-) "pal_zoi_detector/GetPointZoIRequest")
(setf (get pal_zoi_detector::GetPointZoIRequest :definition-)
      "geometry_msgs/Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
CurrentZoI zois


================================================================================
MSG: pal_zoi_detector/CurrentZoI
string[] zois
")

(setf (get pal_zoi_detector::GetPointZoIResponse :md5sum-) "5b5a8a2cb9fa5d51a60a241ebea72cc6")
(setf (get pal_zoi_detector::GetPointZoIResponse :datatype-) "pal_zoi_detector/GetPointZoIResponse")
(setf (get pal_zoi_detector::GetPointZoIResponse :definition-)
      "geometry_msgs/Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
CurrentZoI zois


================================================================================
MSG: pal_zoi_detector/CurrentZoI
string[] zois
")



(provide :pal_zoi_detector/GetPointZoI "5b5a8a2cb9fa5d51a60a241ebea72cc6")


