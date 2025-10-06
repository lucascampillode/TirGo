;; Auto-generated. Do not edit!


(when (boundp 'pal_pcl::ComputeTransform)
  (if (not (find-package "PAL_PCL"))
    (make-package "PAL_PCL"))
  (shadow 'ComputeTransform (find-package "PAL_PCL")))
(unless (find-package "PAL_PCL::COMPUTETRANSFORM")
  (make-package "PAL_PCL::COMPUTETRANSFORM"))
(unless (find-package "PAL_PCL::COMPUTETRANSFORMREQUEST")
  (make-package "PAL_PCL::COMPUTETRANSFORMREQUEST"))
(unless (find-package "PAL_PCL::COMPUTETRANSFORMRESPONSE")
  (make-package "PAL_PCL::COMPUTETRANSFORMRESPONSE"))

(in-package "ROS")





(defclass pal_pcl::ComputeTransformRequest
  :super ros::object
  :slots (_camera_topic _optical_frame _camera_frame _max_dist ))

(defmethod pal_pcl::ComputeTransformRequest
  (:init
   (&key
    ((:camera_topic __camera_topic) "")
    ((:optical_frame __optical_frame) "")
    ((:camera_frame __camera_frame) "")
    ((:max_dist __max_dist) 0.0)
    )
   (send-super :init)
   (setq _camera_topic (string __camera_topic))
   (setq _optical_frame (string __optical_frame))
   (setq _camera_frame (string __camera_frame))
   (setq _max_dist (float __max_dist))
   self)
  (:camera_topic
   (&optional __camera_topic)
   (if __camera_topic (setq _camera_topic __camera_topic)) _camera_topic)
  (:optical_frame
   (&optional __optical_frame)
   (if __optical_frame (setq _optical_frame __optical_frame)) _optical_frame)
  (:camera_frame
   (&optional __camera_frame)
   (if __camera_frame (setq _camera_frame __camera_frame)) _camera_frame)
  (:max_dist
   (&optional __max_dist)
   (if __max_dist (setq _max_dist __max_dist)) _max_dist)
  (:serialization-length
   ()
   (+
    ;; string _camera_topic
    4 (length _camera_topic)
    ;; string _optical_frame
    4 (length _optical_frame)
    ;; string _camera_frame
    4 (length _camera_frame)
    ;; float64 _max_dist
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _camera_topic
       (write-long (length _camera_topic) s) (princ _camera_topic s)
     ;; string _optical_frame
       (write-long (length _optical_frame) s) (princ _optical_frame s)
     ;; string _camera_frame
       (write-long (length _camera_frame) s) (princ _camera_frame s)
     ;; float64 _max_dist
       (sys::poke _max_dist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _camera_topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _optical_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _optical_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _camera_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _max_dist
     (setq _max_dist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass pal_pcl::ComputeTransformResponse
  :super ros::object
  :slots (_height _height_error _roll _roll_error _pitch _pitch_error _error ))

(defmethod pal_pcl::ComputeTransformResponse
  (:init
   (&key
    ((:height __height) 0.0)
    ((:height_error __height_error) 0.0)
    ((:roll __roll) 0.0)
    ((:roll_error __roll_error) 0.0)
    ((:pitch __pitch) 0.0)
    ((:pitch_error __pitch_error) 0.0)
    ((:error __error) nil)
    )
   (send-super :init)
   (setq _height (float __height))
   (setq _height_error (float __height_error))
   (setq _roll (float __roll))
   (setq _roll_error (float __roll_error))
   (setq _pitch (float __pitch))
   (setq _pitch_error (float __pitch_error))
   (setq _error __error)
   self)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:height_error
   (&optional __height_error)
   (if __height_error (setq _height_error __height_error)) _height_error)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:roll_error
   (&optional __roll_error)
   (if __roll_error (setq _roll_error __roll_error)) _roll_error)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:pitch_error
   (&optional __pitch_error)
   (if __pitch_error (setq _pitch_error __pitch_error)) _pitch_error)
  (:error
   (&optional (__error :null))
   (if (not (eq __error :null)) (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; float64 _height
    8
    ;; float64 _height_error
    8
    ;; float64 _roll
    8
    ;; float64 _roll_error
    8
    ;; float64 _pitch
    8
    ;; float64 _pitch_error
    8
    ;; bool _error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height_error
       (sys::poke _height_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll_error
       (sys::poke _roll_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch_error
       (sys::poke _pitch_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _error
       (if _error (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height_error
     (setq _height_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll_error
     (setq _roll_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch_error
     (setq _pitch_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _error
     (setq _error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pal_pcl::ComputeTransform
  :super ros::object
  :slots ())

(setf (get pal_pcl::ComputeTransform :md5sum-) "63c14dea37df6909b0547d1d22800cc9")
(setf (get pal_pcl::ComputeTransform :datatype-) "pal_pcl/ComputeTransform")
(setf (get pal_pcl::ComputeTransform :request) pal_pcl::ComputeTransformRequest)
(setf (get pal_pcl::ComputeTransform :response) pal_pcl::ComputeTransformResponse)

(defmethod pal_pcl::ComputeTransformRequest
  (:response () (instance pal_pcl::ComputeTransformResponse :init)))

(setf (get pal_pcl::ComputeTransformRequest :md5sum-) "63c14dea37df6909b0547d1d22800cc9")
(setf (get pal_pcl::ComputeTransformRequest :datatype-) "pal_pcl/ComputeTransformRequest")
(setf (get pal_pcl::ComputeTransformRequest :definition-)
      "string camera_topic
string optical_frame
string camera_frame
float64 max_dist
---
float64 height
float64 height_error
float64 roll
float64 roll_error
float64 pitch
float64 pitch_error
bool error

")

(setf (get pal_pcl::ComputeTransformResponse :md5sum-) "63c14dea37df6909b0547d1d22800cc9")
(setf (get pal_pcl::ComputeTransformResponse :datatype-) "pal_pcl/ComputeTransformResponse")
(setf (get pal_pcl::ComputeTransformResponse :definition-)
      "string camera_topic
string optical_frame
string camera_frame
float64 max_dist
---
float64 height
float64 height_error
float64 roll
float64 roll_error
float64 pitch
float64 pitch_error
bool error

")



(provide :pal_pcl/ComputeTransform "63c14dea37df6909b0547d1d22800cc9")


