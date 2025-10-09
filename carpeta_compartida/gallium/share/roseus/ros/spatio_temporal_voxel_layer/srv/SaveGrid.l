;; Auto-generated. Do not edit!


(when (boundp 'spatio_temporal_voxel_layer::SaveGrid)
  (if (not (find-package "SPATIO_TEMPORAL_VOXEL_LAYER"))
    (make-package "SPATIO_TEMPORAL_VOXEL_LAYER"))
  (shadow 'SaveGrid (find-package "SPATIO_TEMPORAL_VOXEL_LAYER")))
(unless (find-package "SPATIO_TEMPORAL_VOXEL_LAYER::SAVEGRID")
  (make-package "SPATIO_TEMPORAL_VOXEL_LAYER::SAVEGRID"))
(unless (find-package "SPATIO_TEMPORAL_VOXEL_LAYER::SAVEGRIDREQUEST")
  (make-package "SPATIO_TEMPORAL_VOXEL_LAYER::SAVEGRIDREQUEST"))
(unless (find-package "SPATIO_TEMPORAL_VOXEL_LAYER::SAVEGRIDRESPONSE")
  (make-package "SPATIO_TEMPORAL_VOXEL_LAYER::SAVEGRIDRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass spatio_temporal_voxel_layer::SaveGridRequest
  :super ros::object
  :slots (_file_name ))

(defmethod spatio_temporal_voxel_layer::SaveGridRequest
  (:init
   (&key
    ((:file_name __file_name) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _file_name __file_name)
   self)
  (:file_name
   (&rest __file_name)
   (if (keywordp (car __file_name))
       (send* _file_name __file_name)
     (progn
       (if __file_name (setq _file_name (car __file_name)))
       _file_name)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _file_name
    (send _file_name :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _file_name
       (send _file_name :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _file_name
     (send _file_name :deserialize buf ptr-) (incf ptr- (send _file_name :serialization-length))
   ;;
   self)
  )

(defclass spatio_temporal_voxel_layer::SaveGridResponse
  :super ros::object
  :slots (_map_size_bytes _status ))

(defmethod spatio_temporal_voxel_layer::SaveGridResponse
  (:init
   (&key
    ((:map_size_bytes __map_size_bytes) 0.0)
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _map_size_bytes (float __map_size_bytes))
   (setq _status __status)
   self)
  (:map_size_bytes
   (&optional __map_size_bytes)
   (if __map_size_bytes (setq _map_size_bytes __map_size_bytes)) _map_size_bytes)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; float64 _map_size_bytes
    8
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _map_size_bytes
       (sys::poke _map_size_bytes (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _map_size_bytes
     (setq _map_size_bytes (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass spatio_temporal_voxel_layer::SaveGrid
  :super ros::object
  :slots ())

(setf (get spatio_temporal_voxel_layer::SaveGrid :md5sum-) "793bb7917a99799f9a78324a148a1c17")
(setf (get spatio_temporal_voxel_layer::SaveGrid :datatype-) "spatio_temporal_voxel_layer/SaveGrid")
(setf (get spatio_temporal_voxel_layer::SaveGrid :request) spatio_temporal_voxel_layer::SaveGridRequest)
(setf (get spatio_temporal_voxel_layer::SaveGrid :response) spatio_temporal_voxel_layer::SaveGridResponse)

(defmethod spatio_temporal_voxel_layer::SaveGridRequest
  (:response () (instance spatio_temporal_voxel_layer::SaveGridResponse :init)))

(setf (get spatio_temporal_voxel_layer::SaveGridRequest :md5sum-) "793bb7917a99799f9a78324a148a1c17")
(setf (get spatio_temporal_voxel_layer::SaveGridRequest :datatype-) "spatio_temporal_voxel_layer/SaveGridRequest")
(setf (get spatio_temporal_voxel_layer::SaveGridRequest :definition-)
      "std_msgs/String file_name


================================================================================
MSG: std_msgs/String
string data
---

float64 map_size_bytes
bool status
")

(setf (get spatio_temporal_voxel_layer::SaveGridResponse :md5sum-) "793bb7917a99799f9a78324a148a1c17")
(setf (get spatio_temporal_voxel_layer::SaveGridResponse :datatype-) "spatio_temporal_voxel_layer/SaveGridResponse")
(setf (get spatio_temporal_voxel_layer::SaveGridResponse :definition-)
      "std_msgs/String file_name


================================================================================
MSG: std_msgs/String
string data
---

float64 map_size_bytes
bool status
")



(provide :spatio_temporal_voxel_layer/SaveGrid "793bb7917a99799f9a78324a148a1c17")


