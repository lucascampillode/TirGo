;; Auto-generated. Do not edit!


(when (boundp 'pal_pcl::save)
  (if (not (find-package "PAL_PCL"))
    (make-package "PAL_PCL"))
  (shadow 'save (find-package "PAL_PCL")))
(unless (find-package "PAL_PCL::SAVE")
  (make-package "PAL_PCL::SAVE"))
(unless (find-package "PAL_PCL::SAVEREQUEST")
  (make-package "PAL_PCL::SAVEREQUEST"))
(unless (find-package "PAL_PCL::SAVERESPONSE")
  (make-package "PAL_PCL::SAVERESPONSE"))

(in-package "ROS")





(defclass pal_pcl::saveRequest
  :super ros::object
  :slots ())

(defmethod pal_pcl::saveRequest
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

(defclass pal_pcl::saveResponse
  :super ros::object
  :slots ())

(defmethod pal_pcl::saveResponse
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

(defclass pal_pcl::save
  :super ros::object
  :slots ())

(setf (get pal_pcl::save :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get pal_pcl::save :datatype-) "pal_pcl/save")
(setf (get pal_pcl::save :request) pal_pcl::saveRequest)
(setf (get pal_pcl::save :response) pal_pcl::saveResponse)

(defmethod pal_pcl::saveRequest
  (:response () (instance pal_pcl::saveResponse :init)))

(setf (get pal_pcl::saveRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get pal_pcl::saveRequest :datatype-) "pal_pcl/saveRequest")
(setf (get pal_pcl::saveRequest :definition-)
      "---

")

(setf (get pal_pcl::saveResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get pal_pcl::saveResponse :datatype-) "pal_pcl/saveResponse")
(setf (get pal_pcl::saveResponse :definition-)
      "---

")



(provide :pal_pcl/save "d41d8cd98f00b204e9800998ecf8427e")


