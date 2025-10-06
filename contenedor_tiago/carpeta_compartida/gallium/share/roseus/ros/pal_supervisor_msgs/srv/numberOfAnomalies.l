;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::numberOfAnomalies)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'numberOfAnomalies (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::NUMBEROFANOMALIES")
  (make-package "PAL_SUPERVISOR_MSGS::NUMBEROFANOMALIES"))
(unless (find-package "PAL_SUPERVISOR_MSGS::NUMBEROFANOMALIESREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::NUMBEROFANOMALIESREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::NUMBEROFANOMALIESRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::NUMBEROFANOMALIESRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::numberOfAnomaliesRequest
  :super ros::object
  :slots ())

(defmethod pal_supervisor_msgs::numberOfAnomaliesRequest
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

(defclass pal_supervisor_msgs::numberOfAnomaliesResponse
  :super ros::object
  :slots (_numErrors _numWarnings ))

(defmethod pal_supervisor_msgs::numberOfAnomaliesResponse
  (:init
   (&key
    ((:numErrors __numErrors) 0)
    ((:numWarnings __numWarnings) 0)
    )
   (send-super :init)
   (setq _numErrors (round __numErrors))
   (setq _numWarnings (round __numWarnings))
   self)
  (:numErrors
   (&optional __numErrors)
   (if __numErrors (setq _numErrors __numErrors)) _numErrors)
  (:numWarnings
   (&optional __numWarnings)
   (if __numWarnings (setq _numWarnings __numWarnings)) _numWarnings)
  (:serialization-length
   ()
   (+
    ;; int32 _numErrors
    4
    ;; int32 _numWarnings
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _numErrors
       (write-long _numErrors s)
     ;; int32 _numWarnings
       (write-long _numWarnings s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _numErrors
     (setq _numErrors (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _numWarnings
     (setq _numWarnings (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::numberOfAnomalies
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::numberOfAnomalies :md5sum-) "256e37138065fcef5d440c4ba2011920")
(setf (get pal_supervisor_msgs::numberOfAnomalies :datatype-) "pal_supervisor_msgs/numberOfAnomalies")
(setf (get pal_supervisor_msgs::numberOfAnomalies :request) pal_supervisor_msgs::numberOfAnomaliesRequest)
(setf (get pal_supervisor_msgs::numberOfAnomalies :response) pal_supervisor_msgs::numberOfAnomaliesResponse)

(defmethod pal_supervisor_msgs::numberOfAnomaliesRequest
  (:response () (instance pal_supervisor_msgs::numberOfAnomaliesResponse :init)))

(setf (get pal_supervisor_msgs::numberOfAnomaliesRequest :md5sum-) "256e37138065fcef5d440c4ba2011920")
(setf (get pal_supervisor_msgs::numberOfAnomaliesRequest :datatype-) "pal_supervisor_msgs/numberOfAnomaliesRequest")
(setf (get pal_supervisor_msgs::numberOfAnomaliesRequest :definition-)
      "---
int32 numErrors #Number of errors
int32 numWarnings #Number of warnings

")

(setf (get pal_supervisor_msgs::numberOfAnomaliesResponse :md5sum-) "256e37138065fcef5d440c4ba2011920")
(setf (get pal_supervisor_msgs::numberOfAnomaliesResponse :datatype-) "pal_supervisor_msgs/numberOfAnomaliesResponse")
(setf (get pal_supervisor_msgs::numberOfAnomaliesResponse :definition-)
      "---
int32 numErrors #Number of errors
int32 numWarnings #Number of warnings

")



(provide :pal_supervisor_msgs/numberOfAnomalies "256e37138065fcef5d440c4ba2011920")


