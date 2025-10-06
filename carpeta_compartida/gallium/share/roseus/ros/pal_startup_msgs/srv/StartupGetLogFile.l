;; Auto-generated. Do not edit!


(when (boundp 'pal_startup_msgs::StartupGetLogFile)
  (if (not (find-package "PAL_STARTUP_MSGS"))
    (make-package "PAL_STARTUP_MSGS"))
  (shadow 'StartupGetLogFile (find-package "PAL_STARTUP_MSGS")))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPGETLOGFILE")
  (make-package "PAL_STARTUP_MSGS::STARTUPGETLOGFILE"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPGETLOGFILEREQUEST")
  (make-package "PAL_STARTUP_MSGS::STARTUPGETLOGFILEREQUEST"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPGETLOGFILERESPONSE")
  (make-package "PAL_STARTUP_MSGS::STARTUPGETLOGFILERESPONSE"))

(in-package "ROS")





(defclass pal_startup_msgs::StartupGetLogFileRequest
  :super ros::object
  :slots (_app ))

(defmethod pal_startup_msgs::StartupGetLogFileRequest
  (:init
   (&key
    ((:app __app) "")
    )
   (send-super :init)
   (setq _app (string __app))
   self)
  (:app
   (&optional __app)
   (if __app (setq _app __app)) _app)
  (:serialization-length
   ()
   (+
    ;; string _app
    4 (length _app)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _app
       (write-long (length _app) s) (princ _app s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _app
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _app (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_startup_msgs::StartupGetLogFileResponse
  :super ros::object
  :slots (_log_file ))

(defmethod pal_startup_msgs::StartupGetLogFileResponse
  (:init
   (&key
    ((:log_file __log_file) "")
    )
   (send-super :init)
   (setq _log_file (string __log_file))
   self)
  (:log_file
   (&optional __log_file)
   (if __log_file (setq _log_file __log_file)) _log_file)
  (:serialization-length
   ()
   (+
    ;; string _log_file
    4 (length _log_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _log_file
       (write-long (length _log_file) s) (princ _log_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _log_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _log_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_startup_msgs::StartupGetLogFile
  :super ros::object
  :slots ())

(setf (get pal_startup_msgs::StartupGetLogFile :md5sum-) "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(setf (get pal_startup_msgs::StartupGetLogFile :datatype-) "pal_startup_msgs/StartupGetLogFile")
(setf (get pal_startup_msgs::StartupGetLogFile :request) pal_startup_msgs::StartupGetLogFileRequest)
(setf (get pal_startup_msgs::StartupGetLogFile :response) pal_startup_msgs::StartupGetLogFileResponse)

(defmethod pal_startup_msgs::StartupGetLogFileRequest
  (:response () (instance pal_startup_msgs::StartupGetLogFileResponse :init)))

(setf (get pal_startup_msgs::StartupGetLogFileRequest :md5sum-) "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(setf (get pal_startup_msgs::StartupGetLogFileRequest :datatype-) "pal_startup_msgs/StartupGetLogFileRequest")
(setf (get pal_startup_msgs::StartupGetLogFileRequest :definition-)
      "string app
---
string log_file

")

(setf (get pal_startup_msgs::StartupGetLogFileResponse :md5sum-) "ccb1bd5394209c4a6d7f5a5e0db3dd26")
(setf (get pal_startup_msgs::StartupGetLogFileResponse :datatype-) "pal_startup_msgs/StartupGetLogFileResponse")
(setf (get pal_startup_msgs::StartupGetLogFileResponse :definition-)
      "string app
---
string log_file

")



(provide :pal_startup_msgs/StartupGetLogFile "ccb1bd5394209c4a6d7f5a5e0db3dd26")


