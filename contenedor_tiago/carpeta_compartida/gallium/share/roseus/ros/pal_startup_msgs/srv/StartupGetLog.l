;; Auto-generated. Do not edit!


(when (boundp 'pal_startup_msgs::StartupGetLog)
  (if (not (find-package "PAL_STARTUP_MSGS"))
    (make-package "PAL_STARTUP_MSGS"))
  (shadow 'StartupGetLog (find-package "PAL_STARTUP_MSGS")))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPGETLOG")
  (make-package "PAL_STARTUP_MSGS::STARTUPGETLOG"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPGETLOGREQUEST")
  (make-package "PAL_STARTUP_MSGS::STARTUPGETLOGREQUEST"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPGETLOGRESPONSE")
  (make-package "PAL_STARTUP_MSGS::STARTUPGETLOGRESPONSE"))

(in-package "ROS")





(defclass pal_startup_msgs::StartupGetLogRequest
  :super ros::object
  :slots (_app _nlines ))

(defmethod pal_startup_msgs::StartupGetLogRequest
  (:init
   (&key
    ((:app __app) "")
    ((:nlines __nlines) 0)
    )
   (send-super :init)
   (setq _app (string __app))
   (setq _nlines (round __nlines))
   self)
  (:app
   (&optional __app)
   (if __app (setq _app __app)) _app)
  (:nlines
   (&optional __nlines)
   (if __nlines (setq _nlines __nlines)) _nlines)
  (:serialization-length
   ()
   (+
    ;; string _app
    4 (length _app)
    ;; int32 _nlines
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _app
       (write-long (length _app) s) (princ _app s)
     ;; int32 _nlines
       (write-long _nlines s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _app
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _app (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _nlines
     (setq _nlines (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_startup_msgs::StartupGetLogResponse
  :super ros::object
  :slots (_log ))

(defmethod pal_startup_msgs::StartupGetLogResponse
  (:init
   (&key
    ((:log __log) "")
    )
   (send-super :init)
   (setq _log (string __log))
   self)
  (:log
   (&optional __log)
   (if __log (setq _log __log)) _log)
  (:serialization-length
   ()
   (+
    ;; string _log
    4 (length _log)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _log
       (write-long (length _log) s) (princ _log s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _log
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _log (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_startup_msgs::StartupGetLog
  :super ros::object
  :slots ())

(setf (get pal_startup_msgs::StartupGetLog :md5sum-) "6bc85f88abf333b9c46f2b5155e5e9f0")
(setf (get pal_startup_msgs::StartupGetLog :datatype-) "pal_startup_msgs/StartupGetLog")
(setf (get pal_startup_msgs::StartupGetLog :request) pal_startup_msgs::StartupGetLogRequest)
(setf (get pal_startup_msgs::StartupGetLog :response) pal_startup_msgs::StartupGetLogResponse)

(defmethod pal_startup_msgs::StartupGetLogRequest
  (:response () (instance pal_startup_msgs::StartupGetLogResponse :init)))

(setf (get pal_startup_msgs::StartupGetLogRequest :md5sum-) "6bc85f88abf333b9c46f2b5155e5e9f0")
(setf (get pal_startup_msgs::StartupGetLogRequest :datatype-) "pal_startup_msgs/StartupGetLogRequest")
(setf (get pal_startup_msgs::StartupGetLogRequest :definition-)
      "# Returns nlines last lines of app's log
string app
int32 nlines
---
string log

")

(setf (get pal_startup_msgs::StartupGetLogResponse :md5sum-) "6bc85f88abf333b9c46f2b5155e5e9f0")
(setf (get pal_startup_msgs::StartupGetLogResponse :datatype-) "pal_startup_msgs/StartupGetLogResponse")
(setf (get pal_startup_msgs::StartupGetLogResponse :definition-)
      "# Returns nlines last lines of app's log
string app
int32 nlines
---
string log

")



(provide :pal_startup_msgs/StartupGetLog "6bc85f88abf333b9c46f2b5155e5e9f0")


