;; Auto-generated. Do not edit!


(when (boundp 'pal_startup_msgs::StartupStop)
  (if (not (find-package "PAL_STARTUP_MSGS"))
    (make-package "PAL_STARTUP_MSGS"))
  (shadow 'StartupStop (find-package "PAL_STARTUP_MSGS")))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTOP")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTOP"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTOPREQUEST")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTOPREQUEST"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTOPRESPONSE")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTOPRESPONSE"))

(in-package "ROS")





(defclass pal_startup_msgs::StartupStopRequest
  :super ros::object
  :slots (_app ))

(defmethod pal_startup_msgs::StartupStopRequest
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

(defclass pal_startup_msgs::StartupStopResponse
  :super ros::object
  :slots (_result ))

(defmethod pal_startup_msgs::StartupStopResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_startup_msgs::StartupStop
  :super ros::object
  :slots ())

(setf (get pal_startup_msgs::StartupStop :md5sum-) "d0a14343d8976d7512bdd0e94c3d391e")
(setf (get pal_startup_msgs::StartupStop :datatype-) "pal_startup_msgs/StartupStop")
(setf (get pal_startup_msgs::StartupStop :request) pal_startup_msgs::StartupStopRequest)
(setf (get pal_startup_msgs::StartupStop :response) pal_startup_msgs::StartupStopResponse)

(defmethod pal_startup_msgs::StartupStopRequest
  (:response () (instance pal_startup_msgs::StartupStopResponse :init)))

(setf (get pal_startup_msgs::StartupStopRequest :md5sum-) "d0a14343d8976d7512bdd0e94c3d391e")
(setf (get pal_startup_msgs::StartupStopRequest :datatype-) "pal_startup_msgs/StartupStopRequest")
(setf (get pal_startup_msgs::StartupStopRequest :definition-)
      "string app
---
string result

")

(setf (get pal_startup_msgs::StartupStopResponse :md5sum-) "d0a14343d8976d7512bdd0e94c3d391e")
(setf (get pal_startup_msgs::StartupStopResponse :datatype-) "pal_startup_msgs/StartupStopResponse")
(setf (get pal_startup_msgs::StartupStopResponse :definition-)
      "string app
---
string result

")



(provide :pal_startup_msgs/StartupStop "d0a14343d8976d7512bdd0e94c3d391e")


