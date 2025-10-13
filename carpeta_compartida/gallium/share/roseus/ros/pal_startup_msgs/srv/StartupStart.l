;; Auto-generated. Do not edit!


(when (boundp 'pal_startup_msgs::StartupStart)
  (if (not (find-package "PAL_STARTUP_MSGS"))
    (make-package "PAL_STARTUP_MSGS"))
  (shadow 'StartupStart (find-package "PAL_STARTUP_MSGS")))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTART")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTART"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTARTREQUEST")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTARTREQUEST"))
(unless (find-package "PAL_STARTUP_MSGS::STARTUPSTARTRESPONSE")
  (make-package "PAL_STARTUP_MSGS::STARTUPSTARTRESPONSE"))

(in-package "ROS")





(defclass pal_startup_msgs::StartupStartRequest
  :super ros::object
  :slots (_app _args ))

(defmethod pal_startup_msgs::StartupStartRequest
  (:init
   (&key
    ((:app __app) "")
    ((:args __args) "")
    )
   (send-super :init)
   (setq _app (string __app))
   (setq _args (string __args))
   self)
  (:app
   (&optional __app)
   (if __app (setq _app __app)) _app)
  (:args
   (&optional __args)
   (if __args (setq _args __args)) _args)
  (:serialization-length
   ()
   (+
    ;; string _app
    4 (length _app)
    ;; string _args
    4 (length _args)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _app
       (write-long (length _app) s) (princ _app s)
     ;; string _args
       (write-long (length _args) s) (princ _args s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _app
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _app (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _args
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _args (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_startup_msgs::StartupStartResponse
  :super ros::object
  :slots (_result ))

(defmethod pal_startup_msgs::StartupStartResponse
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

(defclass pal_startup_msgs::StartupStart
  :super ros::object
  :slots ())

(setf (get pal_startup_msgs::StartupStart :md5sum-) "9b78b04bdfab704be898cd6637524866")
(setf (get pal_startup_msgs::StartupStart :datatype-) "pal_startup_msgs/StartupStart")
(setf (get pal_startup_msgs::StartupStart :request) pal_startup_msgs::StartupStartRequest)
(setf (get pal_startup_msgs::StartupStart :response) pal_startup_msgs::StartupStartResponse)

(defmethod pal_startup_msgs::StartupStartRequest
  (:response () (instance pal_startup_msgs::StartupStartResponse :init)))

(setf (get pal_startup_msgs::StartupStartRequest :md5sum-) "9b78b04bdfab704be898cd6637524866")
(setf (get pal_startup_msgs::StartupStartRequest :datatype-) "pal_startup_msgs/StartupStartRequest")
(setf (get pal_startup_msgs::StartupStartRequest :definition-)
      "string app
string args
---
string result

")

(setf (get pal_startup_msgs::StartupStartResponse :md5sum-) "9b78b04bdfab704be898cd6637524866")
(setf (get pal_startup_msgs::StartupStartResponse :datatype-) "pal_startup_msgs/StartupStartResponse")
(setf (get pal_startup_msgs::StartupStartResponse :definition-)
      "string app
string args
---
string result

")



(provide :pal_startup_msgs/StartupStart "9b78b04bdfab704be898cd6637524866")


