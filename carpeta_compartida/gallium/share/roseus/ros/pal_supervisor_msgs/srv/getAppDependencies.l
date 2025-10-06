;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::getAppDependencies)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'getAppDependencies (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::GETAPPDEPENDENCIES")
  (make-package "PAL_SUPERVISOR_MSGS::GETAPPDEPENDENCIES"))
(unless (find-package "PAL_SUPERVISOR_MSGS::GETAPPDEPENDENCIESREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::GETAPPDEPENDENCIESREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::GETAPPDEPENDENCIESRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::GETAPPDEPENDENCIESRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::getAppDependenciesRequest
  :super ros::object
  :slots (_appName ))

(defmethod pal_supervisor_msgs::getAppDependenciesRequest
  (:init
   (&key
    ((:appName __appName) "")
    )
   (send-super :init)
   (setq _appName (string __appName))
   self)
  (:appName
   (&optional __appName)
   (if __appName (setq _appName __appName)) _appName)
  (:serialization-length
   ()
   (+
    ;; string _appName
    4 (length _appName)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _appName
       (write-long (length _appName) s) (princ _appName s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _appName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _appName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::getAppDependenciesResponse
  :super ros::object
  :slots (_dependencies ))

(defmethod pal_supervisor_msgs::getAppDependenciesResponse
  (:init
   (&key
    ((:dependencies __dependencies) ())
    )
   (send-super :init)
   (setq _dependencies __dependencies)
   self)
  (:dependencies
   (&rest __dependencies)
   (if (keywordp (car __dependencies))
       (send* _dependencies __dependencies)
     (progn
       (if __dependencies (setq _dependencies (car __dependencies)))
       _dependencies)))
  (:serialization-length
   ()
   (+
    ;; pal_supervisor_msgs/DependencyStatus[] _dependencies
    (apply #'+ (send-all _dependencies :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pal_supervisor_msgs/DependencyStatus[] _dependencies
     (write-long (length _dependencies) s)
     (dolist (elem _dependencies)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pal_supervisor_msgs/DependencyStatus[] _dependencies
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dependencies (let (r) (dotimes (i n) (push (instance pal_supervisor_msgs::DependencyStatus :init) r)) r))
     (dolist (elem- _dependencies)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::getAppDependencies
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::getAppDependencies :md5sum-) "f7faf0b37d011444aea3a2fa0a051c43")
(setf (get pal_supervisor_msgs::getAppDependencies :datatype-) "pal_supervisor_msgs/getAppDependencies")
(setf (get pal_supervisor_msgs::getAppDependencies :request) pal_supervisor_msgs::getAppDependenciesRequest)
(setf (get pal_supervisor_msgs::getAppDependencies :response) pal_supervisor_msgs::getAppDependenciesResponse)

(defmethod pal_supervisor_msgs::getAppDependenciesRequest
  (:response () (instance pal_supervisor_msgs::getAppDependenciesResponse :init)))

(setf (get pal_supervisor_msgs::getAppDependenciesRequest :md5sum-) "f7faf0b37d011444aea3a2fa0a051c43")
(setf (get pal_supervisor_msgs::getAppDependenciesRequest :datatype-) "pal_supervisor_msgs/getAppDependenciesRequest")
(setf (get pal_supervisor_msgs::getAppDependenciesRequest :definition-)
      "string appName #Name of the application 
---
pal_supervisor_msgs/DependencyStatus[] dependencies #Array of dependencies


================================================================================
MSG: pal_supervisor_msgs/DependencyStatus
string name # dependency name
# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3 #Application not running
byte level # level of operation enumerated above 
")

(setf (get pal_supervisor_msgs::getAppDependenciesResponse :md5sum-) "f7faf0b37d011444aea3a2fa0a051c43")
(setf (get pal_supervisor_msgs::getAppDependenciesResponse :datatype-) "pal_supervisor_msgs/getAppDependenciesResponse")
(setf (get pal_supervisor_msgs::getAppDependenciesResponse :definition-)
      "string appName #Name of the application 
---
pal_supervisor_msgs/DependencyStatus[] dependencies #Array of dependencies


================================================================================
MSG: pal_supervisor_msgs/DependencyStatus
string name # dependency name
# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3 #Application not running
byte level # level of operation enumerated above 
")



(provide :pal_supervisor_msgs/getAppDependencies "f7faf0b37d011444aea3a2fa0a051c43")


