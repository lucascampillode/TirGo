;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::shMemIsAvailable)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'shMemIsAvailable (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISAVAILABLE")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISAVAILABLE"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISAVAILABLEREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISAVAILABLEREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISAVAILABLERESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISAVAILABLERESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::shMemIsAvailableRequest
  :super ros::object
  :slots (_appName _key ))

(defmethod pal_supervisor_msgs::shMemIsAvailableRequest
  (:init
   (&key
    ((:appName __appName) "")
    ((:key __key) 0)
    )
   (send-super :init)
   (setq _appName (string __appName))
   (setq _key (round __key))
   self)
  (:appName
   (&optional __appName)
   (if __appName (setq _appName __appName)) _appName)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:serialization-length
   ()
   (+
    ;; string _appName
    4 (length _appName)
    ;; int32 _key
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _appName
       (write-long (length _appName) s) (princ _appName s)
     ;; int32 _key
       (write-long _key s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _appName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _appName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _key
     (setq _key (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::shMemIsAvailableResponse
  :super ros::object
  :slots (_available ))

(defmethod pal_supervisor_msgs::shMemIsAvailableResponse
  (:init
   (&key
    ((:available __available) nil)
    )
   (send-super :init)
   (setq _available __available)
   self)
  (:available
   (&optional (__available :null))
   (if (not (eq __available :null)) (setq _available __available)) _available)
  (:serialization-length
   ()
   (+
    ;; bool _available
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _available
       (if _available (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _available
     (setq _available (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::shMemIsAvailable
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::shMemIsAvailable :md5sum-) "ecf0a07e30d52a9c701ab569c846bf9a")
(setf (get pal_supervisor_msgs::shMemIsAvailable :datatype-) "pal_supervisor_msgs/shMemIsAvailable")
(setf (get pal_supervisor_msgs::shMemIsAvailable :request) pal_supervisor_msgs::shMemIsAvailableRequest)
(setf (get pal_supervisor_msgs::shMemIsAvailable :response) pal_supervisor_msgs::shMemIsAvailableResponse)

(defmethod pal_supervisor_msgs::shMemIsAvailableRequest
  (:response () (instance pal_supervisor_msgs::shMemIsAvailableResponse :init)))

(setf (get pal_supervisor_msgs::shMemIsAvailableRequest :md5sum-) "ecf0a07e30d52a9c701ab569c846bf9a")
(setf (get pal_supervisor_msgs::shMemIsAvailableRequest :datatype-) "pal_supervisor_msgs/shMemIsAvailableRequest")
(setf (get pal_supervisor_msgs::shMemIsAvailableRequest :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
---
bool available #True if the sharedMemory with the given key is active and opened by it's owner.

")

(setf (get pal_supervisor_msgs::shMemIsAvailableResponse :md5sum-) "ecf0a07e30d52a9c701ab569c846bf9a")
(setf (get pal_supervisor_msgs::shMemIsAvailableResponse :datatype-) "pal_supervisor_msgs/shMemIsAvailableResponse")
(setf (get pal_supervisor_msgs::shMemIsAvailableResponse :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
---
bool available #True if the sharedMemory with the given key is active and opened by it's owner.

")



(provide :pal_supervisor_msgs/shMemIsAvailable "ecf0a07e30d52a9c701ab569c846bf9a")


