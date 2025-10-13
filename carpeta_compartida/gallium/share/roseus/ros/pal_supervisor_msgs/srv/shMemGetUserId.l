;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::shMemGetUserId)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'shMemGetUserId (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMGETUSERID")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMGETUSERID"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMGETUSERIDREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMGETUSERIDREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMGETUSERIDRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMGETUSERIDRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::shMemGetUserIdRequest
  :super ros::object
  :slots (_appName _key ))

(defmethod pal_supervisor_msgs::shMemGetUserIdRequest
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

(defclass pal_supervisor_msgs::shMemGetUserIdResponse
  :super ros::object
  :slots (_userId ))

(defmethod pal_supervisor_msgs::shMemGetUserIdResponse
  (:init
   (&key
    ((:userId __userId) "")
    )
   (send-super :init)
   (setq _userId (string __userId))
   self)
  (:userId
   (&optional __userId)
   (if __userId (setq _userId __userId)) _userId)
  (:serialization-length
   ()
   (+
    ;; string _userId
    4 (length _userId)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _userId
       (write-long (length _userId) s) (princ _userId s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _userId
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _userId (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::shMemGetUserId
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::shMemGetUserId :md5sum-) "4c3734a520e124bb4c1f359d31845707")
(setf (get pal_supervisor_msgs::shMemGetUserId :datatype-) "pal_supervisor_msgs/shMemGetUserId")
(setf (get pal_supervisor_msgs::shMemGetUserId :request) pal_supervisor_msgs::shMemGetUserIdRequest)
(setf (get pal_supervisor_msgs::shMemGetUserId :response) pal_supervisor_msgs::shMemGetUserIdResponse)

(defmethod pal_supervisor_msgs::shMemGetUserIdRequest
  (:response () (instance pal_supervisor_msgs::shMemGetUserIdResponse :init)))

(setf (get pal_supervisor_msgs::shMemGetUserIdRequest :md5sum-) "4c3734a520e124bb4c1f359d31845707")
(setf (get pal_supervisor_msgs::shMemGetUserIdRequest :datatype-) "pal_supervisor_msgs/shMemGetUserIdRequest")
(setf (get pal_supervisor_msgs::shMemGetUserIdRequest :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
---
string userId #Returns the user ID related to the Shared memory

")

(setf (get pal_supervisor_msgs::shMemGetUserIdResponse :md5sum-) "4c3734a520e124bb4c1f359d31845707")
(setf (get pal_supervisor_msgs::shMemGetUserIdResponse :datatype-) "pal_supervisor_msgs/shMemGetUserIdResponse")
(setf (get pal_supervisor_msgs::shMemGetUserIdResponse :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
---
string userId #Returns the user ID related to the Shared memory

")



(provide :pal_supervisor_msgs/shMemGetUserId "4c3734a520e124bb4c1f359d31845707")


