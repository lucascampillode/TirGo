;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::shMemIsTimestampValid)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'shMemIsTimestampValid (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISTIMESTAMPVALID")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISTIMESTAMPVALID"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISTIMESTAMPVALIDREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISTIMESTAMPVALIDREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISTIMESTAMPVALIDRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISTIMESTAMPVALIDRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::shMemIsTimestampValidRequest
  :super ros::object
  :slots (_appName _key _offset ))

(defmethod pal_supervisor_msgs::shMemIsTimestampValidRequest
  (:init
   (&key
    ((:appName __appName) "")
    ((:key __key) 0)
    ((:offset __offset) 0)
    )
   (send-super :init)
   (setq _appName (string __appName))
   (setq _key (round __key))
   (setq _offset (round __offset))
   self)
  (:appName
   (&optional __appName)
   (if __appName (setq _appName __appName)) _appName)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:serialization-length
   ()
   (+
    ;; string _appName
    4 (length _appName)
    ;; int32 _key
    4
    ;; uint32 _offset
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
     ;; uint32 _offset
       (write-long _offset s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _appName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _appName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _key
     (setq _key (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _offset
     (setq _offset (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::shMemIsTimestampValidResponse
  :super ros::object
  :slots (_isValid ))

(defmethod pal_supervisor_msgs::shMemIsTimestampValidResponse
  (:init
   (&key
    ((:isValid __isValid) nil)
    )
   (send-super :init)
   (setq _isValid __isValid)
   self)
  (:isValid
   (&optional (__isValid :null))
   (if (not (eq __isValid :null)) (setq _isValid __isValid)) _isValid)
  (:serialization-length
   ()
   (+
    ;; bool _isValid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isValid
       (if _isValid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isValid
     (setq _isValid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::shMemIsTimestampValid
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::shMemIsTimestampValid :md5sum-) "8fd93cf90468f6698ba910ec3df45dba")
(setf (get pal_supervisor_msgs::shMemIsTimestampValid :datatype-) "pal_supervisor_msgs/shMemIsTimestampValid")
(setf (get pal_supervisor_msgs::shMemIsTimestampValid :request) pal_supervisor_msgs::shMemIsTimestampValidRequest)
(setf (get pal_supervisor_msgs::shMemIsTimestampValid :response) pal_supervisor_msgs::shMemIsTimestampValidResponse)

(defmethod pal_supervisor_msgs::shMemIsTimestampValidRequest
  (:response () (instance pal_supervisor_msgs::shMemIsTimestampValidResponse :init)))

(setf (get pal_supervisor_msgs::shMemIsTimestampValidRequest :md5sum-) "8fd93cf90468f6698ba910ec3df45dba")
(setf (get pal_supervisor_msgs::shMemIsTimestampValidRequest :datatype-) "pal_supervisor_msgs/shMemIsTimestampValidRequest")
(setf (get pal_supervisor_msgs::shMemIsTimestampValidRequest :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
uint32 offset #Offset inside the ShMem of the timeout
---
bool isValid #Returns if the timeout check is valid

")

(setf (get pal_supervisor_msgs::shMemIsTimestampValidResponse :md5sum-) "8fd93cf90468f6698ba910ec3df45dba")
(setf (get pal_supervisor_msgs::shMemIsTimestampValidResponse :datatype-) "pal_supervisor_msgs/shMemIsTimestampValidResponse")
(setf (get pal_supervisor_msgs::shMemIsTimestampValidResponse :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
uint32 offset #Offset inside the ShMem of the timeout
---
bool isValid #Returns if the timeout check is valid

")



(provide :pal_supervisor_msgs/shMemIsTimestampValid "8fd93cf90468f6698ba910ec3df45dba")


