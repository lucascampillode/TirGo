;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::shMemIsClosed)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'shMemIsClosed (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISCLOSED")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISCLOSED"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISCLOSEDREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISCLOSEDREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SHMEMISCLOSEDRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::SHMEMISCLOSEDRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::shMemIsClosedRequest
  :super ros::object
  :slots (_appName _key ))

(defmethod pal_supervisor_msgs::shMemIsClosedRequest
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

(defclass pal_supervisor_msgs::shMemIsClosedResponse
  :super ros::object
  :slots (_isClosed ))

(defmethod pal_supervisor_msgs::shMemIsClosedResponse
  (:init
   (&key
    ((:isClosed __isClosed) nil)
    )
   (send-super :init)
   (setq _isClosed __isClosed)
   self)
  (:isClosed
   (&optional (__isClosed :null))
   (if (not (eq __isClosed :null)) (setq _isClosed __isClosed)) _isClosed)
  (:serialization-length
   ()
   (+
    ;; bool _isClosed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isClosed
       (if _isClosed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isClosed
     (setq _isClosed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::shMemIsClosed
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::shMemIsClosed :md5sum-) "7d5da756f3c73d0365ab7b15abe7a3f4")
(setf (get pal_supervisor_msgs::shMemIsClosed :datatype-) "pal_supervisor_msgs/shMemIsClosed")
(setf (get pal_supervisor_msgs::shMemIsClosed :request) pal_supervisor_msgs::shMemIsClosedRequest)
(setf (get pal_supervisor_msgs::shMemIsClosed :response) pal_supervisor_msgs::shMemIsClosedResponse)

(defmethod pal_supervisor_msgs::shMemIsClosedRequest
  (:response () (instance pal_supervisor_msgs::shMemIsClosedResponse :init)))

(setf (get pal_supervisor_msgs::shMemIsClosedRequest :md5sum-) "7d5da756f3c73d0365ab7b15abe7a3f4")
(setf (get pal_supervisor_msgs::shMemIsClosedRequest :datatype-) "pal_supervisor_msgs/shMemIsClosedRequest")
(setf (get pal_supervisor_msgs::shMemIsClosedRequest :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
---
bool isClosed #True if the sharedMemory with the given key has been closed

")

(setf (get pal_supervisor_msgs::shMemIsClosedResponse :md5sum-) "7d5da756f3c73d0365ab7b15abe7a3f4")
(setf (get pal_supervisor_msgs::shMemIsClosedResponse :datatype-) "pal_supervisor_msgs/shMemIsClosedResponse")
(setf (get pal_supervisor_msgs::shMemIsClosedResponse :definition-)
      "string appName #Name of the application sending the command
int32 key #Shared memory Key
---
bool isClosed #True if the sharedMemory with the given key has been closed

")



(provide :pal_supervisor_msgs/shMemIsClosed "7d5da756f3c73d0365ab7b15abe7a3f4")


