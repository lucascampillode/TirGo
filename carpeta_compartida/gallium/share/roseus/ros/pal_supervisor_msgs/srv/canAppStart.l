;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::canAppStart)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'canAppStart (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::CANAPPSTART")
  (make-package "PAL_SUPERVISOR_MSGS::CANAPPSTART"))
(unless (find-package "PAL_SUPERVISOR_MSGS::CANAPPSTARTREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::CANAPPSTARTREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::CANAPPSTARTRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::CANAPPSTARTRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::canAppStartRequest
  :super ros::object
  :slots (_appName ))

(defmethod pal_supervisor_msgs::canAppStartRequest
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

(defclass pal_supervisor_msgs::canAppStartResponse
  :super ros::object
  :slots (_canStart _description ))

(defmethod pal_supervisor_msgs::canAppStartResponse
  (:init
   (&key
    ((:canStart __canStart) nil)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _canStart __canStart)
   (setq _description (string __description))
   self)
  (:canStart
   (&optional (__canStart :null))
   (if (not (eq __canStart :null)) (setq _canStart __canStart)) _canStart)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; bool _canStart
    1
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _canStart
       (if _canStart (write-byte -1 s) (write-byte 0 s))
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _canStart
     (setq _canStart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::canAppStart
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::canAppStart :md5sum-) "2a4675c414f30fa71f2e7af8b08bf502")
(setf (get pal_supervisor_msgs::canAppStart :datatype-) "pal_supervisor_msgs/canAppStart")
(setf (get pal_supervisor_msgs::canAppStart :request) pal_supervisor_msgs::canAppStartRequest)
(setf (get pal_supervisor_msgs::canAppStart :response) pal_supervisor_msgs::canAppStartResponse)

(defmethod pal_supervisor_msgs::canAppStartRequest
  (:response () (instance pal_supervisor_msgs::canAppStartResponse :init)))

(setf (get pal_supervisor_msgs::canAppStartRequest :md5sum-) "2a4675c414f30fa71f2e7af8b08bf502")
(setf (get pal_supervisor_msgs::canAppStartRequest :datatype-) "pal_supervisor_msgs/canAppStartRequest")
(setf (get pal_supervisor_msgs::canAppStartRequest :definition-)
      "string appName #Name of the application 
---
bool canStart #True if the application can start
string description  #Reason why an application can't start. empty if can start

")

(setf (get pal_supervisor_msgs::canAppStartResponse :md5sum-) "2a4675c414f30fa71f2e7af8b08bf502")
(setf (get pal_supervisor_msgs::canAppStartResponse :datatype-) "pal_supervisor_msgs/canAppStartResponse")
(setf (get pal_supervisor_msgs::canAppStartResponse :definition-)
      "string appName #Name of the application 
---
bool canStart #True if the application can start
string description  #Reason why an application can't start. empty if can start

")



(provide :pal_supervisor_msgs/canAppStart "2a4675c414f30fa71f2e7af8b08bf502")


