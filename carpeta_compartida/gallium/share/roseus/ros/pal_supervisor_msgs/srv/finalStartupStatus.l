;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::finalStartupStatus)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'finalStartupStatus (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::FINALSTARTUPSTATUS")
  (make-package "PAL_SUPERVISOR_MSGS::FINALSTARTUPSTATUS"))
(unless (find-package "PAL_SUPERVISOR_MSGS::FINALSTARTUPSTATUSREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::FINALSTARTUPSTATUSREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::FINALSTARTUPSTATUSRESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::FINALSTARTUPSTATUSRESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::finalStartupStatusRequest
  :super ros::object
  :slots (_status ))

(defmethod pal_supervisor_msgs::finalStartupStatusRequest
  (:init
   (&key
    ((:status __status) (instance pal_supervisor_msgs::StartupStatus :init))
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:serialization-length
   ()
   (+
    ;; pal_supervisor_msgs/StartupStatus _status
    (send _status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pal_supervisor_msgs/StartupStatus _status
       (send _status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pal_supervisor_msgs/StartupStatus _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::finalStartupStatusResponse
  :super ros::object
  :slots ())

(defmethod pal_supervisor_msgs::finalStartupStatusResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass pal_supervisor_msgs::finalStartupStatus
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::finalStartupStatus :md5sum-) "3fe91db533913eea244a861ae3cea188")
(setf (get pal_supervisor_msgs::finalStartupStatus :datatype-) "pal_supervisor_msgs/finalStartupStatus")
(setf (get pal_supervisor_msgs::finalStartupStatus :request) pal_supervisor_msgs::finalStartupStatusRequest)
(setf (get pal_supervisor_msgs::finalStartupStatus :response) pal_supervisor_msgs::finalStartupStatusResponse)

(defmethod pal_supervisor_msgs::finalStartupStatusRequest
  (:response () (instance pal_supervisor_msgs::finalStartupStatusResponse :init)))

(setf (get pal_supervisor_msgs::finalStartupStatusRequest :md5sum-) "3fe91db533913eea244a861ae3cea188")
(setf (get pal_supervisor_msgs::finalStartupStatusRequest :datatype-) "pal_supervisor_msgs/finalStartupStatusRequest")
(setf (get pal_supervisor_msgs::finalStartupStatusRequest :definition-)
      "#Final status reported by startup launch script. This should be repeated by the supervisor forever
pal_supervisor_msgs/StartupStatus status

================================================================================
MSG: pal_supervisor_msgs/StartupStatus
string name # application name
string message # description of the status if required
# Possible status of startup process
byte RUNNING=0
byte WAITING=1
byte TIMEOUT=2
byte ABORTED=3 #Application not running
byte status # status of the startup process for the application. Enumerated above 

---

")

(setf (get pal_supervisor_msgs::finalStartupStatusResponse :md5sum-) "3fe91db533913eea244a861ae3cea188")
(setf (get pal_supervisor_msgs::finalStartupStatusResponse :datatype-) "pal_supervisor_msgs/finalStartupStatusResponse")
(setf (get pal_supervisor_msgs::finalStartupStatusResponse :definition-)
      "#Final status reported by startup launch script. This should be repeated by the supervisor forever
pal_supervisor_msgs/StartupStatus status

================================================================================
MSG: pal_supervisor_msgs/StartupStatus
string name # application name
string message # description of the status if required
# Possible status of startup process
byte RUNNING=0
byte WAITING=1
byte TIMEOUT=2
byte ABORTED=3 #Application not running
byte status # status of the startup process for the application. Enumerated above 

---

")



(provide :pal_supervisor_msgs/finalStartupStatus "3fe91db533913eea244a861ae3cea188")


