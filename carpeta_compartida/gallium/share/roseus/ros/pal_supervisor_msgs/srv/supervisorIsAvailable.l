;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::supervisorIsAvailable)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'supervisorIsAvailable (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::SUPERVISORISAVAILABLE")
  (make-package "PAL_SUPERVISOR_MSGS::SUPERVISORISAVAILABLE"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SUPERVISORISAVAILABLEREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::SUPERVISORISAVAILABLEREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::SUPERVISORISAVAILABLERESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::SUPERVISORISAVAILABLERESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::supervisorIsAvailableRequest
  :super ros::object
  :slots ())

(defmethod pal_supervisor_msgs::supervisorIsAvailableRequest
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

(defclass pal_supervisor_msgs::supervisorIsAvailableResponse
  :super ros::object
  :slots (_available ))

(defmethod pal_supervisor_msgs::supervisorIsAvailableResponse
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

(defclass pal_supervisor_msgs::supervisorIsAvailable
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::supervisorIsAvailable :md5sum-) "efb9ac7739f299decb8575015d81760c")
(setf (get pal_supervisor_msgs::supervisorIsAvailable :datatype-) "pal_supervisor_msgs/supervisorIsAvailable")
(setf (get pal_supervisor_msgs::supervisorIsAvailable :request) pal_supervisor_msgs::supervisorIsAvailableRequest)
(setf (get pal_supervisor_msgs::supervisorIsAvailable :response) pal_supervisor_msgs::supervisorIsAvailableResponse)

(defmethod pal_supervisor_msgs::supervisorIsAvailableRequest
  (:response () (instance pal_supervisor_msgs::supervisorIsAvailableResponse :init)))

(setf (get pal_supervisor_msgs::supervisorIsAvailableRequest :md5sum-) "efb9ac7739f299decb8575015d81760c")
(setf (get pal_supervisor_msgs::supervisorIsAvailableRequest :datatype-) "pal_supervisor_msgs/supervisorIsAvailableRequest")
(setf (get pal_supervisor_msgs::supervisorIsAvailableRequest :definition-)
      "---
bool available #True if the Supervisor is available, false if it's not

")

(setf (get pal_supervisor_msgs::supervisorIsAvailableResponse :md5sum-) "efb9ac7739f299decb8575015d81760c")
(setf (get pal_supervisor_msgs::supervisorIsAvailableResponse :datatype-) "pal_supervisor_msgs/supervisorIsAvailableResponse")
(setf (get pal_supervisor_msgs::supervisorIsAvailableResponse :definition-)
      "---
bool available #True if the Supervisor is available, false if it's not

")



(provide :pal_supervisor_msgs/supervisorIsAvailable "efb9ac7739f299decb8575015d81760c")


