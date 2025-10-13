;; Auto-generated. Do not edit!


(when (boundp 'pal_moveit_plugin_msgs::LoadCapabilities)
  (if (not (find-package "PAL_MOVEIT_PLUGIN_MSGS"))
    (make-package "PAL_MOVEIT_PLUGIN_MSGS"))
  (shadow 'LoadCapabilities (find-package "PAL_MOVEIT_PLUGIN_MSGS")))
(unless (find-package "PAL_MOVEIT_PLUGIN_MSGS::LOADCAPABILITIES")
  (make-package "PAL_MOVEIT_PLUGIN_MSGS::LOADCAPABILITIES"))
(unless (find-package "PAL_MOVEIT_PLUGIN_MSGS::LOADCAPABILITIESREQUEST")
  (make-package "PAL_MOVEIT_PLUGIN_MSGS::LOADCAPABILITIESREQUEST"))
(unless (find-package "PAL_MOVEIT_PLUGIN_MSGS::LOADCAPABILITIESRESPONSE")
  (make-package "PAL_MOVEIT_PLUGIN_MSGS::LOADCAPABILITIESRESPONSE"))

(in-package "ROS")





(defclass pal_moveit_plugin_msgs::LoadCapabilitiesRequest
  :super ros::object
  :slots (_capabilities ))

(defmethod pal_moveit_plugin_msgs::LoadCapabilitiesRequest
  (:init
   (&key
    ((:capabilities __capabilities) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _capabilities __capabilities)
   self)
  (:capabilities
   (&optional __capabilities)
   (if __capabilities (setq _capabilities __capabilities)) _capabilities)
  (:serialization-length
   ()
   (+
    ;; string[] _capabilities
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _capabilities)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _capabilities
     (write-long (length _capabilities) s)
     (dolist (elem _capabilities)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _capabilities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _capabilities (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _capabilities i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass pal_moveit_plugin_msgs::LoadCapabilitiesResponse
  :super ros::object
  :slots (_success _error ))

(defmethod pal_moveit_plugin_msgs::LoadCapabilitiesResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error __error) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _error (string __error))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _error
    4 (length _error)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _error
       (write-long (length _error) s) (princ _error s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _error
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pal_moveit_plugin_msgs::LoadCapabilities
  :super ros::object
  :slots ())

(setf (get pal_moveit_plugin_msgs::LoadCapabilities :md5sum-) "c6f702db26dbc58fc6ddc660a0bd1ac9")
(setf (get pal_moveit_plugin_msgs::LoadCapabilities :datatype-) "pal_moveit_plugin_msgs/LoadCapabilities")
(setf (get pal_moveit_plugin_msgs::LoadCapabilities :request) pal_moveit_plugin_msgs::LoadCapabilitiesRequest)
(setf (get pal_moveit_plugin_msgs::LoadCapabilities :response) pal_moveit_plugin_msgs::LoadCapabilitiesResponse)

(defmethod pal_moveit_plugin_msgs::LoadCapabilitiesRequest
  (:response () (instance pal_moveit_plugin_msgs::LoadCapabilitiesResponse :init)))

(setf (get pal_moveit_plugin_msgs::LoadCapabilitiesRequest :md5sum-) "c6f702db26dbc58fc6ddc660a0bd1ac9")
(setf (get pal_moveit_plugin_msgs::LoadCapabilitiesRequest :datatype-) "pal_moveit_plugin_msgs/LoadCapabilitiesRequest")
(setf (get pal_moveit_plugin_msgs::LoadCapabilitiesRequest :definition-)
      "# list of capabilities
string[] capabilities
---
bool success
string error

")

(setf (get pal_moveit_plugin_msgs::LoadCapabilitiesResponse :md5sum-) "c6f702db26dbc58fc6ddc660a0bd1ac9")
(setf (get pal_moveit_plugin_msgs::LoadCapabilitiesResponse :datatype-) "pal_moveit_plugin_msgs/LoadCapabilitiesResponse")
(setf (get pal_moveit_plugin_msgs::LoadCapabilitiesResponse :definition-)
      "# list of capabilities
string[] capabilities
---
bool success
string error

")



(provide :pal_moveit_plugin_msgs/LoadCapabilities "c6f702db26dbc58fc6ddc660a0bd1ac9")


