;; Auto-generated. Do not edit!


(when (boundp 'pal_supervisor_msgs::tcpIpIsAvailable)
  (if (not (find-package "PAL_SUPERVISOR_MSGS"))
    (make-package "PAL_SUPERVISOR_MSGS"))
  (shadow 'tcpIpIsAvailable (find-package "PAL_SUPERVISOR_MSGS")))
(unless (find-package "PAL_SUPERVISOR_MSGS::TCPIPISAVAILABLE")
  (make-package "PAL_SUPERVISOR_MSGS::TCPIPISAVAILABLE"))
(unless (find-package "PAL_SUPERVISOR_MSGS::TCPIPISAVAILABLEREQUEST")
  (make-package "PAL_SUPERVISOR_MSGS::TCPIPISAVAILABLEREQUEST"))
(unless (find-package "PAL_SUPERVISOR_MSGS::TCPIPISAVAILABLERESPONSE")
  (make-package "PAL_SUPERVISOR_MSGS::TCPIPISAVAILABLERESPONSE"))

(in-package "ROS")





(defclass pal_supervisor_msgs::tcpIpIsAvailableRequest
  :super ros::object
  :slots (_ipAddress _port ))

(defmethod pal_supervisor_msgs::tcpIpIsAvailableRequest
  (:init
   (&key
    ((:ipAddress __ipAddress) "")
    ((:port __port) 0)
    )
   (send-super :init)
   (setq _ipAddress (string __ipAddress))
   (setq _port (round __port))
   self)
  (:ipAddress
   (&optional __ipAddress)
   (if __ipAddress (setq _ipAddress __ipAddress)) _ipAddress)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:serialization-length
   ()
   (+
    ;; string _ipAddress
    4 (length _ipAddress)
    ;; int32 _port
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ipAddress
       (write-long (length _ipAddress) s) (princ _ipAddress s)
     ;; int32 _port
       (write-long _port s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ipAddress
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ipAddress (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::tcpIpIsAvailableResponse
  :super ros::object
  :slots (_available _suggestedSleepTime ))

(defmethod pal_supervisor_msgs::tcpIpIsAvailableResponse
  (:init
   (&key
    ((:available __available) nil)
    ((:suggestedSleepTime __suggestedSleepTime) 0)
    )
   (send-super :init)
   (setq _available __available)
   (setq _suggestedSleepTime (round __suggestedSleepTime))
   self)
  (:available
   (&optional (__available :null))
   (if (not (eq __available :null)) (setq _available __available)) _available)
  (:suggestedSleepTime
   (&optional __suggestedSleepTime)
   (if __suggestedSleepTime (setq _suggestedSleepTime __suggestedSleepTime)) _suggestedSleepTime)
  (:serialization-length
   ()
   (+
    ;; bool _available
    1
    ;; uint32 _suggestedSleepTime
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _available
       (if _available (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _suggestedSleepTime
       (write-long _suggestedSleepTime s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _available
     (setq _available (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _suggestedSleepTime
     (setq _suggestedSleepTime (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pal_supervisor_msgs::tcpIpIsAvailable
  :super ros::object
  :slots ())

(setf (get pal_supervisor_msgs::tcpIpIsAvailable :md5sum-) "c87e9a2a51d895bccdb0157125a69381")
(setf (get pal_supervisor_msgs::tcpIpIsAvailable :datatype-) "pal_supervisor_msgs/tcpIpIsAvailable")
(setf (get pal_supervisor_msgs::tcpIpIsAvailable :request) pal_supervisor_msgs::tcpIpIsAvailableRequest)
(setf (get pal_supervisor_msgs::tcpIpIsAvailable :response) pal_supervisor_msgs::tcpIpIsAvailableResponse)

(defmethod pal_supervisor_msgs::tcpIpIsAvailableRequest
  (:response () (instance pal_supervisor_msgs::tcpIpIsAvailableResponse :init)))

(setf (get pal_supervisor_msgs::tcpIpIsAvailableRequest :md5sum-) "c87e9a2a51d895bccdb0157125a69381")
(setf (get pal_supervisor_msgs::tcpIpIsAvailableRequest :datatype-) "pal_supervisor_msgs/tcpIpIsAvailableRequest")
(setf (get pal_supervisor_msgs::tcpIpIsAvailableRequest :definition-)
      "string ipAddress #Ip of the TCP Server
int32 port #Port to use
---
bool available #True if the TCP Server is available, false if it's not
uint32 suggestedSleepTime #Ms suggested to the client to sleep before trying again

")

(setf (get pal_supervisor_msgs::tcpIpIsAvailableResponse :md5sum-) "c87e9a2a51d895bccdb0157125a69381")
(setf (get pal_supervisor_msgs::tcpIpIsAvailableResponse :datatype-) "pal_supervisor_msgs/tcpIpIsAvailableResponse")
(setf (get pal_supervisor_msgs::tcpIpIsAvailableResponse :definition-)
      "string ipAddress #Ip of the TCP Server
int32 port #Port to use
---
bool available #True if the TCP Server is available, false if it's not
uint32 suggestedSleepTime #Ms suggested to the client to sleep before trying again

")



(provide :pal_supervisor_msgs/tcpIpIsAvailable "c87e9a2a51d895bccdb0157125a69381")


