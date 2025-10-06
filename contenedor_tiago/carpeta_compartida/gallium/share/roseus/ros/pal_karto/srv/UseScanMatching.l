;; Auto-generated. Do not edit!


(when (boundp 'pal_karto::UseScanMatching)
  (if (not (find-package "PAL_KARTO"))
    (make-package "PAL_KARTO"))
  (shadow 'UseScanMatching (find-package "PAL_KARTO")))
(unless (find-package "PAL_KARTO::USESCANMATCHING")
  (make-package "PAL_KARTO::USESCANMATCHING"))
(unless (find-package "PAL_KARTO::USESCANMATCHINGREQUEST")
  (make-package "PAL_KARTO::USESCANMATCHINGREQUEST"))
(unless (find-package "PAL_KARTO::USESCANMATCHINGRESPONSE")
  (make-package "PAL_KARTO::USESCANMATCHINGRESPONSE"))

(in-package "ROS")





(defclass pal_karto::UseScanMatchingRequest
  :super ros::object
  :slots (_value ))

(defmethod pal_karto::UseScanMatchingRequest
  (:init
   (&key
    ((:value __value) nil)
    )
   (send-super :init)
   (setq _value __value)
   self)
  (:value
   (&optional (__value :null))
   (if (not (eq __value :null)) (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; bool _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _value
       (if _value (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _value
     (setq _value (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass pal_karto::UseScanMatchingResponse
  :super ros::object
  :slots ())

(defmethod pal_karto::UseScanMatchingResponse
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

(defclass pal_karto::UseScanMatching
  :super ros::object
  :slots ())

(setf (get pal_karto::UseScanMatching :md5sum-) "e431d687bf4b2c65fbd94b12ae0cb5d9")
(setf (get pal_karto::UseScanMatching :datatype-) "pal_karto/UseScanMatching")
(setf (get pal_karto::UseScanMatching :request) pal_karto::UseScanMatchingRequest)
(setf (get pal_karto::UseScanMatching :response) pal_karto::UseScanMatchingResponse)

(defmethod pal_karto::UseScanMatchingRequest
  (:response () (instance pal_karto::UseScanMatchingResponse :init)))

(setf (get pal_karto::UseScanMatchingRequest :md5sum-) "e431d687bf4b2c65fbd94b12ae0cb5d9")
(setf (get pal_karto::UseScanMatchingRequest :datatype-) "pal_karto/UseScanMatchingRequest")
(setf (get pal_karto::UseScanMatchingRequest :definition-)
      "bool value
---

")

(setf (get pal_karto::UseScanMatchingResponse :md5sum-) "e431d687bf4b2c65fbd94b12ae0cb5d9")
(setf (get pal_karto::UseScanMatchingResponse :datatype-) "pal_karto/UseScanMatchingResponse")
(setf (get pal_karto::UseScanMatchingResponse :definition-)
      "bool value
---

")



(provide :pal_karto/UseScanMatching "e431d687bf4b2c65fbd94b12ae0cb5d9")


