;; Auto-generated. Do not edit!


(when (boundp 'pal_local_planner::traffic_manager)
  (if (not (find-package "PAL_LOCAL_PLANNER"))
    (make-package "PAL_LOCAL_PLANNER"))
  (shadow 'traffic_manager (find-package "PAL_LOCAL_PLANNER")))
(unless (find-package "PAL_LOCAL_PLANNER::TRAFFIC_MANAGER")
  (make-package "PAL_LOCAL_PLANNER::TRAFFIC_MANAGER"))
(unless (find-package "PAL_LOCAL_PLANNER::TRAFFIC_MANAGERREQUEST")
  (make-package "PAL_LOCAL_PLANNER::TRAFFIC_MANAGERREQUEST"))
(unless (find-package "PAL_LOCAL_PLANNER::TRAFFIC_MANAGERRESPONSE")
  (make-package "PAL_LOCAL_PLANNER::TRAFFIC_MANAGERRESPONSE"))

(in-package "ROS")





(defclass pal_local_planner::traffic_managerRequest
  :super ros::object
  :slots ())

(defmethod pal_local_planner::traffic_managerRequest
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

(defclass pal_local_planner::traffic_managerResponse
  :super ros::object
  :slots (_situation ))

(defmethod pal_local_planner::traffic_managerResponse
  (:init
   (&key
    ((:situation __situation) 0)
    )
   (send-super :init)
   (setq _situation (round __situation))
   self)
  (:situation
   (&optional __situation)
   (if __situation (setq _situation __situation)) _situation)
  (:serialization-length
   ()
   (+
    ;; int64 _situation
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _situation
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _situation (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _situation) (= (length (_situation . bv)) 2)) ;; bignum
              (write-long (ash (elt (_situation . bv) 0) 0) s)
              (write-long (ash (elt (_situation . bv) 1) -1) s))
             ((and (class _situation) (= (length (_situation . bv)) 1)) ;; big1
              (write-long (elt (_situation . bv) 0) s)
              (write-long (if (>= _situation 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _situation s)(write-long (if (>= _situation 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _situation
#+(or :alpha :irix6 :x86_64)
      (setf _situation (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _situation (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass pal_local_planner::traffic_manager
  :super ros::object
  :slots ())

(setf (get pal_local_planner::traffic_manager :md5sum-) "2687553acfba2efaeee5da06c95db2d1")
(setf (get pal_local_planner::traffic_manager :datatype-) "pal_local_planner/traffic_manager")
(setf (get pal_local_planner::traffic_manager :request) pal_local_planner::traffic_managerRequest)
(setf (get pal_local_planner::traffic_manager :response) pal_local_planner::traffic_managerResponse)

(defmethod pal_local_planner::traffic_managerRequest
  (:response () (instance pal_local_planner::traffic_managerResponse :init)))

(setf (get pal_local_planner::traffic_managerRequest :md5sum-) "2687553acfba2efaeee5da06c95db2d1")
(setf (get pal_local_planner::traffic_managerRequest :datatype-) "pal_local_planner/traffic_managerRequest")
(setf (get pal_local_planner::traffic_managerRequest :definition-)
      "
---
int64 situation

")

(setf (get pal_local_planner::traffic_managerResponse :md5sum-) "2687553acfba2efaeee5da06c95db2d1")
(setf (get pal_local_planner::traffic_managerResponse :datatype-) "pal_local_planner/traffic_managerResponse")
(setf (get pal_local_planner::traffic_managerResponse :definition-)
      "
---
int64 situation

")



(provide :pal_local_planner/traffic_manager "2687553acfba2efaeee5da06c95db2d1")


