;; Auto-generated. Do not edit!


(when (boundp 'sba::Projection)
  (if (not (find-package "SBA"))
    (make-package "SBA"))
  (shadow 'Projection (find-package "SBA")))
(unless (find-package "SBA::PROJECTION")
  (make-package "SBA::PROJECTION"))

(in-package "ROS")
;;//! \htmlinclude Projection.msg.html


(defclass sba::Projection
  :super ros::object
  :slots (_camindex _pointindex _u _v _d _stereo _usecovariance _covariance ))

(defmethod sba::Projection
  (:init
   (&key
    ((:camindex __camindex) 0)
    ((:pointindex __pointindex) 0)
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    ((:d __d) 0.0)
    ((:stereo __stereo) nil)
    ((:usecovariance __usecovariance) nil)
    ((:covariance __covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _camindex (round __camindex))
   (setq _pointindex (round __pointindex))
   (setq _u (float __u))
   (setq _v (float __v))
   (setq _d (float __d))
   (setq _stereo __stereo)
   (setq _usecovariance __usecovariance)
   (setq _covariance __covariance)
   self)
  (:camindex
   (&optional __camindex)
   (if __camindex (setq _camindex __camindex)) _camindex)
  (:pointindex
   (&optional __pointindex)
   (if __pointindex (setq _pointindex __pointindex)) _pointindex)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:stereo
   (&optional (__stereo :null))
   (if (not (eq __stereo :null)) (setq _stereo __stereo)) _stereo)
  (:usecovariance
   (&optional (__usecovariance :null))
   (if (not (eq __usecovariance :null)) (setq _usecovariance __usecovariance)) _usecovariance)
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; uint32 _camindex
    4
    ;; uint32 _pointindex
    4
    ;; float64 _u
    8
    ;; float64 _v
    8
    ;; float64 _d
    8
    ;; bool _stereo
    1
    ;; bool _usecovariance
    1
    ;; float64[9] _covariance
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _camindex
       (write-long _camindex s)
     ;; uint32 _pointindex
       (write-long _pointindex s)
     ;; float64 _u
       (sys::poke _u (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d
       (sys::poke _d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _stereo
       (if _stereo (write-byte -1 s) (write-byte 0 s))
     ;; bool _usecovariance
       (if _usecovariance (write-byte -1 s) (write-byte 0 s))
     ;; float64[9] _covariance
     (dotimes (i 9)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _camindex
     (setq _camindex (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _pointindex
     (setq _pointindex (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _u
     (setq _u (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d
     (setq _d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _stereo
     (setq _stereo (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _usecovariance
     (setq _usecovariance (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64[9] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get sba::Projection :md5sum-) "3d5a2ac666ca0038812f550185cfc756")
(setf (get sba::Projection :datatype-) "sba/Projection")
(setf (get sba::Projection :definition-)
      "# Projection

# Camera index
uint32 camindex

# Point index
uint32 pointindex

# Projection into the image plane
float64 u
float64 v
float64 d

# Is this a stereo projection? (true if stereo, false if monocular)
bool stereo

# Use a covariance matrix?
bool usecovariance

# A 3x3 covariance matrix describing the error
float64[9] covariance


")



(provide :sba/Projection "3d5a2ac666ca0038812f550185cfc756")


