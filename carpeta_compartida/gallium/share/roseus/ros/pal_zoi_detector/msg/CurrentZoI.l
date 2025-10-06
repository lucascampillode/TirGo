;; Auto-generated. Do not edit!


(when (boundp 'pal_zoi_detector::CurrentZoI)
  (if (not (find-package "PAL_ZOI_DETECTOR"))
    (make-package "PAL_ZOI_DETECTOR"))
  (shadow 'CurrentZoI (find-package "PAL_ZOI_DETECTOR")))
(unless (find-package "PAL_ZOI_DETECTOR::CURRENTZOI")
  (make-package "PAL_ZOI_DETECTOR::CURRENTZOI"))

(in-package "ROS")
;;//! \htmlinclude CurrentZoI.msg.html


(defclass pal_zoi_detector::CurrentZoI
  :super ros::object
  :slots (_zois ))

(defmethod pal_zoi_detector::CurrentZoI
  (:init
   (&key
    ((:zois __zois) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _zois __zois)
   self)
  (:zois
   (&optional __zois)
   (if __zois (setq _zois __zois)) _zois)
  (:serialization-length
   ()
   (+
    ;; string[] _zois
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _zois)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _zois
     (write-long (length _zois) s)
     (dolist (elem _zois)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _zois
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _zois (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _zois i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get pal_zoi_detector::CurrentZoI :md5sum-) "901e944bf726cc1afe5ae5dbf3a91620")
(setf (get pal_zoi_detector::CurrentZoI :datatype-) "pal_zoi_detector/CurrentZoI")
(setf (get pal_zoi_detector::CurrentZoI :definition-)
      "string[] zois

")



(provide :pal_zoi_detector/CurrentZoI "901e944bf726cc1afe5ae5dbf3a91620")


