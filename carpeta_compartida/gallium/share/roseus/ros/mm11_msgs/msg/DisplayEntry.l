;; Auto-generated. Do not edit!


(when (boundp 'mm11_msgs::DisplayEntry)
  (if (not (find-package "MM11_MSGS"))
    (make-package "MM11_MSGS"))
  (shadow 'DisplayEntry (find-package "MM11_MSGS")))
(unless (find-package "MM11_MSGS::DISPLAYENTRY")
  (make-package "MM11_MSGS::DISPLAYENTRY"))

(in-package "ROS")
;;//! \htmlinclude DisplayEntry.msg.html


(intern "*BUTTON*" (find-package "MM11_MSGS::DISPLAYENTRY"))
(shadow '*BUTTON* (find-package "MM11_MSGS::DISPLAYENTRY"))
(defconstant mm11_msgs::DisplayEntry::*BUTTON* 0)
(intern "*LABEL*" (find-package "MM11_MSGS::DISPLAYENTRY"))
(shadow '*LABEL* (find-package "MM11_MSGS::DISPLAYENTRY"))
(defconstant mm11_msgs::DisplayEntry::*LABEL* 1)

(defun mm11_msgs::DisplayEntry-to-symbol (const)
  (cond
        ((= const 0) 'mm11_msgs::DisplayEntry::*BUTTON*)
        ((= const 1) 'mm11_msgs::DisplayEntry::*LABEL*)
        (t nil)))

(defclass mm11_msgs::DisplayEntry
  :super ros::object
  :slots (_type _text ))

(defmethod mm11_msgs::DisplayEntry
  (:init
   (&key
    ((:type __type) 0)
    ((:text __text) "")
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _text (string __text))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get mm11_msgs::DisplayEntry :md5sum-) "e3e7126b9c0f9fa8a31e38b081f54f81")
(setf (get mm11_msgs::DisplayEntry :datatype-) "mm11_msgs/DisplayEntry")
(setf (get mm11_msgs::DisplayEntry :definition-)
      "uint8 BUTTON=0
uint8 LABEL=1

# LABEL or BUTTON
uint8 type
# label or button text
string text

")



(provide :mm11_msgs/DisplayEntry "e3e7126b9c0f9fa8a31e38b081f54f81")


