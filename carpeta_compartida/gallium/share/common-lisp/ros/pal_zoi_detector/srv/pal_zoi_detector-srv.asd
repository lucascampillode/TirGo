
(cl:in-package :asdf)

(defsystem "pal_zoi_detector-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :pal_zoi_detector-msg
)
  :components ((:file "_package")
    (:file "GetPointZoI" :depends-on ("_package_GetPointZoI"))
    (:file "_package_GetPointZoI" :depends-on ("_package"))
  ))