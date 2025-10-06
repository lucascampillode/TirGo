
(cl:in-package :asdf)

(defsystem "pal_pcl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ComputeTransform" :depends-on ("_package_ComputeTransform"))
    (:file "_package_ComputeTransform" :depends-on ("_package"))
    (:file "save" :depends-on ("_package_save"))
    (:file "_package_save" :depends-on ("_package"))
  ))