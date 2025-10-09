
(cl:in-package :asdf)

(defsystem "pal_zoi_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CurrentZoI" :depends-on ("_package_CurrentZoI"))
    (:file "_package_CurrentZoI" :depends-on ("_package"))
  ))