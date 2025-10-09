
(cl:in-package :asdf)

(defsystem "pal_startup_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StartupStatus" :depends-on ("_package_StartupStatus"))
    (:file "_package_StartupStatus" :depends-on ("_package"))
  ))