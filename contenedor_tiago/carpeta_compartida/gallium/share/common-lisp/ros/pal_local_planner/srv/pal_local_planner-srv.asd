
(cl:in-package :asdf)

(defsystem "pal_local_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "traffic_manager" :depends-on ("_package_traffic_manager"))
    (:file "_package_traffic_manager" :depends-on ("_package"))
  ))