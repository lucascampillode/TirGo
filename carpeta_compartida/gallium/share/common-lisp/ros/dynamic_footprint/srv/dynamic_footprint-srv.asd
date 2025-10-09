
(cl:in-package :asdf)

(defsystem "dynamic_footprint-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChangeParams" :depends-on ("_package_ChangeParams"))
    (:file "_package_ChangeParams" :depends-on ("_package"))
  ))