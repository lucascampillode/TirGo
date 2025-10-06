
(cl:in-package :asdf)

(defsystem "plotjuggler_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DataPoint" :depends-on ("_package_DataPoint"))
    (:file "_package_DataPoint" :depends-on ("_package"))
    (:file "DataPoints" :depends-on ("_package_DataPoints"))
    (:file "_package_DataPoints" :depends-on ("_package"))
    (:file "Dictionary" :depends-on ("_package_Dictionary"))
    (:file "_package_Dictionary" :depends-on ("_package"))
  ))