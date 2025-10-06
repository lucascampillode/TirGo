
(cl:in-package :asdf)

(defsystem "pal_karto-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "UseScanMatching" :depends-on ("_package_UseScanMatching"))
    (:file "_package_UseScanMatching" :depends-on ("_package"))
  ))