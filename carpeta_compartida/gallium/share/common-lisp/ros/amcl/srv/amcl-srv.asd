
(cl:in-package :asdf)

(defsystem "amcl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Randomize" :depends-on ("_package_Randomize"))
    (:file "_package_Randomize" :depends-on ("_package"))
    (:file "SetInitialPose" :depends-on ("_package_SetInitialPose"))
    (:file "_package_SetInitialPose" :depends-on ("_package"))
  ))