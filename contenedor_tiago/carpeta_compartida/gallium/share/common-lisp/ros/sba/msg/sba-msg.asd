
(cl:in-package :asdf)

(defsystem "sba-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CameraNode" :depends-on ("_package_CameraNode"))
    (:file "_package_CameraNode" :depends-on ("_package"))
    (:file "Frame" :depends-on ("_package_Frame"))
    (:file "_package_Frame" :depends-on ("_package"))
    (:file "Projection" :depends-on ("_package_Projection"))
    (:file "_package_Projection" :depends-on ("_package"))
    (:file "WorldPoint" :depends-on ("_package_WorldPoint"))
    (:file "_package_WorldPoint" :depends-on ("_package"))
  ))