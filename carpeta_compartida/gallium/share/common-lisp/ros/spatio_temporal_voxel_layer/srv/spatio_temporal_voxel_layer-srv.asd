
(cl:in-package :asdf)

(defsystem "spatio_temporal_voxel_layer-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SaveGrid" :depends-on ("_package_SaveGrid"))
    (:file "_package_SaveGrid" :depends-on ("_package"))
  ))