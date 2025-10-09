
(cl:in-package :asdf)

(defsystem "pal_moveit_plugin_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LoadCapabilities" :depends-on ("_package_LoadCapabilities"))
    (:file "_package_LoadCapabilities" :depends-on ("_package"))
  ))