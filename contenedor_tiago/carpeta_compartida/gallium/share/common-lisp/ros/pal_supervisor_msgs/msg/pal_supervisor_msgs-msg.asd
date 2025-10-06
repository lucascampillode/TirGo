
(cl:in-package :asdf)

(defsystem "pal_supervisor_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :diagnostic_msgs-msg
)
  :components ((:file "_package")
    (:file "DependencyStatus" :depends-on ("_package_DependencyStatus"))
    (:file "_package_DependencyStatus" :depends-on ("_package"))
    (:file "Parameter" :depends-on ("_package_Parameter"))
    (:file "_package_Parameter" :depends-on ("_package"))
    (:file "RemoteContinuousTest" :depends-on ("_package_RemoteContinuousTest"))
    (:file "_package_RemoteContinuousTest" :depends-on ("_package"))
    (:file "StartupAction" :depends-on ("_package_StartupAction"))
    (:file "_package_StartupAction" :depends-on ("_package"))
    (:file "StartupStatus" :depends-on ("_package_StartupStatus"))
    (:file "_package_StartupStatus" :depends-on ("_package"))
    (:file "SupervisorCommand" :depends-on ("_package_SupervisorCommand"))
    (:file "_package_SupervisorCommand" :depends-on ("_package"))
  ))