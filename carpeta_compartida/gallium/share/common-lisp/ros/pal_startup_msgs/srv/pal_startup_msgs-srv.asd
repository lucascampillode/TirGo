
(cl:in-package :asdf)

(defsystem "pal_startup_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StartupGetLog" :depends-on ("_package_StartupGetLog"))
    (:file "_package_StartupGetLog" :depends-on ("_package"))
    (:file "StartupGetLogFile" :depends-on ("_package_StartupGetLogFile"))
    (:file "_package_StartupGetLogFile" :depends-on ("_package"))
    (:file "StartupStart" :depends-on ("_package_StartupStart"))
    (:file "_package_StartupStart" :depends-on ("_package"))
    (:file "StartupStop" :depends-on ("_package_StartupStop"))
    (:file "_package_StartupStop" :depends-on ("_package"))
  ))