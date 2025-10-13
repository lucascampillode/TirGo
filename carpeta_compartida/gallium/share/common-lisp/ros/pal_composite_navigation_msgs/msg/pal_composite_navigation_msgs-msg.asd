
(cl:in-package :asdf)

(defsystem "pal_composite_navigation_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GoToFloorPOIAction" :depends-on ("_package_GoToFloorPOIAction"))
    (:file "_package_GoToFloorPOIAction" :depends-on ("_package"))
    (:file "GoToFloorPOIActionFeedback" :depends-on ("_package_GoToFloorPOIActionFeedback"))
    (:file "_package_GoToFloorPOIActionFeedback" :depends-on ("_package"))
    (:file "GoToFloorPOIActionGoal" :depends-on ("_package_GoToFloorPOIActionGoal"))
    (:file "_package_GoToFloorPOIActionGoal" :depends-on ("_package"))
    (:file "GoToFloorPOIActionResult" :depends-on ("_package_GoToFloorPOIActionResult"))
    (:file "_package_GoToFloorPOIActionResult" :depends-on ("_package"))
    (:file "GoToFloorPOIFeedback" :depends-on ("_package_GoToFloorPOIFeedback"))
    (:file "_package_GoToFloorPOIFeedback" :depends-on ("_package"))
    (:file "GoToFloorPOIGoal" :depends-on ("_package_GoToFloorPOIGoal"))
    (:file "_package_GoToFloorPOIGoal" :depends-on ("_package"))
    (:file "GoToFloorPOIResult" :depends-on ("_package_GoToFloorPOIResult"))
    (:file "_package_GoToFloorPOIResult" :depends-on ("_package"))
  ))