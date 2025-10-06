
(cl:in-package :asdf)

(defsystem "pal_waypoint_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DoWaypointNavigationAction" :depends-on ("_package_DoWaypointNavigationAction"))
    (:file "_package_DoWaypointNavigationAction" :depends-on ("_package"))
    (:file "DoWaypointNavigationActionFeedback" :depends-on ("_package_DoWaypointNavigationActionFeedback"))
    (:file "_package_DoWaypointNavigationActionFeedback" :depends-on ("_package"))
    (:file "DoWaypointNavigationActionGoal" :depends-on ("_package_DoWaypointNavigationActionGoal"))
    (:file "_package_DoWaypointNavigationActionGoal" :depends-on ("_package"))
    (:file "DoWaypointNavigationActionResult" :depends-on ("_package_DoWaypointNavigationActionResult"))
    (:file "_package_DoWaypointNavigationActionResult" :depends-on ("_package"))
    (:file "DoWaypointNavigationFeedback" :depends-on ("_package_DoWaypointNavigationFeedback"))
    (:file "_package_DoWaypointNavigationFeedback" :depends-on ("_package"))
    (:file "DoWaypointNavigationGoal" :depends-on ("_package_DoWaypointNavigationGoal"))
    (:file "_package_DoWaypointNavigationGoal" :depends-on ("_package"))
    (:file "DoWaypointNavigationResult" :depends-on ("_package_DoWaypointNavigationResult"))
    (:file "_package_DoWaypointNavigationResult" :depends-on ("_package"))
  ))