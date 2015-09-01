
(cl:in-package :asdf)

(defsystem "robotino_local_move-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LocalMoveActionFeedback" :depends-on ("_package_LocalMoveActionFeedback"))
    (:file "_package_LocalMoveActionFeedback" :depends-on ("_package"))
    (:file "LocalMoveActionResult" :depends-on ("_package_LocalMoveActionResult"))
    (:file "_package_LocalMoveActionResult" :depends-on ("_package"))
    (:file "LocalMoveAction" :depends-on ("_package_LocalMoveAction"))
    (:file "_package_LocalMoveAction" :depends-on ("_package"))
    (:file "LocalMoveGoal" :depends-on ("_package_LocalMoveGoal"))
    (:file "_package_LocalMoveGoal" :depends-on ("_package"))
    (:file "PathDisplacements" :depends-on ("_package_PathDisplacements"))
    (:file "_package_PathDisplacements" :depends-on ("_package"))
    (:file "LocalMoveResult" :depends-on ("_package_LocalMoveResult"))
    (:file "_package_LocalMoveResult" :depends-on ("_package"))
    (:file "LocalMoveFeedback" :depends-on ("_package_LocalMoveFeedback"))
    (:file "_package_LocalMoveFeedback" :depends-on ("_package"))
    (:file "LocalMoveActionGoal" :depends-on ("_package_LocalMoveActionGoal"))
    (:file "_package_LocalMoveActionGoal" :depends-on ("_package"))
    (:file "PathStatus" :depends-on ("_package_PathStatus"))
    (:file "_package_PathStatus" :depends-on ("_package"))
  ))