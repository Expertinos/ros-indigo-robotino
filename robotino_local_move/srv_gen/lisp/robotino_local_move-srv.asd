
(cl:in-package :asdf)

(defsystem "robotino_local_move-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FullPath" :depends-on ("_package_FullPath"))
    (:file "_package_FullPath" :depends-on ("_package"))
    (:file "Stop" :depends-on ("_package_Stop"))
    (:file "_package_Stop" :depends-on ("_package"))
  ))