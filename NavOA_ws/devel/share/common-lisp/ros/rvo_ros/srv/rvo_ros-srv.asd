
(cl:in-package :asdf)

(defsystem "rvo_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SetGoals" :depends-on ("_package_SetGoals"))
    (:file "_package_SetGoals" :depends-on ("_package"))
  ))