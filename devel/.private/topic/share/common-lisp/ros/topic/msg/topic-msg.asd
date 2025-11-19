
(cl:in-package :asdf)

(defsystem "topic-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "U_control_vector" :depends-on ("_package_U_control_vector"))
    (:file "_package_U_control_vector" :depends-on ("_package"))
  ))