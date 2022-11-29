
(cl:in-package :asdf)

(defsystem "statistics_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Stats1D" :depends-on ("_package_Stats1D"))
    (:file "_package_Stats1D" :depends-on ("_package"))
  ))