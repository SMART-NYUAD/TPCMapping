
(cl:in-package :asdf)

(defsystem "ptu_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pan_tilt" :depends-on ("_package_pan_tilt"))
    (:file "_package_pan_tilt" :depends-on ("_package"))
  ))