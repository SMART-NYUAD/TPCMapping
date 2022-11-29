
(cl:in-package :asdf)

(defsystem "wfov_camera_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WFOVCompressedImage" :depends-on ("_package_WFOVCompressedImage"))
    (:file "_package_WFOVCompressedImage" :depends-on ("_package"))
    (:file "WFOVImage" :depends-on ("_package_WFOVImage"))
    (:file "_package_WFOVImage" :depends-on ("_package"))
    (:file "WFOVTrigger" :depends-on ("_package_WFOVTrigger"))
    (:file "_package_WFOVTrigger" :depends-on ("_package"))
  ))