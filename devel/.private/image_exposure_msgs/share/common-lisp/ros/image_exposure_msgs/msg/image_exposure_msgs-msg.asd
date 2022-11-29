
(cl:in-package :asdf)

(defsystem "image_exposure_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :statistics_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ExposureSequence" :depends-on ("_package_ExposureSequence"))
    (:file "_package_ExposureSequence" :depends-on ("_package"))
    (:file "ImageExposureStatistics" :depends-on ("_package_ImageExposureStatistics"))
    (:file "_package_ImageExposureStatistics" :depends-on ("_package"))
    (:file "SequenceExposureStatistics" :depends-on ("_package_SequenceExposureStatistics"))
    (:file "_package_SequenceExposureStatistics" :depends-on ("_package"))
  ))