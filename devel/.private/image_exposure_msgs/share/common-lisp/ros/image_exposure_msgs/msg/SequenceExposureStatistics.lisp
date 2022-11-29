; Auto-generated. Do not edit!


(cl:in-package image_exposure_msgs-msg)


;//! \htmlinclude SequenceExposureStatistics.msg.html

(cl:defclass <SequenceExposureStatistics> (roslisp-msg-protocol:ros-message)
  ((exposures
    :reader exposures
    :initarg :exposures
    :type (cl:vector image_exposure_msgs-msg:ImageExposureStatistics)
   :initform (cl:make-array 0 :element-type 'image_exposure_msgs-msg:ImageExposureStatistics :initial-element (cl:make-instance 'image_exposure_msgs-msg:ImageExposureStatistics)))
   (underExposed
    :reader underExposed
    :initarg :underExposed
    :type cl:integer
    :initform 0)
   (overExposed
    :reader overExposed
    :initarg :overExposed
    :type cl:integer
    :initform 0)
   (meanIrradiance
    :reader meanIrradiance
    :initarg :meanIrradiance
    :type cl:float
    :initform 0.0)
   (minMeasurableIrradiance
    :reader minMeasurableIrradiance
    :initarg :minMeasurableIrradiance
    :type cl:float
    :initform 0.0)
   (maxMeasurableIrradiance
    :reader maxMeasurableIrradiance
    :initarg :maxMeasurableIrradiance
    :type cl:float
    :initform 0.0)
   (minObservedIrradiance
    :reader minObservedIrradiance
    :initarg :minObservedIrradiance
    :type cl:float
    :initform 0.0)
   (maxObservedIrradiance
    :reader maxObservedIrradiance
    :initarg :maxObservedIrradiance
    :type cl:float
    :initform 0.0))
)

(cl:defclass SequenceExposureStatistics (<SequenceExposureStatistics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SequenceExposureStatistics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SequenceExposureStatistics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_exposure_msgs-msg:<SequenceExposureStatistics> is deprecated: use image_exposure_msgs-msg:SequenceExposureStatistics instead.")))

(cl:ensure-generic-function 'exposures-val :lambda-list '(m))
(cl:defmethod exposures-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:exposures-val is deprecated.  Use image_exposure_msgs-msg:exposures instead.")
  (exposures m))

(cl:ensure-generic-function 'underExposed-val :lambda-list '(m))
(cl:defmethod underExposed-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:underExposed-val is deprecated.  Use image_exposure_msgs-msg:underExposed instead.")
  (underExposed m))

(cl:ensure-generic-function 'overExposed-val :lambda-list '(m))
(cl:defmethod overExposed-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:overExposed-val is deprecated.  Use image_exposure_msgs-msg:overExposed instead.")
  (overExposed m))

(cl:ensure-generic-function 'meanIrradiance-val :lambda-list '(m))
(cl:defmethod meanIrradiance-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:meanIrradiance-val is deprecated.  Use image_exposure_msgs-msg:meanIrradiance instead.")
  (meanIrradiance m))

(cl:ensure-generic-function 'minMeasurableIrradiance-val :lambda-list '(m))
(cl:defmethod minMeasurableIrradiance-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:minMeasurableIrradiance-val is deprecated.  Use image_exposure_msgs-msg:minMeasurableIrradiance instead.")
  (minMeasurableIrradiance m))

(cl:ensure-generic-function 'maxMeasurableIrradiance-val :lambda-list '(m))
(cl:defmethod maxMeasurableIrradiance-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:maxMeasurableIrradiance-val is deprecated.  Use image_exposure_msgs-msg:maxMeasurableIrradiance instead.")
  (maxMeasurableIrradiance m))

(cl:ensure-generic-function 'minObservedIrradiance-val :lambda-list '(m))
(cl:defmethod minObservedIrradiance-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:minObservedIrradiance-val is deprecated.  Use image_exposure_msgs-msg:minObservedIrradiance instead.")
  (minObservedIrradiance m))

(cl:ensure-generic-function 'maxObservedIrradiance-val :lambda-list '(m))
(cl:defmethod maxObservedIrradiance-val ((m <SequenceExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:maxObservedIrradiance-val is deprecated.  Use image_exposure_msgs-msg:maxObservedIrradiance instead.")
  (maxObservedIrradiance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SequenceExposureStatistics>) ostream)
  "Serializes a message object of type '<SequenceExposureStatistics>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'exposures))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'exposures))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'overExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'overExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'overExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'overExposed)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'meanIrradiance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minMeasurableIrradiance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxMeasurableIrradiance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minObservedIrradiance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxObservedIrradiance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SequenceExposureStatistics>) istream)
  "Deserializes a message object of type '<SequenceExposureStatistics>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'exposures) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'exposures)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'image_exposure_msgs-msg:ImageExposureStatistics))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'meanIrradiance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minMeasurableIrradiance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxMeasurableIrradiance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minObservedIrradiance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxObservedIrradiance) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SequenceExposureStatistics>)))
  "Returns string type for a message object of type '<SequenceExposureStatistics>"
  "image_exposure_msgs/SequenceExposureStatistics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SequenceExposureStatistics)))
  "Returns string type for a message object of type 'SequenceExposureStatistics"
  "image_exposure_msgs/SequenceExposureStatistics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SequenceExposureStatistics>)))
  "Returns md5sum for a message object of type '<SequenceExposureStatistics>"
  "2a4f3187c50e7b3544984e9e28ce4328")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SequenceExposureStatistics)))
  "Returns md5sum for a message object of type 'SequenceExposureStatistics"
  "2a4f3187c50e7b3544984e9e28ce4328")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SequenceExposureStatistics>)))
  "Returns full string definition for message of type '<SequenceExposureStatistics>"
  (cl:format cl:nil "# exposure statistics for a multi-exposure HDR sequence~%ImageExposureStatistics[] exposures~%uint32 underExposed 		# number of pixels underexposed in longest exposure~%uint32 overExposed  		# number of pixels overexposed in shortest~%float64 meanIrradiance 		# mean irradiance of all properly exposed pixels in all images in the sequence~%float64 minMeasurableIrradiance~%float64 maxMeasurableIrradiance~%float64 minObservedIrradiance~%float64 maxObservedIrradiance~%~%~%================================================================================~%MSG: image_exposure_msgs/ImageExposureStatistics~%# message for exposure statistics reported a single image~%time stamp         # image time stamp~%string time_reference # The name of the reference clock for this message's timestamp.~%float32 shutterms  # shutter durations in ms~%float32 gaindb     # gain in decibels~%# pixel exposure and latency statistics~%uint32 underExposed # number of pixels underexposed~%uint32 overExposed  # number of pixels overexposed~%statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image~%statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames~%# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))~%float64 meanIrradiance~%float64 minMeasurableIrradiance~%float64 maxMeasurableIrradiance~%float64 minObservedIrradiance~%float64 maxObservedIrradiance~%~%~%~%================================================================================~%MSG: statistics_msgs/Stats1D~%# statistics of a 1-D distribution~%float64 min~%float64 max ~%float64 mean ~%float64 variance~%int64 N~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SequenceExposureStatistics)))
  "Returns full string definition for message of type 'SequenceExposureStatistics"
  (cl:format cl:nil "# exposure statistics for a multi-exposure HDR sequence~%ImageExposureStatistics[] exposures~%uint32 underExposed 		# number of pixels underexposed in longest exposure~%uint32 overExposed  		# number of pixels overexposed in shortest~%float64 meanIrradiance 		# mean irradiance of all properly exposed pixels in all images in the sequence~%float64 minMeasurableIrradiance~%float64 maxMeasurableIrradiance~%float64 minObservedIrradiance~%float64 maxObservedIrradiance~%~%~%================================================================================~%MSG: image_exposure_msgs/ImageExposureStatistics~%# message for exposure statistics reported a single image~%time stamp         # image time stamp~%string time_reference # The name of the reference clock for this message's timestamp.~%float32 shutterms  # shutter durations in ms~%float32 gaindb     # gain in decibels~%# pixel exposure and latency statistics~%uint32 underExposed # number of pixels underexposed~%uint32 overExposed  # number of pixels overexposed~%statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image~%statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames~%# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))~%float64 meanIrradiance~%float64 minMeasurableIrradiance~%float64 maxMeasurableIrradiance~%float64 minObservedIrradiance~%float64 maxObservedIrradiance~%~%~%~%================================================================================~%MSG: statistics_msgs/Stats1D~%# statistics of a 1-D distribution~%float64 min~%float64 max ~%float64 mean ~%float64 variance~%int64 N~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SequenceExposureStatistics>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exposures) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
     4
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SequenceExposureStatistics>))
  "Converts a ROS message object to a list"
  (cl:list 'SequenceExposureStatistics
    (cl:cons ':exposures (exposures msg))
    (cl:cons ':underExposed (underExposed msg))
    (cl:cons ':overExposed (overExposed msg))
    (cl:cons ':meanIrradiance (meanIrradiance msg))
    (cl:cons ':minMeasurableIrradiance (minMeasurableIrradiance msg))
    (cl:cons ':maxMeasurableIrradiance (maxMeasurableIrradiance msg))
    (cl:cons ':minObservedIrradiance (minObservedIrradiance msg))
    (cl:cons ':maxObservedIrradiance (maxObservedIrradiance msg))
))
