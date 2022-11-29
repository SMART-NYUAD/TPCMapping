; Auto-generated. Do not edit!


(cl:in-package image_exposure_msgs-msg)


;//! \htmlinclude ImageExposureStatistics.msg.html

(cl:defclass <ImageExposureStatistics> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (time_reference
    :reader time_reference
    :initarg :time_reference
    :type cl:string
    :initform "")
   (shutterms
    :reader shutterms
    :initarg :shutterms
    :type cl:float
    :initform 0.0)
   (gaindb
    :reader gaindb
    :initarg :gaindb
    :type cl:float
    :initform 0.0)
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
   (pixelVal
    :reader pixelVal
    :initarg :pixelVal
    :type statistics_msgs-msg:Stats1D
    :initform (cl:make-instance 'statistics_msgs-msg:Stats1D))
   (pixelAge
    :reader pixelAge
    :initarg :pixelAge
    :type statistics_msgs-msg:Stats1D
    :initform (cl:make-instance 'statistics_msgs-msg:Stats1D))
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

(cl:defclass ImageExposureStatistics (<ImageExposureStatistics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageExposureStatistics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageExposureStatistics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_exposure_msgs-msg:<ImageExposureStatistics> is deprecated: use image_exposure_msgs-msg:ImageExposureStatistics instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:stamp-val is deprecated.  Use image_exposure_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'time_reference-val :lambda-list '(m))
(cl:defmethod time_reference-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:time_reference-val is deprecated.  Use image_exposure_msgs-msg:time_reference instead.")
  (time_reference m))

(cl:ensure-generic-function 'shutterms-val :lambda-list '(m))
(cl:defmethod shutterms-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:shutterms-val is deprecated.  Use image_exposure_msgs-msg:shutterms instead.")
  (shutterms m))

(cl:ensure-generic-function 'gaindb-val :lambda-list '(m))
(cl:defmethod gaindb-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:gaindb-val is deprecated.  Use image_exposure_msgs-msg:gaindb instead.")
  (gaindb m))

(cl:ensure-generic-function 'underExposed-val :lambda-list '(m))
(cl:defmethod underExposed-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:underExposed-val is deprecated.  Use image_exposure_msgs-msg:underExposed instead.")
  (underExposed m))

(cl:ensure-generic-function 'overExposed-val :lambda-list '(m))
(cl:defmethod overExposed-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:overExposed-val is deprecated.  Use image_exposure_msgs-msg:overExposed instead.")
  (overExposed m))

(cl:ensure-generic-function 'pixelVal-val :lambda-list '(m))
(cl:defmethod pixelVal-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:pixelVal-val is deprecated.  Use image_exposure_msgs-msg:pixelVal instead.")
  (pixelVal m))

(cl:ensure-generic-function 'pixelAge-val :lambda-list '(m))
(cl:defmethod pixelAge-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:pixelAge-val is deprecated.  Use image_exposure_msgs-msg:pixelAge instead.")
  (pixelAge m))

(cl:ensure-generic-function 'meanIrradiance-val :lambda-list '(m))
(cl:defmethod meanIrradiance-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:meanIrradiance-val is deprecated.  Use image_exposure_msgs-msg:meanIrradiance instead.")
  (meanIrradiance m))

(cl:ensure-generic-function 'minMeasurableIrradiance-val :lambda-list '(m))
(cl:defmethod minMeasurableIrradiance-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:minMeasurableIrradiance-val is deprecated.  Use image_exposure_msgs-msg:minMeasurableIrradiance instead.")
  (minMeasurableIrradiance m))

(cl:ensure-generic-function 'maxMeasurableIrradiance-val :lambda-list '(m))
(cl:defmethod maxMeasurableIrradiance-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:maxMeasurableIrradiance-val is deprecated.  Use image_exposure_msgs-msg:maxMeasurableIrradiance instead.")
  (maxMeasurableIrradiance m))

(cl:ensure-generic-function 'minObservedIrradiance-val :lambda-list '(m))
(cl:defmethod minObservedIrradiance-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:minObservedIrradiance-val is deprecated.  Use image_exposure_msgs-msg:minObservedIrradiance instead.")
  (minObservedIrradiance m))

(cl:ensure-generic-function 'maxObservedIrradiance-val :lambda-list '(m))
(cl:defmethod maxObservedIrradiance-val ((m <ImageExposureStatistics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:maxObservedIrradiance-val is deprecated.  Use image_exposure_msgs-msg:maxObservedIrradiance instead.")
  (maxObservedIrradiance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageExposureStatistics>) ostream)
  "Serializes a message object of type '<ImageExposureStatistics>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'time_reference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'time_reference))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shutterms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gaindb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'underExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'overExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'overExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'overExposed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'overExposed)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pixelVal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pixelAge) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageExposureStatistics>) istream)
  "Deserializes a message object of type '<ImageExposureStatistics>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_reference) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'time_reference) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shutterms) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gaindb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'underExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'overExposed)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pixelVal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pixelAge) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageExposureStatistics>)))
  "Returns string type for a message object of type '<ImageExposureStatistics>"
  "image_exposure_msgs/ImageExposureStatistics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageExposureStatistics)))
  "Returns string type for a message object of type 'ImageExposureStatistics"
  "image_exposure_msgs/ImageExposureStatistics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageExposureStatistics>)))
  "Returns md5sum for a message object of type '<ImageExposureStatistics>"
  "334dc170ce6287d1de470f25be78dd9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageExposureStatistics)))
  "Returns md5sum for a message object of type 'ImageExposureStatistics"
  "334dc170ce6287d1de470f25be78dd9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageExposureStatistics>)))
  "Returns full string definition for message of type '<ImageExposureStatistics>"
  (cl:format cl:nil "# message for exposure statistics reported a single image~%time stamp         # image time stamp~%string time_reference # The name of the reference clock for this message's timestamp.~%float32 shutterms  # shutter durations in ms~%float32 gaindb     # gain in decibels~%# pixel exposure and latency statistics~%uint32 underExposed # number of pixels underexposed~%uint32 overExposed  # number of pixels overexposed~%statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image~%statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames~%# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))~%float64 meanIrradiance~%float64 minMeasurableIrradiance~%float64 maxMeasurableIrradiance~%float64 minObservedIrradiance~%float64 maxObservedIrradiance~%~%~%~%================================================================================~%MSG: statistics_msgs/Stats1D~%# statistics of a 1-D distribution~%float64 min~%float64 max ~%float64 mean ~%float64 variance~%int64 N~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageExposureStatistics)))
  "Returns full string definition for message of type 'ImageExposureStatistics"
  (cl:format cl:nil "# message for exposure statistics reported a single image~%time stamp         # image time stamp~%string time_reference # The name of the reference clock for this message's timestamp.~%float32 shutterms  # shutter durations in ms~%float32 gaindb     # gain in decibels~%# pixel exposure and latency statistics~%uint32 underExposed # number of pixels underexposed~%uint32 overExposed  # number of pixels overexposed~%statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image~%statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames~%# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))~%float64 meanIrradiance~%float64 minMeasurableIrradiance~%float64 maxMeasurableIrradiance~%float64 minObservedIrradiance~%float64 maxObservedIrradiance~%~%~%~%================================================================================~%MSG: statistics_msgs/Stats1D~%# statistics of a 1-D distribution~%float64 min~%float64 max ~%float64 mean ~%float64 variance~%int64 N~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageExposureStatistics>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'time_reference))
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pixelVal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pixelAge))
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageExposureStatistics>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageExposureStatistics
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':time_reference (time_reference msg))
    (cl:cons ':shutterms (shutterms msg))
    (cl:cons ':gaindb (gaindb msg))
    (cl:cons ':underExposed (underExposed msg))
    (cl:cons ':overExposed (overExposed msg))
    (cl:cons ':pixelVal (pixelVal msg))
    (cl:cons ':pixelAge (pixelAge msg))
    (cl:cons ':meanIrradiance (meanIrradiance msg))
    (cl:cons ':minMeasurableIrradiance (minMeasurableIrradiance msg))
    (cl:cons ':maxMeasurableIrradiance (maxMeasurableIrradiance msg))
    (cl:cons ':minObservedIrradiance (minObservedIrradiance msg))
    (cl:cons ':maxObservedIrradiance (maxObservedIrradiance msg))
))
