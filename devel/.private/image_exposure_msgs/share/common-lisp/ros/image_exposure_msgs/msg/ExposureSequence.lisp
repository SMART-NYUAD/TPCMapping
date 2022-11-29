; Auto-generated. Do not edit!


(cl:in-package image_exposure_msgs-msg)


;//! \htmlinclude ExposureSequence.msg.html

(cl:defclass <ExposureSequence> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (shutter
    :reader shutter
    :initarg :shutter
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (gain
    :reader gain
    :initarg :gain
    :type cl:float
    :initform 0.0)
   (white_balance_blue
    :reader white_balance_blue
    :initarg :white_balance_blue
    :type cl:fixnum
    :initform 0)
   (white_balance_red
    :reader white_balance_red
    :initarg :white_balance_red
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ExposureSequence (<ExposureSequence>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExposureSequence>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExposureSequence)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_exposure_msgs-msg:<ExposureSequence> is deprecated: use image_exposure_msgs-msg:ExposureSequence instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ExposureSequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:header-val is deprecated.  Use image_exposure_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'shutter-val :lambda-list '(m))
(cl:defmethod shutter-val ((m <ExposureSequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:shutter-val is deprecated.  Use image_exposure_msgs-msg:shutter instead.")
  (shutter m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <ExposureSequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:gain-val is deprecated.  Use image_exposure_msgs-msg:gain instead.")
  (gain m))

(cl:ensure-generic-function 'white_balance_blue-val :lambda-list '(m))
(cl:defmethod white_balance_blue-val ((m <ExposureSequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:white_balance_blue-val is deprecated.  Use image_exposure_msgs-msg:white_balance_blue instead.")
  (white_balance_blue m))

(cl:ensure-generic-function 'white_balance_red-val :lambda-list '(m))
(cl:defmethod white_balance_red-val ((m <ExposureSequence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_exposure_msgs-msg:white_balance_red-val is deprecated.  Use image_exposure_msgs-msg:white_balance_red instead.")
  (white_balance_red m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExposureSequence>) ostream)
  "Serializes a message object of type '<ExposureSequence>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'shutter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'shutter))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'white_balance_blue)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'white_balance_blue)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'white_balance_red)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'white_balance_red)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExposureSequence>) istream)
  "Deserializes a message object of type '<ExposureSequence>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'shutter) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'shutter)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'white_balance_blue)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'white_balance_blue)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'white_balance_red)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'white_balance_red)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExposureSequence>)))
  "Returns string type for a message object of type '<ExposureSequence>"
  "image_exposure_msgs/ExposureSequence")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExposureSequence)))
  "Returns string type for a message object of type 'ExposureSequence"
  "image_exposure_msgs/ExposureSequence")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExposureSequence>)))
  "Returns md5sum for a message object of type '<ExposureSequence>"
  "81d98e1e20eab8beb4bd07beeba6a398")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExposureSequence)))
  "Returns md5sum for a message object of type 'ExposureSequence"
  "81d98e1e20eab8beb4bd07beeba6a398")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExposureSequence>)))
  "Returns full string definition for message of type '<ExposureSequence>"
  (cl:format cl:nil "# exposure sequence (for mult-exposure HDR)~%Header header~%uint32[]  shutter    	     # Time duration (microseconds) the shutter was open.~%float32 gain                 # Gain (in dB) applied to the images in the sequence.~%uint16 white_balance_blue    # White balance in blue.~%uint16 white_balance_red     # White balance in red.~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExposureSequence)))
  "Returns full string definition for message of type 'ExposureSequence"
  (cl:format cl:nil "# exposure sequence (for mult-exposure HDR)~%Header header~%uint32[]  shutter    	     # Time duration (microseconds) the shutter was open.~%float32 gain                 # Gain (in dB) applied to the images in the sequence.~%uint16 white_balance_blue    # White balance in blue.~%uint16 white_balance_red     # White balance in red.~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExposureSequence>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'shutter) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExposureSequence>))
  "Converts a ROS message object to a list"
  (cl:list 'ExposureSequence
    (cl:cons ':header (header msg))
    (cl:cons ':shutter (shutter msg))
    (cl:cons ':gain (gain msg))
    (cl:cons ':white_balance_blue (white_balance_blue msg))
    (cl:cons ':white_balance_red (white_balance_red msg))
))
