; Auto-generated. Do not edit!


(cl:in-package wfov_camera_msgs-msg)


;//! \htmlinclude WFOVTrigger.msg.html

(cl:defclass <WFOVTrigger> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (time_reference
    :reader time_reference
    :initarg :time_reference
    :type cl:string
    :initform "")
   (trigger_time
    :reader trigger_time
    :initarg :trigger_time
    :type cl:real
    :initform 0)
   (trigger_time_reference
    :reader trigger_time_reference
    :initarg :trigger_time_reference
    :type cl:string
    :initform "")
   (shutter
    :reader shutter
    :initarg :shutter
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (trigger_seq
    :reader trigger_seq
    :initarg :trigger_seq
    :type cl:integer
    :initform 0))
)

(cl:defclass WFOVTrigger (<WFOVTrigger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WFOVTrigger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WFOVTrigger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wfov_camera_msgs-msg:<WFOVTrigger> is deprecated: use wfov_camera_msgs-msg:WFOVTrigger instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:header-val is deprecated.  Use wfov_camera_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'time_reference-val :lambda-list '(m))
(cl:defmethod time_reference-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:time_reference-val is deprecated.  Use wfov_camera_msgs-msg:time_reference instead.")
  (time_reference m))

(cl:ensure-generic-function 'trigger_time-val :lambda-list '(m))
(cl:defmethod trigger_time-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:trigger_time-val is deprecated.  Use wfov_camera_msgs-msg:trigger_time instead.")
  (trigger_time m))

(cl:ensure-generic-function 'trigger_time_reference-val :lambda-list '(m))
(cl:defmethod trigger_time_reference-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:trigger_time_reference-val is deprecated.  Use wfov_camera_msgs-msg:trigger_time_reference instead.")
  (trigger_time_reference m))

(cl:ensure-generic-function 'shutter-val :lambda-list '(m))
(cl:defmethod shutter-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:shutter-val is deprecated.  Use wfov_camera_msgs-msg:shutter instead.")
  (shutter m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:id-val is deprecated.  Use wfov_camera_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'trigger_seq-val :lambda-list '(m))
(cl:defmethod trigger_seq-val ((m <WFOVTrigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wfov_camera_msgs-msg:trigger_seq-val is deprecated.  Use wfov_camera_msgs-msg:trigger_seq instead.")
  (trigger_seq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WFOVTrigger>) ostream)
  "Serializes a message object of type '<WFOVTrigger>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'time_reference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'time_reference))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'trigger_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'trigger_time) (cl:floor (cl:slot-value msg 'trigger_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'trigger_time_reference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'trigger_time_reference))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shutter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shutter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'shutter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'shutter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trigger_seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trigger_seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'trigger_seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'trigger_seq)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WFOVTrigger>) istream)
  "Deserializes a message object of type '<WFOVTrigger>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_reference) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'time_reference) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trigger_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trigger_time_reference) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'trigger_time_reference) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shutter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shutter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'shutter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'shutter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trigger_seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'trigger_seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'trigger_seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'trigger_seq)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WFOVTrigger>)))
  "Returns string type for a message object of type '<WFOVTrigger>"
  "wfov_camera_msgs/WFOVTrigger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WFOVTrigger)))
  "Returns string type for a message object of type 'WFOVTrigger"
  "wfov_camera_msgs/WFOVTrigger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WFOVTrigger>)))
  "Returns md5sum for a message object of type '<WFOVTrigger>"
  "e38c040150f1be3148468f6b9974f8bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WFOVTrigger)))
  "Returns md5sum for a message object of type 'WFOVTrigger"
  "e38c040150f1be3148468f6b9974f8bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WFOVTrigger>)))
  "Returns full string definition for message of type '<WFOVTrigger>"
  (cl:format cl:nil "# Message for the time synchronization packets for the WFOV cameras.~%~%Header header			# stamp is the time at which the trigger was received.~%~%string time_reference		# The name of the reference clock for this message's timestamp.~%~%time trigger_time		# The trigger time at which the data was valid.~%~%string trigger_time_reference	# The name of the reference clock for this message's trigger timestamp.~%~%uint32 shutter			# Time duration (microseconds) the shutter was open.~%~%uint32 id			# Camera ID Number~%~%uint32 trigger_seq		# Number of triggers sent to this camera ID number~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WFOVTrigger)))
  "Returns full string definition for message of type 'WFOVTrigger"
  (cl:format cl:nil "# Message for the time synchronization packets for the WFOV cameras.~%~%Header header			# stamp is the time at which the trigger was received.~%~%string time_reference		# The name of the reference clock for this message's timestamp.~%~%time trigger_time		# The trigger time at which the data was valid.~%~%string trigger_time_reference	# The name of the reference clock for this message's trigger timestamp.~%~%uint32 shutter			# Time duration (microseconds) the shutter was open.~%~%uint32 id			# Camera ID Number~%~%uint32 trigger_seq		# Number of triggers sent to this camera ID number~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WFOVTrigger>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'time_reference))
     8
     4 (cl:length (cl:slot-value msg 'trigger_time_reference))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WFOVTrigger>))
  "Converts a ROS message object to a list"
  (cl:list 'WFOVTrigger
    (cl:cons ':header (header msg))
    (cl:cons ':time_reference (time_reference msg))
    (cl:cons ':trigger_time (trigger_time msg))
    (cl:cons ':trigger_time_reference (trigger_time_reference msg))
    (cl:cons ':shutter (shutter msg))
    (cl:cons ':id (id msg))
    (cl:cons ':trigger_seq (trigger_seq msg))
))
