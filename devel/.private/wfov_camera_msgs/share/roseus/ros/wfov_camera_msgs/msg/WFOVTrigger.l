;; Auto-generated. Do not edit!


(when (boundp 'wfov_camera_msgs::WFOVTrigger)
  (if (not (find-package "WFOV_CAMERA_MSGS"))
    (make-package "WFOV_CAMERA_MSGS"))
  (shadow 'WFOVTrigger (find-package "WFOV_CAMERA_MSGS")))
(unless (find-package "WFOV_CAMERA_MSGS::WFOVTRIGGER")
  (make-package "WFOV_CAMERA_MSGS::WFOVTRIGGER"))

(in-package "ROS")
;;//! \htmlinclude WFOVTrigger.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wfov_camera_msgs::WFOVTrigger
  :super ros::object
  :slots (_header _time_reference _trigger_time _trigger_time_reference _shutter _id _trigger_seq ))

(defmethod wfov_camera_msgs::WFOVTrigger
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time_reference __time_reference) "")
    ((:trigger_time __trigger_time) (instance ros::time :init))
    ((:trigger_time_reference __trigger_time_reference) "")
    ((:shutter __shutter) 0)
    ((:id __id) 0)
    ((:trigger_seq __trigger_seq) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time_reference (string __time_reference))
   (setq _trigger_time __trigger_time)
   (setq _trigger_time_reference (string __trigger_time_reference))
   (setq _shutter (round __shutter))
   (setq _id (round __id))
   (setq _trigger_seq (round __trigger_seq))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time_reference
   (&optional __time_reference)
   (if __time_reference (setq _time_reference __time_reference)) _time_reference)
  (:trigger_time
   (&optional __trigger_time)
   (if __trigger_time (setq _trigger_time __trigger_time)) _trigger_time)
  (:trigger_time_reference
   (&optional __trigger_time_reference)
   (if __trigger_time_reference (setq _trigger_time_reference __trigger_time_reference)) _trigger_time_reference)
  (:shutter
   (&optional __shutter)
   (if __shutter (setq _shutter __shutter)) _shutter)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:trigger_seq
   (&optional __trigger_seq)
   (if __trigger_seq (setq _trigger_seq __trigger_seq)) _trigger_seq)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _time_reference
    4 (length _time_reference)
    ;; time _trigger_time
    8
    ;; string _trigger_time_reference
    4 (length _trigger_time_reference)
    ;; uint32 _shutter
    4
    ;; uint32 _id
    4
    ;; uint32 _trigger_seq
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _time_reference
       (write-long (length _time_reference) s) (princ _time_reference s)
     ;; time _trigger_time
       (write-long (send _trigger_time :sec) s) (write-long (send _trigger_time :nsec) s)
     ;; string _trigger_time_reference
       (write-long (length _trigger_time_reference) s) (princ _trigger_time_reference s)
     ;; uint32 _shutter
       (write-long _shutter s)
     ;; uint32 _id
       (write-long _id s)
     ;; uint32 _trigger_seq
       (write-long _trigger_seq s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _time_reference
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _time_reference (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _trigger_time
     (send _trigger_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _trigger_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _trigger_time_reference
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _trigger_time_reference (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _shutter
     (setq _shutter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _trigger_seq
     (setq _trigger_seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wfov_camera_msgs::WFOVTrigger :md5sum-) "e38c040150f1be3148468f6b9974f8bf")
(setf (get wfov_camera_msgs::WFOVTrigger :datatype-) "wfov_camera_msgs/WFOVTrigger")
(setf (get wfov_camera_msgs::WFOVTrigger :definition-)
      "# Message for the time synchronization packets for the WFOV cameras.

Header header			# stamp is the time at which the trigger was received.

string time_reference		# The name of the reference clock for this message's timestamp.

time trigger_time		# The trigger time at which the data was valid.

string trigger_time_reference	# The name of the reference clock for this message's trigger timestamp.

uint32 shutter			# Time duration (microseconds) the shutter was open.

uint32 id			# Camera ID Number

uint32 trigger_seq		# Number of triggers sent to this camera ID number
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :wfov_camera_msgs/WFOVTrigger "e38c040150f1be3148468f6b9974f8bf")


