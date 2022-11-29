;; Auto-generated. Do not edit!


(when (boundp 'image_exposure_msgs::ExposureSequence)
  (if (not (find-package "IMAGE_EXPOSURE_MSGS"))
    (make-package "IMAGE_EXPOSURE_MSGS"))
  (shadow 'ExposureSequence (find-package "IMAGE_EXPOSURE_MSGS")))
(unless (find-package "IMAGE_EXPOSURE_MSGS::EXPOSURESEQUENCE")
  (make-package "IMAGE_EXPOSURE_MSGS::EXPOSURESEQUENCE"))

(in-package "ROS")
;;//! \htmlinclude ExposureSequence.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass image_exposure_msgs::ExposureSequence
  :super ros::object
  :slots (_header _shutter _gain _white_balance_blue _white_balance_red ))

(defmethod image_exposure_msgs::ExposureSequence
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:shutter __shutter) (make-array 0 :initial-element 0 :element-type :integer))
    ((:gain __gain) 0.0)
    ((:white_balance_blue __white_balance_blue) 0)
    ((:white_balance_red __white_balance_red) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _shutter __shutter)
   (setq _gain (float __gain))
   (setq _white_balance_blue (round __white_balance_blue))
   (setq _white_balance_red (round __white_balance_red))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:shutter
   (&optional __shutter)
   (if __shutter (setq _shutter __shutter)) _shutter)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:white_balance_blue
   (&optional __white_balance_blue)
   (if __white_balance_blue (setq _white_balance_blue __white_balance_blue)) _white_balance_blue)
  (:white_balance_red
   (&optional __white_balance_red)
   (if __white_balance_red (setq _white_balance_red __white_balance_red)) _white_balance_red)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32[] _shutter
    (* 4    (length _shutter)) 4
    ;; float32 _gain
    4
    ;; uint16 _white_balance_blue
    2
    ;; uint16 _white_balance_red
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32[] _shutter
     (write-long (length _shutter) s)
     (dotimes (i (length _shutter))
       (write-long (elt _shutter i) s)
       )
     ;; float32 _gain
       (sys::poke _gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _white_balance_blue
       (write-word _white_balance_blue s)
     ;; uint16 _white_balance_red
       (write-word _white_balance_red s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32[] _shutter
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _shutter (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _shutter i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32 _gain
     (setq _gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _white_balance_blue
     (setq _white_balance_blue (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _white_balance_red
     (setq _white_balance_red (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get image_exposure_msgs::ExposureSequence :md5sum-) "81d98e1e20eab8beb4bd07beeba6a398")
(setf (get image_exposure_msgs::ExposureSequence :datatype-) "image_exposure_msgs/ExposureSequence")
(setf (get image_exposure_msgs::ExposureSequence :definition-)
      "# exposure sequence (for mult-exposure HDR)
Header header
uint32[]  shutter    	     # Time duration (microseconds) the shutter was open.
float32 gain                 # Gain (in dB) applied to the images in the sequence.
uint16 white_balance_blue    # White balance in blue.
uint16 white_balance_red     # White balance in red.


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



(provide :image_exposure_msgs/ExposureSequence "81d98e1e20eab8beb4bd07beeba6a398")


