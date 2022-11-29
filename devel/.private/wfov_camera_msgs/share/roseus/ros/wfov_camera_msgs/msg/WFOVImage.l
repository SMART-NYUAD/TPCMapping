;; Auto-generated. Do not edit!


(when (boundp 'wfov_camera_msgs::WFOVImage)
  (if (not (find-package "WFOV_CAMERA_MSGS"))
    (make-package "WFOV_CAMERA_MSGS"))
  (shadow 'WFOVImage (find-package "WFOV_CAMERA_MSGS")))
(unless (find-package "WFOV_CAMERA_MSGS::WFOVIMAGE")
  (make-package "WFOV_CAMERA_MSGS::WFOVIMAGE"))

(in-package "ROS")
;;//! \htmlinclude WFOVImage.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wfov_camera_msgs::WFOVImage
  :super ros::object
  :slots (_header _time_reference _image _info _shutter _gain _white_balance_blue _white_balance_red _temperature ))

(defmethod wfov_camera_msgs::WFOVImage
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time_reference __time_reference) "")
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:info __info) (instance sensor_msgs::CameraInfo :init))
    ((:shutter __shutter) 0.0)
    ((:gain __gain) 0.0)
    ((:white_balance_blue __white_balance_blue) 0)
    ((:white_balance_red __white_balance_red) 0)
    ((:temperature __temperature) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time_reference (string __time_reference))
   (setq _image __image)
   (setq _info __info)
   (setq _shutter (float __shutter))
   (setq _gain (float __gain))
   (setq _white_balance_blue (round __white_balance_blue))
   (setq _white_balance_red (round __white_balance_red))
   (setq _temperature (float __temperature))
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
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:info
   (&rest __info)
   (if (keywordp (car __info))
       (send* _info __info)
     (progn
       (if __info (setq _info (car __info)))
       _info)))
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
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _time_reference
    4 (length _time_reference)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; sensor_msgs/CameraInfo _info
    (send _info :serialization-length)
    ;; float32 _shutter
    4
    ;; float32 _gain
    4
    ;; uint16 _white_balance_blue
    2
    ;; uint16 _white_balance_red
    2
    ;; float32 _temperature
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
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; sensor_msgs/CameraInfo _info
       (send _info :serialize s)
     ;; float32 _shutter
       (sys::poke _shutter (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gain
       (sys::poke _gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _white_balance_blue
       (write-word _white_balance_blue s)
     ;; uint16 _white_balance_red
       (write-word _white_balance_red s)
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _time_reference
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _time_reference (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; sensor_msgs/CameraInfo _info
     (send _info :deserialize buf ptr-) (incf ptr- (send _info :serialization-length))
   ;; float32 _shutter
     (setq _shutter (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gain
     (setq _gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _white_balance_blue
     (setq _white_balance_blue (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _white_balance_red
     (setq _white_balance_red (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wfov_camera_msgs::WFOVImage :md5sum-) "807d0db423ab5e1561cfeba09a10bc88")
(setf (get wfov_camera_msgs::WFOVImage :datatype-) "wfov_camera_msgs/WFOVImage")
(setf (get wfov_camera_msgs::WFOVImage :definition-)
      "# Message for holding an image and metadata for the NDVI cameras.
Header header			# stamp is the time at which the data was valid.
				# frame_id is the location of the camera.

string time_reference		# The name of the reference clock for this message's timestamp.

sensor_msgs/Image image      	# Standard ROS image

sensor_msgs/CameraInfo info  	# Standard ROS camera info

float32 shutter              	# Time duration (seconds) the shutter was open.  Given by the FPGA timesync packet.  Negative shutter is an uninitialized (invalid) value.

float32 gain		     	# Gain (in dB) applied to the image.

uint16 white_balance_blue    	# White balance in blue.

uint16 white_balance_red     	# White balance in red.

float32 temperature          	# Temperature reported by the camera (Celsius).
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

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: sensor_msgs/CameraInfo
# This message defines meta information for a camera. It should be in a
# camera namespace on topic \"camera_info\" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may \"recalibrate\" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, \"plumb_bob\" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] P # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi

================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify

")



(provide :wfov_camera_msgs/WFOVImage "807d0db423ab5e1561cfeba09a10bc88")


