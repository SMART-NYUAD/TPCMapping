;; Auto-generated. Do not edit!


(when (boundp 'image_exposure_msgs::SequenceExposureStatistics)
  (if (not (find-package "IMAGE_EXPOSURE_MSGS"))
    (make-package "IMAGE_EXPOSURE_MSGS"))
  (shadow 'SequenceExposureStatistics (find-package "IMAGE_EXPOSURE_MSGS")))
(unless (find-package "IMAGE_EXPOSURE_MSGS::SEQUENCEEXPOSURESTATISTICS")
  (make-package "IMAGE_EXPOSURE_MSGS::SEQUENCEEXPOSURESTATISTICS"))

(in-package "ROS")
;;//! \htmlinclude SequenceExposureStatistics.msg.html


(defclass image_exposure_msgs::SequenceExposureStatistics
  :super ros::object
  :slots (_exposures _underExposed _overExposed _meanIrradiance _minMeasurableIrradiance _maxMeasurableIrradiance _minObservedIrradiance _maxObservedIrradiance ))

(defmethod image_exposure_msgs::SequenceExposureStatistics
  (:init
   (&key
    ((:exposures __exposures) ())
    ((:underExposed __underExposed) 0)
    ((:overExposed __overExposed) 0)
    ((:meanIrradiance __meanIrradiance) 0.0)
    ((:minMeasurableIrradiance __minMeasurableIrradiance) 0.0)
    ((:maxMeasurableIrradiance __maxMeasurableIrradiance) 0.0)
    ((:minObservedIrradiance __minObservedIrradiance) 0.0)
    ((:maxObservedIrradiance __maxObservedIrradiance) 0.0)
    )
   (send-super :init)
   (setq _exposures __exposures)
   (setq _underExposed (round __underExposed))
   (setq _overExposed (round __overExposed))
   (setq _meanIrradiance (float __meanIrradiance))
   (setq _minMeasurableIrradiance (float __minMeasurableIrradiance))
   (setq _maxMeasurableIrradiance (float __maxMeasurableIrradiance))
   (setq _minObservedIrradiance (float __minObservedIrradiance))
   (setq _maxObservedIrradiance (float __maxObservedIrradiance))
   self)
  (:exposures
   (&rest __exposures)
   (if (keywordp (car __exposures))
       (send* _exposures __exposures)
     (progn
       (if __exposures (setq _exposures (car __exposures)))
       _exposures)))
  (:underExposed
   (&optional __underExposed)
   (if __underExposed (setq _underExposed __underExposed)) _underExposed)
  (:overExposed
   (&optional __overExposed)
   (if __overExposed (setq _overExposed __overExposed)) _overExposed)
  (:meanIrradiance
   (&optional __meanIrradiance)
   (if __meanIrradiance (setq _meanIrradiance __meanIrradiance)) _meanIrradiance)
  (:minMeasurableIrradiance
   (&optional __minMeasurableIrradiance)
   (if __minMeasurableIrradiance (setq _minMeasurableIrradiance __minMeasurableIrradiance)) _minMeasurableIrradiance)
  (:maxMeasurableIrradiance
   (&optional __maxMeasurableIrradiance)
   (if __maxMeasurableIrradiance (setq _maxMeasurableIrradiance __maxMeasurableIrradiance)) _maxMeasurableIrradiance)
  (:minObservedIrradiance
   (&optional __minObservedIrradiance)
   (if __minObservedIrradiance (setq _minObservedIrradiance __minObservedIrradiance)) _minObservedIrradiance)
  (:maxObservedIrradiance
   (&optional __maxObservedIrradiance)
   (if __maxObservedIrradiance (setq _maxObservedIrradiance __maxObservedIrradiance)) _maxObservedIrradiance)
  (:serialization-length
   ()
   (+
    ;; image_exposure_msgs/ImageExposureStatistics[] _exposures
    (apply #'+ (send-all _exposures :serialization-length)) 4
    ;; uint32 _underExposed
    4
    ;; uint32 _overExposed
    4
    ;; float64 _meanIrradiance
    8
    ;; float64 _minMeasurableIrradiance
    8
    ;; float64 _maxMeasurableIrradiance
    8
    ;; float64 _minObservedIrradiance
    8
    ;; float64 _maxObservedIrradiance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; image_exposure_msgs/ImageExposureStatistics[] _exposures
     (write-long (length _exposures) s)
     (dolist (elem _exposures)
       (send elem :serialize s)
       )
     ;; uint32 _underExposed
       (write-long _underExposed s)
     ;; uint32 _overExposed
       (write-long _overExposed s)
     ;; float64 _meanIrradiance
       (sys::poke _meanIrradiance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _minMeasurableIrradiance
       (sys::poke _minMeasurableIrradiance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxMeasurableIrradiance
       (sys::poke _maxMeasurableIrradiance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _minObservedIrradiance
       (sys::poke _minObservedIrradiance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxObservedIrradiance
       (sys::poke _maxObservedIrradiance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; image_exposure_msgs/ImageExposureStatistics[] _exposures
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _exposures (let (r) (dotimes (i n) (push (instance image_exposure_msgs::ImageExposureStatistics :init) r)) r))
     (dolist (elem- _exposures)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _underExposed
     (setq _underExposed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _overExposed
     (setq _overExposed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _meanIrradiance
     (setq _meanIrradiance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _minMeasurableIrradiance
     (setq _minMeasurableIrradiance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxMeasurableIrradiance
     (setq _maxMeasurableIrradiance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _minObservedIrradiance
     (setq _minObservedIrradiance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxObservedIrradiance
     (setq _maxObservedIrradiance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get image_exposure_msgs::SequenceExposureStatistics :md5sum-) "2a4f3187c50e7b3544984e9e28ce4328")
(setf (get image_exposure_msgs::SequenceExposureStatistics :datatype-) "image_exposure_msgs/SequenceExposureStatistics")
(setf (get image_exposure_msgs::SequenceExposureStatistics :definition-)
      "# exposure statistics for a multi-exposure HDR sequence
ImageExposureStatistics[] exposures
uint32 underExposed 		# number of pixels underexposed in longest exposure
uint32 overExposed  		# number of pixels overexposed in shortest
float64 meanIrradiance 		# mean irradiance of all properly exposed pixels in all images in the sequence
float64 minMeasurableIrradiance
float64 maxMeasurableIrradiance
float64 minObservedIrradiance
float64 maxObservedIrradiance


================================================================================
MSG: image_exposure_msgs/ImageExposureStatistics
# message for exposure statistics reported a single image
time stamp         # image time stamp
string time_reference # The name of the reference clock for this message's timestamp.
float32 shutterms  # shutter durations in ms
float32 gaindb     # gain in decibels
# pixel exposure and latency statistics
uint32 underExposed # number of pixels underexposed
uint32 overExposed  # number of pixels overexposed
statistics_msgs/Stats1D pixelVal   # distribution of pixel values in the image
statistics_msgs/Stats1D pixelAge   # distribution of pixel ages in frames
# irradiance = pixelval/(shutterTime*10^(gaindb/10.0))
float64 meanIrradiance
float64 minMeasurableIrradiance
float64 maxMeasurableIrradiance
float64 minObservedIrradiance
float64 maxObservedIrradiance



================================================================================
MSG: statistics_msgs/Stats1D
# statistics of a 1-D distribution
float64 min
float64 max 
float64 mean 
float64 variance
int64 N

")



(provide :image_exposure_msgs/SequenceExposureStatistics "2a4f3187c50e7b3544984e9e28ce4328")


