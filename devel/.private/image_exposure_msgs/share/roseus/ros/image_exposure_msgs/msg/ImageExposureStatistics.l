;; Auto-generated. Do not edit!


(when (boundp 'image_exposure_msgs::ImageExposureStatistics)
  (if (not (find-package "IMAGE_EXPOSURE_MSGS"))
    (make-package "IMAGE_EXPOSURE_MSGS"))
  (shadow 'ImageExposureStatistics (find-package "IMAGE_EXPOSURE_MSGS")))
(unless (find-package "IMAGE_EXPOSURE_MSGS::IMAGEEXPOSURESTATISTICS")
  (make-package "IMAGE_EXPOSURE_MSGS::IMAGEEXPOSURESTATISTICS"))

(in-package "ROS")
;;//! \htmlinclude ImageExposureStatistics.msg.html
(if (not (find-package "STATISTICS_MSGS"))
  (ros::roseus-add-msgs "statistics_msgs"))


(defclass image_exposure_msgs::ImageExposureStatistics
  :super ros::object
  :slots (_stamp _time_reference _shutterms _gaindb _underExposed _overExposed _pixelVal _pixelAge _meanIrradiance _minMeasurableIrradiance _maxMeasurableIrradiance _minObservedIrradiance _maxObservedIrradiance ))

(defmethod image_exposure_msgs::ImageExposureStatistics
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:time_reference __time_reference) "")
    ((:shutterms __shutterms) 0.0)
    ((:gaindb __gaindb) 0.0)
    ((:underExposed __underExposed) 0)
    ((:overExposed __overExposed) 0)
    ((:pixelVal __pixelVal) (instance statistics_msgs::Stats1D :init))
    ((:pixelAge __pixelAge) (instance statistics_msgs::Stats1D :init))
    ((:meanIrradiance __meanIrradiance) 0.0)
    ((:minMeasurableIrradiance __minMeasurableIrradiance) 0.0)
    ((:maxMeasurableIrradiance __maxMeasurableIrradiance) 0.0)
    ((:minObservedIrradiance __minObservedIrradiance) 0.0)
    ((:maxObservedIrradiance __maxObservedIrradiance) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _time_reference (string __time_reference))
   (setq _shutterms (float __shutterms))
   (setq _gaindb (float __gaindb))
   (setq _underExposed (round __underExposed))
   (setq _overExposed (round __overExposed))
   (setq _pixelVal __pixelVal)
   (setq _pixelAge __pixelAge)
   (setq _meanIrradiance (float __meanIrradiance))
   (setq _minMeasurableIrradiance (float __minMeasurableIrradiance))
   (setq _maxMeasurableIrradiance (float __maxMeasurableIrradiance))
   (setq _minObservedIrradiance (float __minObservedIrradiance))
   (setq _maxObservedIrradiance (float __maxObservedIrradiance))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:time_reference
   (&optional __time_reference)
   (if __time_reference (setq _time_reference __time_reference)) _time_reference)
  (:shutterms
   (&optional __shutterms)
   (if __shutterms (setq _shutterms __shutterms)) _shutterms)
  (:gaindb
   (&optional __gaindb)
   (if __gaindb (setq _gaindb __gaindb)) _gaindb)
  (:underExposed
   (&optional __underExposed)
   (if __underExposed (setq _underExposed __underExposed)) _underExposed)
  (:overExposed
   (&optional __overExposed)
   (if __overExposed (setq _overExposed __overExposed)) _overExposed)
  (:pixelVal
   (&rest __pixelVal)
   (if (keywordp (car __pixelVal))
       (send* _pixelVal __pixelVal)
     (progn
       (if __pixelVal (setq _pixelVal (car __pixelVal)))
       _pixelVal)))
  (:pixelAge
   (&rest __pixelAge)
   (if (keywordp (car __pixelAge))
       (send* _pixelAge __pixelAge)
     (progn
       (if __pixelAge (setq _pixelAge (car __pixelAge)))
       _pixelAge)))
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
    ;; time _stamp
    8
    ;; string _time_reference
    4 (length _time_reference)
    ;; float32 _shutterms
    4
    ;; float32 _gaindb
    4
    ;; uint32 _underExposed
    4
    ;; uint32 _overExposed
    4
    ;; statistics_msgs/Stats1D _pixelVal
    (send _pixelVal :serialization-length)
    ;; statistics_msgs/Stats1D _pixelAge
    (send _pixelAge :serialization-length)
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
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; string _time_reference
       (write-long (length _time_reference) s) (princ _time_reference s)
     ;; float32 _shutterms
       (sys::poke _shutterms (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gaindb
       (sys::poke _gaindb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _underExposed
       (write-long _underExposed s)
     ;; uint32 _overExposed
       (write-long _overExposed s)
     ;; statistics_msgs/Stats1D _pixelVal
       (send _pixelVal :serialize s)
     ;; statistics_msgs/Stats1D _pixelAge
       (send _pixelAge :serialize s)
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
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _time_reference
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _time_reference (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _shutterms
     (setq _shutterms (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gaindb
     (setq _gaindb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _underExposed
     (setq _underExposed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _overExposed
     (setq _overExposed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; statistics_msgs/Stats1D _pixelVal
     (send _pixelVal :deserialize buf ptr-) (incf ptr- (send _pixelVal :serialization-length))
   ;; statistics_msgs/Stats1D _pixelAge
     (send _pixelAge :deserialize buf ptr-) (incf ptr- (send _pixelAge :serialization-length))
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

(setf (get image_exposure_msgs::ImageExposureStatistics :md5sum-) "334dc170ce6287d1de470f25be78dd9e")
(setf (get image_exposure_msgs::ImageExposureStatistics :datatype-) "image_exposure_msgs/ImageExposureStatistics")
(setf (get image_exposure_msgs::ImageExposureStatistics :definition-)
      "# message for exposure statistics reported a single image
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



(provide :image_exposure_msgs/ImageExposureStatistics "334dc170ce6287d1de470f25be78dd9e")


