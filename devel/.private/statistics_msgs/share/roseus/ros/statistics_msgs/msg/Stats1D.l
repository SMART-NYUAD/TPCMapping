;; Auto-generated. Do not edit!


(when (boundp 'statistics_msgs::Stats1D)
  (if (not (find-package "STATISTICS_MSGS"))
    (make-package "STATISTICS_MSGS"))
  (shadow 'Stats1D (find-package "STATISTICS_MSGS")))
(unless (find-package "STATISTICS_MSGS::STATS1D")
  (make-package "STATISTICS_MSGS::STATS1D"))

(in-package "ROS")
;;//! \htmlinclude Stats1D.msg.html


(defclass statistics_msgs::Stats1D
  :super ros::object
  :slots (_min _max _mean _variance _N ))

(defmethod statistics_msgs::Stats1D
  (:init
   (&key
    ((:min __min) 0.0)
    ((:max __max) 0.0)
    ((:mean __mean) 0.0)
    ((:variance __variance) 0.0)
    ((:N __N) 0)
    )
   (send-super :init)
   (setq _min (float __min))
   (setq _max (float __max))
   (setq _mean (float __mean))
   (setq _variance (float __variance))
   (setq _N (round __N))
   self)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:mean
   (&optional __mean)
   (if __mean (setq _mean __mean)) _mean)
  (:variance
   (&optional __variance)
   (if __variance (setq _variance __variance)) _variance)
  (:N
   (&optional __N)
   (if __N (setq _N __N)) _N)
  (:serialization-length
   ()
   (+
    ;; float64 _min
    8
    ;; float64 _max
    8
    ;; float64 _mean
    8
    ;; float64 _variance
    8
    ;; int64 _N
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _min
       (sys::poke _min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max
       (sys::poke _max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mean
       (sys::poke _mean (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _variance
       (sys::poke _variance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _N
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _N (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _N) (= (length (_N . bv)) 2)) ;; bignum
              (write-long (ash (elt (_N . bv) 0) 0) s)
              (write-long (ash (elt (_N . bv) 1) -1) s))
             ((and (class _N) (= (length (_N . bv)) 1)) ;; big1
              (write-long (elt (_N . bv) 0) s)
              (write-long (if (>= _N 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _N s)(write-long (if (>= _N 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _min
     (setq _min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max
     (setq _max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mean
     (setq _mean (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _variance
     (setq _variance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _N
#+(or :alpha :irix6 :x86_64)
      (setf _N (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _N (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get statistics_msgs::Stats1D :md5sum-) "5e29efbcd70d63a82b5ddfac24a5bc4b")
(setf (get statistics_msgs::Stats1D :datatype-) "statistics_msgs/Stats1D")
(setf (get statistics_msgs::Stats1D :definition-)
      "# statistics of a 1-D distribution
float64 min
float64 max 
float64 mean 
float64 variance
int64 N

")



(provide :statistics_msgs/Stats1D "5e29efbcd70d63a82b5ddfac24a5bc4b")


