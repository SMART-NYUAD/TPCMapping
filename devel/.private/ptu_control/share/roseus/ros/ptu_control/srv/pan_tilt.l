;; Auto-generated. Do not edit!


(when (boundp 'ptu_control::pan_tilt)
  (if (not (find-package "PTU_CONTROL"))
    (make-package "PTU_CONTROL"))
  (shadow 'pan_tilt (find-package "PTU_CONTROL")))
(unless (find-package "PTU_CONTROL::PAN_TILT")
  (make-package "PTU_CONTROL::PAN_TILT"))
(unless (find-package "PTU_CONTROL::PAN_TILTREQUEST")
  (make-package "PTU_CONTROL::PAN_TILTREQUEST"))
(unless (find-package "PTU_CONTROL::PAN_TILTRESPONSE")
  (make-package "PTU_CONTROL::PAN_TILTRESPONSE"))

(in-package "ROS")





(defclass ptu_control::pan_tiltRequest
  :super ros::object
  :slots (_position _velocity _effort ))

(defmethod ptu_control::pan_tiltRequest
  (:init
   (&key
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort __effort) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _effort __effort)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:serialization-length
   ()
   (+
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _effort
    (* 8    (length _effort)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort
     (write-long (length _effort) s)
     (dotimes (i (length _effort))
       (sys::poke (elt _effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass ptu_control::pan_tiltResponse
  :super ros::object
  :slots (_success ))

(defmethod ptu_control::pan_tiltResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ptu_control::pan_tilt
  :super ros::object
  :slots ())

(setf (get ptu_control::pan_tilt :md5sum-) "2e2b5f145e29f6b78ab4fe0698b7903a")
(setf (get ptu_control::pan_tilt :datatype-) "ptu_control/pan_tilt")
(setf (get ptu_control::pan_tilt :request) ptu_control::pan_tiltRequest)
(setf (get ptu_control::pan_tilt :response) ptu_control::pan_tiltResponse)

(defmethod ptu_control::pan_tiltRequest
  (:response () (instance ptu_control::pan_tiltResponse :init)))

(setf (get ptu_control::pan_tiltRequest :md5sum-) "2e2b5f145e29f6b78ab4fe0698b7903a")
(setf (get ptu_control::pan_tiltRequest :datatype-) "ptu_control/pan_tiltRequest")
(setf (get ptu_control::pan_tiltRequest :definition-)
      "float64[] position
float64[] velocity
float64[] effort
---
bool success
")

(setf (get ptu_control::pan_tiltResponse :md5sum-) "2e2b5f145e29f6b78ab4fe0698b7903a")
(setf (get ptu_control::pan_tiltResponse :datatype-) "ptu_control/pan_tiltResponse")
(setf (get ptu_control::pan_tiltResponse :definition-)
      "float64[] position
float64[] velocity
float64[] effort
---
bool success
")



(provide :ptu_control/pan_tilt "2e2b5f145e29f6b78ab4fe0698b7903a")


