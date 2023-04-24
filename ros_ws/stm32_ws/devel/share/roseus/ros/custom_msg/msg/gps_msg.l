;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::gps_msg)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'gps_msg (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::GPS_MSG")
  (make-package "CUSTOM_MSG::GPS_MSG"))

(in-package "ROS")
;;//! \htmlinclude gps_msg.msg.html


(defclass custom_msg::gps_msg
  :super ros::object
  :slots (_latitude _longitude _speed_kmh _northing _easting _tracking_angle ))

(defmethod custom_msg::gps_msg
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:speed_kmh __speed_kmh) 0.0)
    ((:northing __northing) 0.0)
    ((:easting __easting) 0.0)
    ((:tracking_angle __tracking_angle) 0.0)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _speed_kmh (float __speed_kmh))
   (setq _northing (float __northing))
   (setq _easting (float __easting))
   (setq _tracking_angle (float __tracking_angle))
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:speed_kmh
   (&optional __speed_kmh)
   (if __speed_kmh (setq _speed_kmh __speed_kmh)) _speed_kmh)
  (:northing
   (&optional __northing)
   (if __northing (setq _northing __northing)) _northing)
  (:easting
   (&optional __easting)
   (if __easting (setq _easting __easting)) _easting)
  (:tracking_angle
   (&optional __tracking_angle)
   (if __tracking_angle (setq _tracking_angle __tracking_angle)) _tracking_angle)
  (:serialization-length
   ()
   (+
    ;; float32 _latitude
    4
    ;; float32 _longitude
    4
    ;; float32 _speed_kmh
    4
    ;; float32 _northing
    4
    ;; float32 _easting
    4
    ;; float32 _tracking_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_kmh
       (sys::poke _speed_kmh (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _northing
       (sys::poke _northing (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _easting
       (sys::poke _easting (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tracking_angle
       (sys::poke _tracking_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _latitude
     (setq _latitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longitude
     (setq _longitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_kmh
     (setq _speed_kmh (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _northing
     (setq _northing (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _easting
     (setq _easting (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tracking_angle
     (setq _tracking_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msg::gps_msg :md5sum-) "9a89ca61072a2258a2e6eec048f89329")
(setf (get custom_msg::gps_msg :datatype-) "custom_msg/gps_msg")
(setf (get custom_msg::gps_msg :definition-)
      "float32 latitude
float32 longitude
float32 speed_kmh
float32 northing
float32 easting
float32 tracking_angle

")



(provide :custom_msg/gps_msg "9a89ca61072a2258a2e6eec048f89329")


