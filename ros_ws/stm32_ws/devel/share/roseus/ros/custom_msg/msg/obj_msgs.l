;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::obj_msgs)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'obj_msgs (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::OBJ_MSGS")
  (make-package "CUSTOM_MSG::OBJ_MSGS"))

(in-package "ROS")
;;//! \htmlinclude obj_msgs.msg.html


(defclass custom_msg::obj_msgs
  :super ros::object
  :slots (_distance _northing _easting ))

(defmethod custom_msg::obj_msgs
  (:init
   (&key
    ((:distance __distance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:northing __northing) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:easting __easting) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _distance __distance)
   (setq _northing __northing)
   (setq _easting __easting)
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:northing
   (&optional __northing)
   (if __northing (setq _northing __northing)) _northing)
  (:easting
   (&optional __easting)
   (if __easting (setq _easting __easting)) _easting)
  (:serialization-length
   ()
   (+
    ;; float32[] _distance
    (* 4    (length _distance)) 4
    ;; float32[] _northing
    (* 4    (length _northing)) 4
    ;; float32[] _easting
    (* 4    (length _easting)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _distance
     (write-long (length _distance) s)
     (dotimes (i (length _distance))
       (sys::poke (elt _distance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _northing
     (write-long (length _northing) s)
     (dotimes (i (length _northing))
       (sys::poke (elt _northing i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _easting
     (write-long (length _easting) s)
     (dotimes (i (length _easting))
       (sys::poke (elt _easting i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _distance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _northing
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _northing (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _northing i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _easting
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _easting (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _easting i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get custom_msg::obj_msgs :md5sum-) "2a1bea06901aaaf6ec5c025d3a77a953")
(setf (get custom_msg::obj_msgs :datatype-) "custom_msg/obj_msgs")
(setf (get custom_msg::obj_msgs :definition-)
      "float32[] distance
float32[] northing
float32[] easting

")



(provide :custom_msg/obj_msgs "2a1bea06901aaaf6ec5c025d3a77a953")


