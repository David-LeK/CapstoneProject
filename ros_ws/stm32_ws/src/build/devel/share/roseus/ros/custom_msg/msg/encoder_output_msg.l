;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::encoder_output_msg)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'encoder_output_msg (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::ENCODER_OUTPUT_MSG")
  (make-package "CUSTOM_MSG::ENCODER_OUTPUT_MSG"))

(in-package "ROS")
;;//! \htmlinclude encoder_output_msg.msg.html


(defclass custom_msg::encoder_output_msg
  :super ros::object
  :slots (_output_rpm_m1 _output_controller_m1 _output_rpm_m2 _output_controller_m2 ))

(defmethod custom_msg::encoder_output_msg
  (:init
   (&key
    ((:output_rpm_m1 __output_rpm_m1) 0.0)
    ((:output_controller_m1 __output_controller_m1) 0.0)
    ((:output_rpm_m2 __output_rpm_m2) 0.0)
    ((:output_controller_m2 __output_controller_m2) 0.0)
    )
   (send-super :init)
   (setq _output_rpm_m1 (float __output_rpm_m1))
   (setq _output_controller_m1 (float __output_controller_m1))
   (setq _output_rpm_m2 (float __output_rpm_m2))
   (setq _output_controller_m2 (float __output_controller_m2))
   self)
  (:output_rpm_m1
   (&optional __output_rpm_m1)
   (if __output_rpm_m1 (setq _output_rpm_m1 __output_rpm_m1)) _output_rpm_m1)
  (:output_controller_m1
   (&optional __output_controller_m1)
   (if __output_controller_m1 (setq _output_controller_m1 __output_controller_m1)) _output_controller_m1)
  (:output_rpm_m2
   (&optional __output_rpm_m2)
   (if __output_rpm_m2 (setq _output_rpm_m2 __output_rpm_m2)) _output_rpm_m2)
  (:output_controller_m2
   (&optional __output_controller_m2)
   (if __output_controller_m2 (setq _output_controller_m2 __output_controller_m2)) _output_controller_m2)
  (:serialization-length
   ()
   (+
    ;; float32 _output_rpm_m1
    4
    ;; float32 _output_controller_m1
    4
    ;; float32 _output_rpm_m2
    4
    ;; float32 _output_controller_m2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _output_rpm_m1
       (sys::poke _output_rpm_m1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _output_controller_m1
       (sys::poke _output_controller_m1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _output_rpm_m2
       (sys::poke _output_rpm_m2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _output_controller_m2
       (sys::poke _output_controller_m2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _output_rpm_m1
     (setq _output_rpm_m1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _output_controller_m1
     (setq _output_controller_m1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _output_rpm_m2
     (setq _output_rpm_m2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _output_controller_m2
     (setq _output_controller_m2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msg::encoder_output_msg :md5sum-) "1245866a2cb06bf0708dba963cb3a6ee")
(setf (get custom_msg::encoder_output_msg :datatype-) "custom_msg/encoder_output_msg")
(setf (get custom_msg::encoder_output_msg :definition-)
      "float32 output_rpm_m1
float32 output_controller_m1
float32 output_rpm_m2
float32 output_controller_m2

")



(provide :custom_msg/encoder_output_msg "1245866a2cb06bf0708dba963cb3a6ee")


