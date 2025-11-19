;; Auto-generated. Do not edit!


(when (boundp 'topic::U_control_vector)
  (if (not (find-package "TOPIC"))
    (make-package "TOPIC"))
  (shadow 'U_control_vector (find-package "TOPIC")))
(unless (find-package "TOPIC::U_CONTROL_VECTOR")
  (make-package "TOPIC::U_CONTROL_VECTOR"))

(in-package "ROS")
;;//! \htmlinclude U_control_vector.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass topic::U_control_vector
  :super ros::object
  :slots (_header _u_ControlVector ))

(defmethod topic::U_control_vector
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:u_ControlVector __u_ControlVector) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _u_ControlVector __u_ControlVector)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:u_ControlVector
   (&optional __u_ControlVector)
   (if __u_ControlVector (setq _u_ControlVector __u_ControlVector)) _u_ControlVector)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _u_ControlVector
    (* 8    (length _u_ControlVector)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _u_ControlVector
     (write-long (length _u_ControlVector) s)
     (dotimes (i (length _u_ControlVector))
       (sys::poke (elt _u_ControlVector i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _u_ControlVector
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _u_ControlVector (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _u_ControlVector i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get topic::U_control_vector :md5sum-) "9b7e7f682f6b2c79d3a6cfee564de60e")
(setf (get topic::U_control_vector :datatype-) "topic/U_control_vector")
(setf (get topic::U_control_vector :definition-)
      "std_msgs/Header header
float64[] u_ControlVector

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



(provide :topic/U_control_vector "9b7e7f682f6b2c79d3a6cfee564de60e")


