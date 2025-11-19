; Auto-generated. Do not edit!


(cl:in-package topic-msg)


;//! \htmlinclude U_control_vector.msg.html

(cl:defclass <U_control_vector> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (u_ControlVector
    :reader u_ControlVector
    :initarg :u_ControlVector
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass U_control_vector (<U_control_vector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <U_control_vector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'U_control_vector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name topic-msg:<U_control_vector> is deprecated: use topic-msg:U_control_vector instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <U_control_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topic-msg:header-val is deprecated.  Use topic-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'u_ControlVector-val :lambda-list '(m))
(cl:defmethod u_ControlVector-val ((m <U_control_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topic-msg:u_ControlVector-val is deprecated.  Use topic-msg:u_ControlVector instead.")
  (u_ControlVector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <U_control_vector>) ostream)
  "Serializes a message object of type '<U_control_vector>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'u_ControlVector))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'u_ControlVector))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <U_control_vector>) istream)
  "Deserializes a message object of type '<U_control_vector>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'u_ControlVector) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'u_ControlVector)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<U_control_vector>)))
  "Returns string type for a message object of type '<U_control_vector>"
  "topic/U_control_vector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'U_control_vector)))
  "Returns string type for a message object of type 'U_control_vector"
  "topic/U_control_vector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<U_control_vector>)))
  "Returns md5sum for a message object of type '<U_control_vector>"
  "9b7e7f682f6b2c79d3a6cfee564de60e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'U_control_vector)))
  "Returns md5sum for a message object of type 'U_control_vector"
  "9b7e7f682f6b2c79d3a6cfee564de60e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<U_control_vector>)))
  "Returns full string definition for message of type '<U_control_vector>"
  (cl:format cl:nil "std_msgs/Header header~%float64[] u_ControlVector~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'U_control_vector)))
  "Returns full string definition for message of type 'U_control_vector"
  (cl:format cl:nil "std_msgs/Header header~%float64[] u_ControlVector~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <U_control_vector>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'u_ControlVector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <U_control_vector>))
  "Converts a ROS message object to a list"
  (cl:list 'U_control_vector
    (cl:cons ':header (header msg))
    (cl:cons ':u_ControlVector (u_ControlVector msg))
))
