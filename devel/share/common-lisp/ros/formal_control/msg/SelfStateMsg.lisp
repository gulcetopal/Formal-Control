; Auto-generated. Do not edit!


(cl:in-package formal_control-msg)


;//! \htmlinclude SelfStateMsg.msg.html

(cl:defclass <SelfStateMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rfdist
    :reader rfdist
    :initarg :rfdist
    :type cl:float
    :initform 0.0)
   (lfdist
    :reader lfdist
    :initarg :lfdist
    :type cl:float
    :initform 0.0)
   (bdist
    :reader bdist
    :initarg :bdist
    :type cl:float
    :initform 0.0)
   (v_relative
    :reader v_relative
    :initarg :v_relative
    :type cl:float
    :initform 0.0)
   (policy
    :reader policy
    :initarg :policy
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (timestep
    :reader timestep
    :initarg :timestep
    :type cl:integer
    :initform 0)
   (v_refx
    :reader v_refx
    :initarg :v_refx
    :type cl:float
    :initform 0.0)
   (yaw_ref
    :reader yaw_ref
    :initarg :yaw_ref
    :type cl:float
    :initform 0.0)
   (got_new_plan
    :reader got_new_plan
    :initarg :got_new_plan
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SelfStateMsg (<SelfStateMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelfStateMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelfStateMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name formal_control-msg:<SelfStateMsg> is deprecated: use formal_control-msg:SelfStateMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:header-val is deprecated.  Use formal_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rfdist-val :lambda-list '(m))
(cl:defmethod rfdist-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:rfdist-val is deprecated.  Use formal_control-msg:rfdist instead.")
  (rfdist m))

(cl:ensure-generic-function 'lfdist-val :lambda-list '(m))
(cl:defmethod lfdist-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:lfdist-val is deprecated.  Use formal_control-msg:lfdist instead.")
  (lfdist m))

(cl:ensure-generic-function 'bdist-val :lambda-list '(m))
(cl:defmethod bdist-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:bdist-val is deprecated.  Use formal_control-msg:bdist instead.")
  (bdist m))

(cl:ensure-generic-function 'v_relative-val :lambda-list '(m))
(cl:defmethod v_relative-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:v_relative-val is deprecated.  Use formal_control-msg:v_relative instead.")
  (v_relative m))

(cl:ensure-generic-function 'policy-val :lambda-list '(m))
(cl:defmethod policy-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:policy-val is deprecated.  Use formal_control-msg:policy instead.")
  (policy m))

(cl:ensure-generic-function 'timestep-val :lambda-list '(m))
(cl:defmethod timestep-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:timestep-val is deprecated.  Use formal_control-msg:timestep instead.")
  (timestep m))

(cl:ensure-generic-function 'v_refx-val :lambda-list '(m))
(cl:defmethod v_refx-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:v_refx-val is deprecated.  Use formal_control-msg:v_refx instead.")
  (v_refx m))

(cl:ensure-generic-function 'yaw_ref-val :lambda-list '(m))
(cl:defmethod yaw_ref-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:yaw_ref-val is deprecated.  Use formal_control-msg:yaw_ref instead.")
  (yaw_ref m))

(cl:ensure-generic-function 'got_new_plan-val :lambda-list '(m))
(cl:defmethod got_new_plan-val ((m <SelfStateMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader formal_control-msg:got_new_plan-val is deprecated.  Use formal_control-msg:got_new_plan instead.")
  (got_new_plan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelfStateMsg>) ostream)
  "Serializes a message object of type '<SelfStateMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rfdist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lfdist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bdist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_relative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'policy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'policy))
  (cl:let* ((signed (cl:slot-value msg 'timestep)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_refx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'got_new_plan) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelfStateMsg>) istream)
  "Deserializes a message object of type '<SelfStateMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rfdist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lfdist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bdist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_relative) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'policy) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'policy)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestep) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_refx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_ref) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'got_new_plan) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelfStateMsg>)))
  "Returns string type for a message object of type '<SelfStateMsg>"
  "formal_control/SelfStateMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelfStateMsg)))
  "Returns string type for a message object of type 'SelfStateMsg"
  "formal_control/SelfStateMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelfStateMsg>)))
  "Returns md5sum for a message object of type '<SelfStateMsg>"
  "443649e18f869d4801a49ae1419aa278")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelfStateMsg)))
  "Returns md5sum for a message object of type 'SelfStateMsg"
  "443649e18f869d4801a49ae1419aa278")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelfStateMsg>)))
  "Returns full string definition for message of type '<SelfStateMsg>"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 rfdist~%float32 lfdist~%float32 bdist~%float32 v_relative~%~%int32[] policy~%int32 timestep~%~%float32 v_refx~%float32 yaw_ref~%~%bool got_new_plan~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelfStateMsg)))
  "Returns full string definition for message of type 'SelfStateMsg"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 rfdist~%float32 lfdist~%float32 bdist~%float32 v_relative~%~%int32[] policy~%int32 timestep~%~%float32 v_refx~%float32 yaw_ref~%~%bool got_new_plan~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelfStateMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'policy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelfStateMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'SelfStateMsg
    (cl:cons ':header (header msg))
    (cl:cons ':rfdist (rfdist msg))
    (cl:cons ':lfdist (lfdist msg))
    (cl:cons ':bdist (bdist msg))
    (cl:cons ':v_relative (v_relative msg))
    (cl:cons ':policy (policy msg))
    (cl:cons ':timestep (timestep msg))
    (cl:cons ':v_refx (v_refx msg))
    (cl:cons ':yaw_ref (yaw_ref msg))
    (cl:cons ':got_new_plan (got_new_plan msg))
))
