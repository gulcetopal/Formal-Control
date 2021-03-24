;; Auto-generated. Do not edit!


(when (boundp 'formal_control::SelfStateMsg)
  (if (not (find-package "FORMAL_CONTROL"))
    (make-package "FORMAL_CONTROL"))
  (shadow 'SelfStateMsg (find-package "FORMAL_CONTROL")))
(unless (find-package "FORMAL_CONTROL::SELFSTATEMSG")
  (make-package "FORMAL_CONTROL::SELFSTATEMSG"))

(in-package "ROS")
;;//! \htmlinclude SelfStateMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass formal_control::SelfStateMsg
  :super ros::object
  :slots (_header _rfdist _lfdist _bdist _v_relative _policy _timestep _v_refx _yaw_ref _got_new_plan ))

(defmethod formal_control::SelfStateMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rfdist __rfdist) 0.0)
    ((:lfdist __lfdist) 0.0)
    ((:bdist __bdist) 0.0)
    ((:v_relative __v_relative) 0.0)
    ((:policy __policy) (make-array 0 :initial-element 0 :element-type :integer))
    ((:timestep __timestep) 0)
    ((:v_refx __v_refx) 0.0)
    ((:yaw_ref __yaw_ref) 0.0)
    ((:got_new_plan __got_new_plan) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _rfdist (float __rfdist))
   (setq _lfdist (float __lfdist))
   (setq _bdist (float __bdist))
   (setq _v_relative (float __v_relative))
   (setq _policy __policy)
   (setq _timestep (round __timestep))
   (setq _v_refx (float __v_refx))
   (setq _yaw_ref (float __yaw_ref))
   (setq _got_new_plan __got_new_plan)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rfdist
   (&optional __rfdist)
   (if __rfdist (setq _rfdist __rfdist)) _rfdist)
  (:lfdist
   (&optional __lfdist)
   (if __lfdist (setq _lfdist __lfdist)) _lfdist)
  (:bdist
   (&optional __bdist)
   (if __bdist (setq _bdist __bdist)) _bdist)
  (:v_relative
   (&optional __v_relative)
   (if __v_relative (setq _v_relative __v_relative)) _v_relative)
  (:policy
   (&optional __policy)
   (if __policy (setq _policy __policy)) _policy)
  (:timestep
   (&optional __timestep)
   (if __timestep (setq _timestep __timestep)) _timestep)
  (:v_refx
   (&optional __v_refx)
   (if __v_refx (setq _v_refx __v_refx)) _v_refx)
  (:yaw_ref
   (&optional __yaw_ref)
   (if __yaw_ref (setq _yaw_ref __yaw_ref)) _yaw_ref)
  (:got_new_plan
   (&optional __got_new_plan)
   (if __got_new_plan (setq _got_new_plan __got_new_plan)) _got_new_plan)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _rfdist
    4
    ;; float32 _lfdist
    4
    ;; float32 _bdist
    4
    ;; float32 _v_relative
    4
    ;; int32[] _policy
    (* 4    (length _policy)) 4
    ;; int32 _timestep
    4
    ;; float32 _v_refx
    4
    ;; float32 _yaw_ref
    4
    ;; bool _got_new_plan
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _rfdist
       (sys::poke _rfdist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lfdist
       (sys::poke _lfdist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bdist
       (sys::poke _bdist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v_relative
       (sys::poke _v_relative (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32[] _policy
     (write-long (length _policy) s)
     (dotimes (i (length _policy))
       (write-long (elt _policy i) s)
       )
     ;; int32 _timestep
       (write-long _timestep s)
     ;; float32 _v_refx
       (sys::poke _v_refx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_ref
       (sys::poke _yaw_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _got_new_plan
       (if _got_new_plan (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _rfdist
     (setq _rfdist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lfdist
     (setq _lfdist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bdist
     (setq _bdist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v_relative
     (setq _v_relative (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32[] _policy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _policy (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _policy i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _timestep
     (setq _timestep (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _v_refx
     (setq _v_refx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_ref
     (setq _yaw_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _got_new_plan
     (setq _got_new_plan (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get formal_control::SelfStateMsg :md5sum-) "443649e18f869d4801a49ae1419aa278")
(setf (get formal_control::SelfStateMsg :datatype-) "formal_control/SelfStateMsg")
(setf (get formal_control::SelfStateMsg :definition-)
      "std_msgs/Header header

float32 rfdist
float32 lfdist
float32 bdist
float32 v_relative

int32[] policy
int32 timestep

float32 v_refx
float32 yaw_ref

bool got_new_plan




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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :formal_control/SelfStateMsg "443649e18f869d4801a49ae1419aa278")


