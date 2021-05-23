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
  :slots (_header _rfdist _lfdist _bdist _v_relative _actions _policy _old_policy _v_emg _yaw_ref _got_new_plan _emergency _crit_check _current_state _lane _timestep _request ))

(defmethod formal_control::SelfStateMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rfdist __rfdist) 0.0)
    ((:lfdist __lfdist) 0.0)
    ((:bdist __bdist) 0.0)
    ((:v_relative __v_relative) 0.0)
    ((:actions __actions) (make-array 0 :initial-element 0 :element-type :integer))
    ((:policy __policy) (make-array 0 :initial-element 0 :element-type :integer))
    ((:old_policy __old_policy) (make-array 0 :initial-element 0 :element-type :integer))
    ((:v_emg __v_emg) 0.0)
    ((:yaw_ref __yaw_ref) 0.0)
    ((:got_new_plan __got_new_plan) nil)
    ((:emergency __emergency) 0)
    ((:crit_check __crit_check) "")
    ((:current_state __current_state) 0)
    ((:lane __lane) 0)
    ((:timestep __timestep) 0)
    ((:request __request) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _rfdist (float __rfdist))
   (setq _lfdist (float __lfdist))
   (setq _bdist (float __bdist))
   (setq _v_relative (float __v_relative))
   (setq _actions __actions)
   (setq _policy __policy)
   (setq _old_policy __old_policy)
   (setq _v_emg (float __v_emg))
   (setq _yaw_ref (float __yaw_ref))
   (setq _got_new_plan __got_new_plan)
   (setq _emergency (round __emergency))
   (setq _crit_check (string __crit_check))
   (setq _current_state (round __current_state))
   (setq _lane (round __lane))
   (setq _timestep (round __timestep))
   (setq _request __request)
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
  (:actions
   (&optional __actions)
   (if __actions (setq _actions __actions)) _actions)
  (:policy
   (&optional __policy)
   (if __policy (setq _policy __policy)) _policy)
  (:old_policy
   (&optional __old_policy)
   (if __old_policy (setq _old_policy __old_policy)) _old_policy)
  (:v_emg
   (&optional __v_emg)
   (if __v_emg (setq _v_emg __v_emg)) _v_emg)
  (:yaw_ref
   (&optional __yaw_ref)
   (if __yaw_ref (setq _yaw_ref __yaw_ref)) _yaw_ref)
  (:got_new_plan
   (&optional __got_new_plan)
   (if __got_new_plan (setq _got_new_plan __got_new_plan)) _got_new_plan)
  (:emergency
   (&optional __emergency)
   (if __emergency (setq _emergency __emergency)) _emergency)
  (:crit_check
   (&optional __crit_check)
   (if __crit_check (setq _crit_check __crit_check)) _crit_check)
  (:current_state
   (&optional __current_state)
   (if __current_state (setq _current_state __current_state)) _current_state)
  (:lane
   (&optional __lane)
   (if __lane (setq _lane __lane)) _lane)
  (:timestep
   (&optional __timestep)
   (if __timestep (setq _timestep __timestep)) _timestep)
  (:request
   (&optional __request)
   (if __request (setq _request __request)) _request)
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
    ;; int32[] _actions
    (* 4    (length _actions)) 4
    ;; int32[] _policy
    (* 4    (length _policy)) 4
    ;; int32[] _old_policy
    (* 4    (length _old_policy)) 4
    ;; float32 _v_emg
    4
    ;; float32 _yaw_ref
    4
    ;; bool _got_new_plan
    1
    ;; int32 _emergency
    4
    ;; string _crit_check
    4 (length _crit_check)
    ;; int32 _current_state
    4
    ;; int32 _lane
    4
    ;; int32 _timestep
    4
    ;; bool _request
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
     ;; int32[] _actions
     (write-long (length _actions) s)
     (dotimes (i (length _actions))
       (write-long (elt _actions i) s)
       )
     ;; int32[] _policy
     (write-long (length _policy) s)
     (dotimes (i (length _policy))
       (write-long (elt _policy i) s)
       )
     ;; int32[] _old_policy
     (write-long (length _old_policy) s)
     (dotimes (i (length _old_policy))
       (write-long (elt _old_policy i) s)
       )
     ;; float32 _v_emg
       (sys::poke _v_emg (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_ref
       (sys::poke _yaw_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _got_new_plan
       (if _got_new_plan (write-byte -1 s) (write-byte 0 s))
     ;; int32 _emergency
       (write-long _emergency s)
     ;; string _crit_check
       (write-long (length _crit_check) s) (princ _crit_check s)
     ;; int32 _current_state
       (write-long _current_state s)
     ;; int32 _lane
       (write-long _lane s)
     ;; int32 _timestep
       (write-long _timestep s)
     ;; bool _request
       (if _request (write-byte -1 s) (write-byte 0 s))
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
   ;; int32[] _actions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actions (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _actions i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _policy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _policy (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _policy i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _old_policy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _old_policy (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _old_policy i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32 _v_emg
     (setq _v_emg (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_ref
     (setq _yaw_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _got_new_plan
     (setq _got_new_plan (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _emergency
     (setq _emergency (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _crit_check
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _crit_check (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _current_state
     (setq _current_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _lane
     (setq _lane (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _timestep
     (setq _timestep (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _request
     (setq _request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get formal_control::SelfStateMsg :md5sum-) "e16fb09f7f887b94d9bd0e0a250f07a7")
(setf (get formal_control::SelfStateMsg :datatype-) "formal_control/SelfStateMsg")
(setf (get formal_control::SelfStateMsg :definition-)
      "std_msgs/Header header

float32 rfdist
float32 lfdist
float32 bdist
float32 v_relative

int32[] actions
int32[] policy
int32[] old_policy

float32 v_emg
float32 yaw_ref

bool got_new_plan

int32 emergency
string crit_check
int32 current_state
int32 lane
int32 timestep
bool request

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



(provide :formal_control/SelfStateMsg "e16fb09f7f887b94d9bd0e0a250f07a7")


