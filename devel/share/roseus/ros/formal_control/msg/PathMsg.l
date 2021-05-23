;; Auto-generated. Do not edit!


(when (boundp 'formal_control::PathMsg)
  (if (not (find-package "FORMAL_CONTROL"))
    (make-package "FORMAL_CONTROL"))
  (shadow 'PathMsg (find-package "FORMAL_CONTROL")))
(unless (find-package "FORMAL_CONTROL::PATHMSG")
  (make-package "FORMAL_CONTROL::PATHMSG"))

(in-package "ROS")
;;//! \htmlinclude PathMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass formal_control::PathMsg
  :super ros::object
  :slots (_header _m _y_start _y_finish _x_start _x_finish _check ))

(defmethod formal_control::PathMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:m __m) 0.0)
    ((:y_start __y_start) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y_finish __y_finish) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:x_start __x_start) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:x_finish __x_finish) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:check __check) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _m (float __m))
   (setq _y_start __y_start)
   (setq _y_finish __y_finish)
   (setq _x_start __x_start)
   (setq _x_finish __x_finish)
   (setq _check __check)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:m
   (&optional __m)
   (if __m (setq _m __m)) _m)
  (:y_start
   (&optional __y_start)
   (if __y_start (setq _y_start __y_start)) _y_start)
  (:y_finish
   (&optional __y_finish)
   (if __y_finish (setq _y_finish __y_finish)) _y_finish)
  (:x_start
   (&optional __x_start)
   (if __x_start (setq _x_start __x_start)) _x_start)
  (:x_finish
   (&optional __x_finish)
   (if __x_finish (setq _x_finish __x_finish)) _x_finish)
  (:check
   (&optional __check)
   (if __check (setq _check __check)) _check)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _m
    4
    ;; float32[] _y_start
    (* 4    (length _y_start)) 4
    ;; float32[] _y_finish
    (* 4    (length _y_finish)) 4
    ;; float32[] _x_start
    (* 4    (length _x_start)) 4
    ;; float32[] _x_finish
    (* 4    (length _x_finish)) 4
    ;; bool _check
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _m
       (sys::poke _m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _y_start
     (write-long (length _y_start) s)
     (dotimes (i (length _y_start))
       (sys::poke (elt _y_start i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y_finish
     (write-long (length _y_finish) s)
     (dotimes (i (length _y_finish))
       (sys::poke (elt _y_finish i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _x_start
     (write-long (length _x_start) s)
     (dotimes (i (length _x_start))
       (sys::poke (elt _x_start i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _x_finish
     (write-long (length _x_finish) s)
     (dotimes (i (length _x_finish))
       (sys::poke (elt _x_finish i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _check
       (if _check (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _m
     (setq _m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _y_start
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y_start (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y_start i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y_finish
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y_finish (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y_finish i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _x_start
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_start (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_start i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _x_finish
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_finish (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_finish i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _check
     (setq _check (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get formal_control::PathMsg :md5sum-) "768ebd03b31b8172a50d65a6c3f01ec4")
(setf (get formal_control::PathMsg :datatype-) "formal_control/PathMsg")
(setf (get formal_control::PathMsg :definition-)
      "std_msgs/Header header

float32 m
float32[] y_start
float32[] y_finish
float32[] x_start
float32[] x_finish
bool check

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



(provide :formal_control/PathMsg "768ebd03b31b8172a50d65a6c3f01ec4")


