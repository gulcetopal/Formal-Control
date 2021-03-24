;; Auto-generated. Do not edit!


(when (boundp 'robot_localization_243::SetDatum)
  (if (not (find-package "ROBOT_LOCALIZATION_243"))
    (make-package "ROBOT_LOCALIZATION_243"))
  (shadow 'SetDatum (find-package "ROBOT_LOCALIZATION_243")))
(unless (find-package "ROBOT_LOCALIZATION_243::SETDATUM")
  (make-package "ROBOT_LOCALIZATION_243::SETDATUM"))
(unless (find-package "ROBOT_LOCALIZATION_243::SETDATUMREQUEST")
  (make-package "ROBOT_LOCALIZATION_243::SETDATUMREQUEST"))
(unless (find-package "ROBOT_LOCALIZATION_243::SETDATUMRESPONSE")
  (make-package "ROBOT_LOCALIZATION_243::SETDATUMRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOGRAPHIC_MSGS"))
  (ros::roseus-add-msgs "geographic_msgs"))




(defclass robot_localization_243::SetDatumRequest
  :super ros::object
  :slots (_geo_pose ))

(defmethod robot_localization_243::SetDatumRequest
  (:init
   (&key
    ((:geo_pose __geo_pose) (instance geographic_msgs::GeoPose :init))
    )
   (send-super :init)
   (setq _geo_pose __geo_pose)
   self)
  (:geo_pose
   (&rest __geo_pose)
   (if (keywordp (car __geo_pose))
       (send* _geo_pose __geo_pose)
     (progn
       (if __geo_pose (setq _geo_pose (car __geo_pose)))
       _geo_pose)))
  (:serialization-length
   ()
   (+
    ;; geographic_msgs/GeoPose _geo_pose
    (send _geo_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geographic_msgs/GeoPose _geo_pose
       (send _geo_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geographic_msgs/GeoPose _geo_pose
     (send _geo_pose :deserialize buf ptr-) (incf ptr- (send _geo_pose :serialization-length))
   ;;
   self)
  )

(defclass robot_localization_243::SetDatumResponse
  :super ros::object
  :slots ())

(defmethod robot_localization_243::SetDatumResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass robot_localization_243::SetDatum
  :super ros::object
  :slots ())

(setf (get robot_localization_243::SetDatum :md5sum-) "fe903ca95d0210defda73a1629604439")
(setf (get robot_localization_243::SetDatum :datatype-) "robot_localization_243/SetDatum")
(setf (get robot_localization_243::SetDatum :request) robot_localization_243::SetDatumRequest)
(setf (get robot_localization_243::SetDatum :response) robot_localization_243::SetDatumResponse)

(defmethod robot_localization_243::SetDatumRequest
  (:response () (instance robot_localization_243::SetDatumResponse :init)))

(setf (get robot_localization_243::SetDatumRequest :md5sum-) "fe903ca95d0210defda73a1629604439")
(setf (get robot_localization_243::SetDatumRequest :datatype-) "robot_localization_243/SetDatumRequest")
(setf (get robot_localization_243::SetDatumRequest :definition-)
      "geographic_msgs/GeoPose geo_pose

================================================================================
MSG: geographic_msgs/GeoPose
# Geographic pose, using the WGS 84 reference ellipsoid.
#
# Orientation uses the East-North-Up (ENU) frame of reference.
# (But, what about singularities at the poles?)

GeoPoint position
geometry_msgs/Quaternion orientation

================================================================================
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

")

(setf (get robot_localization_243::SetDatumResponse :md5sum-) "fe903ca95d0210defda73a1629604439")
(setf (get robot_localization_243::SetDatumResponse :datatype-) "robot_localization_243/SetDatumResponse")
(setf (get robot_localization_243::SetDatumResponse :definition-)
      "geographic_msgs/GeoPose geo_pose

================================================================================
MSG: geographic_msgs/GeoPose
# Geographic pose, using the WGS 84 reference ellipsoid.
#
# Orientation uses the East-North-Up (ENU) frame of reference.
# (But, what about singularities at the poles?)

GeoPoint position
geometry_msgs/Quaternion orientation

================================================================================
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---

")



(provide :robot_localization_243/SetDatum "fe903ca95d0210defda73a1629604439")


