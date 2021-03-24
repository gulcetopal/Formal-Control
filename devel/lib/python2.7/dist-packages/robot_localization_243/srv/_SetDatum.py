# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from robot_localization_243/SetDatumRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import geographic_msgs.msg

class SetDatumRequest(genpy.Message):
  _md5sum = "fe903ca95d0210defda73a1629604439"
  _type = "robot_localization_243/SetDatumRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geographic_msgs/GeoPose geo_pose

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
"""
  __slots__ = ['geo_pose']
  _slot_types = ['geographic_msgs/GeoPose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       geo_pose

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetDatumRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.geo_pose is None:
        self.geo_pose = geographic_msgs.msg.GeoPose()
    else:
      self.geo_pose = geographic_msgs.msg.GeoPose()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_7d().pack(_x.geo_pose.position.latitude, _x.geo_pose.position.longitude, _x.geo_pose.position.altitude, _x.geo_pose.orientation.x, _x.geo_pose.orientation.y, _x.geo_pose.orientation.z, _x.geo_pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.geo_pose is None:
        self.geo_pose = geographic_msgs.msg.GeoPose()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.geo_pose.position.latitude, _x.geo_pose.position.longitude, _x.geo_pose.position.altitude, _x.geo_pose.orientation.x, _x.geo_pose.orientation.y, _x.geo_pose.orientation.z, _x.geo_pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_7d().pack(_x.geo_pose.position.latitude, _x.geo_pose.position.longitude, _x.geo_pose.position.altitude, _x.geo_pose.orientation.x, _x.geo_pose.orientation.y, _x.geo_pose.orientation.z, _x.geo_pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.geo_pose is None:
        self.geo_pose = geographic_msgs.msg.GeoPose()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.geo_pose.position.latitude, _x.geo_pose.position.longitude, _x.geo_pose.position.altitude, _x.geo_pose.orientation.x, _x.geo_pose.orientation.y, _x.geo_pose.orientation.z, _x.geo_pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from robot_localization_243/SetDatumResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetDatumResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "robot_localization_243/SetDatumResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetDatumResponse, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
class SetDatum(object):
  _type          = 'robot_localization_243/SetDatum'
  _md5sum = 'fe903ca95d0210defda73a1629604439'
  _request_class  = SetDatumRequest
  _response_class = SetDatumResponse
