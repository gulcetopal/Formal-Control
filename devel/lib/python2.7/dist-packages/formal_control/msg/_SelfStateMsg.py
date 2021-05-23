# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from formal_control/SelfStateMsg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class SelfStateMsg(genpy.Message):
  _md5sum = "e16fb09f7f887b94d9bd0e0a250f07a7"
  _type = "formal_control/SelfStateMsg"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header

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
"""
  __slots__ = ['header','rfdist','lfdist','bdist','v_relative','actions','policy','old_policy','v_emg','yaw_ref','got_new_plan','emergency','crit_check','current_state','lane','timestep','request']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','int32[]','int32[]','int32[]','float32','float32','bool','int32','string','int32','int32','int32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,rfdist,lfdist,bdist,v_relative,actions,policy,old_policy,v_emg,yaw_ref,got_new_plan,emergency,crit_check,current_state,lane,timestep,request

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SelfStateMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.rfdist is None:
        self.rfdist = 0.
      if self.lfdist is None:
        self.lfdist = 0.
      if self.bdist is None:
        self.bdist = 0.
      if self.v_relative is None:
        self.v_relative = 0.
      if self.actions is None:
        self.actions = []
      if self.policy is None:
        self.policy = []
      if self.old_policy is None:
        self.old_policy = []
      if self.v_emg is None:
        self.v_emg = 0.
      if self.yaw_ref is None:
        self.yaw_ref = 0.
      if self.got_new_plan is None:
        self.got_new_plan = False
      if self.emergency is None:
        self.emergency = 0
      if self.crit_check is None:
        self.crit_check = ''
      if self.current_state is None:
        self.current_state = 0
      if self.lane is None:
        self.lane = 0
      if self.timestep is None:
        self.timestep = 0
      if self.request is None:
        self.request = False
    else:
      self.header = std_msgs.msg.Header()
      self.rfdist = 0.
      self.lfdist = 0.
      self.bdist = 0.
      self.v_relative = 0.
      self.actions = []
      self.policy = []
      self.old_policy = []
      self.v_emg = 0.
      self.yaw_ref = 0.
      self.got_new_plan = False
      self.emergency = 0
      self.crit_check = ''
      self.current_state = 0
      self.lane = 0
      self.timestep = 0
      self.request = False

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_4f().pack(_x.rfdist, _x.lfdist, _x.bdist, _x.v_relative))
      length = len(self.actions)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.actions))
      length = len(self.policy)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.policy))
      length = len(self.old_policy)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.old_policy))
      _x = self
      buff.write(_get_struct_2fBi().pack(_x.v_emg, _x.yaw_ref, _x.got_new_plan, _x.emergency))
      _x = self.crit_check
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3iB().pack(_x.current_state, _x.lane, _x.timestep, _x.request))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.rfdist, _x.lfdist, _x.bdist, _x.v_relative,) = _get_struct_4f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.actions = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.policy = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.old_policy = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 13
      (_x.v_emg, _x.yaw_ref, _x.got_new_plan, _x.emergency,) = _get_struct_2fBi().unpack(str[start:end])
      self.got_new_plan = bool(self.got_new_plan)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.crit_check = str[start:end].decode('utf-8')
      else:
        self.crit_check = str[start:end]
      _x = self
      start = end
      end += 13
      (_x.current_state, _x.lane, _x.timestep, _x.request,) = _get_struct_3iB().unpack(str[start:end])
      self.request = bool(self.request)
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_4f().pack(_x.rfdist, _x.lfdist, _x.bdist, _x.v_relative))
      length = len(self.actions)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.actions.tostring())
      length = len(self.policy)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.policy.tostring())
      length = len(self.old_policy)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.old_policy.tostring())
      _x = self
      buff.write(_get_struct_2fBi().pack(_x.v_emg, _x.yaw_ref, _x.got_new_plan, _x.emergency))
      _x = self.crit_check
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3iB().pack(_x.current_state, _x.lane, _x.timestep, _x.request))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.rfdist, _x.lfdist, _x.bdist, _x.v_relative,) = _get_struct_4f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.actions = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.policy = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.old_policy = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 13
      (_x.v_emg, _x.yaw_ref, _x.got_new_plan, _x.emergency,) = _get_struct_2fBi().unpack(str[start:end])
      self.got_new_plan = bool(self.got_new_plan)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.crit_check = str[start:end].decode('utf-8')
      else:
        self.crit_check = str[start:end]
      _x = self
      start = end
      end += 13
      (_x.current_state, _x.lane, _x.timestep, _x.request,) = _get_struct_3iB().unpack(str[start:end])
      self.request = bool(self.request)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4f = None
def _get_struct_4f():
    global _struct_4f
    if _struct_4f is None:
        _struct_4f = struct.Struct("<4f")
    return _struct_4f
_struct_3iB = None
def _get_struct_3iB():
    global _struct_3iB
    if _struct_3iB is None:
        _struct_3iB = struct.Struct("<3iB")
    return _struct_3iB
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_2fBi = None
def _get_struct_2fBi():
    global _struct_2fBi
    if _struct_2fBi is None:
        _struct_2fBi = struct.Struct("<2fBi")
    return _struct_2fBi
