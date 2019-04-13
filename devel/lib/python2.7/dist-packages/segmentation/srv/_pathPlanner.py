# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from segmentation/pathPlannerRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class pathPlannerRequest(genpy.Message):
  _md5sum = "5d1799e0b491e3c6d6f148bd3d07bda9"
  _type = "segmentation/pathPlannerRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """std_msgs/Float32MultiArray sorted_objects
int16 target_id
geometry_msgs/Point grasp_point

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = "height"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = "width"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = "channel"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['sorted_objects','target_id','grasp_point']
  _slot_types = ['std_msgs/Float32MultiArray','int16','geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       sorted_objects,target_id,grasp_point

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(pathPlannerRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sorted_objects is None:
        self.sorted_objects = std_msgs.msg.Float32MultiArray()
      if self.target_id is None:
        self.target_id = 0
      if self.grasp_point is None:
        self.grasp_point = geometry_msgs.msg.Point()
    else:
      self.sorted_objects = std_msgs.msg.Float32MultiArray()
      self.target_id = 0
      self.grasp_point = geometry_msgs.msg.Point()

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
      length = len(self.sorted_objects.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.sorted_objects.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        if python3:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.size, _x.stride))
      buff.write(_struct_I.pack(self.sorted_objects.layout.data_offset))
      length = len(self.sorted_objects.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.sorted_objects.data))
      _x = self
      buff.write(_struct_h3d.pack(_x.target_id, _x.grasp_point.x, _x.grasp_point.y, _x.grasp_point.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.sorted_objects is None:
        self.sorted_objects = std_msgs.msg.Float32MultiArray()
      if self.grasp_point is None:
        self.grasp_point = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sorted_objects.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _struct_2I.unpack(str[start:end])
        self.sorted_objects.layout.dim.append(val1)
      start = end
      end += 4
      (self.sorted_objects.layout.data_offset,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.sorted_objects.data = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 26
      (_x.target_id, _x.grasp_point.x, _x.grasp_point.y, _x.grasp_point.z,) = _struct_h3d.unpack(str[start:end])
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
      length = len(self.sorted_objects.layout.dim)
      buff.write(_struct_I.pack(length))
      for val1 in self.sorted_objects.layout.dim:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        if python3:
          buff.write(struct.pack('<I%sB'%length, length, *_x))
        else:
          buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2I.pack(_x.size, _x.stride))
      buff.write(_struct_I.pack(self.sorted_objects.layout.data_offset))
      length = len(self.sorted_objects.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.sorted_objects.data.tostring())
      _x = self
      buff.write(_struct_h3d.pack(_x.target_id, _x.grasp_point.x, _x.grasp_point.y, _x.grasp_point.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.sorted_objects is None:
        self.sorted_objects = std_msgs.msg.Float32MultiArray()
      if self.grasp_point is None:
        self.grasp_point = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sorted_objects.layout.dim = []
      for i in range(0, length):
        val1 = std_msgs.msg.MultiArrayDimension()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 8
        (_x.size, _x.stride,) = _struct_2I.unpack(str[start:end])
        self.sorted_objects.layout.dim.append(val1)
      start = end
      end += 4
      (self.sorted_objects.layout.data_offset,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.sorted_objects.data = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 26
      (_x.target_id, _x.grasp_point.x, _x.grasp_point.y, _x.grasp_point.z,) = _struct_h3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2I = struct.Struct("<2I")
_struct_h3d = struct.Struct("<h3d")
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from segmentation/pathPlannerResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class pathPlannerResponse(genpy.Message):
  _md5sum = "d5550685e8802ccb55d7b325d3ef4459"
  _type = "segmentation/pathPlannerResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool path_found

"""
  __slots__ = ['path_found']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       path_found

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(pathPlannerResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.path_found is None:
        self.path_found = False
    else:
      self.path_found = False

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
      buff.write(_struct_B.pack(self.path_found))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.path_found,) = _struct_B.unpack(str[start:end])
      self.path_found = bool(self.path_found)
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
      buff.write(_struct_B.pack(self.path_found))
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
      start = end
      end += 1
      (self.path_found,) = _struct_B.unpack(str[start:end])
      self.path_found = bool(self.path_found)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class pathPlanner(object):
  _type          = 'segmentation/pathPlanner'
  _md5sum = '29b555fe93d25b5dcf4f386d833186f5'
  _request_class  = pathPlannerRequest
  _response_class = pathPlannerResponse
