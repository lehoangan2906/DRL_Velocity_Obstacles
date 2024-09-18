# generated from rosidl_generator_py/resource/_idl.py.em
# with input from track_ped_msgs:msg/TrackedPerson.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrackedPerson(type):
    """Metaclass of message 'TrackedPerson'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('track_ped_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'track_ped_msgs.msg.TrackedPerson')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tracked_person
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tracked_person
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tracked_person
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tracked_person
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tracked_person

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrackedPerson(metaclass=Metaclass_TrackedPerson):
    """Message class 'TrackedPerson'."""

    __slots__ = [
        '_bbox_upper_left_x',
        '_bbox_upper_left_y',
        '_bbox_bottom_right_x',
        '_bbox_bottom_right_y',
        '_id',
        '_depth',
        '_angle',
        '_twist',
        '_pose',
    ]

    _fields_and_field_types = {
        'bbox_upper_left_x': 'int32',
        'bbox_upper_left_y': 'int32',
        'bbox_bottom_right_x': 'int32',
        'bbox_bottom_right_y': 'int32',
        'id': 'int32',
        'depth': 'float',
        'angle': 'float',
        'twist': 'geometry_msgs/Twist',
        'pose': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.bbox_upper_left_x = kwargs.get('bbox_upper_left_x', int())
        self.bbox_upper_left_y = kwargs.get('bbox_upper_left_y', int())
        self.bbox_bottom_right_x = kwargs.get('bbox_bottom_right_x', int())
        self.bbox_bottom_right_y = kwargs.get('bbox_bottom_right_y', int())
        self.id = kwargs.get('id', int())
        self.depth = kwargs.get('depth', float())
        self.angle = kwargs.get('angle', float())
        from geometry_msgs.msg import Twist
        self.twist = kwargs.get('twist', Twist())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.bbox_upper_left_x != other.bbox_upper_left_x:
            return False
        if self.bbox_upper_left_y != other.bbox_upper_left_y:
            return False
        if self.bbox_bottom_right_x != other.bbox_bottom_right_x:
            return False
        if self.bbox_bottom_right_y != other.bbox_bottom_right_y:
            return False
        if self.id != other.id:
            return False
        if self.depth != other.depth:
            return False
        if self.angle != other.angle:
            return False
        if self.twist != other.twist:
            return False
        if self.pose != other.pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bbox_upper_left_x(self):
        """Message field 'bbox_upper_left_x'."""
        return self._bbox_upper_left_x

    @bbox_upper_left_x.setter
    def bbox_upper_left_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bbox_upper_left_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bbox_upper_left_x' field must be an integer in [-2147483648, 2147483647]"
        self._bbox_upper_left_x = value

    @builtins.property
    def bbox_upper_left_y(self):
        """Message field 'bbox_upper_left_y'."""
        return self._bbox_upper_left_y

    @bbox_upper_left_y.setter
    def bbox_upper_left_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bbox_upper_left_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bbox_upper_left_y' field must be an integer in [-2147483648, 2147483647]"
        self._bbox_upper_left_y = value

    @builtins.property
    def bbox_bottom_right_x(self):
        """Message field 'bbox_bottom_right_x'."""
        return self._bbox_bottom_right_x

    @bbox_bottom_right_x.setter
    def bbox_bottom_right_x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bbox_bottom_right_x' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bbox_bottom_right_x' field must be an integer in [-2147483648, 2147483647]"
        self._bbox_bottom_right_x = value

    @builtins.property
    def bbox_bottom_right_y(self):
        """Message field 'bbox_bottom_right_y'."""
        return self._bbox_bottom_right_y

    @bbox_bottom_right_y.setter
    def bbox_bottom_right_y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bbox_bottom_right_y' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bbox_bottom_right_y' field must be an integer in [-2147483648, 2147483647]"
        self._bbox_bottom_right_y = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def depth(self):
        """Message field 'depth'."""
        return self._depth

    @depth.setter
    def depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'depth' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._depth = value

    @builtins.property
    def angle(self):
        """Message field 'angle'."""
        return self._angle

    @angle.setter
    def angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle = value

    @builtins.property
    def twist(self):
        """Message field 'twist'."""
        return self._twist

    @twist.setter
    def twist(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'twist' field must be a sub message of type 'Twist'"
        self._twist = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value
