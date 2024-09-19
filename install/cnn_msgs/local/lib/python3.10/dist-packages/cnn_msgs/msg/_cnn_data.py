# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cnn_msgs:msg/CnnData.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'ped_pos_map'
# Member 'scan'
# Member 'scan_all'
# Member 'image_gray'
# Member 'depth'
# Member 'goal_cart'
# Member 'goal_final_polar'
# Member 'vel'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CnnData(type):
    """Metaclass of message 'CnnData'."""

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
            module = import_type_support('cnn_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cnn_msgs.msg.CnnData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cnn_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cnn_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cnn_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cnn_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cnn_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CnnData(metaclass=Metaclass_CnnData):
    """Message class 'CnnData'."""

    __slots__ = [
        '_ped_pos_map',
        '_scan',
        '_scan_all',
        '_image_gray',
        '_depth',
        '_goal_cart',
        '_goal_final_polar',
        '_vel',
    ]

    _fields_and_field_types = {
        'ped_pos_map': 'sequence<float>',
        'scan': 'sequence<float>',
        'scan_all': 'sequence<float>',
        'image_gray': 'sequence<float>',
        'depth': 'sequence<float>',
        'goal_cart': 'sequence<float>',
        'goal_final_polar': 'sequence<float>',
        'vel': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ped_pos_map = array.array('f', kwargs.get('ped_pos_map', []))
        self.scan = array.array('f', kwargs.get('scan', []))
        self.scan_all = array.array('f', kwargs.get('scan_all', []))
        self.image_gray = array.array('f', kwargs.get('image_gray', []))
        self.depth = array.array('f', kwargs.get('depth', []))
        self.goal_cart = array.array('f', kwargs.get('goal_cart', []))
        self.goal_final_polar = array.array('f', kwargs.get('goal_final_polar', []))
        self.vel = array.array('f', kwargs.get('vel', []))

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
        if self.ped_pos_map != other.ped_pos_map:
            return False
        if self.scan != other.scan:
            return False
        if self.scan_all != other.scan_all:
            return False
        if self.image_gray != other.image_gray:
            return False
        if self.depth != other.depth:
            return False
        if self.goal_cart != other.goal_cart:
            return False
        if self.goal_final_polar != other.goal_final_polar:
            return False
        if self.vel != other.vel:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ped_pos_map(self):
        """Message field 'ped_pos_map'."""
        return self._ped_pos_map

    @ped_pos_map.setter
    def ped_pos_map(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'ped_pos_map' array.array() must have the type code of 'f'"
            self._ped_pos_map = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'ped_pos_map' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._ped_pos_map = array.array('f', value)

    @builtins.property
    def scan(self):
        """Message field 'scan'."""
        return self._scan

    @scan.setter
    def scan(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'scan' array.array() must have the type code of 'f'"
            self._scan = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'scan' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._scan = array.array('f', value)

    @builtins.property
    def scan_all(self):
        """Message field 'scan_all'."""
        return self._scan_all

    @scan_all.setter
    def scan_all(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'scan_all' array.array() must have the type code of 'f'"
            self._scan_all = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'scan_all' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._scan_all = array.array('f', value)

    @builtins.property
    def image_gray(self):
        """Message field 'image_gray'."""
        return self._image_gray

    @image_gray.setter
    def image_gray(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'image_gray' array.array() must have the type code of 'f'"
            self._image_gray = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'image_gray' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._image_gray = array.array('f', value)

    @builtins.property
    def depth(self):
        """Message field 'depth'."""
        return self._depth

    @depth.setter
    def depth(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'depth' array.array() must have the type code of 'f'"
            self._depth = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'depth' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._depth = array.array('f', value)

    @builtins.property
    def goal_cart(self):
        """Message field 'goal_cart'."""
        return self._goal_cart

    @goal_cart.setter
    def goal_cart(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'goal_cart' array.array() must have the type code of 'f'"
            self._goal_cart = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'goal_cart' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._goal_cart = array.array('f', value)

    @builtins.property
    def goal_final_polar(self):
        """Message field 'goal_final_polar'."""
        return self._goal_final_polar

    @goal_final_polar.setter
    def goal_final_polar(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'goal_final_polar' array.array() must have the type code of 'f'"
            self._goal_final_polar = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'goal_final_polar' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._goal_final_polar = array.array('f', value)

    @builtins.property
    def vel(self):
        """Message field 'vel'."""
        return self._vel

    @vel.setter
    def vel(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'vel' array.array() must have the type code of 'f'"
            self._vel = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'vel' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel = array.array('f', value)
