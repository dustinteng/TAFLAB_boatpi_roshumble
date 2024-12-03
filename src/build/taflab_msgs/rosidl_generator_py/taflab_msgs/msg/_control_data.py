# generated from rosidl_generator_py/resource/_idl.py.em
# with input from taflab_msgs:msg/ControlData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ControlData(type):
    """Metaclass of message 'ControlData'."""

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
            module = import_type_support('taflab_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'taflab_msgs.msg.ControlData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__control_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__control_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__control_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__control_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__control_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ControlData(metaclass=Metaclass_ControlData):
    """Message class 'ControlData'."""

    __slots__ = [
        '_servo_rudder',
        '_servo_sail',
        '_esc',
    ]

    _fields_and_field_types = {
        'servo_rudder': 'float',
        'servo_sail': 'float',
        'esc': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.servo_rudder = kwargs.get('servo_rudder', float())
        self.servo_sail = kwargs.get('servo_sail', float())
        self.esc = kwargs.get('esc', float())

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
        if self.servo_rudder != other.servo_rudder:
            return False
        if self.servo_sail != other.servo_sail:
            return False
        if self.esc != other.esc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def servo_rudder(self):
        """Message field 'servo_rudder'."""
        return self._servo_rudder

    @servo_rudder.setter
    def servo_rudder(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'servo_rudder' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'servo_rudder' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._servo_rudder = value

    @builtins.property
    def servo_sail(self):
        """Message field 'servo_sail'."""
        return self._servo_sail

    @servo_sail.setter
    def servo_sail(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'servo_sail' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'servo_sail' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._servo_sail = value

    @builtins.property
    def esc(self):
        """Message field 'esc'."""
        return self._esc

    @esc.setter
    def esc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'esc' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'esc' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._esc = value
