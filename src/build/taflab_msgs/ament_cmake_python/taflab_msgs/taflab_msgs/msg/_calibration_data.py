# generated from rosidl_generator_py/resource/_idl.py.em
# with input from taflab_msgs:msg/CalibrationData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CalibrationData(type):
    """Metaclass of message 'CalibrationData'."""

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
                'taflab_msgs.msg.CalibrationData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__calibration_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__calibration_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__calibration_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__calibration_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__calibration_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CalibrationData(metaclass=Metaclass_CalibrationData):
    """Message class 'CalibrationData'."""

    __slots__ = [
        '_rudder_min',
        '_rudder_max',
        '_sail_min',
        '_sail_max',
        '_esc_min',
        '_esc_max',
    ]

    _fields_and_field_types = {
        'rudder_min': 'float',
        'rudder_max': 'float',
        'sail_min': 'float',
        'sail_max': 'float',
        'esc_min': 'float',
        'esc_max': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.rudder_min = kwargs.get('rudder_min', float())
        self.rudder_max = kwargs.get('rudder_max', float())
        self.sail_min = kwargs.get('sail_min', float())
        self.sail_max = kwargs.get('sail_max', float())
        self.esc_min = kwargs.get('esc_min', float())
        self.esc_max = kwargs.get('esc_max', float())

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
        if self.rudder_min != other.rudder_min:
            return False
        if self.rudder_max != other.rudder_max:
            return False
        if self.sail_min != other.sail_min:
            return False
        if self.sail_max != other.sail_max:
            return False
        if self.esc_min != other.esc_min:
            return False
        if self.esc_max != other.esc_max:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def rudder_min(self):
        """Message field 'rudder_min'."""
        return self._rudder_min

    @rudder_min.setter
    def rudder_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rudder_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rudder_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rudder_min = value

    @builtins.property
    def rudder_max(self):
        """Message field 'rudder_max'."""
        return self._rudder_max

    @rudder_max.setter
    def rudder_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rudder_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rudder_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rudder_max = value

    @builtins.property
    def sail_min(self):
        """Message field 'sail_min'."""
        return self._sail_min

    @sail_min.setter
    def sail_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'sail_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'sail_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._sail_min = value

    @builtins.property
    def sail_max(self):
        """Message field 'sail_max'."""
        return self._sail_max

    @sail_max.setter
    def sail_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'sail_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'sail_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._sail_max = value

    @builtins.property
    def esc_min(self):
        """Message field 'esc_min'."""
        return self._esc_min

    @esc_min.setter
    def esc_min(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'esc_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'esc_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._esc_min = value

    @builtins.property
    def esc_max(self):
        """Message field 'esc_max'."""
        return self._esc_max

    @esc_max.setter
    def esc_max(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'esc_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'esc_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._esc_max = value
