from enum import IntEnum
from typing import List, Union, Dict, Tuple

import rospy

from pytidenetworking.message import Message

ROS_TYPE = Union[bool, int, float, str, List[bool], List[int], List[float], List[str]]
ROS_TYPE_EXTENDED = Union[ROS_TYPE, List['ROS_TYPE_EXTENDED'], Dict[str, 'ROS_TYPE_EXTENDED']]

class DataType(IntEnum):
    BOOL = 0
    BOOL_ARRAY = 1
    BYTE = 2
    BYTE_ARRAY = 3
    SBYTE = 4
    SBYTE_ARRAY = 5
    SHORT = 6
    SHORT_ARRAY = 7
    USHORT = 8
    USHORT_ARRAY = 9
    INT = 10
    INT_ARRAY = 11
    UINT = 12
    UINT_ARRAY = 13
    LONG = 14
    LONG_ARRAY = 15
    ULONG = 16
    ULONG_ARRAY = 17
    FLOAT = 18
    FLOAT_ARRAY = 19
    DOUBLE = 20
    DOUBLE_ARRAY = 21
    STRING = 22
    STRING_ARRAY = 23
    NONE_TYPE = 24
    LIST_BEGIN = 252
    LIST_END = 253
    DICT_BEGIN = 254
    DICT_END = 255


TYPE_NAMES = {
    DataType.BOOL: "bool",
    DataType.BOOL_ARRAY: "bool[]",
    DataType.BYTE: "byte",
    DataType.BYTE_ARRAY: "byte[]",
    DataType.SBYTE: "sbyte",
    DataType.SBYTE_ARRAY: "sbyte[]",
    DataType.SHORT: "short",
    DataType.SHORT_ARRAY: "short[]",
    DataType.USHORT: "ushort",
    DataType.USHORT_ARRAY: "ushort[]",
    DataType.INT: "int",
    DataType.INT_ARRAY: "int[]",
    DataType.UINT: "uint",
    DataType.UINT_ARRAY: "uint[]",
    DataType.LONG: "long",
    DataType.LONG_ARRAY: "long[]",
    DataType.ULONG: "ulong",
    DataType.ULONG_ARRAY: "ulong[]",
    DataType.FLOAT: "float",
    DataType.FLOAT_ARRAY: "float[]",
    DataType.DOUBLE: "double",
    DataType.DOUBLE_ARRAY: "double[]",
    DataType.STRING: "string",
    DataType.STRING_ARRAY: "string[]",
    DataType.NONE_TYPE: "None"
}

PARAMETER_TYPE_MAPPING = {
    int: DataType.INT,
    bool: DataType.BOOL,
    str: DataType.STRING,
    float: DataType.DOUBLE,
}


def build_type_array(value: ROS_TYPE_EXTENDED) -> Tuple[List[int], List[ROS_TYPE]]:
    if value is None:
        return [DataType.NONE_TYPE], []

    if isinstance(value, bool):
        return [DataType.BOOL], [value]
    elif isinstance(value, int):
        return [DataType.INT], [value]
    elif isinstance(value, float):
        return [DataType.FLOAT], [value]
    elif isinstance(value, str):
        return [DataType.STRING], [value]
    elif isinstance(value, (list, tuple)):
        typearray = [DataType.LIST_BEGIN]
        valueArray = []
        for val in value:
            t, v = build_type_array(val)
            typearray.extend(t)
            valueArray.extend(v)
        typearray.append(DataType.LIST_END)
        return typearray, valueArray
    elif isinstance(value, dict):
        typearray = [DataType.DICT_BEGIN]
        valueArray = []

        for key in value:
            valueArray.append(key)
            t, v = build_type_array(value[key])
            typearray.extend(t)
            valueArray.extend(v)

        typearray.append(DataType.DICT_END)
        return typearray, valueArray
    else:
        rospy.logwarn("Unknown DataType '{}' of value '{}'".format(type(value), value))
        return [], []

def parse_type_array_data(types: List[int], message: Message, position = 0) -> Tuple[ROS_TYPE_EXTENDED, int]:
    value: ROS_TYPE_EXTENDED = None

    if types[position] == DataType.BOOL:
        value = message.getBool()


    return value, position