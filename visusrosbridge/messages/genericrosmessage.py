from typing import Callable, Dict, Any, List, Tuple

from pytidenetworking.message import Message
from visusrosbridge.messages.rosmessagebase import ROSMessageBase
from visusrosbridge.ros.tools import getMessageDefinition

import rospy
import std_msgs.msg

#region Code Generation

#region Serialization

PRIMITIVE_SERIALIZERS: Dict[str, str] = {
    "bool": "putBool",
    "byte": "putInt8",
    "int8": "putInt8",
    "uint8": "putUInt8",
    "int16": "putInt16",
    "uint16": "putUInt16",
    "int32": "putInt32",
    "uint32": "putUInt32",
    "int64": "putInt64",
    "uint64": "putUInt64",
    "float32": "putFloat",
    "float64": "putDouble",
    "string": "putString",
}

ROS_SERIALIZERS: Dict[str, str] = {
    'time': 'msg.putUInt32(value.{0}.secs)\nmsg.putUInt32(value.{0}.nsecs)',
    'duration': 'msg.putInt32(value.{0}.secs)\nmsg.putInt32(value.{0}.nsecs)'
}

ROS_SERIALIZERS_ARRAY: Dict[str, str] = {
    'time': 'for j in range(len(value.{0})):\n    msg.putUInt32(value.{0}[j].secs)\n    msg.putUInt32(value.{0}[j].nsecs)',
    'duration': 'for j in range(len(value.{0})):\n    msg.putInt32(value.{0}[j].secs)\n    msg.putInt32(value.{0}[j].nsecs)'
}

SERIALIZERS: Dict[str, Any] = {}

VALUE_SERIALIZE_PRIMITIVE_SINGLE = "msg.{}(value.{})"
VALUE_SERIALIZE_PRIMITIVE_ARRAY = "msg.{}Array(value.{})"

VALUE_SERIALIZE_ARRAY_LENGTH = "msg.putArrayLength(len({}))"
VALUE_SERIALIZE_COMPLEX = "exec(SERIALIZERS['{}'], {{'SERIALIZERS': SERIALIZERS, 'msg': msg, 'value': value.{} }})"

VALUE_SERIALIZE_COMPLEX_ARRAY = "for j in range(len(value.{0})):\n    exec(SERIALIZERS['{1}'], __globals={{'SERIALIZERS': SERIALIZERS, 'msg': msg, 'value': value.{0}[j]}})"


def generateSerializer(msg):
    msg_type = msg._type
    types = getMessageDefinition(msg)
    slots = msg.__slots__

    source = ["# " + msg_type]

    for i in range(len(types)):
        type = types[i]
        if type[0] in PRIMITIVE_SERIALIZERS: # Type is in primitives
            if type[1]: # isArray
                if type[2] > 0: # fixedLength array
                    for j in range(type[2]):
                        source.append(VALUE_SERIALIZE_PRIMITIVE_SINGLE.format(PRIMITIVE_SERIALIZERS[type[0]],
                                                                              "{}[{}]".format(slots[i], j)))
                else: # Dynamic Length Array
                    source.append(VALUE_SERIALIZE_PRIMITIVE_ARRAY.format(PRIMITIVE_SERIALIZERS[type[0]], slots[i]))
            else:
                source.append(VALUE_SERIALIZE_PRIMITIVE_SINGLE.format(PRIMITIVE_SERIALIZERS[type[0]], slots[i]))
        elif type[0] in ROS_SERIALIZERS:
            if type[1]: # isArray
                if type[2] > 0: # fixedLength array
                    for j in range(type[2]):
                        source.append(ROS_SERIALIZERS[type[0]].format("{}[{}]".format(slots[i], j)))
                else: # dynamic Length Array
                    source.append(VALUE_SERIALIZE_ARRAY_LENGTH.format(slots[i]))
                    source.append(ROS_SERIALIZERS_ARRAY[type[0]].format(slots[i]))
            else: # Single Value
                source.append(ROS_SERIALIZERS[type[0]].format(slots[i]))
        else:
            if getattr(msg, slots[i])._type not in SERIALIZERS:
                generateSerializer(getattr(msg, slots[i]))

            if type[1]: # isArray
                if type[2] > 0: # fixedLength array
                    for j in range(type[2]):
                        source.append(VALUE_SERIALIZE_COMPLEX.format(type[0], "{}[{}]".format(slots[i], j)))
                else: # dynamic Length Array
                    source.append(VALUE_SERIALIZE_ARRAY_LENGTH.format(slots[i]))
                    source.append(VALUE_SERIALIZE_COMPLEX_ARRAY.format(slots[i], type[0]))
            else: # Single Value
                source.append(VALUE_SERIALIZE_COMPLEX.format(type[0], slots[i]))

    src = "\n".join(source)

    print(src)

    compiled = compile(src, "serialize_" + msg_type, "exec")
    SERIALIZERS[msg_type] = compiled

#endregion

#region deserialize

PRIMITIVE_DESERIALIZERS: Dict[str, str] = {
    "bool": "getBool",
    "byte": "getInt8",
    "int8": "getInt8",
    "uint8": "getUInt8",
    "int16": "getInt16",
    "uint16": "getUInt16",
    "int32": "getInt32",
    "uint32": "getUInt32",
    "int64": "getInt64",
    "uint64": "getUInt64",
    "float32": "getFloat",
    "float64": "getDouble",
    "string": "getString",
}

ROS_DESERIALIZERS: Dict[str, str] = {
    'time': 'value.{0}.secs = msg.getUInt32()\nvalue.{0}.nsecs = msg.getUInt32()',
    'duration': 'value.{0}.secs = msg.getInt32()\nvalue.{0}.nsecs = msg.getInt32()'
}

ROS_DESERIALIZERS_ARRAY: Dict[str, str] = {
    'time': 'for j in range(lenght):\n    value.{0}[j].secs = msg.getUInt32()\n    value.{0}[j].nsecs = msg.getUInt32()',
    'duration': 'for j in range(lenght):\n    value.{0}[j].secs = msg.getInt32()\n    value.{0}[j].nsecs = msg.getInt32()'
}

DESERIALIZERS: Dict[str, Any] = {}

VALUE_DESERIALIZE_PRIMITIVE_SINGLE = "value.{1} = msg.{0}()"
VALUE_DESERIALIZE_PRIMITIVE_ARRAY = "value.{1} = msg.{0}Array()"

VALUE_DESERIALIZE_ARRAY_LENGTH = "lenght = msg.readArrayLength()"
VALUE_DESERIALIZE_COMPLEX = "exec(DESERIALIZERS['{0}'], {{'DESERIALIZERS': DESERIALIZERS, 'msg': msg, 'value': value.{1} }})"

VALUE_DESERIALIZE_COMPLEX_ARRAY = "for j in range(length):\n    exec(DESERIALIZERS['{1}'], __globals={{'DESERIALIZERS': DESERIALIZERS, 'msg': msg, 'value': value.{0}[j]}})"


def generateDeserializer(msg):
    msg_type = msg._type
    types = getMessageDefinition(msg)
    slots = msg.__slots__

    source = ["# " + msg_type]

    for i in range(len(types)):
        type = types[i]
        if type[0] in PRIMITIVE_DESERIALIZERS: # Type is in primitives
            if type[1]: # isArray
                if type[2] > 0: # fixedLength array
                    for j in range(type[2]):
                        source.append(VALUE_DESERIALIZE_PRIMITIVE_SINGLE.format(PRIMITIVE_DESERIALIZERS[type[0]],
                                                                              "{}[{}]".format(slots[i], j)))
                else: # Dynamic Length Array
                    source.append(VALUE_DESERIALIZE_PRIMITIVE_ARRAY.format(PRIMITIVE_DESERIALIZERS[type[0]], slots[i]))
            else:
                source.append(VALUE_DESERIALIZE_PRIMITIVE_SINGLE.format(PRIMITIVE_DESERIALIZERS[type[0]], slots[i]))
        elif type[0] in ROS_DESERIALIZERS:
            if type[1]: # isArray
                if type[2] > 0: # fixedLength array
                    for j in range(type[2]):
                        source.append(ROS_DESERIALIZERS[type[0]].format("{}[{}]".format(slots[i], j)))
                else: # dynamic Length Array
                    source.append(VALUE_DESERIALIZE_ARRAY_LENGTH.format(slots[i]))
                    source.append(ROS_DESERIALIZERS_ARRAY[type[0]].format(slots[i]))
            else: # Single Value
                source.append(ROS_DESERIALIZERS[type[0]].format(type[0], slots[i]))
        else:
            if getattr(msg, slots[i])._type not in DESERIALIZERS:
                generateDeserializer(getattr(msg, slots[i]))

            if type[1]: # isArray
                if type[2] > 0: # fixedLength array
                    for j in range(type[2]):
                        source.append(VALUE_DESERIALIZE_COMPLEX.format(type[0], "{}[{}]".format(slots[i], j)))
                else: # dynamic Length Array
                    source.append(VALUE_DESERIALIZE_ARRAY_LENGTH.format(slots[i]))
                    source.append(VALUE_DESERIALIZE_COMPLEX_ARRAY.format(slots[i], type[0]))
            else: # Single Value
                source.append(VALUE_DESERIALIZE_COMPLEX.format(type[0], slots[i]))

    src = "\n".join(source)
    print(src)
    compiled = compile(src, "deserialize_" + msg_type, "exec")
    DESERIALIZERS[msg_type] = compiled

#endregion

#endregion

def serialize(msg: Message, value: Any):
    if value._type not in SERIALIZERS:
        generateSerializer(value)

    exec(SERIALIZERS[value._type], {"SERIALIZERS": SERIALIZERS, "msg": msg, "value": value})

    return msg

def deserialize(msg: Message, value: Any):
    if value._type not in DESERIALIZERS:
        generateDeserializer(value)
    exec(DESERIALIZERS[value._type], {"DESERIALIZERS": DESERIALIZERS, "msg": msg, "value": value})
    return value

class GenericROSMessage(ROSMessageBase):

    def __init__(self, value: Any):
        super().__init__()
        self.value = value

    def serialize(self, msg: Message):
        if self.value._type not in SERIALIZERS:
            generateSerializer(self.value)
        exec(SERIALIZERS[self.value._type], {"SERIALIZERS": SERIALIZERS, "msg": msg, "value": self.value})

    def deserialize(self, msg: Message):
        if self.value._type not in DESERIALIZERS:
            generateDeserializer(self.value)
        exec(DESERIALIZERS[self.value._type], {"DESERIALIZERS": DESERIALIZERS, "msg": msg, "value": self.value})
