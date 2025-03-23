from typing import Tuple, List, Any, Dict

import rospy

ROS_PRIMITIVES = ["bool", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32",
                  "float64", "string", "time", "duration"]

TYPE_DEFINITION_CACHE: Dict[str, List[Tuple[str, bool, int]]] = {}


def getTypeDefinition(type: str):
    basetype = type
    isArray = False
    length = -1

    try:
        if type.__contains__('['):
            isArray = True
            typevals = type.split('[')
            basetype = typevals[0]
            if len(typevals) > 2:
                rospy.logerr("Multidimensional Arrays are not yet supported")
            if typevals[1] != ']':
                length = int(typevals[1].strip('[]'))

        return basetype, isArray, length
    except ValueError as ex:
        rospy.logerr("Exception while parsing data type: {}".format(ex))


def getMessageDefinition(msg: Any) -> List[Tuple[str, bool, int]]:
    return getDefinition(msg._type, msg._slot_types)


def getDefinition(type, slot_types) -> List[Tuple[str, bool, int]]:
    if type in TYPE_DEFINITION_CACHE:
        return TYPE_DEFINITION_CACHE[type]
    else:
        definition = []
        for t in slot_types:
            definition.append(getTypeDefinition(t))
        TYPE_DEFINITION_CACHE[type] = definition
        return definition