from typing import List, Union, Tuple, Any

import rospy

from pytidenetworking.message import create, MessageSendMode, Message
from visusrosbridge.messages.genericrosmessage import serialize
from visusrosbridge.messages.messagetypes import ClientToServer, ServerToClient
from visusrosbridge.messages.rosmessagebase import ROSMessageBase
from visusrosbridge.messages.value_types import DataType, ROS_TYPE_EXTENDED, build_type_array, PARAMETER_TYPE_MAPPING

ROS_TYPE = Union[bool, int, float, str, List[bool], List[int], List[float], List[str]]


def addTypedValueToMessage(msg: Message, type: int, value: ROS_TYPE):
    """
    Append the value of the specified type to the message

    :param msg: Message to add the data to
    :param type: DataType to add value as
    :param value: value to add
    :return:
    """
    if type == DataType.BOOL:
        msg.putBool(value)
    elif type == DataType.BOOL_ARRAY:
        msg.putBoolArray(value)
    elif type == DataType.BYTE:
        msg.putUInt8(value)
    elif type == DataType.BYTE_ARRAY:
        msg.putUInt8Array(value)
    elif type == DataType.SBYTE:
        msg.putInt8(value)
    elif type == DataType.SBYTE_ARRAY:
        msg.putInt8Array(value)
    elif type == DataType.SHORT:
        msg.putInt16(value)
    elif type == DataType.SHORT_ARRAY:
        msg.putInt16Array(value)
    elif type == DataType.USHORT:
        msg.putUInt16(value)
    elif type == DataType.USHORT_ARRAY:
        msg.putUInt16Array(value)
    elif type == DataType.INT:
        msg.putInt32(value)
    elif type == DataType.INT_ARRAY:
        msg.putInt32Array(value)
    elif type == DataType.UINT:
        msg.putUInt32(value)
    elif type == DataType.UINT_ARRAY:
        msg.putUInt32Array(value)
    elif type == DataType.LONG:
        msg.putInt64(value)
    elif type == DataType.LONG_ARRAY:
        msg.putInt64Array(value)
    elif type == DataType.ULONG:
        msg.putUInt64(value)
    elif type == DataType.ULONG_ARRAY:
        msg.putUInt64Array(value)
    elif type == DataType.FLOAT:
        msg.putFloat(value)
    elif type == DataType.FLOAT_ARRAY:
        msg.putFloatArray(value)
    elif type == DataType.DOUBLE:
        msg.putDouble(value)
    elif type == DataType.DOUBLE_ARRAY:
        msg.putDoubleArray(value)
    elif type == DataType.STRING:
        msg.putString(value)
    elif type == DataType.STRING_ARRAY:
        msg.putStringArray(value)
    else:
        rospy.logwarn("Attempted to add value '{}' of unknown type {}, not adding any value !".format(value, type))


def getTypedValueFromMessage(msg: Message, type: int) -> ROS_TYPE:
    """
    Append the value of the specified type to the message

    :param msg: Message to add the data to
    :param type: DataType to add value as
    :param value: value to add
    :return:
    """
    if type == DataType.BOOL:
        return msg.getBool()
    elif type == DataType.BOOL_ARRAY:
        return msg.getBoolArray()
    elif type == DataType.BYTE:
        return msg.getUInt8()
    elif type == DataType.BYTE_ARRAY:
        return msg.getUInt8Array()
    elif type == DataType.SBYTE:
        return msg.getInt8()
    elif type == DataType.SBYTE_ARRAY:
        return msg.getInt8Array()
    elif type == DataType.SHORT:
        return msg.getInt16()
    elif type == DataType.SHORT_ARRAY:
        return msg.getInt16Array()
    elif type == DataType.USHORT:
        return msg.getUInt16()
    elif type == DataType.USHORT_ARRAY:
        return msg.getUInt16Array()
    elif type == DataType.INT:
        return msg.getInt32()
    elif type == DataType.INT_ARRAY:
        return msg.getInt32Array()
    elif type == DataType.UINT:
        return msg.getUInt32()
    elif type == DataType.UINT_ARRAY:
        return msg.getUInt32Array()
    elif type == DataType.LONG:
        return msg.getInt64()
    elif type == DataType.LONG_ARRAY:
        return msg.getInt64Array()
    elif type == DataType.ULONG:
        return msg.getUInt64()
    elif type == DataType.ULONG_ARRAY:
        return msg.getUInt64Array()
    elif type == DataType.FLOAT:
        return msg.getFloat()
    elif type == DataType.FLOAT_ARRAY:
        return msg.getFloatArray()
    elif type == DataType.DOUBLE:
        return msg.getDouble()
    elif type == DataType.DOUBLE_ARRAY:
        return msg.getDoubleArray()
    elif type == DataType.STRING:
        return msg.getString()
    elif type == DataType.STRING_ARRAY:
        return msg.getStringArray()
    else:
        rospy.logwarn("Attempted to get value of unknown type {} !".format(type))

#region Mesage Factory
#region Client

def client_unified_log(time_s: int, time_ns: int, origin: str, loglevel: int, name: str, msg: str, file: str,
                       function: str, line: int, topics: List[str]) -> Message:
    """
    Create a client log message

    :param time_s: Time in seconds since UNIX Epoch
    :param time_ns: Time offset in nanoseconds
    :param origin: Component name of the component the message originates from
    :param loglevel: Log Level of the message
    :param name: Name of the Logger to use
    :param msg: Actual log message
    :param file: Filename of the file the error occured in
    :param function: Name of the function the error occured in
    :param line: Line number of the line causing the error
    :param topics: List of Topics
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.LOG)
    message.addInt(time_s)
    message.addInt(time_ns)
    message.addString(origin)
    message.addByte(loglevel)
    message.addString(name)
    message.addString(msg)
    message.addString(file)
    message.addString(function)
    message.addInt(line)
    message.addStringArray(topics)

    return message


def client_list_parameters(requestID: str) -> Message:
    """
    Request a list of parameters on ROS's parameter server

    :param namespace: Namespace of parameters to look for, set ''/'' as default
    :param requestID: Unique Identifier mirrored by the server in the response
    :return: :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.LIST_PARAMETER)

    message.addString(requestID)

    return message


def client_request_parameter(parameterName: str, requestID: str) -> Message:
    """
    Request a parameter

    :param parameterName: Name of the parameter to request
    :param requestID: Unique Identifier mirrored by the server in the response
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.REQUEST_PARAMETER)

    message.addString(parameterName)
    message.addString(requestID)

    return message


def client_set_parameter(parameterName: str, parameterValue: ROS_TYPE) -> Message:
    """
    Set a parameter

    :param parameterName: Name of the parameter to request
    :param parameterType: Byte ID of the parameter's type
    :param parameterValue: Value of the Parameter to set

    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.SET_PARAMETER)

    message.addString(parameterName)

    typeArray, valueArray = build_type_array(parameterValue)
    message.addByteArray(typeArray)
    for val in valueArray:
        addTypedValueToMessage(message, PARAMETER_TYPE_MAPPING[type(val)], val)

    return message


def client_list_topics(namespace: str, requestID: str) -> Message:
    """
    List all topics

    :param namespace: Namespace of topics to look for
    :param requestID: Unique Identifier mirrored by the server in the response

    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.LIST_TOPIC)

    message.addString(namespace)
    message.addString(requestID)

    return message


def client_request_message_structure(messageName: str, requestID: str) -> Message:
    """
    Requests the structure of a message

    :param messageName: Name of the message whose structure to fetch
    :param requestID: Unique Identifier mirrored by the server in the response
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.REQUEST_MESSAGE_STRUCTURE)

    message.addString(messageName)
    message.addString(requestID)

    return message


def client_request_subscription(topic: str, type: str) -> Message:
    """
    Request as a Subscription

    :param topic: Topic name to subscribe to
    :param type: Type of the message
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.SUBSCRIBE)

    message.addString(topic)
    message.addString(type)

    return message


def client_request_unsubscription(topic: str) -> Message:
    """
    Request to unsubscribe

    :param topic: Topic name to unsubscribe from
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.UNSUBSCRIBE)

    message.addString(topic)
    message.addString(type)

    return message


def client_create_publisher(topic: str, type: str, queue: int):
    """
    Register a new publisher

    :param topic: Topic name to publish to
    :param type: Type of the message
    :param queue: Size of the publisher's queue
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ClientToServer.CREATE_PUBLISHER)

    message.addString(topic)
    message.addString(type)
    message.addInt(queue)

    return message


def client_create_ros_message(topic: str, message: Any):
    """
    Sends a message to ROS

    :param topic: Topic name to send the message to
    :param message: ROS Message to send
    :return: the created message
    """
    msg = create(MessageSendMode.Unreliable, ClientToServer.ROS_MESSAGE)

    msg.addString(topic)
    msg.addString(msg._type)
    serialize(msg, value=message)

    return msg


def client_call_service(topic: str, type: str, message: Any, requestID: str) -> Message:
    """
    Calls the service

    :param topic: Topic name of the service
    :param type: Type of the message
    :param message: ROS Message to send
    :param requestID: Unique Identifier mirrored by the server in the response

    :return: the created message
    """
    msg = create(MessageSendMode.Unreliable, ClientToServer.SERVICE_CALL)

    msg.addString(topic)
    msg.addString(type)
    serialize(msg, value=message)
    msg.addString(requestID)

    return msg

#endregion


#region Server

def server_unified_log(time_s: int, time_ns: int, origin: str, loglevel: int, name: str, msg: str, file: str,
                       function: str, line: int, topics: List[str]) -> Message:
    """
    Create a client log message

    :param time_s: Time in seconds since UNIX Epoch
    :param time_ns: Time offset in nanoseconds
    :param origin: Component name of the component the message originates from
    :param loglevel: Log Level of the message
    :param name: Name of the Logger to use
    :param msg: Actual log message
    :param file: Filename of the file the error occured in
    :param function: Name of the function the error occured in
    :param line: Line number of the line causing the error
    :param topics: List of Topics
    :return: the created message
    """
    message = create(MessageSendMode.Unreliable, ServerToClient.LOG)
    message.addInt(time_s)
    message.addInt(time_ns)
    message.addString(origin)
    message.addByte(loglevel)
    message.addString(name)
    message.addString(msg)
    message.addString(file)
    message.addString(function)
    message.addInt(line)
    message.addStringArray(topics)

    return message


def server_parameter_list_response(parameters: List[str], requestID: str) -> Message:
    """
    Sends a list of parameters

    :param parameters: Parameters to send
    :param requestID: Unique Identifier mirrored by the server from the request
    :return: the created Message
    """
    message = create(MessageSendMode.Unreliable, ServerToClient.PARAMETER_LIST)

    message.addStringArray(parameters)
    message.addString(requestID)

    return message


def server_parameter_response(parameterName: str, parameterValue: ROS_TYPE_EXTENDED, requestID: str) -> Message:
    """
    Parameter response

    :param parameterName: Name of the parameter requested
    :param parameterType: Byte ID of the parameter’s type, see DataTypes
    :param parameterValue: Value if the parameter
    :param requestID: Unique Identifier mirrored by the server from the request
    :return: the created Message
    """
    message = create(MessageSendMode.Unreliable, ServerToClient.PARAMETER)

    message.addString(parameterName)

    typeArray, valueArray = build_type_array(parameterValue)
    message.addByteArray(typeArray)
    for val in valueArray:
        addTypedValueToMessage(message, PARAMETER_TYPE_MAPPING[type(val)], val)

    message.addString(requestID)

    return message


def server_topic_list_response(topics: List[Tuple[str, str]], requestID: str) -> Message:
    """
    Response containing a list of topics

    :param topics: Topics to send
    :param requestID: Unique Identifier mirrored by the server in the response
    :return: the created Message
    """
    message = create(MessageSendMode.Unreliable, ServerToClient.TOPIC_LIST)

    message.putArrayLength(len(topics))
    for topic in topics:
        message.addString(topic[0])
        message.addString(topic[1])
    message.addString(requestID)

    return message


def server_message_structure(name: str, types: List[str], requestID: str) -> Message:
    """
    Response containing a message structure

    :param name: Message Name Type
    :param types: List of Types the (expanded) message definition consists of
    :param requestID: Unique Identifier mirrored by the server in the response
    :return: the created Message
    """

    message = create(MessageSendMode.Unreliable, ServerToClient.MESSAGE_STRUCTURE)

    message.addString(name)
    message.addStringArray(types)
    message.addString(requestID)

    return message


def server_message_subscription_ack(topic: str, status: int, message: str) -> Message:
    """
    Acknoledgement of Subscription

    :param topic: Topic name to subscribe to
    :param status: Status
    :param message: Error message if any
    :return: the created Message
    """

    msg = create(MessageSendMode.Unreliable, ServerToClient.SUBSCRIBE_ACK)

    msg.addString(topic)
    msg.addInt(status)
    msg.addString(message)

    return msg


def server_message_unsubscription_ack(topic: str, status: int, message: str) -> Message:
    """
    Acknoledgement of Subscription

    :param topic: Topic name to subscribe to
    :param status: Status
    :param message: Error message if any
    :return: the created Message
    """

    msg = create(MessageSendMode.Unreliable, ServerToClient.UNSUBSCRIBE_ACK)

    msg.addString(topic)
    msg.addInt(status)
    msg.addString(message)

    return msg


def server_create_ros_message(topic: str, message: Any):
    """
    Sends a message to ROS

    :param topic: Topic name to send the message to
    :param message: ROS Message to send
    :return: the created message
    """
    msg = create(MessageSendMode.Unreliable, ServerToClient.ROS_MESSAGE)

    msg.addString(topic)
    msg.addString(message._type)
    serialize(msg, value=message)

    return msg


def server_call_service(requestID: str, type: str, message: ROSMessageBase) -> Message:
    """
    Calls the service

    :param requestID: Unique Identifier mirrored by the server in the response
    :param type: Type of the message
    :param message: ROS Message to send

    :return: the created message
    """
    msg = create(MessageSendMode.Unreliable, ServerToClient.SERVICE_RESPONSE)

    msg.addString(requestID)
    msg.addString(type)
    serialize(msg, value=message)

    return msg


def server_text_broadcast(message: str, format: str, origin: str) -> Message:
    """
    Text Broadcast from server components

    :param msg: Message Content
    :param format: Format of the message
    :param origin: Origin of the message
    :return: the created message
    """
    msg = create(MessageSendMode.Unreliable, ServerToClient.TEXT_BROADCAST)

    msg.addString(message)
    msg.addString(format)
    msg.addString(origin)

    return msg

#endregion
#endregion