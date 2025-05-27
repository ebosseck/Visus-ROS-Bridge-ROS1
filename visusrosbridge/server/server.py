import time
from typing import Dict, List

from roslib import message as roslibmsg
import rospy
from rospy import Publisher, Subscriber, ROSException

from pytidenetworking.connection import Connection
from pytidenetworking.message import Message
from pytidenetworking import message as Msg
from pytidenetworking.peer import DisconnectReason, DISCONNECT_REASON_STR
from pytidenetworking.server import Server
from pytidenetworking.threading.fixedupdatethreads import FixedUpdateThread
from pytidenetworking.transports.tcp.tcp_server import TCPServer
from visusrosbridge.messages import messagefactory
from visusrosbridge.messages.genericrosmessage import deserialize
from visusrosbridge.messages.messagetypes import ClientToServer
from visusrosbridge.unified_logging import logmanager

from rosgraph_msgs.msg import Log as ROSLogMessage

class ROSServer:

    def __init__(self, port, maxClients: int = 10):
        Msg.maxPayloadSize = 63 * 1024 # 63 Kb

        self.tcpTransport = TCPServer()
        self.server: Server = Server(self.tcpTransport)
        self.server.start(port, maxClients)

        self.serverUpdater: FixedUpdateThread = FixedUpdateThread(self.server.update)

        self.publishers: Dict[str, Publisher] = {}
        self.publishingClients: Dict[str, List[int]] = {}

        self.subscriptions: Dict[str, Subscriber] = {}
        self.subscribedClients: Dict[str, List[int]] = {}

        self.server.ClientDisconnected += self.onClientDisconnected

    def onClientConnected(self, client: Connection):
        rospy.loginfo("Client {} Connected".format(client.id))

    def onClientDisconnected(self, client: Connection, reason: DisconnectReason):
        rospy.loginfo("Client {} disconnected with reason {}".format(client.id, DISCONNECT_REASON_STR[reason]))
        clientID = client.id

        for key in self.subscribedClients:
            if clientID in self.subscribedClients[key]:
                self.subscribedClients[key].remove(clientID)
                if len(self.subscribedClients[key]) == 0:
                    self.subscriptions[key].unregister()
                    del self.subscriptions[key]

        for key in self.publishingClients:
            if clientID in self.publishingClients[key]:
                self.publishingClients[key].remove(clientID)
                if len(self.publishingClients[key]) == 0:
                    self.publishers[key].unregister()
                    del self.publishers[key]

    def start(self):
        self.__addCallbacks()
        self.serverUpdater.start()
        self.__subscribeToTopics()
        time.sleep(0.01)
        rospy.loginfo("Started TCP Bridge on port {}".format(self.server.port))

    def close(self):
        self.serverUpdater.requestClose()
        self.serverUpdater.join()

    def __subscribeToTopics(self):
        rospy.Subscriber("/rosout", ROSLogMessage, self.callback_rosout)

    def callback_rosout(self, msg: ROSLogMessage):
        time = msg.header.stamp

        logmanager.log(time.secs, time.nsecs, "ROS", msg.level, msg.name, msg.msg, msg.file, msg.function, msg.line,
                       msg.topics)

        #TODO Uncomment & Fix
#        self.server.sendToAll(
#            messagefactory.server_unified_log(time.secs, time.nsecs, msg.header.frame_id, msg.level, msg.name, msg.msg, msg.file,
#                                              msg.function, msg.line, msg.topics))

    def subscriptionCallback(self, msg, topic: str):
        clientIDs = self.subscribedClients[topic]

        message = messagefactory.server_create_ros_message(topic, msg)

        for client in clientIDs:
            self.server.send(message, client, False)

        message.release()

    def __addCallbacks(self):
        self.server.registerMessageHandler(ClientToServer.LOG, self.onLog)
        self.server.registerMessageHandler(ClientToServer.LIST_PARAMETER, self.onListParameter)
        self.server.registerMessageHandler(ClientToServer.REQUEST_PARAMETER, self.onRequestParameter)
        self.server.registerMessageHandler(ClientToServer.SET_PARAMETER, self.onSetParameter)
        self.server.registerMessageHandler(ClientToServer.LIST_TOPIC, self.onListTopics)
        self.server.registerMessageHandler(ClientToServer.REQUEST_MESSAGE_STRUCTURE, self.onMessageStructureRequest)
        self.server.registerMessageHandler(ClientToServer.SUBSCRIBE, self.onSubscribe)
        self.server.registerMessageHandler(ClientToServer.UNSUBSCRIBE, self.onUnsubscribe)
        self.server.registerMessageHandler(ClientToServer.CREATE_PUBLISHER, self.onCreatePublisher)
        self.server.registerMessageHandler(ClientToServer.ROS_MESSAGE, self.onROSMessage)
        self.server.registerMessageHandler(ClientToServer.SERVICE_CALL, self.onCallService)
        self.server.registerMessageHandler(ClientToServer.SERVICE_RESPONSE, self.onServiceResponse)


    def onLog(self, connectionID: int, message: Message):

        time_s = message.getInt()
        time_ns = message.getInt()
        origin = message.getString()
        logLevel = message.getByte()

        name = message.getString()
        msg = message.getString()
        file = message.getString()
        function = message.getString()
        line = message.getInt()
        topics = message.getStringArray()

        logmanager.log(time_s, time_ns, origin, logLevel, name, msg, file, function, line, topics)

        self.server.sendToAll(
            messagefactory.server_unified_log(time_s, time_ns, origin, logLevel, name, msg, file, function, line, topics),
            connectionID)

    def onListParameter(self, connectionID: int, message: Message):
        requestID = message.getString()
        try:
            parameters = rospy.get_param_names()
            self.server.send(messagefactory.server_parameter_list_response(parameters=parameters, requestID=requestID),
                             connectionID)
        except Exception as ex:
            rospy.logerr(ex)

    def onRequestParameter(self, connectionID: int, message: Message):
        parameter = message.getString()
        requestID = message.getString()

        try:
            param = rospy.get_param(parameter)
        except KeyError:
            rospy.logwarn("Parameter not found on the parameter server: '{}'".format(parameter))
            param = None

        self.server.send(messagefactory.server_parameter_response(parameter, param, requestID), connectionID)

    def onSetParameter(self, connectionID: int, message: Message):
        rospy.logerr("Not Implemented")
        pass # TODO: Handle

    def onListTopics(self, connectionID: int, message: Message):
        namespace = message.getString()
        requestID = message.getString()

        topics = rospy.get_published_topics(namespace)

        self.server.send(messagefactory.server_topic_list_response(topics, requestID), connectionID)

    def onMessageStructureRequest(self, connectionID: int, message: Message):
        rospy.logerr("Not Implemented")
        pass # TODO: Handle

    def onSubscribe(self, connectionID: int, message: Message):
        topic = message.getString()
        type = message.getString()

        rospy.loginfo("Subscribing to {} with type {} from Connection {}".format(topic, type, connectionID))

        status = 0
        msg = "OK"

        if topic not in self.subscriptions:
            try:
                self.subscriptions[topic] = rospy.Subscriber(topic, roslibmsg.get_message_class(type),
                                                         self.subscriptionCallback, callback_args=topic)
            except ROSException as ex:
                rospy.logerr(ex)
                status = 1
                msg = str(ex)

        if topic not in self.subscribedClients:
            self.subscribedClients[topic] = [connectionID]
        else:
            self.subscribedClients[topic].append(connectionID)

        self.server.send(messagefactory.server_message_subscription_ack(topic, status, msg), connectionID)

    def onUnsubscribe(self, connectionID: int, message: Message):
        topic = message.getString()

        if topic in self.subscribedClients:
            if connectionID in self.subscribedClients[topic]:
                self.subscribedClients[topic].remove(connectionID)
            if len(self.subscribedClients[topic]) == 0:
                self.subscriptions[topic].unregister()
                del self.subscriptions[topic]
                del self.subscribedClients[topic]
        else:
            self.subscriptions[topic].unregister()
            del self.subscriptions[topic]

    def onCreatePublisher(self, connectionID: int, message: Message):
        topic = message.getString()
        type = message.getString()
        queue = message.getInt()
        latch = message.getBool()

        status = 0
        msg = "OK"

        if topic not in self.publishers:
            try:
                self.publishers[topic] = rospy.Publisher(topic, roslibmsg.get_message_class(type), queue_size=queue,
                                                         latch=latch)
            except ROSException as ex:
                rospy.logerr(ex)
                status = 1
                msg = str(ex)

        if topic not in self.publishingClients:
            self.publishingClients[topic] = [connectionID]
        else:
            self.publishingClients[topic].append(connectionID)

    def onROSMessage(self, connectionID: int, message: Message):
        topic = message.getString()
        type = message.getString()

        msg = roslibmsg.get_message_class(type)()
        deserialize(message, value=msg)
        if topic in self.publishers:
            self.publishers[topic].publish(msg)

    def onCallService(self, connectionID: int, message: Message):
        pass # TODO: Handle by calling service and sending the response

    def onServiceResponse(self, connectionID: int, message: Message):
        pass #TODO