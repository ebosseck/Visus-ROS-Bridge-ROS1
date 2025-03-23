import rospy
from roslib import message

from visusrosbridge.messages.genericrosmessage import generateSerializer
from visusrosbridge.ros.tools import getTypeDefinition

if __name__ == "__main__":

    topics = rospy.get_published_topics()

    for topic in topics:
        msg = message.get_message_class(topic[1])

        print(topic[0] + '(Type: "' + topic[1] + '")')
        print('\t' + str(msg.__slots__))
        print('\t' + str(msg._slot_types))

        print(msg._type)

        generateSerializer(msg())

        for type in msg._slot_types:
            print(getTypeDefinition(type))

