import unittest

from std_msgs.msg import String
from turtlesim.msg import Color

from pytidenetworking.message import create
from pytidenetworking.message_base import MessageSendMode
from visusrosbridge.messages.genericrosmessage import GenericROSMessage


class MessageSerializationText(unittest.TestCase):
    def test_autoserialization(self):
        rosmessage = Color(255, 0, 0)

        pyMsgGeneric = GenericROSMessage(rosmessage)
        pytideMsg = create(MessageSendMode.Unreliable, 0)

        pyMsgGeneric.serialize(pytideMsg)

        pyMsgGeneric.value.r = 0
        pyMsgGeneric.value.g = 255

        print("({}, {}, {})".format(pyMsgGeneric.value.r, pyMsgGeneric.value.g, pyMsgGeneric.value.b))

        pyMsgGeneric.deserialize(pytideMsg)

        print("({}, {}, {})".format(pyMsgGeneric.value.r, pyMsgGeneric.value.g, pyMsgGeneric.value.b))

        #self.assertEqual("Hello World", value)  # add assertion here


if __name__ == '__main__':
    unittest.main()
