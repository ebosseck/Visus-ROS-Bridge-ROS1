import asyncio
from asyncio import sleep
from asyncio.queues import Queue, QueueEmpty

from threading import Thread, Lock

from nio import AsyncClient, MatrixRoom, RoomMessageText

from visusrosbridge.config import config

class MatrixClient(Thread):

    def __init__(self):
        super(MatrixClient, self).__init__()

        clientConfig = config.CONFIG_DATA["notification"]["matrix"]

        self.server = clientConfig["server"]
        self.username = clientConfig["username"]
        self.password = clientConfig["password"]
        self.roomIDs = clientConfig["roomID"]

        self.shouldClose: bool = False
        self.client: AsyncClient = None
        self.clientLock = Lock()

        self.sendQueue = Queue()

    def run(self) -> None:
        self.connect()

    def connect(self):
        asyncio.run(self.__connectAsync())

    async def message_callback(self, room: MatrixRoom, event: RoomMessageText) -> None:
        pass
        # print(
        #    f"Message received in room {room.display_name}\n"
        #    f"{room.user_name(event.sender)} | {event.body}"
        #)

    async def __connectAsync(self):
        sendLoop = None

        with self.clientLock:

            self.client = AsyncClient(self.server, self.username)
            # noinspection PyTypeChecker
            self.client.add_event_callback(self.message_callback, RoomMessageText)

            try:
                await self.client.login(self.password)
                sendLoop = self._sendLoop()

            except Exception as ex:
                print(ex)

        await asyncio.gather(sendLoop, self.client.sync_forever(timeout=30000), return_exceptions=True) # milliseconds

    async def _sendLoop(self):
        while not self.shouldClose:
            if not self.client.logged_in:
                continue

            try:
                request = self.sendQueue.get_nowait()
            except QueueEmpty:
                await sleep(0.00001)
                continue

            await self.client.room_send(**request)
        await self._close()

    async def _close (self):
        await self.client.logout()

    def close (self):
        self.shouldClose = True

    def sendMessagePlainText (self, message: str, room: str = None, isNotice: bool = False):
        if room is None:
            room = self.roomIDs[0]

        data = {"room_id": room,
        "message_type":"m.room.message",
        "content":{"msgtype": "m.text" if not isNotice else "m.notice", "body": message}}

        self.sendQueue.put_nowait(data)

    def sendMessageHTML (self, message: str, room: str = None, isNotice: bool = False):
        if room is None:
            room = self.roomIDs[0]

        # m.notice
        data = {"room_id": room,
        "message_type":"m.room.message",
        "content":{"msgtype": "m.text" if not isNotice else "m.notice", "format": "org.matrix.custom.html",
                   "body": message,  "formatted_body": message}}

        self.sendQueue.put_nowait(data)