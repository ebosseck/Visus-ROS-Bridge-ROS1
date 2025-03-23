from typing import List

from visusrosbridge.unified_logging.ilogger import ILogger
from base.matrix import MatrixClient

FATAL = 16
ERROR = 8
WARN = 4
INFO = 2
DEBUG = 1

NOTICE_THRESHOLD = 3

class MatrixLogger(ILogger):
    
    def __init__(self):
        super().__init__()
        self.client = MatrixClient()
        self.client.start()

        self.client.sendMessageHTML("<font color=#000000 data-mx-bg-color=#ffaa00> {} </font>".format(
            "".join(["&nbsp;&nbsp;" * 42, "<b>RESTART</b>", "&nbsp;&nbsp;" * 42]),
        ), isNotice=False)
        self.minimum_log_level = DEBUG

    def getType(self) -> str:
        return "MATRIX"

    def log(self, time_s: int, time_ns: int, origin: str, log_level: int, name: str, msg: str, file: str,
            function: str, line: int, topics: List[str]):
        format = ""

        metadata = "<i>at {}:{}, function {}</i>".format(file, line, function)

        if log_level >= FATAL:
            format = "<font color=#ff0088>[FATAL] {} {}</font> <font color=#888888>from component </font><font color=#ff8800>{}</font>"
        elif log_level >= ERROR:
            format = "<font color=#ff0000>[ERROR] {} {}</font> <font color=#888888>from component </font><font color=#ff8800>{}</font>"
        elif log_level >= WARN:
            format = "<font color=#ffaa00>[WARN]</font> {} {} <font color=#888888>from component </font><font color=#ff8800>{}</font>"
        elif log_level >= INFO:
            format = "<font color=#0088ff>[INFO]</font> {} {} <font color=#888888>from component </font><font color=#ff8800>{}</font>"
        else: # DEBUG
            format = "<font color=#00ff00>[DEBUG]</font> {} {} <font color=#888888>from component </font><font color=#ff8800>{}</font>"

        self.client.sendMessageHTML(format.format(msg, metadata, origin), isNotice=log_level < NOTICE_THRESHOLD)