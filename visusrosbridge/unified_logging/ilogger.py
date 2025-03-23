from typing import List


class ILogger:

    def __init__(self):
        pass

    def getType(self) -> str:
        return "BASE"

    def log(self, time_s: int, time_ns: int, origin: str, log_level: int, name: str, msg: str, file: str,
            function: str, line: int, topics: List[str]):
        pass

    def close(self):
        pass