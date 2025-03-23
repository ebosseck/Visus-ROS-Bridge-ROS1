from typing import List

from visusrosbridge.unified_logging.ilogger import ILogger

loggers = {}
enabledLoggers = {}


def loadLoggers():
    loggers = ILogger.__subclasses__()
    for logger in loggers:
        registerLogger(logger())


def registerLogger(logger: ILogger):
    loggers[logger.getType()] = logger


def enableLogger(loggerType: str):
    if loggerType in loggers:
        enabledLoggers[loggerType] = loggers[loggerType]


def disableLogger(loggerType):
    if loggerType in enabledLoggers:
        del enabledLoggers[loggerType]


def log(time_s: int, time_ns: int, origin: str, log_level: int, name: str, msg: str, file: str,
        function: str, line: int, topics: List[str]):
    for logger in enabledLoggers:
        enabledLoggers[logger].log(time_s, time_ns, origin, log_level, name, msg, file,
                                        function, line, topics)


def close():
    for logger in loggers:
        logger.close()