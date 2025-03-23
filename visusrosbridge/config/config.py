import json
import os
from typing import AnyStr

CONFIG_DATA = {}

def loadConfig(path : str):
    global CONFIG_DATA
    CONFIG_DATA = json.loads(readFile(path))


def readFile(path) -> AnyStr:
    """
    Read the complete file

    :param path: Path to read from
    :return: the data of the file
    """
    print(os.getcwd())
    f = open(path, "r")
    result = ""
    try:
        result = f.read()
    finally:
        f.close()

    return result