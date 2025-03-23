#!/usr/bin/env python3
import os
import sys

import rospy

from visusrosbridge.config import config as cfg
from visusrosbridge.plugin import moduleloader
from visusrosbridge.server.server import ROSServer
from visusrosbridge.unified_logging import logmanager

FATAL = 16
ERROR = 8
WARN = 4
INFO = 2
DEBUG = 1

def loadModules(path: str):
    sys.path.append(path)
    moduleloader.loadModulesFromPath(path)

if __name__ == "__main__":
    rospy.init_node('visus_bridge')
    os.chdir("/".join(__file__.split('/')[:-1]))

    cfg.loadConfig(rospy.get_param("config_path", "./config.json"))

    modules = cfg.CONFIG_DATA["modules"]
    for module in modules:
        loadModules(module)

    logmanager.loadLoggers()
    logmanager.enableLogger("MATRIX")

    #print(rospy.get_param_names())

    server = ROSServer(port=rospy.get_param("tcp_port", 10000), maxClients=rospy.get_param("max_clients", 10))
    server.start()

    rospy.spin()