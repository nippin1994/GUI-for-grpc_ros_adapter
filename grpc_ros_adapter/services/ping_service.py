#!/usr/bin/env python3

import sys
import os
import logging

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from protobuf import ping_pb2
from protobuf import ping_pb2_grpc

logging.basicConfig(level=logging.DEBUG)
logging.debug("Starting PingService server...")


class PingService(ping_pb2_grpc.PingServicer):
    """
    Service used to send and receive dummy data to see if connection 
    is established
    """
    def __init__(self):
        pass

    def Ping(self, request, context):
        """
        Handles ping request
        """
        request.value = 1
        return request
