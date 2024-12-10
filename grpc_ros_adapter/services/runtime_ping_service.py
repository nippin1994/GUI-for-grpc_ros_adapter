#!/usr/bin/env python3

import rospy
from grpc_ros_adapter.srv import Ping, PingResponse  # Import the correct service type

def handle_ping_service(request):
    # Add logic for handling service requests here
    rospy.loginfo("Received ping request from client.")  # This is the log message
    return PingResponse("Pong")  # Or other response logic

rospy.init_node('ping_service_node')
service = rospy.Service('ping_service', Ping, handle_ping_service)  # Use Ping instead of PingService
rospy.loginfo("PingService is ready.")
rospy.spin()
