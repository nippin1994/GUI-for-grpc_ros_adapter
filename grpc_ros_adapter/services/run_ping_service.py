#!/usr/bin/env python3
import sys
import os
import rospy
import grpc
from concurrent import futures

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ping_service import PingService
from protobuf import ping_pb2_grpc

def serve():
    # Initialize the ROS node
    rospy.init_node('ping_service_node')

    # Create a gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    ping_pb2_grpc.add_PingServicer_to_server(PingService(), server)

    # Start the server on port 50051
    server.add_insecure_port('[::]:50051')
    server.start()

    rospy.loginfo("Ping service started on port 50051")

    # Keep the script running
    try:
        rospy.spin()
    except KeyboardInterrupt:
        server.stop(0)
        rospy.loginfo("Ping service stopped")

if __name__ == '__main__':
    serve()
