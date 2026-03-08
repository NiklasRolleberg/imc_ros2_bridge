#!/usr/bin/python3

#Fix so relative paths work
import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

#Add path of pylsts so it can be imported correctly in other files
import sys
sys.path.append(os.path.abspath("pyimclsts/src/"))

import threading
import pyimclsts.network as imc_net
import pyimc_generated.messages as imc
import pyimc_generated.enumerations as imc_enum

#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import socket
import sys
import struct
import threading
from std_msgs.msg import String
from datetime import datetime
import struct
import signal
from imc_ros_bridge.msg import PlanControl as ros_PlanControl
from imc_ros_bridge.msg import PlanControlState as ros_PlanControlState
from imc_ros_bridge.msg import VehicleState as ros_VehicleState
from imc_ros_bridge.msg import EstimatedState as ros_EstimatedState
from imc_ros_bridge.msg import RemoteState as ros_RemoteState
from imc_ros_bridge.msg import SonarData as ros_SonarData
from imc_ros_bridge.msg import DesiredHeading as ros_DesiredHeading
from imc_ros_bridge.msg import DesiredHeadingRate as ros_DesiredHeadingRate
from imc_ros_bridge.msg import DesiredPitch as ros_DesiredPitch
from imc_ros_bridge.msg import DesiredRoll as ros_DesiredRoll
from imc_ros_bridge.msg import DesiredSpeed as ros_DesiredSpeed
from imc_ros_bridge.msg import DesiredZ as ros_DesiredZ
from imc_ros_bridge.msg import PlanDB as ros_PlanDB
from imc_ros_bridge.msg import PlanSpecification as ros_PlanSpecification
from imc_ros_bridge.msg import PlanDBInformation as ros_PlanDBInformation
from imc_ros_bridge.msg import PlanDBState as ros_PlanDBState
from imc_ros_bridge.msg import PlanManeuver as ros_PlanManeuver
from imc_ros_bridge.msg import Maneuver as ros_Maneuver


class ImcRosBridge:

    def __init__(self, server_ip, server_port, imc_src, node):

        #imc
        self.imc_to_send_callback = None
        self.sock = imc_net.tcp_interface(server_ip, server_port)
        self.sub = imc_net.subscriber(self.sock, use_mp=False)
        self.imc_src = imc_src

        #ros
        self._node = node

    def get_callback(self, callback):
        print("Get callback called")
        self.imc_to_send_callback = callback

    def run(self):
        #Get send callback
        self.sub.call_once(self.get_callback)

        #Create ros subscribers and publishers
        self.ros_subscribe_publish()

        #Subscribe to messages (how to subscribe to all?)
        self.imc_subscribe()

        #self.sub.periodic_async(_send_area, period=5.0)

        self.sub.run()

    def stop(self):
        self.sub.stop()


    def ros_subscribe_publish(self):
        self._node.get_logger().info("Creating ros publishers and subscribers")
        #Estimated state
        self.ros_subsciber_EstimatedState = self._node.create_subscription(ros_EstimatedState,"imc/out/estimatedstate", self.ros_callback_EstimatedState,10)
        self.ros_publisher_EstimatedState = self._node.create_publisher(ros_EstimatedState, "imc/in/estimatedstate", 10)

        #Desired heading
        self.ros_subsciber_DesiredHeading = self._node.create_subscription(ros_DesiredHeading,"imc/out/desiredheading", self.ros_callback_DesiredHeading,10)
        self.ros_publisher_DesiredHeading = self._node.create_publisher(ros_DesiredHeading, "imc/in/desiredheading", 10)

        #Desired speed
        self.ros_subsciber_DesiredSpeed = self._node.create_subscription(ros_DesiredSpeed,"imc/out/desiredspeed", self.ros_callback_DesiredSpeed,10)
        self.ros_publisher_DesiredSpeed = self._node.create_publisher(ros_DesiredSpeed, "imc/in/desiredspeed", 10)

    def imc_subscribe(self):
        self._node.get_logger().info("Creating imc subscribers")
        self.sub.subscribe_async(self.imc_callback_EstimatedState, msg_id=imc.EstimatedState)
        self.sub.subscribe_async(self.imc_callback_DesiredHeading, msg_id=imc.DesiredHeading)
        self.sub.subscribe_async(self.imc_callback_DesiredSpeed, msg_id=imc.DesiredSpeed)
        #self.sub.subscribe_async(self.imc_callback_VehicleState, msg_id=imc.VehicleState)  
        
        


##################################################################################3##################
#                                ROS callbacks                                                      #
##################################################################################3##################
    def ros_callback_EstimatedState(self,ros_msg : ros_EstimatedState):
        #self._node.get_logger().info(f"Received ROS message: {ros_msg}")
        if(self.imc_to_send_callback == None):
            self._node.get_logger().error("Error could not send")
            return
        imc_msg = imc.EstimatedState(
            lat = ros_msg.lat, 
            lon = ros_msg.lon, 
            height =ros_msg.height, 
            x = ros_msg.x, 
            y = ros_msg.y, 
            z = ros_msg.z, 
            phi = ros_msg.phi, 
            theta = ros_msg.theta, 
            psi = ros_msg.psi, 
            u = ros_msg.u, 
            v = ros_msg.v, 
            w = ros_msg.w, 
            vx = ros_msg.vx, 
            vy = ros_msg.vy, 
            vz = ros_msg.vz, 
            p = ros_msg.p, 
            q = ros_msg.q, 
            r = ros_msg.r, 
            depth = ros_msg.depth, 
            alt = ros_msg.alt
        )
        self.imc_to_send_callback(message = imc_msg, src = self.imc_src, dst = 0xFF)

    def ros_callback_DesiredHeading(self,ros_msg : ros_DesiredHeading):
        #self._node.get_logger().info(f"Received ROS message: {ros_msg}")
        if(self.imc_to_send_callback == None):
            self._node.get_logger().error("Error could not send")
            return
        imc_msg = imc.DesiredHeading(ros_msg.value)
        self.imc_to_send_callback(imc_msg, self.imc_src, dst = 0xFF)

    def ros_callback_DesiredSpeed(self,ros_msg : ros_DesiredSpeed):
        #self._node.get_logger().info(f"Received ROS message: {ros_msg}")
        if(self.imc_to_send_callback == None):
            self._node.get_logger().error("Error could not send")
            return
        imc_msg = imc.DesiredSpeed(ros_msg.value, ros_msg.speed_units)
        self.imc_to_send_callback(imc_msg, src = self.imc_src, dst = 0xFF)
        


##################################################################################3##################
#                                IMC callbacks                                                      #
##################################################################################3##################
    def imc_callback_EstimatedState(self, imc_msg: imc.EstimatedState, callback):
        #print(f"Received imc message: {imc_msg}")
        ros_msg = ros_EstimatedState()
        ros_msg.lat = imc_msg.lat
        ros_msg.lon = imc_msg.lon
        ros_msg.height = imc_msg.height
        ros_msg.x = imc_msg.x
        ros_msg.y = imc_msg.y
        ros_msg.z = imc_msg.z
        ros_msg.phi = imc_msg.phi
        ros_msg.theta = imc_msg.theta
        ros_msg.psi = imc_msg.psi
        ros_msg.u = imc_msg.u
        ros_msg.v = imc_msg.v
        ros_msg.w = imc_msg.w
        ros_msg.vx = imc_msg.vx
        ros_msg.vy = imc_msg.vy
        ros_msg.vz = imc_msg.vz
        ros_msg.p = imc_msg.p
        ros_msg.q = imc_msg.q
        ros_msg.r = imc_msg.r
        ros_msg.depth = imc_msg.depth
        ros_msg.alt = imc_msg.alt
        self.ros_publisher_EstimatedState.publish(ros_msg)


    def imc_callback_DesiredHeading(self, imc_msg: imc.DesiredHeading, callback):
        #print(f"Received imc message: {imc_msg}")
        ros_msg = ros_DesiredHeading()
        ros_msg.value = imc_msg.value
        self.ros_publisher_DesiredHeading.publish(ros_msg)
        

    def imc_callback_DesiredSpeed(self, imc_msg: imc.DesiredSpeed, callback):
        #print(f"Received imc message: {imc_msg}")
        ros_msg = ros_DesiredSpeed()
        ros_msg.value = imc_msg.value
        self.ros_publisher_DesiredSpeed.publish(ros_msg)


def main(args=None, namespace=None):
    rclpy.init(args=args)

    _node = Node('imc_ros_bridge')

    _node.declare_parameter('server_ip', "127.0.0.1")
    ip_address = _node.get_parameter('server_ip').value

    _node.declare_parameter('tcp_port', 7001)
    port = _node.get_parameter('tcp_port').value

    _node.declare_parameter('imc_src', 0x0806)
    imc_src = _node.get_parameter('imc_src').value

    _node.get_logger().info(f"Server ip: {ip_address}")
    _node.get_logger().info(f"Server port: {port}")
    _node.get_logger().info(f"imc src: {imc_src}")


    bridge = ImcRosBridge(ip_address, port, imc_src, _node)
    imc_thread = threading.Thread(target=bridge.run)
    print("Main    : before running thread")
    imc_thread.start()
    print("Main    : wait for the thread to finish")

    while rclpy.ok():
        rclpy.spin_once(_node)
    
    bridge.stop()
    imc_thread.join()
    
    


if __name__ == "__main__":
    main()



    

