#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import tf
import ast
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    BatteryLvl,
    ImuData,
    ImuAck,
    InstantConsumption,
    EnableButton,
    ResourceMonitor,
    CurrentSpeed,
    CurrentSteer,
    WarningSignal,
    Semaphores,
    Location
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber

class threadRosBridgeRead(ThreadWithStop):
    """This thread read data from SerialHandler and publish them to ROS topic .
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """
    # ROS SPEED RANGE : (FLOAT) -5 ~ 5 [M/S]
    # BFMC SPEED RANGE : (INT) -500 ~ 500 [MM/S]

    # ROS STEER RANGE : (FLOAT) -0.401426 ~ 0.401426 [RAD] (ACKERMANN MSG)
    # BFMC STEER RANGE : (INT) -230 ~ 230 [DEGREE * 10]

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size= 10)
        self.subscribe()
        self.rate = rospy.Rate(10)
        
        #self.cur_state_pub = rospy.Publisher('/car_cur_state', CarState, queue_size=10)
        #To do: add car state msg

        super(threadRosBridgeRead, self).__init__()

    def run(self):
        
        while self._running and not rospy.is_shutdown():
            #imu receiver
            imuData = self.imuDataSubscriber.receive()
            if imuData is not None:
                imuData = ast.literal_eval(imuData)
                roll = float(imuData["roll"])
                pitch = float(imuData["pitch"])
                yaw = float(imuData["yaw"])
                accelx = float(imuData["accelx"])
                accely = float(imuData["accely"])
                accelz = float(imuData["accelz"])

                quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = 'imu_link'  # 센서 프레임 지정

                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]

                imu_msg.linear_acceleration.x = accelx
                imu_msg.linear_acceleration.y = accely
                imu_msg.linear_acceleration.z = accelz

                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0
                
                self.imu_pub.publish(imu_msg)
            #car_speed&steer receiver
            cur_speedData = self.currentSpeedSubscriber.receive()
            if cur_speedData is not None:
                print(f"cur_speedData:{cur_speedData}")
            
            cur_steerData = self.currentSteerSubscriber.receive()
            if cur_steerData is not None:
                print(f"cur_steerData:{cur_steerData}")

            #battery_operating time receiver
            # warningData = self.warningSubscriber.receive()
            # if warningData is not None:
            #     print(f"warningData:{warningData}")

            #battery_voltage level receiver
            # batteryLvlData = self.batteryLvlSubscriber.receive()
            # if batteryLvlData is not None:
            #     print(f"batterLvlData:{batteryLvlData}")    
             
            #location(nav) receiver
            locationData = self.locationSubscriber.receive()
            if locationData is not None:
                print(f"locationData:{locationData}")
            #semaphores(traffic) receiver
            semaphoresData = self.semaphoresSubscriber.receive()
            if semaphoresData is not None:
                print(f"semaphoresData:{semaphoresData}")
            

            
            
    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.enableButtonSubscriber = messageHandlerSubscriber(self.queuesList, EnableButton, "lastOnly", True)
        # self.batteryLvlSubscriber = messageHandlerSubscriber(self.queuesList, BatteryLvl, "lastOnly", True)
        # self.instantConsumptionSubscriber = messageHandlerSubscriber(self.queuesList, InstantConsumption, "lastOnly", True)
        self.imuDataSubscriber = messageHandlerSubscriber(self.queuesList, ImuData, "lastOnly", True)
        # self.imuAckSubscriber = messageHandlerSubscriber(self.queuesList, ImuAck, "lastOnly", True)
        # self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ResourceMonitor, "lastOnly", True)
        self.currentSpeedSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastOnly", True)
        self.currentSteerSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSteer, "lastOnly", True)
        # self.warningSubscriber = messageHandlerSubscriber(self.queuesList, WarningSignal, "lastOnly", True)
        self.semaphoresSubscriber = messageHandlerSubscriber(self.queuesList, Semaphores, "lastOnly", True)
        self.locationSubscriber = messageHandlerSubscriber(self.queuesList, Location, "lastOnly", True)

