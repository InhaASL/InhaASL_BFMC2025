#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import ast
import math
import json
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
    Location,
    TrafficData
)

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from std_msgs.msg import String
import time

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

        # ROS 퍼블리셔 초기화
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.speed_pub = rospy.Publisher('/current_speed', AckermannDriveStamped, queue_size=10)
        self.traffic_pub = rospy.Publisher('/traffic_data', String, queue_size=10)
        self.semaphores_pub = rospy.Publisher('/semaphores_data', String, queue_size=10)
        self.cars_pub = rospy.Publisher('/cars_data', String, queue_size=10)

        # 메시지 객체 초기화
        self.drive_msg = AckermannDriveStamped()
        self.current_speed = 0
        self.current_steer = 0
        
        # 구독자 초기화
        self.subscribe()
        
        # 발행 주기 설정 (30Hz)
        self.rate = rospy.Rate(30)
        
        super(threadRosBridgeRead, self).__init__()

    def run(self):
        try:
            while self._running and not rospy.is_shutdown():
                try:
                    # IMU 데이터 처리
                    imuData = self.imuDataSubscriber.receive()
                    if imuData is not None:
                        try:
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
                            imu_msg.header.frame_id = 'imu_link'

                            imu_msg.orientation.x = quaternion[0]
                            imu_msg.orientation.y = quaternion[1]
                            imu_msg.orientation.z = quaternion[2]
                            imu_msg.orientation.w = quaternion[3]

                            imu_msg.linear_acceleration.x = accelx
                            imu_msg.linear_acceleration.y = accely
                            imu_msg.linear_acceleration.z = accelz

                            self.imu_pub.publish(imu_msg)
                            if self.debugging:
                                self.logging.info("Published IMU data")
                        except Exception as e:
                            self.logging.error(f"IMU data processing error: {str(e)}")

                    # 속도 및 조향 데이터 처리
                    cur_speedData = self.currentSpeedSubscriber.receive()
                    if cur_speedData is not None:
                        self.current_speed = cur_speedData
                    
                    cur_steerData = self.currentSteerSubscriber.receive()
                    if cur_steerData is not None:
                        self.current_steer = cur_steerData

                    self.drive_msg.header.stamp = rospy.Time.now()
                    self.drive_msg.header.frame_id = "base_link"
                    self.drive_msg.drive.speed = self.current_speed / 100
                    self.drive_msg.drive.steering_angle = math.radians(self.current_steer / 10)
                    self.speed_pub.publish(self.drive_msg)
                    if self.debugging:
                        self.logging.info("Published speed data")

                    # Traffic 데이터 처리
                    traffic_data = self.trafficSubscriber.receive()
                    if traffic_data is not None:
                        try:
                            self.logging.info(f"Received traffic data: {traffic_data}")
                            # 데이터 검증
                            if self.validate_traffic_data(traffic_data):
                                # JSON 형식으로 변환
                                traffic_msg = String()
                                traffic_msg.data = json.dumps(traffic_data)
                                self.traffic_pub.publish(traffic_msg)
                                self.logging.info("Published traffic data to ROS topic")
                            else:
                                self.logging.warning(f"Invalid traffic data format: {traffic_data}")
                        except Exception as e:
                            self.logging.error(f"Traffic data processing error: {str(e)}")

                    # Semaphores 데이터 처리
                    semaphoresData = self.semaphoresSubscriber.receive()
                    if semaphoresData is not None:
                        try:
                            if self.validate_semaphores_data(semaphoresData):
                                semaphores_msg = String()
                                semaphores_msg.data = json.dumps(semaphoresData)
                                self.semaphores_pub.publish(semaphores_msg)
                                if self.debugging:
                                    self.logging.info("Published semaphores data")
                        except Exception as e:
                            self.logging.error(f"Semaphores data processing error: {str(e)}")

                except Exception as e:
                    self.logging.error(f"Data processing error: {str(e)}")
                    continue

                self.rate.sleep()

        except Exception as e:
            self.logging.error(f"Thread execution error: {str(e)}")
            self.stop()

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
        self.trafficSubscriber = messageHandlerSubscriber(self.queuesList, TrafficData, "lastOnly", True)

    def validate_traffic_data(self, data):
        """트래픽 데이터 검증"""
        if not isinstance(data, dict):
            self.logging.warning("Traffic data is not a dictionary")
            return False
        
        # 서버에서 받은 데이터 형식에 맞게 수정
        required_fields = ['type', 'x', 'y', 'z', 'quality']
        if not all(field in data for field in required_fields):
            self.logging.warning(f"Missing required fields in traffic data. Required: {required_fields}, Got: {list(data.keys())}")
            return False
            
        try:
            # 데이터 타입 검증
            if data['type'] != 'traffic':
                self.logging.warning(f"Invalid message type: {data['type']}")
                return False
                
            float(data['x'])
            float(data['y'])
            float(data['z'])
            int(data['quality'])
            return True
        except (ValueError, TypeError) as e:
            self.logging.warning(f"Invalid data type in traffic data: {str(e)}")
            return False

    def validate_semaphores_data(self, data):
        """신호등 데이터 검증"""
        if not isinstance(data, dict):
            return False
        
        if data.get("device") == "semaphore":
            required_fields = ["id", "state", "x", "y"]
        elif data.get("device") == "car":
            required_fields = ["id", "x", "y"]
        else:
            return False
        
        return all(field in data for field in required_fields)

