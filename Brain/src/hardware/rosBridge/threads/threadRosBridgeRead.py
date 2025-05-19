#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import ast
import math
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
import json

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

        # 큐 사이즈 작아서 데이터 손실될 수도 있음. 큐 사이즈 체크 필요함 (오버플로우) ex)예상 지연 허용: 0.2초 정도 → 30 × 0.2 = 6개
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size= 5)
        self.speed_pub = rospy.Publisher('/current_speed', AckermannDriveStamped, queue_size= 5)
        self.subscribe()
        self.rate = rospy.Rate(30) # 60--> 30Hz  변경했음 ,부담되는지 리소스 사용 확인 필요

        self.drive_msg = AckermannDriveStamped()
        self.current_speed = 0
        self.current_steer = 0
        
        #self.cur_state_pub = rospy.Publisher('/car_cur_state', CarState, queue_size=10)
        #To do: add car state msg

        self.traffic_pub = rospy.Publisher('/traffic_data', String, queue_size=10) # 트래픽 데이터 토픽 발행

        #semaphore & car data 토픽 발행
        self.semaphores_pub = rospy.Publisher('/semaphores_data', String, queue_size=10)
        self.cars_pub = rospy.Publisher('/cars_data', String, queue_size=10)

        super(threadRosBridgeRead, self).__init__()

    def run(self):
        try:
            while self._running and not rospy.is_shutdown():
                start_time = time.time()
                
                try:
                    # 토픽 구독자 수 확인 추가
                    if not self.semaphores_pub.get_num_connections():
                        self.logging.warning("Semaphores 토픽 구독자 없음")
                    if not self.cars_pub.get_num_connections():
                        self.logging.warning("Cars 토픽 구독자 없음")
                    if not self.traffic_pub.get_num_connections():
                        self.logging.warning("Traffic 토픽 구독자 없음")
                    if not self.imu_pub.get_num_connections():
                        self.logging.warning("IMU 토픽 구독자 없음")
                    if not self.speed_pub.get_num_connections():
                        self.logging.warning("Speed 토픽 구독자 없음")
                    
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
                        imu_msg.header.frame_id = 'imu_link' 

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
                    '''car_speed&steer receiver'''
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

                    '''battery_operating time receiver'''
                    # warningData = self.warningSubscriber.receive()
                    # if warningData is not None:
                    #     print(f"warningData:{warningData}")

                    '''battery_voltage level receiver'''
                    # batteryLvlData = self.batteryLvlSubscriber.receive()
                    # if batteryLvlData is not None:
                    #     print(f"batterLvlData:{batteryLvlData}")    
                     
                    '''location(nav) receiver'''
                    locationData = self.locationSubscriber.receive()
                    if locationData is not None:
                        print(f"locationData:{locationData}")
                    '''semaphores(traffic) receiver'''
                    semaphoresData = self.semaphoresSubscriber.receive()
                    if semaphoresData is not None:
                        # 데이터 검증
                        if self.validate_semaphores_data(semaphoresData):
                            # ROS 메시지로 변환
                            semaphores_msg = String()
                            semaphores_msg.data = json.dumps(semaphoresData)
                            self.semaphores_pub.publish(semaphores_msg)
                            
                            if self.debugging:
                                self.logging.info(f"Published semaphores data: {semaphoresData}")
                    
                    # Cars 데이터 처리 (semaphoresData에 car 정보도 포함되어 있음)
                    if semaphoresData is not None and semaphoresData.get("device") == "car":
                        cars_msg = String()
                        cars_msg.data = json.dumps(semaphoresData)
                        self.cars_pub.publish(cars_msg)
                        
                        if self.debugging:
                            self.logging.info(f"Published cars data: {semaphoresData}")

                    # TrafficCommunication 데이터 처리 추가
                    traffic_data = self.trafficSubscriber.receive()
                    if traffic_data is not None:
                        # 데이터 형식에 따라 적절한 ROS 메시지로 변환
                        traffic_msg = String()
                        traffic_msg.data = str(traffic_data)
                        self.traffic_pub.publish(traffic_msg)

                    # 처리 시간 모니터링
                    if time.time() - start_time > 0.1:
                        self.logging.warning("데이터 처리 지연 발생")
                    
                except Exception as e:
                    self.logging.error(f"데이터 처리 중 오류: {str(e)}")
                    continue
                
                self.rate.sleep()
        except Exception as e:
            self.logging.error(f"실행 중 치명적 오류: {str(e)}")
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

    def validate_traffic_data(self, data): # 데이터 검증 추가
        if not isinstance(data, dict):
            return False
        required_fields = ['devicePos', 'deviceRot', 'deviceSpeed']
        return all(field in data for field in required_fields)

    def validate_semaphores_data(self, data):
        """신호등 및 차량 데이터 검증"""
        if not isinstance(data, dict):
            return False
        
        if data.get("device") == "semaphore":
            required_fields = ["id", "state", "x", "y"]
        elif data.get("device") == "car":
            required_fields = ["id", "x", "y"]
        else:
            return False
        
        return all(field in data for field in required_fields)

