#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray , String
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
)

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
import time

class threadRosBridgeRead(ThreadWithStop):
    """Gateway → ROS 퍼블리셔 스레드."""
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

        # ───── ROS Publishers ─────초기화
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.speed_pub = rospy.Publisher('/current_speed', AckermannDriveStamped, queue_size=10)
        # self.traffic_pub = rospy.Publisher('/traffic_data', Float64MultiArray, queue_size=10)
        self.cars_pub = rospy.Publisher('/cars_data', String, queue_size=10)

        # 디버깅 true일때 디버깅 메세지 출력 
        if self.debugging:
            self.logging.info("ROS 퍼블리셔가 초기화되었습니다.")

        # ───── Gateway Subscribers ────초기화
        self.subscribe()

        # 메시지 객체 초기화
        self.drive_msg = AckermannDriveStamped()
        self.current_speed = 0
        self.current_steer = 0
        
        # 발행 주기 설정 (30Hz)
        self.rate = rospy.Rate(30)
        super(threadRosBridgeRead, self).__init__()

    # ──────────────────────────────────────────────────────────
    def run(self):
        try:
            if self.debugging: # 디버깅 메시지
                self.logging.info("threadRosBridgeRead 스레드가 시작되었습니다.")
                self.logging.info("ROS 노드 상태: " + ("초기화됨" if rospy.core.is_initialized() else "초기화되지 않음"))

            while self._running and not rospy.is_shutdown():
                try:
                    # 1) Location 처리 --------------------------------------------
                    loc = self.locationSubscriber.receive()                    
                    if loc and self._validate_location(loc):    # 이게 안 됐을때 방어 필요 
                        msg = Float64MultiArray()
                        msg.data = [
                            float(loc["x"]),
                            float(loc["y"]),
                            float(loc["z"]),
                            float(loc.get("quality", 1)),
                        ]
                        if self.debugging:
                            self.logging.info(f"/location_data {msg.data}")

                        # rospy.loginfo(f"Published location data: {msg.data}") #이게 한번만 publish되고 잇음
                        
                        if self.debugging:
                            self.logging.info(f"[location] Published to ROS topic /traffic_data: {msg.data}")
                    
                                

                    # 2) IMU -----------------------------------------------------
                    # imuData = self.imuDataSubscriber.receive()
                    # if imuData is not None:
                    #     try:
                    #         imuData = ast.literal_eval(imuData)
                    #         roll = float(imuData["roll"])
                    #         pitch = float(imuData["pitch"])
                    #         yaw = float(imuData["yaw"])
                    #         accelx = float(imuData["accelx"])
                    #         accely = float(imuData["accely"])
                    #         accelz = float(imuData["accelz"])

                    #         quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

                    #         imu_msg = Imu()
                    #         imu_msg.header.stamp = rospy.Time.now()
                    #         imu_msg.header.frame_id = 'imu_link'

                    #         imu_msg.orientation.x = quaternion[0]
                    #         imu_msg.orientation.y = quaternion[1]
                    #         imu_msg.orientation.z = quaternion[2]
                    #         imu_msg.orientation.w = quaternion[3]

                    #         imu_msg.linear_acceleration.x = accelx
                    #         imu_msg.linear_acceleration.y = accely
                    #         imu_msg.linear_acceleration.z = accelz

                    #         self.imu_pub.publish(imu_msg)
                    #         if self.debugging:
                    #             self.logging.info("Published IMU data")
                    #     except Exception as e:
                    #         self.logging.error(f"IMU data processing error: {str(e)}")

                    # 3) 속도/조향 ------------------------------------------------
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

                    # 4) 신호등 ---------------------------------------------------
                    semaphoresData = self.semaphoresSubscriber.receive()
                    if semaphoresData is not None:
                        try:
                            semaphores_msg = String()
                            semaphores_msg.data = json.dumps(semaphoresData)
                            if self.debugging:
                                self.logging.info("Published semaphores data")
                        except Exception as e:
                            self.logging.error(f"Semaphores data processing error: {str(e)}")

                    self.rate.sleep()  # Semaphores 데이터 처리 후 sleep

                except Exception as e:
                    self.logging.error(f"ROS loop error: {e}", exc_info=True)

        except Exception as e:
            self.logging.error(f"Thread execution error: {str(e)}")
            self.logging.error("Stack trace:", exc_info=True)
        finally:
            # ROS 노드 정리
            if not rospy.is_shutdown():
                rospy.signal_shutdown('Thread stopped')
            self.stop()
            
    # ──────────────────────────────────────────────────────────
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
        # self.trafficSubscriber = messageHandlerSubscriber(self.queuesList, TrafficData, "lastOnly", True)

        if self.debugging:
            self.logging.info("모든 구독자가 초기화되었습니다.")

    # ───── Helper: Location 검증 ─────
    def _validate_location(self, d):
        need = ("type", "x", "y", "z")
        return all(k in d for k in need) and d["type"] == "location"

    def stop(self):
        """스레드를 안전하게 종료하고 ROS 리소스를 정리합니다."""
        self._running = False
        # ROS 퍼블리셔 정리
        self.imu_pub.unregister()
        self.speed_pub.unregister()
        self.cars_pub.unregister()
        super().stop()

