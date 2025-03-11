import threading
import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    ROSKlem,
    ROSSpeedMotor,
    ROSSteerMotor,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender

class threadRosBridgeWrite(ThreadWithStop):
    """ROS → BFMC 통신 스레드.
    /control, /kl 토픽 데이터를 받아 BFMC 쪽으로 전달합니다."""

    # ROS SPEED RANGE : (FLOAT) -5 ~ 5 [M/S]
    # BFMC SPEED RANGE : (INT) -500 ~ 500 [MM/S]

    # ROS STEER RANGE : (FLOAT) -0.401426 ~ 0.401426 [RAD] (ACKERMANN MSG)
    # BFMC STEER RANGE : (INT) -230 ~ 230 [DEGREE * 10]

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.bfmc_speed = None
        self.bfmc_steer = None
        self.kl_state = None

        self.prev_speed = 0
        self.prev_steer = 0

        self.max_speed = 500
        self.min_speed = -500

        self.max_steer = 230
        self.min_steer = -230

        rospy.Subscriber('/ackermann_cmd_mux/output', AckermannDriveStamped, self.ack_cb)
        rospy.Subscriber('/kl', String, self.kl_cb)

        self.speedMotorSender = messageHandlerSender(self.queuesList, ROSSpeedMotor)
        self.steerMotorSender = messageHandlerSender(self.queuesList, ROSSteerMotor)
        self.klSender = messageHandlerSender(self.queuesList, ROSKlem)
        

        super(threadRosBridgeWrite, self).__init__()


    def run(self):
        rospy.spin()

        
    def stop(self):
        rospy.signal_shutdown("Stopping threadRosBridgeWrite")

    def ack_cb(self, msg):
        ros_speed = msg.drive.speed
        ros_steer = msg.drive.steering_angle
        self.bfmc_speed, self.bfmc_steer = self.msg_converter(ros_speed, ros_steer)
        # RUN IN THE SAFE AREA
        self.bfmc_speed = self.clip(self.bfmc_speed, self.min_speed, self.max_speed)
        self.bfmc_steer = self.clip(self.bfmc_steer, self.min_steer, self.max_steer)
        if(self.bfmc_speed != self.prev_speed):
            self.speedMotorSender.send(str(self.bfmc_speed))
            self.prev_speed = self.bfmc_speed
        if(self.bfmc_steer != self.prev_steer):
            self.steerMotorSender.send(str(self.bfmc_steer))
            self.prev_steer = self.bfmc_steer

    def kl_cb(self, msg):
        self.kl_state = msg.data
        if self.kl_state == "30" or "15" or "0":
            self.klSender.send(self.kl_state)


    def msg_converter(self, speed_val, steer_val):
        conv_speed = int(speed_val * 100)
        conv_steer = int(math.degrees(steer_val) * 10)
        return conv_speed, conv_steer
    
    def clip(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))