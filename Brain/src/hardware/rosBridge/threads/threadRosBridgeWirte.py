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

        rospy.Subscriber('/control', AckermannDriveStamped, self.ack_cb)
        rospy.Subscriber('/kl', String, self.kl_cb)

        self.speedMotorSender = messageHandlerSender(self.queuesList, ROSSpeedMotor)
        self.steerMotorSender = messageHandlerSender(self.queuesList, ROSSteerMotor)
        self.klSender = messageHandlerSender(self.queuesList, ROSKlem)

        self._spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        
        super(threadRosBridgeWrite, self).__init__()

    def _spin_ros(self):
        rospy.spin()

    def run(self):
        self._spin_thread.start()
        
        rate = rospy.Rate(50) 
        while self._running and not rospy.is_shutdown():
            if self.bfmc_speed is not None and self.bfmc_steer is not None:
                self.speedMotorSender.send(str(self.bfmc_speed))
                self.steerMotorSender.send(str(self.bfmc_steer))

            if self.kl_state is not None:
                self.klSender.send(self.kl_state)

            rate.sleep()


    def stop(self):
        self._running = False
        rospy.signal_shutdown("Stopping threadRosBridgeWrite")

        if self._spin_thread.is_alive():
            self._spin_thread.join()


    def ack_cb(self, msg):
        ros_speed = msg.drive.speed
        ros_steer = msg.drive.steering_angle
        self.bfmc_speed, self.bfmc_steer = self.msg_converter(ros_speed, ros_steer)

    def kl_cb(self, msg):
        self.kl_state = msg.data

    def msg_converter(self, speed_val, steer_val):
        conv_speed = int(speed_val * 100)
        conv_steer = int(math.degrees(steer_val) * 10)
        return conv_speed, conv_steer