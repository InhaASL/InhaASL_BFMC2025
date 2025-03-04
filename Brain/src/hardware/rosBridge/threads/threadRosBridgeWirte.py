import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    ToggleBatteryLvl,
    ToggleImuData,
    ToggleInstant,
    ToggleResourceMonitor
)
from src.utils.messages.messageHandlerSender import messageHandlerSender

class threadRosBridgeWrite(ThreadWithStop):
    """This thread read data from ROS topic and deliver them to SerialHandler.
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

        self.bfmc_speed = None
        self.bfmc_steer = None
        self.kl_state = None

        rospy.Subscriber('/control', AckermannDriveStamped, self.ack_cb)
        rospy.Subscriber('/kl', String(), self.kl_cb)
        #rospy.Subscriber('/car_state)
        # ADD CAR STATE MSG FOR BRING UP CAR FROM ROS NODE

        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.klSender = messageHandlerSender(self.queuesList, Klem)

        super(threadRosBridgeWrite, self).__init__()

    def run(self):
        while self._running:
            if self.bfmc_speed != None and self.bfmc_steer != None:
                self.speedMotorSender.send(self.bfmc_speed)
                self.steerMotorSender.send(self.bfmc_steer)
            
            if self.kl_state != None:
                self.klSender.send(self.kl_state)
                

    def ack_cb(self, msg):
       ros_speed = msg.AckermannDrive.speed
       ros_steer = msg.AckermannDrive.steering_angle

       self.bfmc_speed, self.bfmc_steer = self.msg_converter(ros_speed, ros_steer)

    def kl_cb(self, msg):
        self.kl_state = msg.data

    def msg_converter(self, speed_val, steer_val):
        conv_speed = int(speed_val * 100)
        conv_steer = int(math.degrees(steer_val))
        return conv_speed, conv_steer
