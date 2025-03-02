import rospy
from sensor_msgs.msg import Imu
import tf
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
    WarningSignal
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
        self.subscribe()

        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size= 10)
        self.ros_imu = Imu()
        #self.cur_state_pub = rospy.Publisher('/car_cur_state', CarState, queue_size=10)
        #ADD CAR STATE LATER
        
        super(threadRosBridgeRead, self).__init__()

    def run(self):
        while self._running:
            enableButton = self.enableButtonSubscriber.receive()
            if enableButton:
                imuData = self.imuDataSubscriber.receive()
                if imuData is not None:
                    roll = float(imuData["roll"])
                    pitch = float(imuData["pitch"])
                    yaw = float(imuData["yaw"])
                    accelx = float(imuData["accelx"])
                    accely = float(imuData["accely"])
                    accelz = float(imuData["accelz"])

                    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

                    # IMU 메시지 생성
                    imu_msg = Imu()
                    imu_msg.header.stamp = rospy.Time.now()
                    imu_msg.header.frame_id = "imu"  # 센서 프레임 지정

                    # Orientation (쿼터니언)
                    imu_msg.orientation.x = quaternion[0]
                    imu_msg.orientation.y = quaternion[1]
                    imu_msg.orientation.z = quaternion[2]
                    imu_msg.orientation.w = quaternion[3]

                    # Linear Acceleration
                    imu_msg.linear_acceleration.x = accelx
                    imu_msg.linear_acceleration.y = accely
                    imu_msg.linear_acceleration.z = accelz

                    # Angular Velocity (필요한 경우 설정)
                    imu_msg.angular_velocity.x = 0.0
                    imu_msg.angular_velocity.y = 0.0
                    imu_msg.angular_velocity.z = 0.0

                    # 메시지 퍼블리시
                    self.imu_pub.publish()
                


            pass

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.enableButtonSubscriber = messageHandlerSubscriber(self.queuesList, EnableButton)
        self.batteryLvlSubscriber = messageHandlerSubscriber(self.queuesList, BatteryLvl)
        self.instantConsumptionSubscriber = messageHandlerSubscriber(self.queuesList, InstantConsumption)
        self.imuDataSubscriber = messageHandlerSubscriber(self.queuesList, ImuData)
        self.imuAckSubscriber = messageHandlerSubscriber(self.queuesList, ImuAck)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ResourceMonitor)
        self.currentSpeedSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSpeed)
        self.currentSteerSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSteer)
        self.warningSubscriber = messageHandlerSubscriber(self.queuesList, WarningSignal)
        pass
