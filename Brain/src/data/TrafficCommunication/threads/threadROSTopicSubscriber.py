import rospy
from geometry_msgs.msg import PoseStamped
from threading import Thread

class threadROSTopicSubscriber(Thread):
    def __init__(self, shared_memory):
        super(threadROSTopicSubscriber, self).__init__()
        self.shared_memory = shared_memory
        self._running = True

        if not rospy.core.is_initialized():
            rospy.init_node("ros_subscriber_thread", anonymous=True)

        self.sub = rospy.Subscriber("/traffic_info", PoseStamped, self.callback) #토픽명 localizationd으로 변경해주기

    def callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # 회전각은 필요하면 계산 (예: yaw)
        self.shared_memory.insert("devicePos", [x, y]) #서버로 보낼 데이터 넣어주기 

    def stop(self):
        self._running = False
        self.sub.unregister()
