#!/usr/bin/env python3

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")


import rospy
import time
from src.templates.workerprocess import WorkerProcess
from src.hardware.serialhandler.threads.filehandler import FileHandler
from src.hardware.rosBridge.threads.threadRosBridgeRead import threadRosBridgeRead
from src.hardware.rosBridge.threads.threadRosBridgeWrite import threadRosBridgeWrite

class processRosBridge(WorkerProcess):
    """This process Handle RosBridge which connnect between ROS and SerialHandler
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging=False, debugging=False):
        logFile = "RosHistoryFile.txt"

        self.historyFile = FileHandler(logFile)
        self.queuesList = queueList
        self.debugging = debugging
        self.logging = logging
        super(processRosBridge, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        try:
            if not rospy.core.is_initialized():
                # ROS 마스터 연결 시도
                try:
                    rospy.init_node("ROS_Bridge", anonymous=True, disable_signals=True)
                    # 초기화 확인
                    if not rospy.core.is_initialized():
                        raise RuntimeError("ROS 노드 초기화 실패")
                    self.logging.info("ROS 노드가 성공적으로 초기화되었습니다.")
                except Exception as e:
                    self.logging.error(f"ROS 마스터 연결 실패: {str(e)}")
                    raise
            super(processRosBridge, self).run()
        except Exception as e:
            self.logging.error(f"ROS 노드 초기화 중 오류 발생: {str(e)}")
            raise

    def _init_threads(self):
        """Create the rosBridge Publisher thread and add to the list of threads."""
        rosBridgeReadTh = threadRosBridgeRead(self.queuesList, self.logging, self.debugging)
        self.threads.append(rosBridgeReadTh)
        rosBridgeWriteTh = threadRosBridgeWrite(self.queuesList, self.logging, self.debugging)
        self.threads.append(rosBridgeWriteTh)

    def stop(self):
        """Function for stopping threads and the process."""
        try:
            # 먼저 스레드들을 정지
            for thread in self.threads:
                if thread.is_alive():
                    thread.stop()
                    thread.join(timeout=5.0)
                    if thread.is_alive():
                        self.logging.error(f"Thread {thread.__class__.__name__} did not stop properly")
            
            # ROS 노드 정상 종료
            if rospy.core.is_initialized():
                rospy.signal_shutdown('Process stopping')
                rospy.spinOnce()
                
        except Exception as e:
            self.logging.error(f"Error stopping process: {str(e)}")
        finally:
            super(processRosBridge, self).stop()


