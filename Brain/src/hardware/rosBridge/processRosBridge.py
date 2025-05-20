#!/usr/bin/env python3

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")


import rospy
import time
import signal
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
        self._shutdown = False
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
                    self.logging.error("Stack trace:", exc_info=True)
                    raise
            else:
                self.logging.info("ROS 노드가 이미 초기화되어 있습니다.")
            
            super(processRosBridge, self).run()
        except Exception as e:
            self.logging.error(f"ROS 노드 초기화 중 오류 발생: {str(e)}")
            self.logging.error("Stack trace:", exc_info=True)
            raise

    def _signal_handler(self, signum, frame):
        """시그널 핸들러"""
        self.logging.info(f"시그널 수신: {signum}")
        self._shutdown = True
        self.stop()

    def _init_threads(self):
        """Create the rosBridge Publisher thread and add to the list of threads."""
        rosBridgeReadTh = threadRosBridgeRead(self.queuesList, self.logging, self.debugging)
        self.threads.append(rosBridgeReadTh)
        rosBridgeWriteTh = threadRosBridgeWrite(self.queuesList, self.logging, self.debugging)
        self.threads.append(rosBridgeWriteTh)

    def stop(self):
        if self._shutdown:
            return
            
        try:
            self._shutdown = True
            self.logging.info("ROS Bridge 프로세스 종료 시작...")
            
            # 스레드 종료
            for thread in self.threads:
                if thread.is_alive():
                    self.logging.info(f"{thread.__class__.__name__} 스레드 종료 시도...")
                    thread.stop()
                    thread.join(timeout=2.0)
                    
                    if thread.is_alive():
                        self.logging.warning(f"{thread.__class__.__name__} 스레드가 정상적으로 종료되지 않음. 강제 종료 시도...")
                        thread.join(timeout=1.0)
                        if thread.is_alive():
                            self.logging.error(f"{thread.__class__.__name__} 스레드 강제 종료 실패")
            
            # ROS 노드 종료
            if rospy.core.is_initialized():
                self.logging.info("ROS 노드 종료 중...")
                rospy.signal_shutdown('Process stopping')
                time.sleep(1.0)  # ROS 노드가 완전히 종료될 때까지 대기
                
                if rospy.core.is_initialized():
                    self.logging.warning("ROS 노드가 정상적으로 종료되지 않음")
                else:
                    self.logging.info("ROS 노드가 성공적으로 종료됨")
            
        except Exception as e:
            self.logging.error(f"프로세스 종료 중 오류 발생: {str(e)}")
        finally:
            super(processRosBridge, self).stop()
            self.logging.info("ROS Bridge 프로세스 종료 완료")


