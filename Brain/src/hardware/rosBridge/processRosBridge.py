#!/usr/bin/env python3

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")


import rospy
import time
from src.templates.workerprocess import WorkerProcess
from src.hardware.serialhandler.threads.filehandler import FileHandler
from src.hardware.rosBridge.threads.threadRosBridgeRead import threadRosBridgeRead
from src.hardware.rosBridge.threads.threadRosBridgeWirte import threadRosBridgeWrite

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
        rospy.init_node("ROS_Bridge", anonymous=False, disable_signals=True) 
        super(processRosBridge, self).run()
        


    def _init_threads(self):
        """Create the rosBridge Publisher thread and add to the list of threads."""
        rosBridgeReadTh = threadRosBridgeRead(self.queuesList, self.logging, self.debugging)
        self.threads.append(rosBridgeReadTh)
        rosBridgeWriteTh = threadRosBridgeWrite(self.queuesList, self.logging, self.debugging)
        self.threads.append(rosBridgeWriteTh)


