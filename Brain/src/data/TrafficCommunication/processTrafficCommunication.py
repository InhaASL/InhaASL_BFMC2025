# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

# Import necessary modules
from multiprocessing import Pipe
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.templates.workerprocess import WorkerProcess
from src.data.TrafficCommunication.threads.threadTrafficCommunicaiton import (
    threadTrafficCommunication,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import TrafficData


import rospy
class processTrafficCommunication(WorkerProcess):
    """This process receives the location of the car and sends it to the processGateway.
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Used for debugging.
        deviceID (int): The ID of the device.
        frequency (float): The frequency of communication.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, deviceID, debugging, frequency=1):
        self.queuesList = queueList
        self.logging = logging
        self.shared_memory = sharedMem()
        self.filename = "src/data/TrafficCommunication/useful/publickey_server_test.pem" #테스트용, 실제 대회에서는 test를 제거 
        # self.filename = "src/data/TrafficCommunication/useful/privatekey_server_test.pem"

        self.deviceID = deviceID
        self.frequency = frequency
        self.debugging = debugging
        super(processTrafficCommunication, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        
        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processTrafficCommunication, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        try:
            if self.debugging:
                self.logging.info("Traffic Communication 프로세스 시작")
                self.logging.info(f"Device ID: {self.deviceID}")
                self.logging.info(f"Frequency: {self.frequency}")
                self.logging.info(f"Key file: {self.filename}")

            # 실제 데이터를 받아오는 부분
            device_pos = self.shared_memory.get()
            if self.debugging:
                self.logging.info(f"Received device position: {device_pos}")

            # device_pos가 리스트인 경우 처리
            if isinstance(device_pos, list) and len(device_pos) >= 2:
                x, y = device_pos[0], device_pos[1]
            else:
                x, y = 0.0, 0.0

            traffic_data = {
                "type": "traffic",
                "x": x,
                "y": y,
                "z": 0.0,
                "quality": 1
            }

            if self.debugging:
                self.logging.info(f"Preparing to send traffic data: {traffic_data}")

            # TrafficData 메시지로 전송
            self.queuesList["TrafficData"].put({
                "Owner": "TrafficCommunication",
                "msgID": "TrafficData",
                "msgType": "dict",
                "msgValue": traffic_data
            })

            if self.debugging:
                self.logging.info("Traffic data sent to TrafficData queue")

            # General 큐에도 전송 (기존 방식 유지)
            sender = messageHandlerSender(self.queuesList, TrafficData)
            sender.send(traffic_data)

            if self.debugging:
                self.logging.info("Traffic data sent via messageHandlerSender")

        except Exception as e:
            if self.debugging:
                self.logging.error(f"Error sending traffic data: {str(e)}")
                self.logging.error(f"Stack trace:", exc_info=True)
        
        super(processTrafficCommunication, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Traffic Communication thread and add it to the list of threads."""

        TrafficComTh = threadTrafficCommunication(
            self.shared_memory, self.queuesList, self.deviceID, self.frequency, self.filename
        )
        self.threads.append(TrafficComTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processTrafficCommunication.py

if __name__ == "__main__": # 이 파일을 직접 실행할때만 작동, 아니라면 main.py 클래스만 불러와짐 
    from multiprocessing import Queue
    import time
    import logging

    shared_memory = sharedMem()
    locsysReceivePipe, locsysSendPipe = Pipe(duplex=False)


    # logging.basicConfig(level=logging.DEBUG,format = "%(asctime)s [%(levelname)s] %(name)s : %(message)s")
    # logger = logging.getLogger("TrafficComm")
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    # filename = "useful/publickey_server.pem"
    filename = "useful/publickey_server_test.pem"
    deviceID = 3
    frequency = 0.4
    # frequency = 1.0
    traffic_communication = threadTrafficCommunication(
        shared_memory, queueList, deviceID, frequency, filename
    )


    traffic_communication.start()    

    start_time = time.time()
    duration = 10  # specify the duration in seconds
    
    shared_memory.insert("devicePos", [1.2, 2.3]) # send a position to the server
    shared_memory.insert("deviceRot", [3.4]) # send a rotation to the server
    shared_memory.insert("deviceSpeed", [4.5]) # send a speed to the server
    shared_memory.insert("historyData", [5.6, 6.7, 8]) # send a history data point to the server

    while time.time() - start_time < duration:
        try:
            print(queueList["General"].get(timeout=1))
        except:pass
    traffic_communication.stop()
