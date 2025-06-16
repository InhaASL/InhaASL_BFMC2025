#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# To start the project: 
#
#       sudo apt update
#       sudo apt upgrade
#       xargs sudo apt install -y < "requirement.txt" 
#       cd src/dashboard/frontend/
#       curl -fsSL https://fnm.vercel.app/install | bash
#       source ~/.bashrc
#       fnm install --lts
#       npm install -g @angular/cli@17
#       npm install
#       if needed: npm audit fix
#
# ===================================== GENERAL IMPORTS ==================================
import sys
import subprocess
import time

sys.path.append(".")
from multiprocessing import Queue, Event
import logging

logging.basicConfig(level=logging.INFO)

# ===================================== PROCESS IMPORTS ==================================

import rospy
from std_msgs.msg import String  # ROS 메시지 타입 추가
from src.gateway.processGateway import processGateway
from src.dashboard.processDashboard import processDashboard
# from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.data.Semaphores.Semaphores import processSemaphores
from src.data.TrafficCommunication.processTrafficCommunication import processTrafficCommunication
from src.utils.ipManager.IpReplacement import IPManager
# ------ New component imports starts here ------#
from src.hardware.rosBridge.processRosBridge import processRosBridge
# ------ New component imports ends here ------#
# from src.utils.router.dummyRouter import DummyRouter  # gateway에서 대신해주고 있어서 필요없음


# ======================================== SETTING UP ====================================
allProcesses = list() #모든 프로세스 리스트에 추가 

queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
    "TrafficData": Queue(),  # TrafficData 큐 추가
}
logging = logging.getLogger()

# # ROS 노드 초기화
# rospy.init_node('main_node', anonymous=True)
# # ROS 퍼블리셔 생성
# pub = rospy.Publisher('your_topic_name', String, queue_size=10)

# # 메시지 발행 함수
# def publish_message():
#     msg = String()
#     msg.data = "Hello from main.py!"
#     pub.publish(msg)
#     # rospy.loginfo("Published: %s", msg.data)  #디버깅용 

# # 지속적인 메시지 발행을 위한 함수
# def continuous_publish():
#     rate = rospy.Rate(10)  # 10Hz로 메시지 발행
#     while not rospy.is_shutdown():
#         publish_message()
#         rate.sleep()

Dashboard = False
Camera = False
Semaphores = True
TrafficCommunication = True
SerialHandler = True

# ------ New component flags starts here ------#
RosBridge = True
# ------ New component flags ends here ------#


# ===================================== SETUP PROCESSES ==================================

# Initializing gateway
processGateway = processGateway(queueList, logging)
processGateway.start()

# Ip replacement
path = './src/dashboard/frontend/src/app/webSocket/web-socket.service.ts'
IpChanger = IPManager(path)
IpChanger.replace_ip_in_file()


# Initializing dashboard
if Dashboard:
    processDashboard = processDashboard( queueList, logging, debugging = False)
    allProcesses.append(processDashboard)

# Initializing camera
# if Camera:
#     processCamera = processCamera(queueList, logging , debugging = False)
#     allProcesses.append(processCamera)

# Initializing semaphores
if Semaphores:
    processSemaphores = processSemaphores(queueList, logging, debugging = False)
    allProcesses.append(processSemaphores)

# Initializing GPS
if TrafficCommunication:
    processTrafficCommunication = processTrafficCommunication(queueList, logging, 3, debugging = True)
    allProcesses.append(processTrafficCommunication)

# Initializing serial connection NUCLEO - > PI
if SerialHandler:
    processSerialHandler = processSerialHandler(queueList, logging, debugging = False)
    allProcesses.append(processSerialHandler)

# ------ New component runs starts here ------#
if RosBridge:
    processRosBridge = processRosBridge(queueList, logging, debugging = True)
    allProcesses.append(processRosBridge)
# ------ New component runs ends here ------#

# router = DummyRouter(queueList, logging)
# allProcesses.append(router)


print(allProcesses)
# ===================================== START PROCESSES ==================================
for process in allProcesses:
    process.daemon = True
    process.start()

time.sleep(10)

# 지속적인 메시지 발행 시작
# continuous_publish()

def verify_processes(process_list):
    """모든 프로세스의 is_alive() 상태를 확인하는 간단한 함수."""
    print("=== 프로세스 상태 점검 시작 ===")
    for idx, proc in enumerate(process_list, start=1):
        if proc.is_alive():
            print(f"[{idx}] {proc.__class__.__name__} 프로세스가 정상적으로 동작 중입니다.")
        else:
            print(f"[{idx}] {proc.__class__.__name__} 프로세스가 동작하지 않습니다!")
    print("=== 프로세스 상태 점검 종료 ===")

# verify_processes(allProcesses)

c4_bomb = r"""
  _______________________
 /                       \
| [██████]    [██████]    |
| [██████]    [██████]    |
| [██████]    [██████]    |
|       TIMER: 00:10      |
|_________________________|
 \_______________________/
        LET'S GO!!!

        Press ctrl+C to close
"""

print(c4_bomb)

# ===================================== STAYING ALIVE ====================================
blocker = Event()
try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    big_text = """
    PPPP   L        EEEEE    A    SSSS  EEEEE       W      W   A   III TTTTT
    P   P  L        E       A A   S     E           W      W  A A   I    T  
    PPPP   L        EEEE   A   A   SSS  EEEE        W  W   W A   A  I    T  
    P      L        E      AAAAA      S E           W W W W  AAAAA  I    T  
    P      LLLLLL   EEEEE  A   A  SSSS  EEEEE        W   W   A   A III   T  
    """
    print(big_text)
    
    # 모든 프로세스 종료
    for process in allProcesses:
        if process.is_alive():
            process.stop()
            process.join(timeout=2.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
    
    # gateway 프로세스 종료
    if processGateway.is_alive():
        processGateway.stop()
        processGateway.join(timeout=2.0)
        if processGateway.is_alive():
            processGateway.terminate()
            processGateway.join(timeout=1.0)
    
    print("모든 프로세스가 종료되었습니다.")
    