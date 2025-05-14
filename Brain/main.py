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
# ======================================== SETTING UP ====================================
allProcesses = list()

queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
}
logging = logging.getLogger()


Dashboard = True
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
    processTrafficCommunication = processTrafficCommunication(queueList, logging, 3, debugging = False)
    allProcesses.append(processTrafficCommunication)

# Initializing serial connection NUCLEO - > PI
if SerialHandler:
    processSerialHandler = processSerialHandler(queueList, logging, debugging = False)
    allProcesses.append(processSerialHandler)

# ------ New component runs starts here ------#
if RosBridge:
    processRosBridge = processRosBridge(queueList, logging, debugging = False)
    allProcesses.append(processRosBridge)
# ------ New component runs ends here ------#

print(allProcesses)
# ===================================== START PROCESSES ==================================
# rospy.init_node("ROS_Bridge", anonymous=False, disable_signals=True) # disable_signals=True 제거 

for process in allProcesses:
    process.daemon = True
    process.start()

time.sleep(10)

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
    '''
# -------- 프로세스 종료 & 디버깅 로깅 --------
    print("[메인] 모든 자식 프로세스 중지 시도 (역순)...\n")
    for proc in reversed(allProcesses):
        name = proc.__class__.__name__
        pid = proc.pid
        print(f"→ {name} (PID={pid}) 종료 시도")
        proc.stop()
        time.sleep(1)  # 잠시 대기 후 is_alive() 확인
        
        if proc.is_alive():
            print(f"→ {name} (PID={pid}) 여전히 종료되지 않음!")
        else:
            print(f"→ {name} (PID={pid}) 정상적으로 종료됨")

    # Gateway 프로세스도 멈춤
    print(f"[메인] Gateway 프로세스 {processGateway.__class__.__name__} (PID={processGateway.pid}) 종료 시도")
    processGateway.stop()
    time.sleep(1)
    if processGateway.is_alive():
        print(f"→ Gateway (PID={processGateway.pid}) 여전히 종료되지 않음!")
    else:
        print("→ Gateway 정상적으로 종료됨")
    '''

    big_text = """
    PPPP   RRRR   EEEEE  SSSS  SSSS       CCCC  TTTTT RRRR    L          ++      CCCC      !!! 
    P   P  R   R  E     S     S          C        T   R   R   L          ++      C         !!! 
    PPPP   RRRR   EEEE   SSS   SSS       C        T   RRRR    L      ++++++++++  C         !!! 
    P      R R    E         S     S      C        T   R R     L          ++      C         !!! 
    P      R  R   EEEEE  SSSS  SSSS       CCCC    T   R  R    LLLLL      ++      CCCC      !!!
    """

    print(big_text)