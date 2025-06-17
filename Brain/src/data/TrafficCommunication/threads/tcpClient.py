#!/usr/bin/env python3
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

import json
import time
import logging
from src.utils.messages.allMessages import Location
from src.utils.messages.messageHandlerSender import messageHandlerSender
from twisted.internet import protocol

# 로깅 설정
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('TrafficClient')

# The server itself. Creates a new Protocol for each new connection and has the info for all of them.
class tcpClient(protocol.ClientFactory):
    """Factory that 유지 TCP 연결 및 메시지 송수신."""
    def __init__(self, connectionBrokenCllbck, locsysID, locsysFrequency, queue):
        self.connectiondata = None
        self.connection = None
        self.retry_delay = 1
        self.connectionBrokenCllbck = connectionBrokenCllbck
        self.locsysID = locsysID
        self.locsysFrequency = locsysFrequency
        self.queue = queue
        self.sendLocation = messageHandlerSender(self.queue, Location)
        #self.sendTrafficData = messageHandlerSender(self.queue, TrafficData)
        logger.info("TCP Client initialized")

    def clientConnectionLost(self, connector, reason):
        logger.warning(f"Connection lost with server {self.connectiondata}")
        try:
            self.connectiondata = None
            self.connection = None
            self.connectionBrokenCllbck()
        except Exception as e:
            logger.error(f"Error in connection lost handler: {str(e)}")

    def clientConnectionFailed(self, connector, reason):
        logger.warning(f"Connection failed. Retrying in {self.retry_delay} seconds...")
        time.sleep(self.retry_delay)
        connector.connect()

    def buildProtocol(self, addr):
        conn = SingleConnection()
        conn.factory = self
        return conn
    
    # 편의 함수: 서버로 직접 전송
    def send_data_to_server(self, message):
        logger.debug(f"Sending data to server: {message}")
        self.connection.send_data(message)


# One class is generated for each new connection
class SingleConnection(protocol.Protocol):
    """실제 소켓 connection마다 하나씩 생성."""

    # ───────── Connect ─────────  ㅡㅡ> 문제없음
    def connectionMade(self):
        peer = self.transport.getPeer()
        self.factory.connectiondata = peer.host + ":" + str(peer.port)
        self.factory.connection = self
        self.subscribeToLocaitonData(self.factory.locsysID, self.factory.locsysFrequency)
        logger.info(f"Connection with server established: {self.factory.connectiondata}")

        #서버에 위치 데이터 구독 요청
        #self._subscribe_to_location(self.factory.locsysID, self.factory.locsysFrequency)


    # ───────── Receive ─────────
    def dataReceived(self, data):
        """
        서버에서 오는 JSON 문자열을 파싱해서
        내부 큐 → Gateway → ROS 파이프라인으로 넣는다.
        """
        try:
            dat = data.decode()
            logger.debug(f"Raw data received: {dat}")
            
            tmp_data = dat.replace("}{","}}{{")
            if tmp_data != dat:
                tmp_dat = tmp_data.split("}{")
                dat = tmp_dat[-1]
            
            da = json.loads(dat)
            logger.debug(f"Parsed data: {da}")

            # ① location 패킷 → traffic 포맷 변환
            if da["type"] == "location":

                #traffic_data 딕셔너리 형태 포맷 방법
                traffic_data = {
                    "type": "traffic",
                    "x": float(da.get("x", 0.0)),
                    "y": float(da.get("y", 0.0)),
                    "z": float(da.get("z", 0.0)),
                    "quality": int(da.get("quality", 1))
                }
<<<<<<< HEAD
                logger.info(f"Converting location data to traffic data: {traffic_data}")
                
=======
                #logger.info(f"Converting location data to traffic data: {traffic_data}") #확인완료. location으로 들어옴
                try:
                    # TrafficData 큐에 직접 전송
                    message = {
                        "Owner": "TrafficCommunication",
                        "msgID": "TrafficData",
                        "msgType": "dict",
                        "msgValue": traffic_data
                    }
                    #logger.info(f"Sending message to TrafficData queue: {message}")
                    #self.factory.queue["TrafficData"].put(message)
                    self.factory.sendLocation.send(da)                  ### CHANGED
                    #logger.info("Traffic data successfully sent to TrafficData queue") #확인완료 
                except Exception as e:
                    logger.error(f"Error sending traffic data: {str(e)}")
                    logger.error("Stack trace:", exc_info=True)

            # ② 이미 traffic 패킷인 경우
            elif da["type"] == "traffic":
                logger.info(f"Sending traffic data: {da}")
                try:
                    # TrafficData 큐에 직접 전송
                    message = {
                        "Owner": "TrafficCommunication",
                        "msgID": "TrafficData",
                        "msgType": "dict",
                        "msgValue": da
                    }
                    #logger.info(f"Sending message to TrafficData queue: {message}")
                    self.factory.queue["TrafficData"].put(message)
                    #logger.info("Traffic data successfully sent to TrafficData queue")
                except Exception as e:
                    logger.error(f"Error sending traffic data: {str(e)}")
                    logger.error("Stack trace:", exc_info=True)
            # 3. 데이터 타입을 모르겠을때
>>>>>>> 7def637cea30a51beb7eaed5d3363e2476392bc0
            else:
                logger.warning(f"Received unknown message type: {da['type']}")

        except Exception as e:
            logger.error(f"Error TCP client data recieved: {str(e)}") #에러 디버깅

    # ───────── Utility ───────── ㅡㅡ> 서버로 데이터 전송 
    def send_data(self, message):
        msg = json.dumps(message)
        logger.debug(f"Sending message: {msg}")
        self.transport.write(msg.encode())
    
    def subscribeToLocaitonData(self, id, frequency):
        # Sends the id you wish to subscribe to and the frequency you want to receive data. Frequency must be between 0.1 and 5. 
        msg = {
            "reqORinfo": "info",
            "type": "locIDsub",
            "locID": id,
            "freq": frequency,
        }
        logger.info(f"Subscribing to location data: {msg}")
        self.send_data(msg)
    
    def unSubscribeToLocaitonData(self, id, frequency):
        # Unsubscribes from locaiton data. 
        msg = {
            "reqORinfo": "info",
            "type": "locIDubsub",
        }
        logger.info("Unsubscribing from location data")
        self.send_data(msg)
