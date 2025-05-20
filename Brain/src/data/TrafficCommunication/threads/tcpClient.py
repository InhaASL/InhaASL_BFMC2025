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
from src.utils.messages.allMessages import Location, TrafficData
from src.utils.messages.messageHandlerSender import messageHandlerSender
from twisted.internet import protocol

# 로깅 설정
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('TrafficClient')

# The server itself. Creates a new Protocol for each new connection and has the info for all of them.
class tcpClient(protocol.ClientFactory):
    def __init__(self, connectionBrokenCllbck, locsysID, locsysFrequency, queue):
        self.connectiondata = None
        self.connection = None
        self.retry_delay = 1
        self.connectionBrokenCllbck = connectionBrokenCllbck
        self.locsysID = locsysID
        self.locsysFrequency = locsysFrequency
        self.queue = queue
        self.sendLocation = messageHandlerSender(self.queue, Location)
        self.sendTrafficData = messageHandlerSender(self.queue, TrafficData)
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

    def send_data_to_server(self, message):
        logger.debug(f"Sending data to server: {message}")
        self.connection.send_data(message)


# One class is generated for each new connection
class SingleConnection(protocol.Protocol):
    def connectionMade(self):
        peer = self.transport.getPeer()
        self.factory.connectiondata = peer.host + ":" + str(peer.port)
        self.factory.connection = self
        self.subscribeToLocaitonData(self.factory.locsysID, self.factory.locsysFrequency)
        logger.info(f"Connection with server established: {self.factory.connectiondata}")

    def dataReceived(self, data):
        try:
            dat = data.decode()
            logger.debug(f"Raw data received: {dat}")
            
            tmp_data = dat.replace("}{","}}{{")
            if tmp_data != dat:
                tmp_dat = tmp_data.split("}{")
                dat = tmp_dat[-1]
            
            da = json.loads(dat)
            logger.debug(f"Parsed data: {da}")

            if da["type"] == "location":
                da["type"] = traffic # 데이터 타입 변경 시도 
                da["id"] = self.factory.locsysID
                logger.info(f"Sending location data: {da}")
                self.factory.sendLocation.send(da)
            elif da["type"] == "traffic":
                logger.info(f"Sending traffic data: {da}")
                self.factory.sendTrafficData.send(da)
            else:
                logger.warning(f"Received unknown message type: {da['type']}")
        except Exception as e:
            logger.error(f"Error processing received data: {str(e)}")
            logger.error(f"Problematic data: {data}")

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
