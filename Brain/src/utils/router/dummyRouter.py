from multiprocessing import Process

class DummyRouter(Process):
    def __init__(self, queueList, logging):
        super().__init__()
        self.queueList = queueList
        self.logging = logging
        self.subscribers = {}

    def run(self):
        while True:
            # 구독 요청 처리
            if not self.queueList["Config"].empty():
                cfg = self.queueList["Config"].get()
                key = (cfg["Owner"], cfg["msgID"])
                self.subscribers[key] = cfg["To"]["pipe"]
                self.logging.info(f"[Router] Subscribed: {key}")

            # 메시지 전달
            for (owner, msgID), pipe in self.subscribers.items():
                queue_name = "TrafficData"  # 명확하게 사용되는 큐 지정
                if queue_name in self.queueList and not self.queueList[queue_name].empty():
                    data = self.queueList[queue_name].get()
                    pipe.send({"value": data})