import socket
import threading
import time
from jumping.jumping_model import RealTimeSleeper


class UDPSender(object):
    def __init__(self, target_ip="0.0.0.0", target_port=22222, sample_time=0.01):
        self.target_ip = target_ip
        self.target_port = target_port
        self.sample_time = sample_time
        self.RTS = RealTimeSleeper(self.sample_time)

        self._udpStop = False
        self._udp_data = "Hello, World!".encode()
        self._udpThread = None
        self._udpThread_on = False
        self.udp_flag = 0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

    def start_thread(self):
        if not self._udpThread_on:
            self._udpThread = threading.Thread(target=self._udp_worker, args=(), )
            self._udpThread.start()
            time.sleep(self.sample_time)
            self._udpThread_on = True
            print('Upd thread start')
        else:
            print('New upd-send thread is not started')

    def stop_thread(self, ):
        self._udpStop = True
        time.sleep(self.sample_time)
        print('upd thread stopped')

    def send_data(self, data):
        self._udp_data = data

    def _udp_worker(self, ):
        if not self._udpThread_on:
            self.RTS.init()
            while not self._udpStop:
                self.udp_flag = self.udp_flag + 1
                self._sock.sendto(self._udp_data, (self.target_ip, self.target_port))
                self.RTS.sleep()
                # time.sleep(self.sample_time)
            self._udpThread_on = False
