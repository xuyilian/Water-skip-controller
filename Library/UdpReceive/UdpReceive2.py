import socket
import time
import numpy as np
import threading


class UdpRigidBodies(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222, num_bodies=1):
        self.udpStop = False
        self.udp_data = None
        self.udpThread = None
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.num_bodies=num_bodies
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sample_rate = -1  # flag
        self.sample_rate = self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate

    def get_sample_rate(self):
        if self.sample_rate == -1:
            print('computing sample rate...')
            time_list = []
            for i in range(1000): # get 1000 sample
                time_list.append(time.time())
                data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
            dtime = np.diff(time_list)
            sample_time = np.mean(dtime)
            print('Sample rate: ', '%.2f' % (1/sample_time), 'Hz')
            return 1/sample_time
        else:
            return self.sample_rate

    def udp_step(self):
        udp_data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
        return udp_data

    def stop_thread(self, controller_ready):
        self.udpStop = True
        controller_ready.set()
        print('upd thread stoped')

    def start_thread(self,dup_ready, controller_ready):

        self.udpThread = threading.Thread(target=self.udp_thread_worker, args=(dup_ready,controller_ready))
        self.udpThread.start()
        time.sleep(0.1)
        print('upd thread start')

    def get_data_from_thread(self):

        return self.udp_data

    def udp_thread_worker(self, dup_ready, controller_ready):
        # use dup_ready, controller_ready to lock other threads
        if self.num_bodies == 1:
            while not self.udpStop:
                self.udp_data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
                dup_ready.set()
                controller_ready.wait()
                controller_ready.clear()

