import socket
import struct
import threading
from Library.IIR2Filter import IIR2Filter
import math
import time
import numpy as np


class UdpRigidBodies(object):
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222, num_bodies=1,sample_rate=200):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.num_bodies=num_bodies
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind((self.udp_ip, self.udp_port))
        if self.num_bodies == 1:
            self.X = 0
            self.Y = 0
            self.Z = 0
            self.QX = 1
            self.QY = 0
            self.QZ = 0
            self.QW = 0
            self.R31 = 0
            self.R32 = 0

        self.udpStop = False
        self.lock = threading.Lock()
        self.udpThread = threading.Thread(target=self.udp_worker, args=(self.lock,))

        self.sample_rate = -1
        self.sample_rate = self.get_sample_rate()
        self.sampletime = 1/self.sample_rate

        self.FilterX = IIR2Filter.IIR2Filter(4, [16], 'lowpass', design='cheby2', rs=58, fs=sample_rate)
        self.FilterY = IIR2Filter.IIR2Filter(4, [16], 'lowpass', design='cheby2', rs=58, fs=sample_rate)
        self.FilterZ = IIR2Filter.IIR2Filter(4, [16], 'lowpass', design='cheby2', rs=58, fs=sample_rate)
        self.FilterR31 = IIR2Filter.IIR2Filter(4, [16], 'lowpass', design='cheby2', rs=58, fs=sample_rate)
        self.FilterR32 = IIR2Filter.IIR2Filter(4, [16], 'lowpass', design='cheby2', rs=58, fs=sample_rate)

        self.X_F = 0
        self.Y_F = 0
        self.Z_F = 0

        self.R31_F = 0
        self.R32_F = 0

        self.angleY = 0

        self.X_F_d = 0
        self.Y_F_d = 0
        self.R31_F_d = 0
        self.R32_F_d = 0
        self.angleY_d = 0

        self.Delayed_X_F = 0
        self.Delayed_Y_F = 0
        self.Delayed_R31_F = 0
        self.Delayed_R32_F = 0
        self.Delayed_angleY = 0


    def start_thread(self):
        self.udpThread.start()
        print('upd thread start')

    def stop_thread(self):
        self.udpStop = True

    def udp_worker(self, lock,):
        if self.num_bodies ==1:
        # ----- udp settings -----
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            self.sock.bind((self.udp_ip, self.udp_port))
            while not self.udpStop:
                udp_data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
                with lock:
                    x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data)
                    self.X = x * 0.0005
                    self.Y = y * 0.0005
                    self.Z = z * 0.0005
                    self.QX = float(qx * 0.001)
                    self.QY = float(qy * 0.001)
                    self.QZ = float(qz * 0.001)
                    self.QW = float(qw * 0.001)
                    self.angleY = (math.atan2(2 * (self.QW * self.QZ + self.QX * self.QY),
                                         1 - 2 * (self.QZ * self.QZ + self.QY * self.QY))) * 180 / math.pi + 80 + 16 + 5
                    if self.angleY > 180: self.angleY = self.angleY - 360
                    if self.angleY < -180: self.angleY = self.angleY + 360

                    self.R31 = 2 * (self.QX * self.QZ + self.QW * self.QY)
                    self.R32 = 2 * (self.QY * self.QZ - self.QW * self.QX)

                    # delay data
                    Delayed_X_F = self.X_F
                    Delayed_Y_F = self.Y_F
                    Delayed_R31_F = self.R31_F
                    Delayed_R32_F = self.R32_F

                    # filter
                    self.X_F = self.FilterX.filter(self.X)
                    self.Y_F = self.FilterY.filter(self.Y)
                    self.Z_F = self.FilterZ.filter(self.Z)
                    self.R31_F = self.FilterR31.filter(self.R31)
                    self.R32_F = self.FilterR32.filter(self.R32)

                    # diff
                    self.X_F_d = (self.X_F - Delayed_X_F) / self.sampletime
                    self.Y_F_d = (self.Y_F - Delayed_Y_F) / self.sampletime
                    self.R31_F_d = (self.R31_F - Delayed_R31_F) / self.sampletime
                    self.R32_F_d = (self.R32_F - Delayed_R32_F) / self.sampletime

        print('upd thread stop')

    def udp_step_init(self):
        if self.num_bodies ==1:
        # ----- udp settings -----
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            self.sock.bind((self.udp_ip, self.udp_port))

    def udp_step(self):

        udp_data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes

        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data)
        self.X = x * 0.0005
        self.Y = y * 0.0005
        self.Z = z * 0.0005
        self.QX = float(qx * 0.001)
        self.QY = float(qy * 0.001)
        self.QZ = float(qz * 0.001)
        self.QW = float(qw * 0.001)
        self.angleY = (math.atan2(2 * (self.QW * self.QZ + self.QX * self.QY),
                             1 - 2 * (self.QZ * self.QZ + self.QY * self.QY))) * 180 / math.pi + 80 + 16
        if self.angleY > 180: self.angleY = self.angleY - 360
        if self.angleY < -180: self.angleY = self.angleY + 360

        self.R31 = 2 * (self.QX * self.QZ + self.QW * self.QY)
        self.R32 = 2 * (self.QY * self.QZ - self.QW * self.QX)

        # filter
        self.X_F = self.FilterX.filter(self.X)
        self.Y_F = self.FilterY.filter(self.Y)
        self.Z_F = self.FilterZ.filter(self.Z)
        self.R31_F = self.FilterR31.filter(self.R31)
        self.R32_F = self.FilterR32.filter(self.R32)

        # diff
        self.X_F_d = (self.X_F - self.Delayed_X_F) / self.sampletime
        self.Y_F_d = (self.Y_F - self.Delayed_Y_F) / self.sampletime
        self.R31_F_d = (self.R31_F - self.Delayed_R31_F) / self.sampletime
        self.R32_F_d = (self.R32_F - self.Delayed_R32_F) / self.sampletime
        temp_diff_angleY = self.angleY - self.Delayed_angleY
        if temp_diff_angleY > 280:  # depend on the rotation direction of the robot
            temp_diff_angleY = temp_diff_angleY - 360
        self.angleY_d = temp_diff_angleY / self.sampletime

        # delay data
        self.Delayed_X_F = self.X_F
        self.Delayed_Y_F = self.Y_F
        self.Delayed_R31_F = self.R31_F
        self.Delayed_R32_F = self.R32_F
        self.Delayed_angleY = self.angleY

    def get_sample_rate(self):
        if self.sample_rate == -1:
            time_list = []
            for i in range(1000): # get 1000 sample
                time_list.append(time.time())
                data, addr = self.sock.recvfrom(100)  # buffer size is 8192 bytes
            dtime = np.diff(time_list)
            sample_time = np.mean(dtime)
            return 1/sample_time
        else:
            return self.sample_rate

