from Library.IIR2Filter import IIR2Filter
import struct
import math


class RealTimeProcessor(object):
    def __init__(self, num_of_bodies, order, cutoff, ftype, design, rs, sample_rate):
        self.num_of_bodies = num_of_bodies
        self.sample_time = 1/ sample_rate
        self.sample_rate = sample_rate
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.QX = 1
        self.QY = 0
        self.QZ = 0
        self.QW = 0
        self.R31 = 0
        self.R32 = 0
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
        self.FilterX = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterR31 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterR32 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.delayed_angle = 96

        self.Delayed_X_F = 0
        self.Delayed_Y_F = 0
        self.Delayed_R31_F = 0
        self.Delayed_R32_F = 0
        self.Delayed_angleY = 0
        if self.num_of_bodies == 2:
            self.X2 = 0
            self.Y2 = 0
            self.Z2 = 0
            self.QX2 = 1
            self.QY2 = 0
            self.QZ2 = 0
            self.QW2 = 0
            self.R312 = 0
            self.R322 = 0
            self.X_F2 = 0
            self.Y_F2 = 0
            self.Z_F2 = 0
            self.R31_F2 = 0
            self.R32_F2 = 0
            self.angleY2 = 0
            self.X_F_d2 = 0
            self.Y_F_d2 = 0
            self.R31_F_d2 = 0
            self.R32_F_d2 = 0
            self.angleY_d2 = 0
            self.FilterX2 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design, rs, sample_rate)
            self.FilterY2 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design, rs, sample_rate)
            self.FilterZ2 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design, rs, sample_rate)
            self.FilterR312 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design, rs, sample_rate)
            self.FilterR322 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design, rs, sample_rate)
            self.delayed_angle2 = 0

    def step(self,udp_data):
        if self.num_of_bodies == 1:
            x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data)
            self.X = x * 0.0005
            self.Y = y * 0.0005
            self.Z = z * 0.0005
            self.QX = float(qx * 0.001)
            self.QY = float(qy * 0.001)
            self.QZ = float(qz * 0.001)
            self.QW = float(qw * 0.001)
            self.angleY = (math.atan2(2 * (self.QW * self.QZ + self.QX * self.QY),
                                      1 - 2 * (self.QZ * self.QZ + self.QY * self.QY))) * 180 / math.pi + self.delayed_angle
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
            self.X_F_d = (self.X_F - self.Delayed_X_F) / self.sample_time
            self.Y_F_d = (self.Y_F - self.Delayed_Y_F) / self.sample_time
            self.R31_F_d = (self.R31_F - self.Delayed_R31_F) / self.sample_time
            self.R32_F_d = (self.R32_F - self.Delayed_R32_F) / self.sample_time
            temp_diff_angleY = self.angleY - self.Delayed_angleY
            if temp_diff_angleY > 280:  # depend on the rotation direction of the robot
                temp_diff_angleY = temp_diff_angleY - 360
            self.angleY_d = temp_diff_angleY / self.sample_time

            # delay data
            self.Delayed_X_F = self.X_F
            self.Delayed_Y_F = self.Y_F
            self.Delayed_R31_F = self.R31_F
            self.Delayed_R32_F = self.R32_F
            self.Delayed_angleY = self.angleY
        if self.num_of_bodies == 2:
            x, y, z, qx, qy, qz, qw, x2, y2, z2, qx2, qy2, qz2, qw2 = struct.unpack("hhhhhhhhhhhhhh", udp_data)
            print('codes not finished')
