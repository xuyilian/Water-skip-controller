from Library.IIR2Filter import IIR2Filter
import struct
import math


def saturation_fcn(A, satA):
    if A>satA[1]:
        A = satA[1]
    if A<satA[0]:
        A = satA[0]
    return A


def circular_saturation_fcn(U_X, U_Y, sat, ):
    SUM_S = U_X * U_X + U_Y * U_Y
    if SUM_S > sat * sat:
        U_X_new = U_X / math.sqrt(SUM_S) * sat
        U_Y_new = U_Y / math.sqrt(SUM_S) * sat
        return U_X_new, U_Y_new
    else:
        return U_X, U_Y


class RealTimeProcessor(object):
    def __init__(self, order, cutoff, ftype, design, rs, sample_rate, delayed_angle):
        self.flag = 0
        self.sample_time = 1 / sample_rate
        self.sample_rate = sample_rate

        self.X = 0
        self.Y = 0
        self.Z = 0
        self.QX = 1
        self.QY = 0
        self.QZ = 0
        self.QW = 0
        self.R11 = 0
        self.R12 = 0
        self.R13 = 0
        self.R21 = 0
        self.R22 = 0
        self.R23 = 0
        self.R31 = 0
        self.R32 = 0
        self.R33 = 0
        self.X_F = 0
        self.Y_F = 0
        self.Z_F = 0
        self.R13_F = 0
        self.R23_F = 0
        self.angleY = 0
        self.X_F_d = 0
        self.Y_F_d = 0
        self.R13_F_d = 0
        self.R23_F_d = 0
        self.angleY_d = 0

        self.FilterX = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterY = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterZ = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterR13 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterR23 = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)
        self.FilterangleY_d = IIR2Filter.IIR2Filter(order, [cutoff], ftype, design=design, rs=rs, fs=sample_rate)

        self.delayed_angle = delayed_angle

        self.Delayed_X_F = 0
        self.Delayed_Y_F = 0
        self.Delayed_R31_F = 0
        self.Delayed_R32_F = 0
        self.Delayed_angleY = 0
        self.Delayed_R11 = 0
        self.Delayed_R21 = 0
        self.Delayed_R31 = 0
        self.R11_d = 0
        self.R21_d = 0
        self.R31_d = 0

    def step(self,udp_data, fliter_on):
        x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data)
        self.X = x * 0.0005
        self.Y = y * 0.0005
        self.Z = z * 0.0005
        self.QX = float(qx * 0.001)
        self.QY = float(qy * 0.001)
        self.QZ = float(qz * 0.001)
        self.QW = float(qw * 0.001)


        # qaut2rotm
        yy = self.QY * self.QY
        xx = self.QX * self.QX
        zz = self.QZ * self.QZ
        xy = self.QX * self.QY
        xz = self.QX * self.QZ
        yz = self.QY * self.QZ
        wx = self.QW * self.QX
        wy = self.QW * self.QY
        wz = self.QW * self.QZ

        self.R11 = 1 - 2 * (yy + zz)
        self.R12 = 2 * (xy - wz)
        self.R13 = 2 * (xz + wy)
        self.R21 = 2 * (xy + wz)
        self.R22 = 1 - 2 * (xx + zz)
        self.R23 = 2 * (yz - wx)
        self.R31 = 2 * (xz - wy)
        self.R32 = 2 * (yz + wx)
        self.R33 = 1 - 2*(xx + yy)

        # filter
        self.X_F = self.FilterX.filter(self.X)
        self.Y_F = self.FilterY.filter(self.Y)
        self.Z_F = self.FilterZ.filter(self.Z)
        self.R13_F = self.FilterR13.filter(self.R13)
        self.R23_F = self.FilterR23.filter(self.R23)
        if not fliter_on:
            self.X_F = self.X
            self.Y_F = self.Y
            self.Z_F = self.Z
            self.R13_F = self.R13
            self.R23_F = self.R23

        # diff
        self.R11_d = (self.R11 - self.Delayed_R11) / self.sample_time
        self.R21_d = (self.R21 - self.Delayed_R21) / self.sample_time
        self.R31_d = (self.R31 - self.Delayed_R31) / self.sample_time
        self.X_F_d = (self.X_F - self.Delayed_X_F) / self.sample_time
        self.Y_F_d = (self.Y_F - self.Delayed_Y_F) / self.sample_time
        self.R13_F_d = (self.R13_F - self.Delayed_R31_F) / self.sample_time
        self.R23_F_d = (self.R23_F - self.Delayed_R32_F) / self.sample_time

        # delay data
        self.Delayed_X_F = self.X_F
        self.Delayed_Y_F = self.Y_F
        self.Delayed_R31_F = self.R13_F
        self.Delayed_R32_F = self.R23_F
        self.Delayed_angleY = self.angleY
        self.Delayed_R11 = self.R11
        self.Delayed_R21 = self.R21
        self.Delayed_R31 = self.R31

        angleY_d = self.R12 * self.R11_d + self.R22 * self.R21_d + self.R32 * self.R31_d
        self.angleY_d = self.FilterangleY_d.filter(angleY_d)

        self.angleY = (math.atan2(2 * (self.QW * self.QZ + self.QX * self.QY),
                                  1 - 2 * (self.QZ * self.QZ + self.QY * self.QY))) * 180 / math.pi + self.delayed_angle * self.angleY_d # +45+180
        if self.angleY > 180:
            self.angleY = self.angleY - 360
        if self.angleY < -180:
            self.angleY = self.angleY + 360
        if self.angleY > 180:
            self.angleY = self.angleY - 360
        if self.angleY < -180:
            self.angleY = self.angleY + 360
        if self.angleY > 180:
            self.angleY = self.angleY - 360
        # if self.angleY < -180:
        #     self.angleY = self.angleY + 360
        # if self.angleY > 180:
        #     self.angleY = self.angleY - 360
        # if self.angleY < -180:
        #     self.angleY = self.angleY + 360
        # if self.angleY > 180: self.angleY = self.angleY - 360
        # if self.angleY < 180: self.angleY = self.angleY + 360
        # if self.angleY > 180: self.angleY = self.angleY - 360
        # if self.angleY < 180: self.angleY = self.angleY + 360
        # if self.angleY > 180: self.angleY = self.angleY - 360
        # if self.angleY < 180: self.angleY = self.angleY + 360

        if self.flag == 0:
            self.angleY_d = 0
            self.flag = 1


