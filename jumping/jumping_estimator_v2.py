import numpy as np
import math
import time
from scipy.spatial.transform import Rotation
import threading
import socket
from tqdm import tqdm
import struct
from scipy.interpolate import RegularGridInterpolator


class JumpingModel3D:
    def __init__(self, model):
        self.model = model
        self.tol = 0.0001

    def fcn1(self, v_landing, v_takeoff_dir):
        # V_landing = np.array([x_dot, y_dot, z_dot])
        # V_takeoff_dir = np.array([x_dot, y_dot, z_dot])

        p_dot_ld = np.linalg.norm(v_landing)

        v_surface = np.cross(- v_landing, v_takeoff_dir)
        v_surface_norm = np.linalg.norm(v_surface)
        if v_surface_norm < self.tol:
            v_surface = np.array([1, 0, 0])
        else:
            v_surface = v_surface / v_surface_norm
        theta_v = math.atan2(v_surface_norm, np.dot(- v_landing, v_takeoff_dir))  # theta_v

        self.model.input_theta_v(theta_v, p_dot_ld)
        r = Rotation.from_rotvec(self.model.theta_l * v_surface)

        if p_dot_ld == 0:
            landing_attitude_z_b = np.array([0, 0, 1])
        else:
            landing_attitude_z_b = r.apply(-v_landing) / np.linalg.norm(p_dot_ld)
        v_takeoff = v_takeoff_dir * self.model.p_dot_tk

        return landing_attitude_z_b, v_takeoff, v_surface, theta_v


class InPlaneJumpingModel:
    def __init__(self, k1, k2):
        # simplest linear jumping model, the landing velocity does not affect the model,
        # see input_theta_l() for details
        self._k1 = k1
        self._k2 = k2

        # theta_l -> from - landing velocity to landing attitude
        # theta_t -> from - landing velocity to takeoff attitude
        # theta_v -> from - landing velocity to takeoff velocity
        # beta    -> from   landing attitude to takeoff attitude
        self.theta_l = 0.0
        self.theta_t = 0.0
        self.theta_v = 0.0
        self.beta = 0.0
        self.p_dot_ld = 2.0
        self.p_dot_tk = 2.0

    def input_theta_l(self, theta_l, p_dot_ld):
        self.p_dot_ld = p_dot_ld
        self.p_dot_tk = self.p_dot_ld

        self.theta_l = theta_l
        self.theta_t = self._k1 * self.theta_l  # main model
        self.theta_v = self._k2 * self.theta_l
        self.beta = self.theta_t - self.theta_l

    def input_beta(self, beta, p_dot_ld):
        self.beta = beta
        self.input_theta_l(self.beta / (self._k1 - 1), p_dot_ld)

    def input_theta_v(self, theta_v, p_dot_ld):
        self.theta_v = theta_v
        self.input_theta_l(self.theta_v / self._k2, p_dot_ld)

    def input_theta_t(self, theta_t, p_dot_ld):
        self.theta_t = theta_t
        self.input_theta_l(self.theta_t / self._k1, p_dot_ld)


class InPlaneJumpingModelLUTSpeedModel:
    def __init__(self, cLUTX, cLUTY, cLUT, vLUTX, vLUTY, vLUT):
        self.interp_func = RegularGridInterpolator((cLUTX, cLUTY), cLUT)
        self.interp_func_speed = RegularGridInterpolator((vLUTX, vLUTY), vLUT)
        self.point = np.array([3, 0.1])
        self.theta_l = 0
        self.p_dot_tk = 3
        self.min_speed = cLUTX[0]
        self.max_speed = cLUTX[-1]
        self.min_angle = 0
        self.max_angle = 180
        self.leg_efficiency = 0.85

    def input_theta_v(self, theta_v, landing_speed):

        landing_speed = abs(landing_speed)
        if landing_speed < self.min_speed:
            landing_speed = self.min_speed
        if landing_speed > self.max_speed:
            landing_speed = self.max_speed

        theta_v_deg = math.degrees(theta_v)
        if theta_v_deg < self.min_angle:
            theta_v_deg = self.min_angle
        if theta_v_deg > self.max_angle:
            theta_v_deg = self.max_angle

        self.point = np.array([landing_speed, theta_v_deg])

        self.theta_l = math.radians(self.interp_func(self.point)[0])
        self.p_dot_tk = self.interp_func_speed(self.point)[0]
        self.leg_efficiency = (self.p_dot_tk/landing_speed) ** 2


class InPlaneJumpingModelLUT:
    def __init__(self, cLUTX, cLUTY, cLUT):
        self.interp_func = RegularGridInterpolator((cLUTX, cLUTY), cLUT)
        self.point = np.array([3, 0.1])
        self.theta_l = 0
        self.p_dot_tk = 3
        self.min_speed = cLUTX[0]
        self.max_speed = cLUTX[-1]
        self.min_angle = 0
        self.max_angle = 180
        self.leg_efficiency = 0.85

    def input_theta_v(self, theta_v, landing_speed):

        landing_speed = abs(landing_speed)
        if landing_speed < self.min_speed:
            landing_speed = self.min_speed
        if landing_speed > self.max_speed:
            landing_speed = self.max_speed

        theta_v_deg = math.degrees(theta_v)
        if theta_v_deg < self.min_angle:
            theta_v_deg = self.min_angle
        if theta_v_deg > self.max_angle:
            theta_v_deg = self.max_angle

        self.point = np.array([landing_speed, theta_v_deg])
        self.theta_l = math.radians(self.interp_func(self.point)[0])
        self.p_dot_tk = landing_speed * 0.95

class DirectJumpingControllerLeg:
    def __init__(self, g=9.81, velocity_limit_ratio=1, velocity_gain=0.1,model=None, leg_efficiency=1):
        # parameters
        self.g = g
        self.slipping_angle = 25/180*math.pi
        self.velocity_limit_ratio = velocity_limit_ratio
        self.velocity_gain = velocity_gain
        self.takeoff_speed_ratio = 1.0/leg_efficiency
        self.leg_efficiency = leg_efficiency

        if model is None:
            self.model_2d = InPlaneJumpingModel(1.5, 2.5)
        else:
            self.model_2d = model
        self.JM3D = JumpingModel3D(self.model_2d)

        self.ready = 0
        self.robot_zb = np.array([0,0,1])
        # variables
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.jumping_altitude = 0.5
        self.landing_velocity = np.array([0, 0, -2])
        self.takeoff_velocity = np.array([0, 0, 2])
        self.landing_x = 0.0
        self.landing_y = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.leg_pitch = 0.0
        self.lroll = 0.0
        self.lpitch = 0.0
        self.lyaw = 0.0
        self.lroll1 = 0.0
        self.lpitch1 = 0.0
        self.lyaw1 = 0.0

        self.time_planed = 1.0

        self.landing_att_x = 0
        self.landing_att_y = 0
        self.landing_att_z = 1

        self.landing_att_xh = 0
        self.landing_att_yh = 0
        self.landing_att_zh = 1

        self.takeoff_att_x = 0
        self.takeoff_att_y = 0
        self.takeoff_att_z = 1

        self.yaw_turning = 0
        self.desired_yaw_update = 0

        self.psi = 0

        self.zbw_sym = np.array([0, 0, 1])


        self.logging_list = ['LJC_landing_att_x', 'LJC_landing_att_y', 'LJC_landing_att_z',]
        self.logging_data = [0.0] * len(self.logging_list)


    def init(self):
        self.roll, self.pitch, self.yaw, self.leg_pitch = 0, 0, 0, 0
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.jumping_altitude = 0.5
        self.landing_velocity = np.array([0, 0, -2])
        self.takeoff_velocity = np.array([0, 0, 2])
        self.landing_x = 0.0
        self.landing_y = 0.0

    def set_reference(self, desired_x, desired_y, jumping_altitude, ):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.jumping_altitude = jumping_altitude

    def update_landing_state(self, landing_x_dot, landing_y_dot, landing_z_dot,
                             landing_x, landing_y, ):
        self.landing_velocity = np.array([landing_x_dot, landing_y_dot, landing_z_dot])
        self.landing_x = landing_x
        self.landing_y = landing_y

    def update_zbw_sym(self, roll, pitch, yaw):
        """
        根据 roll, pitch, yaw 更新 zbw 和 z 轴对称后的 zbw
        """
        # 计算三角函数
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        # 原始 zbw
        self.zbw = np.array([
            cy*sp*cr + sy*sr,
            sy*sp*cr - cy*sr,
            cp*cr
        ])

        self.zbw_sym = np.array([-self.zbw[0], -self.zbw[1], self.zbw[2]])
        self.zbw_sym = self.zbw_sym / np.linalg.norm(self.zbw_sym)

    def jumping_planning(self, ):
        time_climb = sqrt_safe(2 * self.jumping_altitude / self.g, 0.01)
        # time_climb = math.sqrt(2 * self.jumping_altitude / self.g)
        #time_climb = - self.landing_velocity[2] / self.g 
        time_fall = time_climb
        self.time_planed = time_climb + time_fall
        if self.time_planed == 0:
            u_x = 0
            u_y = 0
        else:
            u_x = self.velocity_gain*(self.desired_x - self.landing_x) / self.time_planed
            u_y = self.velocity_gain*(self.desired_y - self.landing_y) / self.time_planed
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)

        landing_speed = np.linalg.norm(self.landing_velocity)
        predicted_takeoff_speed = landing_speed * self.takeoff_speed_ratio  # considering the powered climbing

        lateral_velocity_limit = predicted_takeoff_speed * self.velocity_limit_ratio
        if norm_v > lateral_velocity_limit:
            u_x = lateral_velocity_limit * u_x / norm_v
            u_y = lateral_velocity_limit * u_y / norm_v
        self.takeoff_velocity = np.array([u_x, u_y, math.sqrt(predicted_takeoff_speed * predicted_takeoff_speed - u_x * u_x - u_y * u_y)])


    def inverse_jumping_model(self, yaw):

        landing_speed = np.linalg.norm(self.landing_velocity)
        if landing_speed < 1e-6:
                # fallback to vertical if velocity is too small
            landing_attitude_z_b = np.array([0.0, 0.0, 1.0])
        else:
            landing_attitude_z_b = -self.landing_velocity / landing_speed

        self.landing_att_x = landing_attitude_z_b[0]
        self.landing_att_y = landing_attitude_z_b[1]
        self.landing_att_z = landing_attitude_z_b[2]

        # directly compute landing roll/pitch from landing velocity direction and yaw
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        v_landing_desired_body = r.apply(landing_attitude_z_b)

        vy_clamped = clamp(v_landing_desired_body[1])
        self.lroll = - math.asin(vy_clamped)

        cos_roll = math.cos(self.lroll)
        if abs(cos_roll) < 1e-6:
            cos_roll = 1e-6 if cos_roll >= 0 else -1e-6

        sx = clamp(v_landing_desired_body[0] / cos_roll)
        self.lpitch = math.asin(sx)

        self.lroll = self.lroll * 180 / math.pi
        self.lpitch = self.lpitch * 180 / math.pi


        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        v_landing_desired_body1 = r.apply(self.zbw_sym)
        vy_clamped = clamp(v_landing_desired_body1[1])
        self.lroll1 = - math.asin(vy_clamped)
        cos_roll = math.cos(self.lroll1)
        if abs(cos_roll) < 1e-6:
            cos_roll = 1e-6 if cos_roll >= 0 else -1e-6
        sx = clamp(v_landing_desired_body1[0] / cos_roll)
        self.lpitch1 = math.asin(sx)
        self.lroll1 = self.lroll1 * 180 / math.pi
        self.lpitch1 = self.lpitch1 * 180 / math.pi


        landing_attitude_z_bh, _, _, _ = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)
        self.landing_att_xh = landing_attitude_z_bh[0]
        self.landing_att_yh = landing_attitude_z_bh[1]
        self.landing_att_zh = landing_attitude_z_bh[2]
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        v_landing_desired_bodyh = r.apply(landing_attitude_z_bh)
        self.roll = - math.asin(v_landing_desired_bodyh[1])
        self.pitch = math.asin(v_landing_desired_bodyh[0] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi
        self.roll = max(-20.0, min(25.0, self.roll))
        self.pitch = max(-20.0, min(25.0, self.pitch))


class LinearJumpingControllerLeg:
    def __init__(self, g=9.81, velocity_limit_ratio=0.3, velocity_gain=1, model=None, leg_efficiency=1):
        # parameters
        self.g = g
        self.slipping_angle = 25/180*math.pi
        self.velocity_limit_ratio = velocity_limit_ratio
        self.velocity_gain = velocity_gain

        self.takeoff_speed_ratio = 1.0/leg_efficiency
        self.leg_efficiency = leg_efficiency

        if model is None:
            self.model_2d = InPlaneJumpingModel(1.5, 2.5)
        else:
            self.model_2d = model
        self.JM3D = JumpingModel3D(self.model_2d)

        # variables
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.jumping_altitude = 0.5
        self.landing_velocity = np.array([0, 0, -2])
        self.takeoff_velocity = np.array([0, 0, 2])
        self.landing_x = 0.0
        self.landing_y = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.leg_pitch = 0.0

        self.time_planed = 1.0

        self.landing_att_x = 0
        self.landing_att_y = 0
        self.landing_att_z = 1

        self.yaw_turning = 0
        self.desired_yaw_update = 0

        self.logging_list = ['LJC_landing_att_x', 'LJC_landing_att_y', 'LJC_landing_att_z',]
        self.logging_data = [0.0] * len(self.logging_list)

    def init(self):
        self.roll, self.pitch, self.yaw, self.leg_pitch = 0, 0, 0, 0
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.jumping_altitude = 0.5
        self.landing_velocity = np.array([0, 0, -2])
        self.takeoff_velocity = np.array([0, 0, 2])
        self.landing_x = 0.0
        self.landing_y = 0.0

    def set_reference(self, desired_x, desired_y, jumping_altitude, ):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.jumping_altitude = jumping_altitude

    def update_landing_state(self, landing_x_dot, landing_y_dot, landing_z_dot,
                             landing_x, landing_y, ):
        self.landing_velocity = np.array([landing_x_dot, landing_y_dot, landing_z_dot])
        self.landing_x = landing_x
        self.landing_y = landing_y

    def jumping_planning(self, ):
        time_climb = sqrt_safe(2 * self.jumping_altitude / self.g, 0.01)
        # time_climb = math.sqrt(2 * self.jumping_altitude / self.g)
        #time_climb = - self.landing_velocity[2] / self.g 
        time_fall = time_climb
        self.time_planed = time_climb + time_fall
        if self.time_planed == 0:
            u_x = 0
            u_y = 0
        else:
            u_x = self.velocity_gain*(self.desired_x - self.landing_x) / self.time_planed
            u_y = self.velocity_gain*(self.desired_y - self.landing_y) / self.time_planed
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)

        landing_speed = np.linalg.norm(self.landing_velocity)
        predicted_takeoff_speed = landing_speed * self.takeoff_speed_ratio  # considering the powered climbing

        lateral_velocity_limit = predicted_takeoff_speed * self.velocity_limit_ratio
        if norm_v > lateral_velocity_limit:
            u_x = lateral_velocity_limit * u_x / norm_v
            u_y = lateral_velocity_limit * u_y / norm_v
        self.takeoff_velocity = np.array([u_x, u_y, math.sqrt(predicted_takeoff_speed * predicted_takeoff_speed - u_x * u_x - u_y * u_y)])

    def jumping_planning_angle(self, u_x, u_y, limit_z=0.7, ):
        temp = math.sqrt(u_x * u_x + u_y * u_y)
        if temp > 1:
            self.takeoff_velocity = np.array([u_x, u_y, temp * limit_z])
        else:
            self.takeoff_velocity = np.array([u_x, u_y, limit_z])

    def jumping_planning_speed(self, u_x, u_y, ratio=1.414, speed_gain=1):
        desired_speed = math.sqrt(u_x * u_x + u_y * u_y)*speed_gain
        landing_speed = np.linalg.norm(self.landing_velocity)
        predicted_takeoff_speed = landing_speed

        # print(desired_speed, landing_speed, predicted_takeoff_speed)

        if desired_speed > predicted_takeoff_speed/ratio:
            desired_speed = predicted_takeoff_speed/ratio
        speed_z = math.sqrt(predicted_takeoff_speed**2 - desired_speed**2)

        self.takeoff_velocity = np.array([u_x*speed_gain, u_y*speed_gain, speed_z])

    def inverse_jumping_model_leg(self, yaw, pitch, roll, anto_turn=False, desired_yaw=0):
        landing_attitude_z_b, _, v_surface, theta_v = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)
        if anto_turn:
            v_surface_project = np.matmul(np.array([[0, 1], [-1, 0]]), v_surface[0:2])
            self.yaw_turning = math.atan2(v_surface_project[1], v_surface_project[0])
            if theta_v > math.radians(40):
                yaw = self.yaw_turning
                self.desired_yaw_update = yaw
            else:
                self.desired_yaw_update = desired_yaw

        # slipping avoidance
        leg_angle = angle_between_vectors(landing_attitude_z_b, np.array([0, 0, 1]))
        if leg_angle > self.slipping_angle:
            rot_angle = leg_angle - self.slipping_angle
            rot_axis = np.cross(landing_attitude_z_b, np.array([0, 0, 1]))
            rot_axis = rot_axis/np.linalg.norm(rot_axis) * rot_angle
            landing_attitude_z_b = np.matmul(Rotation.from_rotvec(rot_axis).as_matrix(), landing_attitude_z_b)

        self.landing_att_x = landing_attitude_z_b[0]
        self.landing_att_y = landing_attitude_z_b[1]
        self.landing_att_z = landing_attitude_z_b[2]
        self.yaw, self.pitch, self.roll, self.leg_pitch = leg_body_pitch_transformation(yaw, pitch, roll, landing_attitude_z_b)
        self.logging_data = [self.landing_att_x, self.landing_att_y, self.landing_att_z]

    def inverse_jumping_model(self, yaw,):
        landing_attitude_z_b, _, _, _ = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)
        self.landing_att_x = landing_attitude_z_b[0]
        self.landing_att_y = landing_attitude_z_b[1]
        self.landing_att_z = landing_attitude_z_b[2]
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        v_landing_desired_body = r.apply(landing_attitude_z_b)
        self.roll = - math.asin(v_landing_desired_body[1])
        self.pitch = math.asin(v_landing_desired_body[0] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi
        self.logging_data = [self.landing_att_x, self.landing_att_y, self.landing_att_z]

def clamp(x):
    return max(-1.0, min(1.0, x))

def leg_body_pitch_transformation(yaw, pitch, roll, z_b):
    zb12 = z_b[0] * z_b[0]
    zb22 = z_b[1] * z_b[1]
    norm_z_b = math.sqrt(zb12 + zb22 + z_b[2] * z_b[2])
    z_b = z_b / norm_z_b

    Cp = math.cos(pitch)
    Sp = math.sin(pitch)
    Cr = math.cos(roll)
    Sr = math.sin(roll)
    Cy = math.cos(yaw)
    Sy = math.sin(yaw)

    SpSr = Sp * Sr
    CrCy = Cr * Cy

    R_imu = np.array([[Cp * Cy, Cy * SpSr - Cr * Sy, Sr * Sy + CrCy * Sp],
                      [Cp * Sy, CrCy + SpSr * Sy, Cr * Sp * Sy - Cy * Sr],
                      [-Sp, Cp * Sr, Cp * Cr]])

    V = R_imu[:, 1]

    roataion_axis = np.array([V[1], -V[0], 0])
    roataion_axis = roataion_axis / math.sqrt(V[1] * V[1] + V[0] * V[0])
    r1 = roataion_axis[0]
    r2 = roataion_axis[1]
    cross_z_b_e_3 = np.array([z_b[1], -z_b[0], 0])
    norm_cross_z_b_e_3 = math.sqrt(zb12 + zb22)

    if norm_cross_z_b_e_3 == 0:
        R0 = np.eye(3)
    else:
        # R0 = R_axang(cross_z_b_e_3 / norm_cross_z_b_e_3, math.atan2(norm_cross_z_b_e_3, z_b[2]))

        R0_axis = cross_z_b_e_3 / norm_cross_z_b_e_3
        R0_theta = math.atan2(norm_cross_z_b_e_3, z_b[2])
        ST = math.sin(R0_theta)
        CT = math.cos(R0_theta)
        CT1 = 1 - CT

        uyST = R0_axis[1] * ST
        uxST = R0_axis[0] * ST

        uxuyCT1 = R0_axis[0] * R0_axis[1] * CT1
        uyuzCT1 = R0_axis[1] * R0_axis[2] * CT1

        R0 = np.array([[R0_axis[0] * R0_axis[0] * CT1 + CT, uxuyCT1, uyST],
                       [uxuyCT1, R0_axis[1] * R0_axis[1] * CT1 + CT, uyuzCT1 - uxST],
                       [- uyST, uyuzCT1 + uxST, CT]])

    R32V2 = R0[2, 1] * V[1]
    R31V1 = R0[2, 0] * V[0]
    r1r2 = r1 * r2
    r12 = r1 ** 2
    r22 = r2 ** 2
    R32V1r1r2 = R0[2, 1] * V[0] * r1r2
    R31V2r1r2 = R0[2, 0] * V[1] * r1r2
    R32V2r22 = R32V2 * r22
    R31V1r12 = R31V1 * r12
    A = R31V1 + R32V2 + R0[2, 2] * V[2] - R31V1r12 - R32V2r22 - R31V2r1r2 - R32V1r1r2
    B = 2 * R0[2, 0] * V[2] * r2 - R0[2, 1] * V[2] * r1 - R0[2, 2] * V[0] * r2 + R0[2, 2] * V[1] * r1
    C = R31V1r12 + R32V2r22 + R31V2r1r2 + R32V1r1r2
    theta = - math.pi / 2 + math.atan2(B, A) - math.asin(C / math.sqrt(A ** 2 + B ** 2))

    ST = math.sin(theta)
    CT = math.cos(theta)
    CT1 = 1 - CT
    uyST = r2 * ST
    uxST = r1 * ST
    uxuyCT1 = r1r2 * CT1
    R1 = np.array([[r12 * CT1 + CT, uxuyCT1, uyST],
                   [uxuyCT1, r22 * CT1 + CT, - uxST],
                   [- uyST, uxST, CT]])
    R_body = np.matmul(R1, R_imu)
    R_body_z_b = R_body[:, 2]
    R_body_y_b = R_body[:, 1]

    leg_pitch = math.asin(np.dot(np.cross(R_body_z_b, z_b), -R_body_y_b)) * 180 / math.pi
    eul = Rotation.from_matrix(R_body).as_euler('ZYX', degrees=True)

    return eul[0], eul[1], eul[2], leg_pitch


class PoweredClimbingTimer:
    def __init__(self, waitting_cycles=10):
        self.takeoff_timestamp = -1
        self.powered_climbing_on = False
        self.RD = RiseDetect()
        self.powered_climbing_on2off = False
        self.powered_climbing_on2off_delay = False
        self.waitting_cycles = waitting_cycles
        self.cycle = waitting_cycles
        self.control_flag = True

        self.logging_list = ['PCT_powered_climbing_on', 'PCT_powered_climbing_on2off', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def step(self, jumping_state, jumping_state_old, abs_time, power_time):
        if jumping_state_old == 2 and jumping_state == 1:
            # takeoff timestamp
            self.takeoff_timestamp = abs_time
            self.powered_climbing_on = True
            self.cycle = self.waitting_cycles
            self.control_flag = False

        elif jumping_state == 2:
            # stance phase
            self.powered_climbing_on = False
        elif jumping_state == 1:
            # aerial phase
            if abs_time - self.takeoff_timestamp > power_time:
                self.powered_climbing_on = False

            self.cycle = self.cycle - 1
            if self.cycle < 0:
                self.control_flag = True


        self.powered_climbing_on2off_delay = self.powered_climbing_on2off
        self.powered_climbing_on2off = self.RD.step(not self.powered_climbing_on)
        self.logging_data = [self.powered_climbing_on, self.powered_climbing_on2off, ]


class PoweredClimbingControl:
    def __init__(self, max_time=0.1, min_time=0.015):
        self.gover2 = 0.5/9.81
        self.RD = RiseDetect()
        self.powered_climbing_on = False
        self.powered_climbing_on2off = False
        self.powered_climbing_end_flag = False
        self.takeoff_timestamp = time.time()
        self.max_time = max_time
        self.min_time = min_time

        self.logging_list = ['PCC_powered_climbing_on', 'PCC_powered_climbing_on2off', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def step(self, jumping_state, jumping_state_old, desired_z, z, vx, vy, vz):
        if jumping_state_old == 2 and jumping_state == 1:
            # takeoff timestamp
            self.powered_climbing_on = True
            self.powered_climbing_end_flag = False
            self.takeoff_timestamp = time.time()

        elif jumping_state == 2:
            # stance phase
            self.powered_climbing_on = False
        elif jumping_state == 1:
            # aerial phase

            if not self.powered_climbing_end_flag:
                if time.time() - self.takeoff_timestamp > self.min_time:
                    total_energy = z + self.gover2 * (vx**2 + vy**2 + vz**2)
                    if (total_energy > desired_z) or (time.time() - self.takeoff_timestamp > self.max_time):
                        self.powered_climbing_on = False
                        self.powered_climbing_end_flag = True
        self.powered_climbing_on2off = self.RD.step(not self.powered_climbing_on)

        self.logging_data = [self.powered_climbing_on, self.powered_climbing_on2off, ]


class JumpingStateTrackerOnboard:
    def __init__(self, acc_z_up_limit=2.0*9.81, show_state_change=False):
        self.acc_z_up_limit = acc_z_up_limit
        self.acc_z_delay = 9.81
        self.jumping_state = 1
        self.jumping_state_old = 1

        self.takeoff_time = -1
        self.landing_time = -1


        self.aerial_time = 1.0
        self.show_state_change = show_state_change
        self.first_takeoff = False

        self.logging_list = ['JSTO_jumping_state', 'JSTO_aerial_time', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def step(self, acc_z, abs_time):
        self.jumping_state_old = self.jumping_state

        # aerial to stance
        if self.acc_z_delay < self.acc_z_up_limit < acc_z:
            self.jumping_state = 2  # stance phase
            self.landing_time = abs_time
            if self.first_takeoff:
                self.aerial_time = self.landing_time - self.takeoff_time
            if self.show_state_change:
                print("->landing")

        # stance to aerial
        if acc_z < self.acc_z_up_limit < self.acc_z_delay:
            self.first_takeoff = True
            self.jumping_state = 1  # aerial phase
            self.takeoff_time = abs_time
            if self.show_state_change:
                print("->takeoff")

        # print(self.jumping_state)

        self.acc_z_delay = acc_z
        self.logging_data = [self.jumping_state, self.aerial_time, ]

    def init(self, ):
        self.first_takeoff = False
        self.jumping_state = 1
        self.jumping_state_old = 1
        self.aerial_time = 0.5


class RiseDetect:
    def __init__(self, ):
        self.flag = False
        self.flag_old = False
        self.force_enable_flag = False

    def step(self, input):
        self.flag_old = self.flag
        self.flag = input

        if self.force_enable_flag:
            self.force_enable_flag = False
            return True
        else:
            if self.flag and not self.flag_old:
                return True
            else:
                return False

    def force_enable(self):
        self.force_enable_flag = True


class JumpingStateEstimator2TOF_leg:
    # using threading to reduce the computational time
    def __init__(self, k1, k2, leg_efficiency, g, direction_gain, altitude_gain, attitude_correction_gain,
                 landing_speed_old=4, max_iteration_count=50, acc_bias=np.array([0.1, 0.045, 0]),
                 direction_error_limit=1, altitude_error_limit=0.04, complementary_filter_gain_z=0.1, ):

        self.tof2com_offset = 0.115
        self.direction_error_limit = direction_error_limit
        self.altitude_error_limit = altitude_error_limit
        # stance model parameters
        self.iteration_count = 0
        self.max_iteration_count_first_run = 3
        self.max_iteration_count = max_iteration_count

        self.attitude_correction_gain = attitude_correction_gain
        self.k1 = k1
        self.k2 = k2
        self.leg_efficiency = leg_efficiency

        # constants
        self.g = g
        self.g2 = g * 2

        self.e_3 = np.array([0, 0, 1])

        # solver parameter
        self.direction_gain = direction_gain
        self.altitude_gain = altitude_gain

        self.landing_speed_old = landing_speed_old

        self.aerial_start_flag = False
        self.RD_aerial_start_flag = RiseDetect()

        self.buffer_flag = True
        # buffer set 1
        self.time_course_1 = [0]
        self.acc_body_1 = [np.array([0, 0, 0])]
        self.quat_1 = [np.array([0, 0, 0, 1])]
        self.quat_leg_1 = [np.array([0, 0, 0, 1])]
        self.z_1 = [0]
        # buffer set 2
        self.time_course_2 = [0]
        self.acc_body_2 = [np.array([0, 0, 0])]
        self.quat_2 = [np.array([0, 0, 0, 1])]
        self.quat_leg_2 = [np.array([0, 0, 0, 1])]
        self.z_2 = [0]

        self.complementary_filter_gain_z_measure = complementary_filter_gain_z
        self.complementary_filter_gain_z_acc_measure = 1 - self.complementary_filter_gain_z_measure

        self.R_TO = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.R_TO_old = self.R_TO
        self.R_LD = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.R_LD_old = self.R_LD

        self.correction_matrix = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.altitude_error = 0
        self.directional_error = 0

        # variables inside the loop
        self.R_c_ini = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.landing_speed_prediction = 3

        self.landing_speed_old_prediction = 3
        self.landing_speed_old_original = 3

        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.takeoff_velocity_original = np.array([0, 0, 3])
        self.landing_velocity_prediction = np.array([0, 0, -3])
        self.landing_location_prediction = np.array([0, 0, 0])

        self.quat_corr = np.array([0, 0, 0, 1])

        self.v_TO_unit = np.array([0, 0, 1])

        self.time_cost = 0
        self.height = 0.458

        self.current_velocity = np.array([0, 0, 0])
        self.current_position = np.array([0, 0, 0])
        self.current_z_dot_old = 0

        self.acc_bias = acc_bias

        self.position_temp = np.array([0, 0, 0])
        self.velocity_temp = np.array([0, 0, -4])
        self.landing_speed_old_temp = 3
        self.max_iteration_count_separate = 5
        self.stop_separate_computation_flag = False
        self.separate_computation_start = True
        self.v_TO_unit_temp = np.array([0, 0, 1])

        self.takeoff_velocity_old = np.array([0, 0, 1])

        self.lock = threading.Lock()  # 创建一个lock对象
        self.iteration_done = False
        self.quat_corr_ready = False
        self.start_iteration = False
        self.landing_location_old_worker = np.array([0, 0, 1])
        self.buffer_flag_worker = False
        self._estimator_thread = threading.Thread(target=self._worker, args=(), )
        self._estimator_thread_stop = False
        time.sleep(0.01)
        self._estimator_thread.start()

        self.logging_list = ['JSE_time_cost', 'JSE_height',
                             'JSE_v_TO_x_predicted', 'JSE_v_TO_y_predicted', 'JSE_v_TO_z_predicted',
                             'JSE_v_LD_x_predicted', 'JSE_v_LD_y_predicted', 'JSE_v_LD_z_predicted',
                             'JSE_v_x', 'JSE_v_y', 'JSE_v_z',
                             'JSE_p_x', 'JSE_p_y', 'JSE_p_z',
                             'JSE_acc_x', 'JSE_acc_y', 'JSE_acc_z',
                             'JSE_start_iteration', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def _worker(self):
        print('JSE: estimator thread start')
        while not self._estimator_thread_stop:
            if self.start_iteration:
                self._iteration_worker()
                self.start_iteration = False

            time.sleep(0.005)

    def stop(self):
        self._estimator_thread_stop = True
        time.sleep(2)
        self._estimator_thread.join()

    def init(self):
        pass

    def update(self, t, acc, quat, estimator_state, z, quat_leg):
        acc = acc - self.acc_bias
        start_time = time.time()

        if estimator_state == 21:
            self._takeoff(quat, self.buffer_flag, quat_leg)

            # change target buffer set
            self.buffer_flag = not self.buffer_flag

        self.lock.acquire()
        if self.aerial_start_flag:
            # append
            if self.buffer_flag:
                self.time_course_1.append(t)
                self.acc_body_1.append(acc)
                self.quat_1.append(quat)
                self.quat_leg_1.append(quat_leg)
                self.z_1.append(z+self.tof2com_offset)
            else:
                self.time_course_2.append(t)
                self.acc_body_2.append(acc)
                self.quat_2.append(quat)
                self.quat_leg_2.append(quat_leg)
                self.z_2.append(z+self.tof2com_offset)
        else:
            # first time (this is the takeoff timestamp)
            self.aerial_start_flag = True

            # clean and init the arrays
            if self.buffer_flag:
                self.time_course_1 = [t]
                self.acc_body_1 = [acc]
                self.quat_1 = [quat]
                self.quat_leg_1 = [quat_leg]
                self.z_1 = [z+self.tof2com_offset]
            else:
                self.time_course_2 = [t]
                self.acc_body_2 = [acc]
                self.quat_2 = [quat]
                self.quat_leg_2 = [quat_leg]
                self.z_2 = [z+self.tof2com_offset]
        self.lock.release()

        if estimator_state == 12:
            self._landing(quat_leg)
            self.aerial_start_flag = False

        if self.iteration_done:
            self.iteration_done = False
            self.update_current_state()
        else:
            self.current_z_dot_old = self.current_velocity[2]
            self.current_state()

        self.height = self.current_position[2] + (self.current_velocity[2] ** 2) / self.g2
        if self.height < 0.3:
            self.height = 0.3

        self.time_cost = time.time() - start_time
        self.logging_data = [self.time_cost, self.height,
                             self.takeoff_velocity_prediction[0], self.takeoff_velocity_prediction[1],
                             self.takeoff_velocity_prediction[2],
                             self.landing_velocity_prediction[0], self.landing_velocity_prediction[1],
                             self.landing_velocity_prediction[2],
                             self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             acc[0], acc[1], acc[2],
                             self.start_iteration, ]

    def _landing(self, quat_leg):
        # quat = np.array([0, 0, 0, 1])
        self.R_LD_old = self.R_LD
        self.R_LD = Rotation.from_quat(quat_leg).as_matrix()

    def _takeoff(self, quat, buffer_flag, quat_leg):
        self.iteration_count = 0
        # update the takeoff attitude
        self.R_TO_old = self.R_TO
        self.R_TO = Rotation.from_quat(quat_leg).as_matrix()

        position = np.array([0, 0, 0])
        velocity = np.array([0, 0, -4])
        v_TO_unit = np.array([0, 0, 1])
        takeoff_velocity_old = np.array([0, 0, 1])
        directional_error = 10
        altitude_error = 10
        height = 0

        landing_location_old = self.landing_location_prediction

        R_c = self.R_c_ini
        landing_speed_old = self.landing_speed_prediction
        self.landing_speed_old_original = landing_speed_old

        while directional_error > self.direction_error_limit or abs(altitude_error) > self.altitude_error_limit:
            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old = self._prediction_error(R_c, landing_speed_old, landing_location_old, buffer_flag)
            temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
            R_c = np.matmul(Rotation.from_rotvec(temp_axis * - self.direction_gain).as_matrix(), R_c)
            landing_speed_old = landing_speed_old - altitude_error * self.altitude_gain

            if self.iteration_count == 0:
                landing_speed_original = np.linalg.norm(velocity)
                takeoff_speed_original = math.sqrt(self.leg_efficiency * landing_speed_original * landing_speed_original)
                self.takeoff_velocity_original = v_TO_unit * takeoff_speed_original
            self.iteration_count += 1
            if self.iteration_count >= self.max_iteration_count_first_run:
                break

        self.takeoff_velocity_old = takeoff_velocity_old
        self.landing_speed_old_prediction = landing_speed_old
        self.landing_location_prediction = position
        self.landing_speed_prediction = np.linalg.norm(velocity)
        self.landing_velocity_prediction = velocity
        self.takeoff_speed_prediction = math.sqrt(self.leg_efficiency * self.landing_speed_prediction * self.landing_speed_prediction)
        self.takeoff_velocity_prediction = v_TO_unit * self.takeoff_speed_prediction
        self.correction_matrix = R_c
        self.altitude_error = altitude_error
        self.directional_error = directional_error

        # self.quat_corr = Rotation.from_rotvec(
        #     Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(canonical=False)

        # run the worker
        self.landing_location_old_worker = landing_location_old
        self.buffer_flag_worker = buffer_flag
        self.start_iteration = True
        print('initial error: ', self.altitude_error, self.directional_error)

    def _iteration_worker(self, ):
        buffer_flag = self.buffer_flag_worker
        landing_location_old = self.landing_location_old_worker

        R_c = self.correction_matrix
        landing_speed_old = self.landing_speed_old_prediction
        while True:
            time.sleep(0.001)
            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old = self._prediction_error(R_c, landing_speed_old, landing_location_old, buffer_flag)
            temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
            R_c = np.matmul(Rotation.from_rotvec(temp_axis * - self.direction_gain).as_matrix(), R_c)
            landing_speed_old = landing_speed_old - altitude_error * self.altitude_gain
            self.iteration_count += 1
            if (self.iteration_count > self.max_iteration_count) or (directional_error < self.direction_error_limit and abs(altitude_error) < self.altitude_error_limit):
                break
        self.takeoff_velocity_old = takeoff_velocity_old
        self.landing_speed_old_prediction = landing_speed_old
        self.landing_location_prediction = position
        self.landing_speed_prediction = np.linalg.norm(velocity)
        self.landing_velocity_prediction = velocity
        self.takeoff_speed_prediction = math.sqrt(
            self.leg_efficiency * self.landing_speed_prediction * self.landing_speed_prediction)
        self.takeoff_velocity_prediction = v_TO_unit * self.takeoff_speed_prediction
        self.correction_matrix = R_c
        self.altitude_error = altitude_error
        self.directional_error = directional_error

        self.quat_corr = Rotation.from_rotvec(Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(canonical=False)
        self.quat_corr_ready = True
        self.iteration_done = True
        print(self.iteration_count)
        # print(directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old, landing_speed_old)
        print('altitude error: ', self.altitude_error, 'directional error: ', self.directional_error)

    def get_quat_corr(self):
        self.quat_corr_ready = False
        return self.quat_corr

    def _prediction_error(self, R_c, landing_speed_old, landing_location_old, buffer_flag):
        # landing_location_old = np.array([0, 0, 0])

        z_b_LD_old = np.matmul(R_c, np.matmul(self.R_LD_old, self.e_3))
        z_b_TO_old = np.matmul(R_c, np.matmul(self.R_TO_old, self.e_3))
        z_b_LD = np.matmul(R_c, np.matmul(self.R_LD, self.e_3))
        z_b_TO = np.matmul(R_c, np.matmul(self.R_TO, self.e_3))
        takeoff_speed_old = math.sqrt(self.leg_efficiency * landing_speed_old * landing_speed_old)
        v_TO_unit_old, v_LD_unit_old = self._compute_velocities(z_b_LD_old, z_b_TO_old)
        v_TO_unit, v_LD_unit = self._compute_velocities(z_b_LD, z_b_TO)
        takeoff_velocity_old = v_TO_unit_old * takeoff_speed_old

        # initial condition
        position = landing_location_old
        velocity = takeoff_velocity_old
        height = 0
        index = 0

        self.lock.acquire()
        if buffer_flag:
            for t in self.time_course_1:

                R = Rotation.from_quat(self.quat_1[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_1[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_1[index] - self.z_1[index-1]) / dt
                    R_prev = Rotation.from_quat(self.quat_leg_1[index - 1]).as_matrix()
                    R_now = Rotation.from_quat(self.quat_leg_1[index]).as_matrix()
                    if self.z_1[index] == self.tof2com_offset or self.z_1[index - 1] == self.tof2com_offset:
                        pass
                    else:
                        velocity_z = (np.matmul(R_c, R_now)[2][2] * self.z_1[index] - np.matmul(R_c, R_prev)[2][2] * self.z_1[index - 1]) / dt
                        velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    # position[2] = self.z_1[index]

                    position = position + velocity * dt
                    position_z = self.z_1[index]
                    # position[2] = position[2] + (position_z - position[2]) * self.complementary_filter_gain_z_measure

                    if position[2] > height:
                        height = position[2]
                index += 1
        else:
            for t in self.time_course_2:

                R = Rotation.from_quat(self.quat_2[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_2[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
                    R_prev = Rotation.from_quat(self.quat_leg_2[index - 1]).as_matrix()
                    R_now = Rotation.from_quat(self.quat_leg_2[index]).as_matrix()
                    if self.z_2[index] == self.tof2com_offset or self.z_2[index - 1] == self.tof2com_offset:
                        pass
                    else:
                        velocity_z = (np.matmul(R_c, R_now)[2][2] * self.z_2[index] - np.matmul(R_c, R_prev)[2][2] * self.z_2[index - 1]) / dt
                        velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    # position[2] = self.z_2[index]
                    position = position + velocity * dt
                    position_z = self.z_2[index]
                    # position[2] = position[2] + (position_z - position[2]) * self.complementary_filter_gain_z_measure

                    if position[2] > height:
                        height = position[2]
                index += 1

        self.lock.release()

        landing_velocity_predicted = velocity
        v_LD_unit_predicted = landing_velocity_predicted / np.linalg.norm(landing_velocity_predicted)

        altitude_error = position[2]
        directional_error = angle_between_vectors(v_LD_unit_predicted, v_LD_unit) * 180 / math.pi

        return directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old

    def update_current_state(self):

        position = self.landing_location_prediction
        velocity = self.takeoff_velocity_prediction
        R_c = self.correction_matrix
        height = 0
        index = 0
        if not self.buffer_flag_worker:  # switch to another buffer set
            for t in self.time_course_1:
                R = Rotation.from_quat(self.quat_1[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_1[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_1[index] - self.z_1[index - 1]) / dt
                    R_prev = Rotation.from_quat(self.quat_leg_1[index - 1]).as_matrix()
                    R_now = Rotation.from_quat(self.quat_leg_1[index]).as_matrix()
                    if self.z_1[index] == self.tof2com_offset or self.z_1[index - 1] == self.tof2com_offset:
                        pass
                    else:
                        velocity_z = (np.matmul(R_c, R_now)[2][2] * self.z_1[index] - np.matmul(R_c, R_prev)[2][2] * self.z_1[index - 1]) / dt
                        velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    # position[2] = self.z_1[index]
                    position = position + velocity * dt
                    position_z = self.z_1[index]
                    # position[2] = position[2] + (position_z - position[2]) * self.complementary_filter_gain_z_measure

                    if position[2] > height:
                        height = position[2]
                index += 1
        else:
            for t in self.time_course_2:
                R = Rotation.from_quat(self.quat_2[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_2[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
                    R_prev = Rotation.from_quat(self.quat_leg_2[index - 1]).as_matrix()
                    R_now = Rotation.from_quat(self.quat_leg_2[index]).as_matrix()
                    if self.z_2[index] == self.tof2com_offset or self.z_2[index - 1] == self.tof2com_offset:
                        pass
                    else:
                        velocity_z = (np.matmul(R_c, R_now)[2][2] * self.z_2[index] - np.matmul(R_c, R_prev)[2][2] *self.z_2[index - 1]) / dt
                        velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    # position[2] = self.z_2[index]
                    position = position + velocity * dt
                    position_z = self.z_2[index]
                    # position[2] = position[2] + (position_z - position[2]) * self.complementary_filter_gain_z_measure

                    if position[2] > height:
                        height = position[2]
                index += 1
        self.current_position = position
        self.current_velocity = velocity

    def current_state(self):
        # run this after self.update()
        if self.RD_aerial_start_flag.step(self.aerial_start_flag):  # takeoff timestamp
            self.current_position = self.landing_location_prediction
            self.current_velocity = self.takeoff_velocity_prediction

        elif self.aerial_start_flag:  # after takeoff
            if self.buffer_flag:
                dt = self.time_course_1[-1] - self.time_course_1[-2]  #xxxxxxx

                R = Rotation.from_quat(self.quat_1[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_1[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * dt

                # velocity_z = (self.z_1[-1] - self.z_1[-2]) / dt
                R_prev = Rotation.from_quat(self.quat_leg_1[-2]).as_matrix()
                R_now = Rotation.from_quat(self.quat_leg_1[-1]).as_matrix()
                if self.z_1[-1] == self.tof2com_offset or self.z_1[-2] == self.tof2com_offset:
                    pass
                else:
                    velocity_z = (R_now[2][2] * self.z_1[-1] - R_prev[2][2] * self.z_1[-2]) / dt
                    self.current_velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * self.current_velocity[2]
                # self.current_position[2] = self.z_1[-1]
                self.current_position = self.current_position + self.current_velocity * dt
                position_z = self.z_1[-1]
                # self.current_position[2] = self.current_position[2] + (position_z - self.current_position[2]) * self.complementary_filter_gain_z_measure

            else:
                dt = self.time_course_2[-1] - self.time_course_2[-2]  #xxxxxxxx

                R = Rotation.from_quat(self.quat_2[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_2[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * dt

                # velocity_z = (self.z_2[-1] - self.z_2[-2]) / dt
                R_prev = Rotation.from_quat(self.quat_leg_2[-2]).as_matrix()
                R_now = Rotation.from_quat(self.quat_leg_2[-1]).as_matrix()
                if self.z_2[-1] == self.tof2com_offset or self.z_2[-2] == self.tof2com_offset:
                    pass
                else:
                    velocity_z = (R_now[2][2] * self.z_2[-1] - R_prev[2][2] * self.z_2[-2]) / dt
                    self.current_velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * self.current_velocity[2]
                # self.current_position[2] = self.z_2[-1]
                self.current_position = self.current_position + self.current_velocity * dt
                position_z = self.z_2[-1]
                # self.current_position[2] = self.current_position[2] + (position_z - self.current_position[2]) * self.complementary_filter_gain_z_measure

        else:  # stance phase
            self.current_velocity = np.array([0, 0, 0])

    def _compute_velocities(self, z_b_LD, z_b_TO):
        e_psi = np.cross(z_b_LD, z_b_TO)
        norm_e_psi = np.linalg.norm(e_psi)
        if norm_e_psi == 0:
            e_psi = np.array([1, 0, 0])
        else:
            e_psi = e_psi / norm_e_psi
        delta_psi = angle_between_vectors(z_b_LD, z_b_TO)
        theta_LD = delta_psi / (self.k1 - 1)
        theta_TO = theta_LD * self.k2 - delta_psi - theta_LD

        v_TO_unit = np.matmul(Rotation.from_rotvec(e_psi*(delta_psi+theta_TO)).as_matrix(), z_b_LD)
        v_LD_unit = - np.matmul(Rotation.from_rotvec(e_psi*(- theta_LD)).as_matrix(), z_b_LD)

        return v_TO_unit, v_LD_unit


class JumpingStateEstimator2TOF:
    # using threading to reduce the computational time
    def __init__(self, k1, k2, leg_efficiency, g, direction_gain, altitude_gain, attitude_correction_gain,
                 landing_speed_old=4, max_iteration_count=50, acc_bias=np.array([0.1, 0.045, 0]),
                 direction_error_limit=1, altitude_error_limit=0.04, complementary_filter_gain_z=0.1, ):
        self.direction_error_limit = direction_error_limit
        self.altitude_error_limit = altitude_error_limit
        # stance model parameters
        self.iteration_count = 0
        self.max_iteration_count_first_run = 3
        self.max_iteration_count = max_iteration_count

        self.attitude_correction_gain = attitude_correction_gain
        self.k1 = k1
        self.k2 = k2
        self.leg_efficiency = leg_efficiency

        # constants
        self.g = g
        self.e_3 = np.array([0, 0, 1])

        # solver parameter
        self.direction_gain = direction_gain
        self.altitude_gain = altitude_gain

        self.landing_speed_old = landing_speed_old

        self.aerial_start_flag = False
        self.RD_aerial_start_flag = RiseDetect()

        self.buffer_flag = True
        # buffer set 1
        self.time_course_1 = [0]
        self.acc_body_1 = [np.array([0, 0, 0])]
        self.quat_1 = [np.array([0, 0, 0, 1])]
        self.z_1 = [0]
        # buffer set 2
        self.time_course_2 = [0]
        self.acc_body_2 = [np.array([0, 0, 0])]
        self.quat_2 = [np.array([0, 0, 0, 1])]
        self.z_2 = [0]

        self.complementary_filter_gain_z_measure = complementary_filter_gain_z
        self.complementary_filter_gain_z_acc_measure = 1 - self.complementary_filter_gain_z_measure

        self.R_TO = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.R_TO_old = self.R_TO
        self.R_LD = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.R_LD_old = self.R_LD

        self.correction_matrix = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.altitude_error = 0
        self.directional_error = 0

        # variables inside the loop
        self.R_c_ini = Rotation.from_quat(self.quat_1[0]).as_matrix()
        self.landing_speed_prediction = 3

        self.landing_speed_old_prediction = 3
        self.landing_speed_old_original = 3

        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.takeoff_velocity_original = np.array([0, 0, 3])
        self.landing_velocity_prediction = np.array([0, 0, -3])
        self.landing_location_prediction = np.array([0, 0, 0])

        self.quat_corr = np.array([0, 0, 0, 1])

        self.v_TO_unit = np.array([0, 0, 1])

        self.time_cost = 0
        self.height = 0.458

        self.current_velocity = np.array([0, 0, 0])
        self.current_position = np.array([0, 0, 0])

        self.acc_bias = acc_bias

        self.position_temp = np.array([0, 0, 0])
        self.velocity_temp = np.array([0, 0, -4])
        self.landing_speed_old_temp = 3
        self.max_iteration_count_separate = 5
        self.stop_separate_computation_flag = False
        self.separate_computation_start = True
        self.v_TO_unit_temp = np.array([0, 0, 1])

        self.takeoff_velocity_old = np.array([0, 0, 1])

        self.lock = threading.Lock()  # 创建一个lock对象
        self.iteration_done = False
        self.quat_corr_ready = False
        self.start_iteration = False
        self.landing_location_old_worker = np.array([0, 0, 1])
        self.buffer_flag_worker = False
        self._estimator_thread = threading.Thread(target=self._worker, args=(), )
        self._estimator_thread_stop = False
        time.sleep(0.01)
        self._estimator_thread.start()


        self.logging_list = ['JSE_time_cost',
                             'JSE_v_TO_x_predicted', 'JSE_v_TO_y_predicted', 'JSE_v_TO_z_predicted',
                             'JSE_v_LD_x_predicted', 'JSE_v_LD_y_predicted', 'JSE_v_LD_z_predicted',
                             'JSE_v_x', 'JSE_v_y', 'JSE_v_z',
                             'JSE_p_x', 'JSE_p_y', 'JSE_p_z',
                             'JSE_acc_x', 'JSE_acc_y', 'JSE_acc_z',
                             'JSE_start_iteration', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def _worker(self):
        print('JSE: estimator thread start')
        while not self._estimator_thread_stop:
            if self.start_iteration:
                self._iteration_worker()
                self.start_iteration = False

            time.sleep(0.005)

    def stop(self):
        self._estimator_thread_stop = True
        time.sleep(2)
        self._estimator_thread.join()

    def init(self):
        pass

    def update(self, t, acc, quat, estimator_state, z):
        acc = acc - self.acc_bias
        start_time = time.time()

        if estimator_state == 21:
            self._takeoff(quat, self.buffer_flag)

            # change target buffer set
            self.buffer_flag = not self.buffer_flag

        self.lock.acquire()
        if self.aerial_start_flag:
            # append
            if self.buffer_flag:
                self.time_course_1.append(t)
                self.acc_body_1.append(acc)
                self.quat_1.append(quat)
                self.z_1.append(z)
            else:
                self.time_course_2.append(t)
                self.acc_body_2.append(acc)
                self.quat_2.append(quat)
                self.z_2.append(z)
        else:
            # first time (this is the takeoff timestamp)
            self.aerial_start_flag = True

            # clean and init the arrays
            if self.buffer_flag:
                self.time_course_1 = [t]
                self.acc_body_1 = [acc]
                self.quat_1 = [quat]
                self.z_1 = [z]
            else:
                self.time_course_2 = [t]
                self.acc_body_2 = [acc]
                self.quat_2 = [quat]
                self.z_2 = [z]
        self.lock.release()

        if estimator_state == 12:
            self._landing(quat)
            self.aerial_start_flag = False

        if self.iteration_done:
            self.iteration_done = False
            self.update_current_state()
        else:
            self.current_state()

        self.time_cost = time.time() - start_time
        self.logging_data = [self.time_cost,
                             self.takeoff_velocity_prediction[0], self.takeoff_velocity_prediction[1],
                             self.takeoff_velocity_prediction[2],
                             self.landing_velocity_prediction[0], self.landing_velocity_prediction[1],
                             self.landing_velocity_prediction[2],
                             self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             acc[0], acc[1], acc[2],
                             self.start_iteration, ]

    def _landing(self, quat):
        # quat = np.array([0, 0, 0, 1])
        self.R_LD_old = self.R_LD
        self.R_LD = Rotation.from_quat(quat).as_matrix()

    def _takeoff(self, quat, buffer_flag):
        self.iteration_count = 0
        # update the takeoff attitude
        self.R_TO_old = self.R_TO
        self.R_TO = Rotation.from_quat(quat).as_matrix()

        position = np.array([0, 0, 0])
        velocity = np.array([0, 0, -4])
        v_TO_unit = np.array([0, 0, 1])
        takeoff_velocity_old = np.array([0, 0, 1])
        directional_error = 10
        altitude_error = 10
        height = 0

        landing_location_old = self.landing_location_prediction


        R_c = self.R_c_ini
        landing_speed_old = self.landing_speed_prediction
        self.landing_speed_old_original = landing_speed_old

        while directional_error > self.direction_error_limit or abs(altitude_error) > self.altitude_error_limit:
            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old = self._prediction_error(R_c, landing_speed_old, landing_location_old, buffer_flag)
            temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
            R_c = np.matmul(Rotation.from_rotvec(temp_axis * - self.direction_gain).as_matrix(), R_c)
            landing_speed_old = landing_speed_old - altitude_error * self.altitude_gain

            if self.iteration_count == 0:
                landing_speed_original = np.linalg.norm(velocity)
                takeoff_speed_original = math.sqrt(self.leg_efficiency * landing_speed_original * landing_speed_original)
                self.takeoff_velocity_original = v_TO_unit * takeoff_speed_original
            self.iteration_count += 1
            if self.iteration_count >= self.max_iteration_count_first_run:
                break

        self.takeoff_velocity_old = takeoff_velocity_old
        self.height = height
        self.landing_speed_old_prediction = landing_speed_old
        self.landing_location_prediction = position
        self.landing_speed_prediction = np.linalg.norm(velocity)
        self.landing_velocity_prediction = velocity
        self.takeoff_speed_prediction = math.sqrt(self.leg_efficiency * self.landing_speed_prediction * self.landing_speed_prediction)
        self.takeoff_velocity_prediction = v_TO_unit * self.takeoff_speed_prediction
        self.correction_matrix = R_c
        self.altitude_error = altitude_error
        self.directional_error = directional_error

        # self.quat_corr = Rotation.from_rotvec(
        #     Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(canonical=False)

        # run the worker
        self.landing_location_old_worker = landing_location_old
        self.buffer_flag_worker = buffer_flag
        self.start_iteration = True
        print('xxx', self.altitude_error, self.directional_error)

    def _iteration_worker(self, ):
        buffer_flag = self.buffer_flag_worker
        landing_location_old = self.landing_location_old_worker

        directional_error = self.directional_error
        altitude_error = self.altitude_error
        R_c = self.correction_matrix
        landing_speed_old = self.landing_speed_old_prediction
        while True:
            time.sleep(0.001)
            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old = self._prediction_error(R_c, landing_speed_old, landing_location_old, buffer_flag)
            temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
            R_c = np.matmul(Rotation.from_rotvec(temp_axis * - self.direction_gain).as_matrix(), R_c)
            landing_speed_old = landing_speed_old - altitude_error * self.altitude_gain
            self.iteration_count += 1
            if (self.iteration_count > self.max_iteration_count) or (directional_error < self.direction_error_limit and abs(altitude_error) < self.altitude_error_limit):
                break
        self.takeoff_velocity_old = takeoff_velocity_old
        self.height = height
        self.landing_speed_old_prediction = landing_speed_old
        self.landing_location_prediction = position
        self.landing_speed_prediction = np.linalg.norm(velocity)
        self.landing_velocity_prediction = velocity
        self.takeoff_speed_prediction = math.sqrt(
            self.leg_efficiency * self.landing_speed_prediction * self.landing_speed_prediction)
        self.takeoff_velocity_prediction = v_TO_unit * self.takeoff_speed_prediction
        self.correction_matrix = R_c
        self.altitude_error = altitude_error
        self.directional_error = directional_error

        self.quat_corr = Rotation.from_rotvec(Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(canonical=False)
        self.quat_corr_ready = True
        self.iteration_done = True
        print(self.iteration_count)
        # print(directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old, landing_speed_old)
        print('altitude error: ', self.altitude_error, 'directional error: ', self.directional_error)

    def get_quat_corr(self):
        self.quat_corr_ready = False
        return self.quat_corr

    def _prediction_error(self, R_c, landing_speed_old, landing_location_old, buffer_flag):
        # landing_location_old = np.array([0, 0, 0])

        z_b_LD_old = np.matmul(R_c, np.matmul(self.R_LD_old, self.e_3))
        z_b_TO_old = np.matmul(R_c, np.matmul(self.R_TO_old, self.e_3))
        z_b_LD = np.matmul(R_c, np.matmul(self.R_LD, self.e_3))
        z_b_TO = np.matmul(R_c, np.matmul(self.R_TO, self.e_3))
        takeoff_speed_old = math.sqrt(self.leg_efficiency * landing_speed_old * landing_speed_old)
        v_TO_unit_old, v_LD_unit_old = self._compute_velocities(z_b_LD_old, z_b_TO_old)
        v_TO_unit, v_LD_unit = self._compute_velocities(z_b_LD, z_b_TO)
        takeoff_velocity_old = v_TO_unit_old * takeoff_speed_old

        # initial condition
        position = landing_location_old
        velocity = takeoff_velocity_old
        height = 0
        index = 0

        self.lock.acquire()
        if buffer_flag:
            for t in self.time_course_1:

                R = Rotation.from_quat(self.quat_1[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_1[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_1[index] - self.z_1[index-1]) / dt
                    R_prev = Rotation.from_quat(self.quat_1[index - 1]).as_matrix()
                    velocity_z = (np.matmul(R_c, R)[2][2] * self.z_1[index] - np.matmul(R_c, R_prev)[2][2] * self.z_1[index - 1]) / dt
                    velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    position = position + velocity * dt

                    if position[2] > height:
                        height = position[2]
                index += 1
        else:
            for t in self.time_course_2:

                R = Rotation.from_quat(self.quat_2[index]).as_matrix()

                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_2[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
                    R_prev = Rotation.from_quat(self.quat_2[index - 1]).as_matrix()
                    velocity_z = (np.matmul(R_c, R)[2][2] * self.z_2[index] - np.matmul(R_c, R_prev)[2][2] * self.z_2[index - 1]) / dt
                    velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    position = position + velocity * dt

                    if position[2] > height:
                        height = position[2]
                index += 1

        self.lock.release()

        landing_velocity_predicted = velocity
        v_LD_unit_predicted = landing_velocity_predicted / np.linalg.norm(landing_velocity_predicted)

        altitude_error = position[2]
        directional_error = angle_between_vectors(v_LD_unit_predicted, v_LD_unit) * 180 / math.pi

        return directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height, takeoff_velocity_old

    def update_current_state(self):

        position = self.landing_location_prediction
        velocity = self.takeoff_velocity_prediction
        R_c = self.correction_matrix
        height = 0
        index = 0
        if not self.buffer_flag_worker:  # switch to another buffer set
            for t in self.time_course_1:
                R = Rotation.from_quat(self.quat_1[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_1[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_1[index] - self.z_1[index - 1]) / dt
                    R_prev = Rotation.from_quat(self.quat_1[index - 1]).as_matrix()
                    velocity_z = (R[2][2] * self.z_1[-1] - R_prev[2][2] * self.z_1[-2]) / dt
                    velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    position = position + velocity * dt

                    if position[2] > height:
                        height = position[2]
                index += 1
        else:
            for t in self.time_course_2:
                R = Rotation.from_quat(self.quat_2[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    dt = t - self.time_course_2[index - 1]

                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    # velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
                    R_prev = Rotation.from_quat(self.quat_2[index - 1]).as_matrix()
                    velocity_z = (R[2][2] * self.z_2[-1] - R_prev[2][2] * self.z_2[-2]) / dt
                    velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    position = position + velocity * dt

                    if position[2] > height:
                        height = position[2]
                index += 1
        self.current_position = position
        self.current_velocity = velocity

    def current_state(self):
        # run this after self.update()
        if self.RD_aerial_start_flag.step(self.aerial_start_flag):  # takeoff timestamp
            self.current_position = self.landing_location_prediction
            self.current_velocity = self.takeoff_velocity_prediction
        elif self.aerial_start_flag:  # after takeoff
            if self.buffer_flag:
                dt = self.time_course_1[-1] - self.time_course_1[-2]  #xxxxxxx

                R = Rotation.from_quat(self.quat_1[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_1[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * dt

                # velocity_z = (self.z_1[-1] - self.z_1[-2]) / dt
                R_prev = Rotation.from_quat(self.quat_1[-2]).as_matrix()
                velocity_z = (R[2][2] * self.z_1[-1] - R_prev[2][2] * self.z_1[-2]) / dt
                self.current_velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * self.current_velocity[2]
                self.current_position = self.current_position + self.current_velocity * dt

            else:
                dt = self.time_course_2[-1] - self.time_course_2[-2]  #xxxxxxxx

                R = Rotation.from_quat(self.quat_2[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_2[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * dt

                # velocity_z = (self.z_2[-1] - self.z_2[-2]) / dt
                R_prev = Rotation.from_quat(self.quat_2[-2]).as_matrix()
                velocity_z = (R[2][2] * self.z_2[-1] - R_prev[2][2] * self.z_2[-2]) / dt
                self.current_velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * self.current_velocity[2]
                self.current_position = self.current_position + self.current_velocity * dt

        else:  # stance phase
            self.current_velocity = np.array([0, 0, 0])

    def _compute_velocities(self, z_b_LD, z_b_TO):
        e_psi = np.cross(z_b_LD, z_b_TO)
        norm_e_psi = np.linalg.norm(e_psi)
        if norm_e_psi == 0:
            e_psi = np.array([1, 0, 0])
        else:
            e_psi = e_psi / norm_e_psi
        delta_psi = angle_between_vectors(z_b_LD, z_b_TO)
        theta_LD = delta_psi / (self.k1 - 1)
        theta_TO = theta_LD * self.k2 - delta_psi - theta_LD

        v_TO_unit = np.matmul(Rotation.from_rotvec(e_psi*(delta_psi+theta_TO)).as_matrix(), z_b_LD)
        v_LD_unit = - np.matmul(Rotation.from_rotvec(e_psi*(- theta_LD)).as_matrix(), z_b_LD)

        return v_TO_unit, v_LD_unit


def angle_between_vectors(u, v):
    angle = math.atan2(np.linalg.norm(np.cross(u, v)), (u * v).sum())
    return angle


class JumpingHeightController:
    def __init__(self, leg_efficiency=0.8, g=9.81, t_p_low=0.04, t_p_high=0.3, thrust_gain=1):
        self.thrust_gain = thrust_gain
        self.t_p_high = t_p_high
        self.t_p_low = t_p_low
        self.g = g
        self.leg_efficiency = leg_efficiency
        pass

    def step(self, h, desired_h, flight_time):

        unpowered_h = self.leg_efficiency * h

        take_off_speed = math.sqrt(2 * self.g * h * self.leg_efficiency)

        if desired_h < h:
            powered_climbing_time = self.t_p_low
        else:
            if take_off_speed * self.thrust_gain == 0:
                powered_climbing_time = self.t_p_low
            else:
                powered_climbing_time = (desired_h - h)/(take_off_speed * self.thrust_gain)
            if powered_climbing_time > self.t_p_high:
                powered_climbing_time = self.t_p_high
            if powered_climbing_time < self.t_p_low:
                powered_climbing_time = self.t_p_low
            if powered_climbing_time > flight_time * 0.4:
                powered_climbing_time = flight_time * 0.4

        # print(take_off_speed, h)
        return powered_climbing_time


def landing_state_prediction_old(position, velocity):
    if velocity[2] > 0:
        t1 = velocity[2] / 9.81
        if position[2] > 0:
            h = position[2] + 0.5 * t1 * velocity[2]
        else:
            h = 0.5 * t1 * velocity[2]
        t2 = math.sqrt(2 * h / 9.81)
        v_z_ld = - math.sqrt(2 * 9.81 * h)
        flight_time = t1 + t2
    else:
        flight_time = (velocity[2] + math.sqrt(
            velocity[2] * velocity[2] + 2 * 9.81 * position[
                2])) / 9.81
        v_z_ld = velocity[2] - 9.81 * flight_time
    landing_x = flight_time * velocity[0] + position[0]
    landing_y = flight_time * velocity[1] + position[1]

    return v_z_ld, landing_x, landing_y


def landing_state_prediction(position, velocity):
    if position[2] <= 0 and velocity[2] < 0:
        landing_x, landing_y = position[0], position[1]
        v_z_ld = velocity[2]

    else:
        # print(velocity[2])
        if position[2] < 0:
            z = 0
        else:
            z = position[2]
        flight_time = (velocity[2] + math.sqrt(velocity[2] * velocity[2] + 2 * 9.81 * z)) / 9.81
        v_z_ld = velocity[2] - 9.81 * flight_time
        landing_x = flight_time * velocity[0] + position[0]
        landing_y = flight_time * velocity[1] + position[1]

    return v_z_ld, landing_x, landing_y


def landing_state_prediction_2(position, velocity):
    if position[2] - 0.28 <= 0:
        landing_x, landing_y = position[0], position[1]
        v_z_ld = velocity[2]
        flight_time = 0
    else:
        # print(velocity[2])
        #flight_time = (velocity[2] + math.sqrt(velocity[2] * velocity[2] + 2 * 9.81 * (position[2]))) / 9.81
        flight_time = (velocity[2] + math.sqrt(velocity[2] * velocity[2] + 2 * 9.81 * (position[2]-0.28))) / 9.81
        v_z_ld = velocity[2] - 9.81 * flight_time
        landing_x = flight_time * velocity[0] + position[0]
        landing_y = flight_time * velocity[1] + position[1]

    return v_z_ld, landing_x, landing_y, flight_time


class PidControl:
    def __init__(self, kp, kd, constant):
        self.kp = kp
        self.kd = kd
        self.constant = constant

    def update(self, desired_x, desired_x_dot,  x, x_dot, ):
        u_x = self.kp * (desired_x - x) + self.kd * (desired_x_dot - x_dot) + self.constant
        return u_x


def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x


class FlightController:
    def __init__(self, kpx, kdx, kpy, kdy, 
                 kpz, kdz, cz, kpyaw, attitude_saturation):
        self.takeoff_timestamp = -1
        self.pitch_bias_flight = 0
        self.roll_bias_flight = 0
        self.kpyaw = kpyaw

        self.attitude_saturation = attitude_saturation

        self.PID_X = PidControl(kpx, kdx, 0)
        self.PID_Y = PidControl(kpy, kdy, 0)
        self.PID_Z = PidControl(kpz, kdz, cz)

        self._const_2pi = 2 * math.pi

        self.roll_flight = 0
        self.pitch_flight = 0
        self.yaw_flight = 0
        self.thrust_flight = 0

        self.logging_list = ['FC_roll_flight', 'FC_pitch_flight', 'FC_yaw_flight', 'FC_thrust_flight', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def update(self, desired_x, desired_y, desired_z, desired_x_dot, desired_y_dot, desired_z_dot, desired_yaw, x, y, z, x_dot, y_dot, z_dot, angle_yaw):
        u_x = self.PID_X.update(desired_x, desired_x_dot, x, x_dot, )
        u_y = self.PID_Y.update(desired_y, desired_y_dot, y, y_dot, )
        u_z = self.PID_Z.update(desired_z, desired_z_dot, z, z_dot, )

        pitch_flight = (u_x * math.cos(angle_yaw) + u_y * math.sin(angle_yaw))
        roll_flight =  - (u_y * math.cos(angle_yaw) - u_x * math.sin(angle_yaw))

        self.pitch_flight = saturation(pitch_flight, 20, -20) + self.pitch_bias_flight
        self.roll_flight = saturation(roll_flight, 20, -20) + self.roll_bias_flight
        self.thrust_flight = round(saturation(u_z, 65000, 3000))

        self.yaw_flight = (desired_yaw - angle_yaw) * 180 / math.pi *0.1

        self.logging_data = [self.roll_flight, self.pitch_flight, self.yaw_flight, self.thrust_flight, ]


class BiController:
    def __init__(self,
                 r13_r23_kp=-1000.0,
                 xy_cmd_limit=200.0,
                 thrust_base=26000,
                 thrust_min=3000,
                 thrust_max=56000,
                 z_kp=15000.0,
                 z_ki=3000.0,
                 z_kd=2000.0,
                 z_int_limit=1000.0,
                 xy_kp=0.2,
                 xy_kd=0.1):
        # R13/R23 projection controller parameters
        self.r13_r23_kp = r13_r23_kp
        self.xy_cmd_limit = xy_cmd_limit

        # Height PID parameters
        self.thrust_base = thrust_base
        self.thrust_min = thrust_min
        self.thrust_max = thrust_max
        self.z_kp = z_kp
        self.z_ki = z_ki
        self.z_kd = z_kd
        self.z_int_limit = z_int_limit
        self.xy_kp = xy_kp
        self.xy_kd = xy_kd

        # Internal states
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0

        self.R13_error = 0.0
        self.R23_error = 0.0
        self.z_error = 0.0
        self.z_error_d = 0.0
        self.z_error_i = 0.0

        # Outputs, same style as FlightController
        self.roll_flight = 0.0
        self.pitch_flight = 0.0
        self.yaw_flight = 0.0
        self.thrust_flight = 0

        self.logging_list = [
            'BI_desired_x', 'BI_desired_y', 'BI_desired_z',
            'BI_R13_error', 'BI_R23_error',
            'BI_z_error', 'BI_z_error_d', 'BI_z_error_i',
            'BI_roll_flight', 'BI_pitch_flight', 'BI_yaw_flight', 'BI_thrust_flight',
        ]
        self.logging_data = [0.0] * len(self.logging_list)

    def init(self, desired_z=0.0):
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0
        self.desired_r13 = 0.0
        self.desired_r23 = 0.0

        self.R13_error = 0.0
        self.R23_error = 0.0
        self.z_error = 0.0
        self.z_error_d = 0.0
        self.z_error_i = 0.0
        self.z_error_prev = 0.0

        self.roll_flight = 0.0
        self.pitch_flight = 0.0
        self.yaw_flight = 0.0
        self.thrust_flight = 0

        self.logging_data = [0.0] * len(self.logging_list)

    def update(self, desired_x, desired_y, desired_z, R13, R23, x, y, z, x_dot, y_dot, z_dot, yaw_deg, dt):
        # Left stick maps to desired R13/R23 references

        self.desired_r13 = self.xy_kp * (desired_x - x) - self.xy_kd * x_dot
        self.desired_r23 = self.xy_kp * (desired_y - y) - self.xy_kd * y_dot

        self.desired_z = desired_z
        # R13/R23 first-order proportional control
        self.R13_error = self.desired_r13 - R13
        self.R23_error = self.desired_r23 - R23

        self.roll_flight = -saturation(
            self.r13_r23_kp * self.R13_error,
            self.xy_cmd_limit,
            -self.xy_cmd_limit,
        )
        self.pitch_flight = -saturation(
            self.r13_r23_kp * self.R23_error,
            self.xy_cmd_limit,
            -self.xy_cmd_limit,
        )

        self.z_error = self.desired_z - z

        self.z_error_i = saturation(
            self.z_error_i * dt, 
            self.z_int_limit,
            -self.z_int_limit,
        )

        # Do not calculate z_dot from error difference.
        # z_dot is directly passed in.
        self.z_error_d = - z_dot

        thrust_pid = (
            self.thrust_base
            + self.z_kp * self.z_error
            + self.z_ki * self.z_error_i
            + self.z_kd * self.z_error_d
        )

        self.thrust_flight = round(
            saturation(thrust_pid, self.thrust_max, self.thrust_min)
        )

        # yaw channel carries mocap yaw angle to firmware BI mode
        self.yaw_flight = yaw_deg

        self.logging_data = [
            self.desired_x, self.desired_y, self.desired_z,
            self.R13_error, self.R23_error,
            self.z_error, self.z_error_d, self.z_error_i,
            self.roll_flight, self.pitch_flight, self.yaw_flight, self.thrust_flight,
        ]

class JoyBiController:
    def __init__(self,
                 r13_r23_kp=-400.0,
                 xy_cmd_limit=120.0,
                 thrust_base=26000,
                 thrust_min=3000,
                 thrust_max=56000,
                 z_kp=3000.0,
                 z_ki=0.0,
                 z_kd=55000.0,
                 z_int_limit=1.0,
                 joy_r13_gain=0.20,
                 joy_r23_gain=0.20,
                 joy_deadzone=0.05,
                 desired_r_limit=0.25):
        """
        Joystick-based bicopter controller.

        Difference from BiController:
            BiController:     x/y position error -> desired_r13/desired_r23 -> cmd
            JoyBiController:  joystick input      -> desired_r13/desired_r23 -> cmd

        Other parts are kept the same style as BiController:
            - R13/R23 proportional controller
            - height PID
            - output fields: roll_flight, pitch_flight, yaw_flight, thrust_flight

        Joystick convention:
            joy_r13 in [-1, 1] -> desired_r13
            joy_r23 in [-1, 1] -> desired_r23

        Command convention follows BiController:
            roll_flight  controls R13 error
            pitch_flight controls R23 error
        """

        # R13/R23 projection controller parameters
        self.r13_r23_kp = r13_r23_kp
        self.xy_cmd_limit = xy_cmd_limit

        # Height PID parameters
        self.thrust_base = thrust_base
        self.thrust_min = thrust_min
        self.thrust_max = thrust_max
        self.z_kp = z_kp
        self.z_ki = z_ki
        self.z_kd = z_kd
        self.z_int_limit = z_int_limit

        # Joystick mapping parameters
        self.joy_r13_gain = joy_r13_gain
        self.joy_r23_gain = joy_r23_gain
        self.joy_deadzone = joy_deadzone
        self.desired_r_limit = desired_r_limit

        # Internal states
        self.desired_r13 = 0.0
        self.desired_r23 = 0.0
        self.desired_z = 0.0

        self.R13_error = 0.0
        self.R23_error = 0.0
        self.z_error = 0.0
        self.z_error_d = 0.0
        self.z_error_i = 0.0

        # Outputs, same style as BiController
        self.roll_flight = 0.0
        self.pitch_flight = 0.0
        self.yaw_flight = 0.0
        self.thrust_flight = 0

        self.logging_list = [
            'JBI_desired_r13', 'JBI_desired_r23', 'JBI_desired_z',
            'JBI_R13_error', 'JBI_R23_error',
            'JBI_z_error', 'JBI_z_error_d', 'JBI_z_error_i',
            'JBI_roll_flight', 'JBI_pitch_flight', 'JBI_yaw_flight', 'JBI_thrust_flight',
        ]
        self.logging_data = [0.0] * len(self.logging_list)

    def init(self, desired_z=0.0):
        self.desired_r13 = 0.0
        self.desired_r23 = 0.0
        self.desired_z = desired_z

        self.R13_error = 0.0
        self.R23_error = 0.0
        self.z_error = 0.0
        self.z_error_d = 0.0
        self.z_error_i = 0.0

        self.roll_flight = 0.0
        self.pitch_flight = 0.0
        self.yaw_flight = 0.0
        self.thrust_flight = 0

        self.logging_data = [0.0] * len(self.logging_list)

    def _apply_deadzone(self, value):
        if abs(value) < self.joy_deadzone:
            return 0.0
        return value

    def update(self, joy_r13, joy_r23, desired_z,
               R13, R23,
               z, z_dot,
               yaw_deg, dt):
        if dt <= 1e-6:
            dt = 1e-6

        # Joystick -> desired R13/R23.
        # Inputs should be normalized to [-1, 1].
        joy_r13 = saturation(joy_r13, 1.0, -1.0)
        joy_r23 = saturation(joy_r23, 1.0, -1.0)

        joy_r13 = self._apply_deadzone(joy_r13)
        joy_r23 = self._apply_deadzone(joy_r23)

        self.desired_r13 = saturation(
            self.joy_r13_gain * joy_r13,
            self.desired_r_limit,
            -self.desired_r_limit,
        )

        self.desired_r23 = saturation(
            self.joy_r23_gain * joy_r23,
            self.desired_r_limit,
            -self.desired_r_limit,
        )

        self.desired_z = desired_z

        # R13/R23 proportional control, same structure as BiController.
        self.R13_error = self.desired_r13 - R13
        self.R23_error = self.desired_r23 - R23

        roll_cmd = -self.r13_r23_kp * self.R13_error
        pitch_cmd = -self.r13_r23_kp * self.R23_error

        self.roll_flight = saturation(
            roll_cmd,
            self.xy_cmd_limit,
            -self.xy_cmd_limit,
        )

        self.pitch_flight = saturation(
            pitch_cmd,
            self.xy_cmd_limit,
            -self.xy_cmd_limit,
        )

        # Keep yaw behavior same as BiController baseline.
        self.yaw_flight = yaw_deg

        # Height PID, same style as BiController.
        self.z_error = desired_z - z
        self.z_error_d = -z_dot

        self.z_error_i = saturation(
            self.z_error_i + self.z_error * dt,
            self.z_int_limit,
            -self.z_int_limit,
        )

        thrust_cmd = (
            self.thrust_base
            + self.z_kp * self.z_error
            + self.z_ki * self.z_error_i
            + self.z_kd * self.z_error_d
        )

        self.thrust_flight = round(
            saturation(thrust_cmd, self.thrust_max, self.thrust_min)
        )

        self.logging_data = [
            self.desired_r13, self.desired_r23, self.desired_z,
            self.R13_error, self.R23_error,
            self.z_error, self.z_error_d, self.z_error_i,
            self.roll_flight, self.pitch_flight, self.yaw_flight, self.thrust_flight,
        ]

# -------------------------------------------------------------------------
# FittedBiController class (inserted before RProjectionLmsEstimator)
# -------------------------------------------------------------------------
class FittedBiController:
    def __init__(self,
                 wn_xy=0.5,
                 cmd_limit=500.0,
                 thrust_base=26000,
                 thrust_min=3000,
                 thrust_max=56000,
                 z_kp=15000.0,
                 z_ki=3000.0,
                 z_kd=2000.0,
                 z_int_limit=1000.0,
                 g=9.81):
        """
        Bicopter horizontal controller using the fitted r13/r23 dynamics.

        State:
            X = [x, y, vx, vy, r13, r23, r13_dot, r23_dot]

        Input convention:
            tau_x = cmd_pitch
            tau_y = cmd_roll

        Fitted model:
            r13_ddot = -0.362275*y_dot - 1.649344*R13_dot + 2.848143*R23_dot
                        + 2.831332*cmd_roll
            r23_ddot =  0.362275*x_dot - 2.848143*R13_dot - 1.649344*R23_dot
                        + 2.845706*cmd_pitch

        The controller enforces the fourth-order horizontal error dynamics:
            e'''' + lambda3*e''' + lambda2*e'' + lambda1*e' + lambda0*e = 0

        For fixed-point tracking, desired_x and desired_y are constants, so
        all reference derivatives are zero.
        """
        self.g = g
        self.wn_xy = wn_xy
        self.cmd_limit = cmd_limit

        # Height PID parameters
        self.thrust_base = thrust_base
        self.thrust_min = thrust_min
        self.thrust_max = thrust_max
        self.z_kp = z_kp
        self.z_ki = z_ki
        self.z_kd = z_kd
        self.z_int_limit = z_int_limit

        # Fourth-order controller gains from (s + wn)^4
        self.lambda0 = self.wn_xy ** 4
        self.lambda1 = 4.0 * self.wn_xy ** 3
        self.lambda2 = 6.0 * self.wn_xy ** 2
        self.lambda3 = 4.0 * self.wn_xy

        # Fitted physically constrained attitude dynamics coefficients
        # r13_ddot = -kv*y_dot - d*R13_dot + gc*R23_dot + by*cmd_roll
        # r23_ddot =  kv*x_dot - gc*R13_dot - d*R23_dot + bx*cmd_pitch
        #self.kv = 0.562275
        #self.d_att = 1.649344
        #self.gc = 2.848143
        #self.by = 2.831332
        #self.bx = 2.845706

        self.kv = 0.271515
        self.d_att = 0.459532
        self.gc = 2.347229
        self.by = 2.434112
        self.bx = 0.34112

        # Internal states
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0

        self.r13_ddot_des = 0.0
        self.r23_ddot_des = 0.0
        self.f13 = 0.0
        self.f23 = 0.0
        self.cmd_roll_unsat = 0.0
        self.cmd_pitch_unsat = 0.0
        self.R13_error = 0.0
        self.R23_error = 0.0
        self.z_error = 0.0
        self.z_error_d = 0.0
        self.z_error_i = 0.0

        # Outputs, same style as FlightController and BiController
        self.roll_flight = 0.0     # cmd_roll
        self.pitch_flight = 0.0    # cmd_pitch
        self.yaw_flight = 0.0
        self.thrust_flight = 0

        self.logging_list = [
            'FBI_desired_x', 'FBI_desired_y', 'FBI_desired_z',
            'FBI_R13_error', 'FBI_R23_error',
            'FBI_r13_ddot_des', 'FBI_r23_ddot_des',
            'FBI_f13', 'FBI_f23',
            'FBI_cmd_roll_unsat', 'FBI_cmd_pitch_unsat',
            'FBI_z_error', 'FBI_z_error_d', 'FBI_z_error_i',
            'FBI_roll_flight', 'FBI_pitch_flight', 'FBI_yaw_flight', 'FBI_thrust_flight',
        ]
        self.logging_data = [0.0] * len(self.logging_list)

    def init(self, desired_z=0.0):
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = desired_z

        self.r13_ddot_des = 0.0
        self.r23_ddot_des = 0.0
        self.f13 = 0.0
        self.f23 = 0.0
        self.cmd_roll_unsat = 0.0
        self.cmd_pitch_unsat = 0.0
        self.R13_error = 0.0
        self.R23_error = 0.0
        self.z_error = 0.0
        self.z_error_d = 0.0
        self.z_error_i = 0.0

        self.roll_flight = 0.0
        self.pitch_flight = 0.0
        self.yaw_flight = 0.0
        self.thrust_flight = 0

        self.logging_data = [0.0] * len(self.logging_list)

    def update(self, desired_x, desired_y, desired_z,
               R13, R23, R13_dot, R23_dot,
               x, y, z, x_dot, y_dot, z_dot,
               yaw_deg, dt):
        if dt <= 1e-6:
            dt = 1e-6

        self.desired_x = desired_x
        self.desired_y = desired_y
        self.desired_z = desired_z

        # Position tracking errors for fixed-point control.
        ex = x - desired_x
        ey = y - desired_y

        # Near-hovering relations:
        #   x_ddot = g*R13,       y_ddot = g*R23
        #   x_3dot = g*R13_dot,   y_3dot = g*R23_dot
        # Enforce:
        #   e'''' + lambda3*e''' + lambda2*e'' + lambda1*e' + lambda0*e = 0
        self.r13_ddot_des = (
            - self.lambda3 * R13_dot
            - self.lambda2 * R13
            - (self.lambda1 / self.g) * x_dot
            - (self.lambda0 / self.g) * ex
        )
        self.r23_ddot_des = (
            - self.lambda3 * R23_dot
            - self.lambda2 * R23
            - (self.lambda1 / self.g) * y_dot
            - (self.lambda0 / self.g) * ey
        )

        # Fitted drift dynamics without command input.
        # r13_ddot = f13 + by*cmd_roll
        # r23_ddot = f23 + bx*cmd_pitch
        self.f13 = (
            - self.kv * y_dot
            - self.d_att * R13_dot
            + self.gc * R23_dot
        )
        self.f23 = (
            self.kv * x_dot
            - self.gc * R13_dot
            - self.d_att * R23_dot
        )

        # Direct inverse of the diagonal input mapping:
        #   cmd_roll  = (r13_ddot_des - f13) / by
        #   cmd_pitch = (r23_ddot_des - f23) / bx
        if abs(self.by) < 1e-9:
            cmd_roll = 0.0
        else:
            cmd_roll = (self.r13_ddot_des - self.f13) / self.by

        if abs(self.bx) < 1e-9:
            cmd_pitch = 0.0
        else:
            cmd_pitch = (self.r23_ddot_des - self.f23) / self.bx

        self.cmd_roll_unsat = cmd_roll
        self.cmd_pitch_unsat = cmd_pitch

        self.roll_flight = saturation(cmd_roll, self.cmd_limit, -self.cmd_limit)
        self.pitch_flight = saturation(cmd_pitch, self.cmd_limit, -self.cmd_limit)

        self.R13_error = self.r13_ddot_des - (self.f13 + self.by * self.roll_flight)
        self.R23_error = self.r23_ddot_des - (self.f23 + self.bx * self.pitch_flight)

        # Height PID, same output convention as BiController.
        self.z_error = self.desired_z - z
        self.z_error_i = saturation(
            self.z_error_i + self.z_error * dt,
            self.z_int_limit,
            -self.z_int_limit,
        )
        self.z_error_d = -z_dot

        thrust_pid = (
            self.thrust_base
            + self.z_kp * self.z_error
            + self.z_ki * self.z_error_i
            + self.z_kd * self.z_error_d
        )

        self.thrust_flight = round(
            saturation(thrust_pid, self.thrust_max, self.thrust_min)
        )

        # yaw channel carries mocap yaw angle to firmware BI mode.
        self.yaw_flight = yaw_deg

        self.logging_data = [
            self.desired_x, self.desired_y, self.desired_z,
            self.R13_error, self.R23_error,
            self.r13_ddot_des, self.r23_ddot_des,
            self.f13, self.f23,
            self.cmd_roll_unsat, self.cmd_pitch_unsat,
            self.z_error, self.z_error_d, self.z_error_i,
            self.roll_flight, self.pitch_flight, self.yaw_flight, self.thrust_flight,
        ]

class RProjectionLmsEstimator:
    def __init__(self, mu=0.3, alpha=0.3, yaw_rate_alpha=0.3):
        """
        Online LMS low-frequency estimator for R13/R23.

        Model:
            R13 = A13 + B13*sin(phase) + C13*cos(phase)
            R23 = A23 + B23*sin(phase) + C23*cos(phase)

        phase is integrated online from:
            f0 = yawrate_deg / 360
            phase_dot = 2*pi*f0

        Outputs:
            R13_filt, R23_filt are the smoothed low-frequency A terms.
        """
        self.mu = mu
        self.alpha = alpha
        self.yaw_rate_alpha = yaw_rate_alpha

        self.initialized = False

        self.phase = 0.0
        self.theta13 = np.zeros(3)
        self.theta23 = np.zeros(3)

        self.R13_low_raw = 0.0
        self.R23_low_raw = 0.0
        self.R13_filt = 0.0
        self.R23_filt = 0.0

        self.last_yaw_raw_deg = 0.0
        self.yaw_unwrapped_deg = 0.0
        self.yawrate_deg = 0.0
        self.yawrate_deg_filt = 0.0
        self.f0_yaw = 0.0

        self.logging_list = [
            'R13_lms_raw', 'R23_lms_raw',
            'R13_filt', 'R23_filt',
            'mocap_yawrate_deg', 'mocap_yawrate_deg_filt', 'f0_yaw',
        ]
        self.logging_data = [0.0] * len(self.logging_list)

    def reset(self):
        self.initialized = False
        self.phase = 0.0
        self.theta13[:] = 0.0
        self.theta23[:] = 0.0

        self.R13_low_raw = 0.0
        self.R23_low_raw = 0.0
        self.R13_filt = 0.0
        self.R23_filt = 0.0

        self.last_yaw_raw_deg = 0.0
        self.yaw_unwrapped_deg = 0.0
        self.yawrate_deg = 0.0
        self.yawrate_deg_filt = 0.0
        self.f0_yaw = 0.0

        self.logging_data = [0.0] * len(self.logging_list)

    def update(self, R13, R23, yaw_deg, dt):
        if dt <= 1e-6:
            dt = 1e-6

        if not self.initialized:
            self.last_yaw_raw_deg = yaw_deg
            self.yaw_unwrapped_deg = yaw_deg

            self.yawrate_deg = 0.0
            self.yawrate_deg_filt = 0.0
            self.f0_yaw = 0.0
            self.phase = 0.0

            self.theta13[:] = [R13, 0.0, 0.0]
            self.theta23[:] = [R23, 0.0, 0.0]

            self.R13_low_raw = R13
            self.R23_low_raw = R23
            self.R13_filt = R13
            self.R23_filt = R23

            self.initialized = True

        else:
            # unwrap yaw online using previous raw yaw
            yaw_delta = yaw_deg - self.last_yaw_raw_deg
            while yaw_delta > 180.0:
                yaw_delta -= 360.0
            while yaw_delta < -180.0:
                yaw_delta += 360.0

            self.yaw_unwrapped_deg += yaw_delta
            self.last_yaw_raw_deg = yaw_deg

            self.yawrate_deg = yaw_delta / dt

            self.yawrate_deg_filt = self.yawrate_deg_filt + self.yaw_rate_alpha * (
                self.yawrate_deg - self.yawrate_deg_filt
            )

            self.f0_yaw = self.yawrate_deg_filt / 360.0

            self.phase = self.phase + 2.0 * math.pi * self.f0_yaw * dt

            phi = np.array([
                1.0,
                math.sin(self.phase),
                math.cos(self.phase),
            ])

            phi_norm = float(np.dot(phi, phi) + 1e-6)

            R13_hat = float(np.dot(self.theta13, phi))
            R13_err = R13 - R13_hat
            self.theta13 = self.theta13 + self.mu * R13_err * phi / phi_norm

            R23_hat = float(np.dot(self.theta23, phi))
            R23_err = R23 - R23_hat
            self.theta23 = self.theta23 + self.mu * R23_err * phi / phi_norm

            self.R13_low_raw = float(self.theta13[0])
            self.R23_low_raw = float(self.theta23[0])

            self.R13_filt = self.R13_filt + self.alpha * (
                self.R13_low_raw - self.R13_filt
            )
            self.R23_filt = self.R23_filt + self.alpha * (
                self.R23_low_raw - self.R23_filt
            )

        self.logging_data = [
            self.R13_low_raw,
            self.R23_low_raw,
            self.R13_filt,
            self.R23_filt,
            self.yawrate_deg,
            self.yawrate_deg_filt,
            self.f0_yaw,
        ]

        return self.R13_filt, self.R23_filt
    
class JumpingHeightRecorder:
    def __init__(self, ):
        self.h_min = 0
        self.h_max = 0.4
        self.jumping_height = self.h_max - self.h_min
        self.RD = RiseDetect()

        self.logging_list = ['JHR_jumping_height', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def update(self, h, jumping_state):
        if self.RD.step(jumping_state == 2):
            self.jumping_height = self.h_max - self.h_min

        if jumping_state == 2:  # stance phase
            self.h_min = h
            self.h_max = h
        else:
            if h > self.h_max:
                self.h_max = h
        self.logging_data = [self.jumping_height, ]


def float_float_compress(a, b):
    return struct.unpack('f', struct.pack('ee',  a, b))[0]


def float_float_decompress(a):
    return struct.unpack('ee', struct.pack('f',  a))


class UdpRigidBodies3:
    def __init__(self, udp_ip="0.0.0.0", udp_port=22222):
        self.len_data = 100
        self.udp_flag = 0
        self._udpStop = False
        self._udp_data = None
        self._udp_data_time = time.time()
        self._udpThread = None
        self._udpThread_on = False
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind((self.udp_ip, self.udp_port))

        self.sample_rate = -1  # flag
        self.get_sample_rate()
        self.sample_time = 1 / self.sample_rate
        print('UDP receiver initialized')

    def get_sample_rate(self):
        if self.sample_rate == -1:
            print('Computing sample rate...')
            time_list = []
            for _ in tqdm(range(100), desc="Processing...", leave=True, position=0):
                time_list.append(time.time())
                udp_data_temp, _ = self._sock.recvfrom(100)  # buffer size is 8192 bytes
                self._udp_data_time = time.time()
                self.len_data = len(udp_data_temp)
            d_time = np.diff(time_list)
            self.sample_time = np.mean(d_time)

            print('Sample rate: ', '%.2f' % (1 / self.sample_time), 'Hz')
            print('UDP data size: ', '%.2f' % self.len_data)
            self.sample_rate = 1 / self.sample_time
            return self.sample_rate
        else:
            return self.sample_rate

    def start_thread(self):
        if not self._udpThread_on:
            self._udpThread = threading.Thread(target=self._udp_worker, args=(), )
            self._udp_data = b'1'
            self._udpThread.start()
            self._udpThread_on = True
            time.sleep(1)
            print('Upd thread start')
        else:
            print('New upd thread is not started')

    def _udp_worker(self, ):
        if not self._udpThread_on:
            while not self._udpStop:
                self.udp_flag = self.udp_flag + 1
                self._udp_data, _ = self._sock.recvfrom(self.len_data)  # buffer size is 8192 bytes
                self._udp_data_time = time.time()

    def stop_thread(self, ):
        self._udpStop = True
        time.sleep(self.sample_time)
        print('upd thread stopped')

    def get_data(self, ):
        return self._udp_data, self._udp_data_time

from Library.IIR2Filter import IIR2Filter
import struct
import math

def saturation_fcn(A, satA):
    if A > satA[1]:
        A = satA[1]
    if A < satA[0]:
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
    def __init__(self, sample_rate):
        self.flag = 0
        self.sample_time = 1 / sample_rate
        self.sample_rate = sample_rate

        self.X = 0
        self.Y = 0
        self.Z = 0
        self.QX = 0
        self.QY = 0
        self.QZ = 0
        self.QW = 1

        self._t_prev = None  # previous mocap timestamp (udp_time) for derivatives

                # ---------------- mocap logging (DataProcessor2 style) ----------------
        # 14B keys
        self.keys = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        # 20B keys (optional)
        self.keys20 = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz']

        # store latest mocap sample (single rigid body)
        self.data_list = {key: 0.0 for key in self.keys20}

        # DataSaver-friendly flat list
        self.logging_list = []
        self.logging_data = []

        # prefix with b{body_id}_ like DataProcessor2
        # (body_id is only for naming, RealTimeProcessor itself is per-body)
        self.body_id = 1

        for key in self.keys20:
            self.logging_list.append('b' + str(self.body_id) + '_' + key)
            self.logging_data.append(0.0)

    def step(self, udp_data, body_index: int = 1, abstime=None):
        self.body_id = body_index
        start = body_index * 14 - 14
        end = body_index * 14
        chunk = udp_data[start:end]
        if chunk is None or len(chunk) < 14:
        # no update if packet incomplete
            return
        self._step_chunk(chunk, abstime=None)


    def _step_chunk(self, udp_data, abstime=None):
    # exactly the same unpacking as DataProcessor2
        if len(udp_data) >= 20:
            x, y, z, qx, qy, qz, qw, vx, vy, vz = struct.unpack("hhhhhhhhhh", udp_data[:20])
        # 可选：把 vx/vy/vz 存下来，未来用
            self.VX = vx * 0.001
            self.VY = vy * 0.001
            self.VZ = vz * 0.001
        else:
            x, y, z, qx, qy, qz, qw = struct.unpack("hhhhhhh", udp_data[:14])
            self.VX = 0.0
            self.VY = 0.0
            self.VZ = 0.0

        self.X = x * 0.0005
        self.Y = y * 0.0005
        self.Z = z * 0.0005
        self.QX = float(qx * 0.001)
        self.QY = float(qy * 0.001)
        self.QZ = float(qz * 0.001)
        self.QW = float(qw * 0.001)
                # ---------------- update mocap logging buffers ----------------
        # write latest values into dict
        self.data_list['x']  = float(self.X)
        self.data_list['y']  = float(self.Y)
        self.data_list['z']  = float(self.Z)
        self.data_list['qx'] = float(self.QX)
        self.data_list['qy'] = float(self.QY)
        self.data_list['qz'] = float(self.QZ)
        self.data_list['qw'] = float(self.QW)
        self.data_list['vx'] = float(getattr(self, 'VX', 0.0))
        self.data_list['vy'] = float(getattr(self, 'VY', 0.0))
        self.data_list['vz'] = float(getattr(self, 'VZ', 0.0))

        # flatten into logging_data with same order as logging_list
        # (logging_list is b{body_id}_x ... b{body_id}_vz)
        # keys20 order must match constructor order
        for i, key in enumerate(self.keys20):
            self.logging_data[i] = self.data_list[key]

class DataProcessor2:
    def __init__(self, len_data):
        self.num_bodies = int(len_data/14)
        #self.num_bodies = int(len_data/20)
        print('Number of rigid bodies: {}'.format(self.num_bodies))
        body_name = [i for i in range(1, self.num_bodies + 1)]
        self.keys = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        #self.keys = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz']
        self.data_list = {name: {key: 0.0 for key in self.keys} for name in body_name}

        self.logging_list = []
        self.logging_data = []

        for body in body_name:
            for key in self.keys:
                self.logging_list.append('b' + str(body) + '_' + key)
                self.logging_data.append(0.0)

    def process_data(self, udp_data):
        for i in range(1, self.num_bodies + 1):
            x, y, z, qx, qy, qz, qw= struct.unpack("hhhhhhh", udp_data[(i*14 - 14):i*14])
            self.data_list[i]['x'] = x * 0.0005
            self.data_list[i]['y'] = y * 0.0005
            self.data_list[i]['z'] = z * 0.0005
            self.data_list[i]['qx'] = qx * 0.001
            self.data_list[i]['qy'] = qy * 0.001
            self.data_list[i]['qz'] = qz * 0.001
            self.data_list[i]['qw'] = qw * 0.001
            #self.data_list[i]['vx'] = vx * 0.001
            #self.data_list[i]['vy'] = vy * 0.001
            #self.data_list[i]['vz'] = vz * 0.001

        i = 0
        for body in self.data_list:
            for key in self.keys:
                self.logging_data[i] = self.data_list[body][key]
                i = i + 1
        return self.data_list
    
    


class ParameterSetQueue:
    def __init__(self, lc):
        self.lc = lc
        self.queue_parameter = []
        self.queue_value = []

        self.parameter = None
        self.value = None

    def enqueue(self, parameter, value):
        self.queue_parameter.append(parameter)
        self.queue_value.append(value)

    def _dequeue(self):
        if not self.queue_parameter:
            raise IndexError("Dequeue from an empty queue.")

        self.parameter = self.queue_parameter.pop(0)
        self.value = self.queue_value.pop(0)

    def _is_empty(self):
        return len(self.queue_parameter) == 0

    def auto_set(self):
        if not self._is_empty():
            self._dequeue()
            self.lc.cf.param.set_value(self.parameter, self.value)
            print('PSQ: set ' + self.parameter + ' as ' + self.value)


class LowpassFilter:
    def __init__(self, gain):
        self.gain = gain
        self.gain_inv = 1 - gain
        self.x_f = 0

    def update(self, x):
        self.x_f = self.x_f * self.gain_inv + x * self.gain
        return self.x_f


def sqrt_safe(a, a_sqrt):
    if a >= 0:
        return math.sqrt(a)
    else:
        return a_sqrt


class AutoRunner():

    def __init__(self):
        self.pitch_init = -18  # 30 works
        self.omega_init = 0
        self.omega = 0  # 45 works
        self.time_init = 0
        self.is_activate = False
        self.time_deadzone = 0.0
        self.pitch_cmd = self.pitch_init
        self.cntr = 0  # cntr = 1, regular control to enter the running mode, cntr > 1, passive running.
        self.steps_lim = 30
        self.pc_time = 0.1
        self.is_terminated = False
        self.cmd_body_pitch = 25

        self.change_gait = False
        self.gait_interval = 4

        self.pitch_list = [-25, -30, -35, -35]
        self.omega_list = [20, 20, 20, 20]
        self.pc_time_list = [0.12, 0.13, 0.13, 0.14]
        self.body_pitch_list = [25, 26, 27, 28]

        self.deacc = False
        self.deacc_cntr = 0
        self.deacc_cntr_lim = 3
        self.pitch_cmd_deacc = -25
        self.omega_cmd_deacc = 0

        self.gait_length = len(self.pc_time_list)
        self.current_gait = 0

    def reset(self, time):
        print('run step:', self.cntr)
        self.time_init = time

        if not self.deacc:
            if not self.change_gait:
                self.pitch_cmd = self.pitch_init
                self.omega = self.omega_init
                self.pc_time = 0.12
            else:
                self.pitch_cmd = self.pitch_list[self.current_gait]
                self.pc_time = self.pc_time_list[self.current_gait]
                self.omega = self.omega_list[self.current_gait]
                self.cmd_body_pitch = self.body_pitch_list[self.current_gait]
        else:
            self.pitch_cmd = self.pitch_cmd_deacc
            self.omega = self.omega_cmd_deacc
            self.pc_time = 0
            self.cmd_body_pitch = 25

        if self.cntr > self.steps_lim:
            self.deacc = True

        if self.cntr > self.steps_lim + self.deacc_cntr_lim:
            self.is_terminated = True
            self.is_activate = False
            self.deacc = False

        if (self.cntr + 1) % self.gait_interval == 0 and self.current_gait < self.gait_length - 1 and self.change_gait:
            self.current_gait += 1

    def step(self, time):
        dt = (time - self.time_init) - self.time_deadzone

        if dt > 0:
            self.pitch_cmd = self.pitch_init + self.omega * dt
            # if self.pitch_cmd >-8:
            #     self.pitch_cmd = -8
        else:
            self.pitch_cmd = self.pitch_init

        return self.pitch_cmd