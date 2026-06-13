import numpy as np
import math
import time
from scipy.spatial.transform import Rotation
from collections import deque


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


def quat_to_rotm(w, x, y, z):
    """
    Converts a quaternion to a rotation matrix.

    :param q: A 4-element numpy array representing the quaternion in the order (w, x, y, z)
    :return: A 3x3 numpy array representing the rotation matrix
    """
    rotm = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                     [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
                     [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]])
    return rotm


def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x


class Differentiator:
    def __init__(self, ):
        self.t_delay = 0
        self.data_delay = -1

    def step(self, data_now, abstime):
        dt = abstime - self.t_delay
        if dt == 0:
            self.t_delay = abstime
            return 0
        else:
            data_rate = (data_now - self.data_delay) / (abstime - self.t_delay)
            self.t_delay = abstime
            self.data_delay = data_now
            return data_rate


class Differentiator2:
    def __init__(self, ):
        self.t_delay = 0
        self.data_delay = -1
        self.data_rate = 0

    def step(self, data_now, abstime):
        dt = abstime - self.t_delay

        if dt == 0:
            self.t_delay = abstime
            return self.data_rate
        else:
            self.data_rate = (data_now - self.data_delay) / (abstime - self.t_delay)
            self.t_delay = abstime
            self.data_delay = data_now
            return self.data_rate


class Differentiator3:
    def __init__(self, diff_steps=1):
        self.t_queue = deque([0] * diff_steps, maxlen=diff_steps)
        self.data_queue = deque([0] * diff_steps, maxlen=diff_steps)
        self.data_rate = 0

    def step(self, data_now, abstime):
        dt = abstime - self.t_queue[0]
        if dt == 0:
            self.t_queue.append(abstime)
        else:
            self.data_rate = (data_now - self.data_queue[0]) / dt
            self.t_queue.append(abstime)
            self.data_queue.append(data_now)

        return self.data_rate


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

        return landing_attitude_z_b, v_takeoff


class LinearJumpingController:
    def __init__(self, g=9.0, velocity_limit_ratio=0.3, velocity_gain=1, model=None):
        # parameters
        self.g = g
        self.velocity_limit_ratio = velocity_limit_ratio
        self.velocity_gain = velocity_gain

        if model is None:
            self.model_2d = InPlaneJumpingModel(1.5, 2)
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
        self.landing_att_x = 0
        self.landing_att_y = 0
        self.landing_att_z = 0
        
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
        time_climb = math.sqrt(2 * self.jumping_altitude / self.g)
        time_fall = time_climb
        time_plan = time_climb + time_fall
        u_x = self.velocity_gain * (self.desired_x - self.landing_x) / time_plan
        u_y = self.velocity_gain * (self.desired_y - self.landing_y) / time_plan
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)

        landing_speed = np.linalg.norm(self.landing_velocity)
        predicted_takeoff_speed = landing_speed

        lateral_velocity_limit = predicted_takeoff_speed * self.velocity_limit_ratio
        if norm_v > lateral_velocity_limit:
            u_x = lateral_velocity_limit * u_x / norm_v
            u_y = lateral_velocity_limit * u_y / norm_v
        self.takeoff_velocity = np.array([u_x, u_y, self.g * time_climb])

    def inverse_jumping_model(self, yaw):
        landing_attitude_z_b, _ = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)
        self.landing_att_x = landing_attitude_z_b[0]
        self.landing_att_y = landing_attitude_z_b[1]
        self.landing_att_z = landing_attitude_z_b[2]
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        v_landing_desired_body = r.apply(landing_attitude_z_b)
        self.roll = - math.asin(v_landing_desired_body[1])
        self.pitch = math.asin(v_landing_desired_body[0] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi


class LinearJumpingController2:
    def __init__(self, g=8.5, velocity_limit_ratio=0.3, velocity_gain=1, model=None):
        # parameters
        self.g = g
        self.velocity_limit_ratio = velocity_limit_ratio
        self.velocity_gain = velocity_gain

        if model is None:
            self.model_2d = InPlaneJumpingModel(1.5, 2)
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
        self.landing_x_dot_estimated = 0
        self.landing_y_dot_estimated = 0
        self.landing_z_dot_estimated = -3

        self.roll = 0.0
        self.pitch = 0.0

        self.run_time = 0
        self.time_planed = 1.0

        self.remove_velocity_gain_flag = False
        self.landing_att_x = 0
        self.landing_att_y = 0
        self.landing_att_z = 1
        # logging
        self.logging_list = [
            'LJC_run_time', 'LJC_desired_x', 'LJC_desired_y', 'LJC_time_planed',
            'LJC_landing_x_dot_estimated', 'LJC_landing_y_dot_estimated', 'LJC_landing_z_dot_estimated', ]
        self.logging_data = [0.0] * len(self.logging_list)
        self.is_velcontrol = False
        self.is_moving = False

    def init(self):
        self.roll, self.pitch = 0, 0
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.jumping_altitude = 0.5
        self.landing_velocity = np.array([0, 0, -2])
        self.takeoff_velocity = np.array([0, 0, 2])
        self.landing_x = 0.0
        self.landing_y = 0.0
        self.landing_x_dot_estimated = 0
        self.landing_y_dot_estimated = 0
        self.landing_z_dot_estimated = -3

        self.run_time = 0
        self.time_planed = 1.0

    def remove_velocity_gain(self):
        self.remove_velocity_gain_flag = True

    def set_reference(self, desired_x, desired_y, jumping_altitude, ):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.jumping_altitude = jumping_altitude

    def update_landing_state(self, landing_x_dot, landing_y_dot, landing_z_dot,
                             landing_x, landing_y, ):

        self.landing_x_dot_estimated = landing_x_dot
        self.landing_y_dot_estimated = landing_y_dot
        self.landing_z_dot_estimated = landing_z_dot
        self.landing_velocity = np.array([landing_x_dot, landing_y_dot, landing_z_dot])
        self.landing_x = landing_x
        self.landing_y = landing_y

    def jumping_planning(self, ):
        time_climb = sqrt_safe(2 * self.jumping_altitude / self.g, 0.01)
        # time_climb = math.sqrt(2 * self.jumping_altitude / self.g)
        time_fall = time_climb
        self.time_planed = time_climb + time_fall

        if self.remove_velocity_gain_flag:
            u_x = (self.desired_x - self.landing_x) / self.time_planed
            u_y = (self.desired_y - self.landing_y) / self.time_planed
            self.remove_velocity_gain_flag = False
        else:
            u_x = self.velocity_gain*(self.desired_x - self.landing_x) / self.time_planed
            u_y = self.velocity_gain*(self.desired_y - self.landing_y) / self.time_planed
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)

        landing_speed = np.linalg.norm(self.landing_velocity)
        # predicted_takeoff_speed = landing_speed
        predicted_takeoff_speed = landing_speed * 1.1  # considering the powered climbing

        lateral_velocity_limit = predicted_takeoff_speed * self.velocity_limit_ratio
        if norm_v > lateral_velocity_limit:
            u_x = lateral_velocity_limit * u_x / norm_v
            u_y = lateral_velocity_limit * u_y / norm_v
        self.takeoff_velocity = np.array([u_x, u_y, math.sqrt(predicted_takeoff_speed * predicted_takeoff_speed - u_x * u_x - u_y * u_y)])
        if self.is_velcontrol:
            lateral_vel_dir = [u_x / norm_v,u_y / norm_v]
            vertical_vel_norm = landing_speed*0.7
            lateral_vel_norm  = landing_speed*0.7
            self.takeoff_velocity = np.array([lateral_vel_dir[0]*lateral_vel_norm,lateral_vel_dir[1]*lateral_vel_norm,vertical_vel_norm])

    def jumping_planning_angle(self, u_x, u_y):
        temp = math.sqrt(u_x * u_x + u_y * u_y)
        if temp > 1:
            self.takeoff_velocity = np.array([u_x, u_y, temp * 1])
        else:
            self.takeoff_velocity = np.array([u_x, u_y, 1])

    def jumping_planning_velocity_control(self, speed):
        # to be removed
        landing_speed = np.linalg.norm(self.landing_velocity)
        predicted_takeoff_speed = landing_speed * 1.1  # considering the powered climbing
        if speed < predicted_takeoff_speed:
            self.takeoff_velocity = np.array([-speed, 0, math.sqrt(predicted_takeoff_speed*predicted_takeoff_speed - speed*speed)])
        else:
            self.takeoff_velocity = np.array([-speed, 0, math.sqrt(predicted_takeoff_speed * predicted_takeoff_speed - speed * speed)])

    def inverse_jumping_model(self, yaw, Abs_time):
        landing_attitude_z_b, _ = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)
        self.landing_att_x = landing_attitude_z_b[0]
        self.landing_att_y = landing_attitude_z_b[1]
        self.landing_att_z = landing_attitude_z_b[2]
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        v_landing_desired_body = r.apply(landing_attitude_z_b)
        self.roll = - math.asin(v_landing_desired_body[1])
        self.pitch = math.asin(v_landing_desired_body[0] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi
        self.run_time = Abs_time
        self.logging_data = [
            self.run_time, self.desired_x, self.desired_y, self.time_planed,
            self.landing_x_dot_estimated, self.landing_y_dot_estimated, self.landing_z_dot_estimated]


    def inverse_jumping_model_leg(self, robot_R_imu, Abs_time):
        landing_attitude_z_b, _ = self.JM3D.fcn1(self.landing_velocity, self.takeoff_velocity)
        self.landing_att_x = landing_attitude_z_b[0]
        self.landing_att_y = landing_attitude_z_b[1]
        self.landing_att_z = landing_attitude_z_b[2]

        robot_R_imu.as_matrix()


class JumpingAttitudeEstimator:
    def __init__(self, g=9.0, estimator_gain=0.1, model=None):
        # parameters
        if model is None:
            self.model_2d = InPlaneJumpingModel(1.5, 2)
        else:
            self.model_2d = model
        self.g = g
        self.estimator_gain = estimator_gain
        self.hold_on = False
        self.flight_time_hold_on = 0.5
        # constants
        self.z_axis_world = np.array([0, 0, 1])
        self.tol = 1e-6

        # variables
        self.landing_timestamp = -1.0
        self.takeoff_timestamp = -1.0

        self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz = 1, 0, 0, 0
        self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz = 1, 0, 0, 0
        self.landing_rotm = quat_to_rotm(self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz)
        self.takeoff_rotm = quat_to_rotm(self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz)

        self.P_dot_l = 2.0

        self.p_dot_LD = np.array([0, 0, -self.P_dot_l])
        self.p_dot_TO = np.array([0.0, 0.0, -self.P_dot_l])
        self.p_dot_LD_delay = np.array([0, 0, -self.P_dot_l])
        self.p_dot_TO_delay = np.array([0, 0, -self.P_dot_l])

        self.jumping_height = 0.5 * self.g * ((1 / 2) ** 2)
        self.vertical_landing_speed = self.g * (1 / 2)
        self.P_dot_l = math.sqrt(self.vertical_landing_speed ** 2 +
                                 (self.p_dot_TO[0]) ** 2 + (self.p_dot_TO[1]) ** 2)
        self.flight_time = 1

        self.takeoff_velocity_estimated = np.array([0, 0, 2])

        # logging
        self.logging_list = [
            'JAE_landing_qw', 'JAE_landing_qx', 'JAE_landing_qy', 'JAE_landing_qz',
            'JAE_takeoff_qw', 'JAE_takeoff_qx', 'JAE_takeoff_qy', 'JAE_takeoff_qz',
            'JAE_p_dot_LD0', 'JAE_p_dot_LD1', 'JAE_p_dot_LD2',
            'JAE_p_dot_TO0', 'JAE_p_dot_TO1', 'JAE_p_dot_TO2',
            'JAE_theta_to', 'JAE_theta_ld',
            'JAE_jumping_height',
            'vel_plane_vertical_angle', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def landing(self, w, x, y, z,):
        self.landing_timestamp = time.time()
        self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz = w, x, y, z
        self.landing_rotm = quat_to_rotm(self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz)

        if self.landing_timestamp != -1.0 and self.takeoff_timestamp != -1.0:
            flying_time = self.landing_timestamp - self.takeoff_timestamp
            self.jumping_height = 0.5 * self.g * ((flying_time / 2)**2)
            self.vertical_landing_speed = self.g * (flying_time / 2)
            if self.hold_on:
                self.vertical_landing_speed = self.g * (self.flight_time_hold_on / 2)

            self.P_dot_l = math.sqrt(self.vertical_landing_speed ** 2 +
                                     (self.p_dot_TO[0]) ** 2 + (self.p_dot_TO[1]) ** 2)
            self.flight_time = flying_time

    def takeoff(self, w, x, y, z):
        self.takeoff_timestamp = time.time()
        self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz = w, x, y, z
        self.takeoff_rotm = quat_to_rotm(self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz)
        e_3_LD = self.landing_rotm[:, 2]
        e_3_TK = self.takeoff_rotm[:, 2]
        e_beta = np.cross(e_3_TK, e_3_LD)
        e_beta_norm = np.linalg.norm(e_beta)
        beta = math.atan2(e_beta_norm, np.dot(e_3_TK, e_3_LD))

        self.model_2d.input_beta(beta, self.P_dot_l)
        self.theta_ld = self.model_2d.theta_l
        self.theta_to = self.model_2d.theta_v - self.model_2d.theta_t

        # self.theta_ld = apply_fit_result_inv(self.P_dot_l, beta,
        #                                      p00=0.05442, p10=-0.1719, p01=-0.04432,
        #                                      p20=-0.2315, p11=0.249, p02=0.007425)
        # self.theta_to = apply_fit_result(self.theta_ld, self.P_dot_l,
        #                                  p00=0.0007468, p10=1.026, p01=-0.0004735,
        #                                  p20=-0.006102, p11=-0.002207, p02=6.335e-05)

        if e_beta_norm < self.tol:
            e_beta = np.array([1, 0, 0])
        else:
            e_beta = e_beta / e_beta_norm

        self.axis_angle_SP = np.array([e_beta[0], e_beta[1], e_beta[2], beta])  # this is not used

        self.p_dot_LD_delay = self.p_dot_LD
        self.p_dot_TO_delay = self.p_dot_TO
        self.p_dot_LD = - np.matmul(Rotation.from_rotvec(-e_beta * - self.theta_ld).as_matrix(),
                                    e_3_LD.T) * self.P_dot_l
        self.p_dot_TO = + np.matmul(Rotation.from_rotvec(-e_beta * + self.theta_to).as_matrix(),
                                    e_3_TK.T) * self.P_dot_l

        vel_plane_vector = np.cross(self.p_dot_TO_delay, - self.p_dot_LD)
        vel_plane_vector_norm = np.linalg.norm(vel_plane_vector)
        if vel_plane_vector_norm < self.tol:
            vel_plane_vector = np.array([1, 0, 0])
        else:
            vel_plane_vector = vel_plane_vector / vel_plane_vector_norm

        self.vel_plane_vertical_angle = math.atan2(np.linalg.norm(np.cross(self.z_axis_world, vel_plane_vector)),
                                                   np.dot(self.z_axis_world, vel_plane_vector))

        VPCH = np.cross(vel_plane_vector, self.z_axis_world)
        VPCH_norm = np.linalg.norm(VPCH)
        if VPCH_norm < self.tol:
            VPCH = np.array([1, 0, 0])
        else:
            VPCH = VPCH / VPCH_norm
        rotvec_correct_1 = VPCH * (self.vel_plane_vertical_angle - math.pi / 2)
        R1 = Rotation.from_rotvec(rotvec_correct_1).as_matrix()

        p_dot_TO_delay_correction_1 = np.matmul(R1, self.p_dot_TO_delay)
        p_dot_LD_correction_1 = np.matmul(R1, self.p_dot_LD)

        mean_vel_vector = p_dot_TO_delay_correction_1 / np.linalg.norm(
            p_dot_TO_delay_correction_1) - p_dot_LD_correction_1 / np.linalg.norm(p_dot_LD_correction_1)
        mean_vel_vector_norm = np.linalg.norm(mean_vel_vector)
        if mean_vel_vector_norm < self.tol:
            mean_vel_vector = np.array([0, 0, 1])
        else:
            mean_vel_vector = mean_vel_vector / mean_vel_vector_norm

        # print(mean_vel_vector)

        self.vel_inplane_angle = math.atan2(np.linalg.norm(np.cross(self.z_axis_world, mean_vel_vector)),
                                            np.dot(self.z_axis_world, mean_vel_vector))

        VVCH = np.cross(mean_vel_vector, self.z_axis_world)
        VVCH_norm = np.linalg.norm(VVCH)
        if VVCH_norm < self.tol:
            VVCH = np.array([1, 0, 0])
        else:
            VVCH = VVCH / VVCH_norm
        rotvec_correct_2 = VVCH * self.vel_inplane_angle
        R2 = Rotation.from_rotvec(rotvec_correct_2).as_matrix()

        correction_matrix = np.matmul(R2, R1)
        takeoff_velocity_estimated = np.matmul(correction_matrix, self.p_dot_TO)


        self.logging_data = [
            self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz,
            self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz,
            self.p_dot_LD[0], self.p_dot_LD[1], self.p_dot_LD[2],
            self.p_dot_TO[0], self.p_dot_TO[1], self.p_dot_TO[2],
            self.theta_to, self.theta_ld,
            self.jumping_height,
            self.vel_plane_vertical_angle, ]

        correction_rotvec = Rotation.from_matrix(correction_matrix).as_rotvec() * self.estimator_gain
        correction_quat = Rotation.from_rotvec(correction_rotvec).as_quat()
        takeoff_velocity_estimated = self.p_dot_TO
        self.takeoff_velocity_estimated = takeoff_velocity_estimated

        return correction_quat

    def takeoff_bidirection(self, w, x, y, z, up_landing):
        self.takeoff_timestamp = time.time()
        self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz = w, x, y, z
        self.takeoff_rotm = quat_to_rotm(self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz)
        if up_landing:
            e_3_LD = self.landing_rotm[:, 2]
            e_3_TK = self.takeoff_rotm[:, 2]
        else:
            e_3_LD = -1 * self.landing_rotm[:, 2]
            e_3_TK = -1 * self.takeoff_rotm[:, 2]
        e_beta = np.cross(e_3_TK, e_3_LD)
        e_beta_norm = np.linalg.norm(e_beta)
        beta = math.atan2(e_beta_norm, np.dot(e_3_TK, e_3_LD))

        self.model_2d.input_beta(beta, self.P_dot_l)
        self.theta_ld = self.model_2d.theta_l
        self.theta_to = self.model_2d.theta_v - self.model_2d.theta_t

        # self.theta_ld = apply_fit_result_inv(self.P_dot_l, beta,
        #                                      p00=0.05442, p10=-0.1719, p01=-0.04432,
        #                                      p20=-0.2315, p11=0.249, p02=0.007425)
        # self.theta_to = apply_fit_result(self.theta_ld, self.P_dot_l,
        #                                  p00=0.0007468, p10=1.026, p01=-0.0004735,
        #                                  p20=-0.006102, p11=-0.002207, p02=6.335e-05)

        if e_beta_norm < self.tol:
            e_beta = np.array([1, 0, 0])
        else:
            e_beta = e_beta / e_beta_norm

        self.axis_angle_SP = np.array([e_beta[0], e_beta[1], e_beta[2], beta])  # this is not used

        self.p_dot_LD_delay = self.p_dot_LD
        self.p_dot_TO_delay = self.p_dot_TO
        self.p_dot_LD = - np.matmul(Rotation.from_rotvec(-e_beta * - self.theta_ld).as_matrix(),
                                    e_3_LD.T) * self.P_dot_l
        self.p_dot_TO = + np.matmul(Rotation.from_rotvec(-e_beta * + self.theta_to).as_matrix(),
                                    e_3_TK.T) * self.P_dot_l * 0.9

        vel_plane_vector = np.cross(self.p_dot_TO_delay, - self.p_dot_LD)
        vel_plane_vector_norm = np.linalg.norm(vel_plane_vector)
        if vel_plane_vector_norm < self.tol:
            vel_plane_vector = np.array([1, 0, 0])
        else:
            vel_plane_vector = vel_plane_vector / vel_plane_vector_norm

        self.vel_plane_vertical_angle = math.atan2(np.linalg.norm(np.cross(self.z_axis_world, vel_plane_vector)),
                                                   np.dot(self.z_axis_world, vel_plane_vector))

        VPCH = np.cross(vel_plane_vector, self.z_axis_world)
        VPCH_norm = np.linalg.norm(VPCH)
        if VPCH_norm < self.tol:
            VPCH = np.array([1, 0, 0])
        else:
            VPCH = VPCH / VPCH_norm
        rotvec_correct_1 = VPCH * (self.vel_plane_vertical_angle - math.pi / 2)
        R1 = Rotation.from_rotvec(rotvec_correct_1).as_matrix()

        p_dot_TO_delay_correction_1 = np.matmul(R1, self.p_dot_TO_delay)
        p_dot_LD_correction_1 = np.matmul(R1, self.p_dot_LD)

        mean_vel_vector = p_dot_TO_delay_correction_1 / np.linalg.norm(
            p_dot_TO_delay_correction_1) - p_dot_LD_correction_1 / np.linalg.norm(p_dot_LD_correction_1)
        mean_vel_vector_norm = np.linalg.norm(mean_vel_vector)
        if mean_vel_vector_norm < self.tol:
            mean_vel_vector = np.array([0, 0, 1])
        else:
            mean_vel_vector = mean_vel_vector / mean_vel_vector_norm

        # print(mean_vel_vector)

        self.vel_inplane_angle = math.atan2(np.linalg.norm(np.cross(self.z_axis_world, mean_vel_vector)),
                                            np.dot(self.z_axis_world, mean_vel_vector))

        VVCH = np.cross(mean_vel_vector, self.z_axis_world)
        VVCH_norm = np.linalg.norm(VVCH)
        if VVCH_norm < self.tol:
            VVCH = np.array([1, 0, 0])
        else:
            VVCH = VVCH / VVCH_norm
        rotvec_correct_2 = VVCH * self.vel_inplane_angle
        R2 = Rotation.from_rotvec(rotvec_correct_2).as_matrix()

        correction_matrix = np.matmul(R2, R1)
        takeoff_velocity_estimated = np.matmul(correction_matrix, self.p_dot_TO)


        self.logging_data = [
            self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz,
            self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz,
            self.p_dot_LD[0], self.p_dot_LD[1], self.p_dot_LD[2],
            self.p_dot_TO[0], self.p_dot_TO[1], self.p_dot_TO[2],
            self.theta_to, self.theta_ld,
            self.jumping_height,
            self.vel_plane_vertical_angle, ]

        correction_rotvec = Rotation.from_matrix(correction_matrix).as_rotvec() * self.estimator_gain
        correction_quat = Rotation.from_rotvec(correction_rotvec).as_quat()
        takeoff_velocity_estimated = self.p_dot_TO
        self.takeoff_velocity_estimated = takeoff_velocity_estimated

        return correction_quat


class RealTimeSleeper:
    def __init__(self, sample_time):
        self._sample_time = sample_time
        self.loop_start_time = time.time()
        self.loop_flag = 0

    def init(self):
        self.loop_start_time = time.time()

    def sleep(self):
        self.loop_flag = self.loop_flag + 1
        current_time = time.time()

        loop_end_time = (self.loop_start_time + self._sample_time)
        sleep_time = loop_end_time - current_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print('warning: loop frequency lower than expected!', self.loop_start_time)
        self.loop_start_time = time.time()


class LandingStateEstimator:
    def __init__(self, ):
        self.previous_landing_x = 0.0
        self.previous_landing_y = 0.0
        self.landing_x = 0.0
        self.landing_y = 0.0

        # logging
        self.logging_list = ['LSE_landing_x', 'LSE_landing_y', ]
        self.logging_data = [0.0] * len(self.logging_list)

    def update_landing_location(self, x, y):
        self.previous_landing_x = x
        self.previous_landing_y = y

    def update_landing_location_self(self):
        self.previous_landing_x = self.landing_x
        self.previous_landing_y = self.landing_y

    def estimation(self, x_dot, y_dot, flight_time):
        landing_x_dot = x_dot
        landing_y_dot = y_dot
        self.landing_x = self.previous_landing_x + landing_x_dot * flight_time
        self.landing_y = self.previous_landing_y + landing_y_dot * flight_time

        # logging
        self.logging_data = [self.landing_x, self.landing_y, ]


class LandingStateEstimator2:
    def __init__(self, ):
        self.previous_landing_x = 0.0
        self.previous_landing_y = 0.0
        self.previous_landing_dx = 0
        self.previous_landing_dy = 0

        self.landing_x = 0.0
        self.landing_y = 0.0
        self.previous_landing_time = 0.0
        self.landing_time_estimated = self.previous_landing_time

        # logging
        self.logging_list = ['LSE_landing_x', 'LSE_landing_y', 'LSE_landing_time_estimated', 'LSE_previous_landing_time']
        self.logging_data = [0.0] * len(self.logging_list)

    def init(self):
        self.previous_landing_x = 0.0
        self.previous_landing_y = 0.0
        self.previous_landing_dx = 0
        self.previous_landing_dy = 0

        self.landing_x = 0.0
        self.landing_y = 0.0
        self.previous_landing_time = 0.0
        self.landing_time_estimated = self.previous_landing_time

    def update_landing_location(self, x, y, Abs_time):
        self.previous_landing_x = x
        self.previous_landing_y = y
        self.previous_landing_time = Abs_time

    def update_landing_location_self(self, Abs_time):
        self.previous_landing_x = self.landing_x
        self.previous_landing_y = self.landing_y
        self.previous_landing_time = Abs_time

    def estimation(self, x_dot, y_dot, flight_time):
        landing_x_dot = x_dot
        landing_y_dot = y_dot
        self.landing_x = self.previous_landing_x + landing_x_dot * flight_time
        self.landing_y = self.previous_landing_y + landing_y_dot * flight_time
        self.landing_time_estimated = self.previous_landing_time + flight_time
        # logging
        self.logging_data = [self.landing_x, self.landing_y, self.landing_time_estimated, self.previous_landing_time]

    def estimation_now(self, x_dot, y_dot, flight_time, x, y, t):
        landing_x_dot = x_dot
        landing_y_dot = y_dot
        self.landing_x = x + landing_x_dot * flight_time
        self.landing_y = y + landing_y_dot * flight_time
        self.landing_time_estimated = t + flight_time
        # logging
        self.logging_data = [self.landing_x, self.landing_y, self.landing_time_estimated, self.previous_landing_time]


def sqrt_safe(a, a_sqrt):
    if a >= 0:
        return math.sqrt(a)
    else:
        return a_sqrt