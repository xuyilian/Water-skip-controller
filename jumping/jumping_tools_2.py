# bsn functions and classes
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation


def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x


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


def apply_fit_result(x, y, p00=0.0, p10=0.0, p01=0.0, p20=0.0, p11=0.0, p02=0.0, ):
    # this function is the resulted polynomial to predict the takeoff state
    z = (
            p00 +
            p10 * x +
            p01 * y +
            p20 * x ** 2 +
            p11 * x * y +
            p02 * y ** 2
    )
    return z


def apply_fit_result_inv(y, z, p00=0.0, p10=0.0, p01=0.0, p20=0.0, p11=0.0, p02=0.0,):
    # this function is the resulted polynomial to predict the takeoff state

    x = -(p10 + p11 * y - (
                p10 ** 2 + 2 * p10 * p11 * y + p11 ** 2 * y ** 2 - 4 * p02 * p20 * y ** 2 - 4 * p01 * p20 * y - 4 * p00 * p20 + 4 * p20 * z) ** 0.5) / (
                    2 * p20)
    return x


class jumping_attitude_estimator:
    def __init__(self, g=9.0, estimator_gain=0.1):
        self.g = g
        self.estimator_gain = estimator_gain
        self.P_dot_l = 3.8
        self.vertical_landing_speed = 3.8
        self.tol = 1e-6
        self.flight_time = 1

        self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz = 1, 0, 0, 0
        self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz = 1, 0, 0, 0
        self.landing_rotm = quat_to_rotm(self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz)
        self.takeoff_rotm = quat_to_rotm(self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz)

        self.axis_angle_SP = np.array([1, 0, 0, 0])  # this variable is not used
        
        self.p_dot_LD = np.array([0, 0, -self.P_dot_l])
        self.p_dot_TO = np.array([0.0, 0.0, -self.P_dot_l])
        self.p_dot_LD_delay = np.array([0, 0, -self.P_dot_l])
        self.p_dot_TO_delay = np.array([0, 0, -self.P_dot_l])

        self.theta_ld = 0.0
        self.theta_to = 0.0
        self.landing_timestamp = -1.0
        self.takeoff_timestamp = -1.0
        self.jumping_height = 1.1

        self.vel_plane_vertical_angle = 0.0
        self.vel_inplane_angle = 0.0
        
        self.z_axis_word = np.array([0, 0, 1])

        self.logging_list = [
            'JAE_landing_qw', 'JAE_landing_qx', 'JAE_landing_qy', 'JAE_landing_qz',
            'JAE_takeoff_qw', 'JAE_takeoff_qx', 'JAE_takeoff_qy', 'JAE_takeoff_qz',
            'JAE_axis_angle_SP0', 'JAE_axis_angle_SP1', 'JAE_axis_angle_SP2', 'JAE_axis_angle_SP3',
            'JAE_p_dot_LD0', 'JAE_p_dot_LD1', 'JAE_p_dot_LD2',
            'JAE_p_dot_TO0', 'JAE_p_dot_TO1', 'JAE_p_dot_TO2',
            'JAE_theta_to', 'JAE_theta_ld',
            'JAE_jumping_height',
            'vel_plane_vertical_angle', ]
        self.logging_data = [0.0] * len(self.logging_list)

        self.logging_list_simplified = [
            'JAE_p_dot_TO0', 'JAE_p_dot_TO1', 'JAE_p_dot_TO2', ]
        self.logging_data_simplified = [0.0] * len(self.logging_list_simplified)

    def landing(self, w, x, y, z,):
        self.landing_timestamp = time.time()

        self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz = w, x, y, z
        self.landing_rotm = quat_to_rotm(self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz)

        if self.landing_timestamp != -1.0 and self.takeoff_timestamp != -1.0:
            flying_time = self.landing_timestamp - self.takeoff_timestamp
            self.jumping_height = 0.5 * self.g * ((flying_time / 2)**2)
            self.vertical_landing_speed = self.g * (flying_time / 2)
            self.P_dot_l = math.sqrt((self.vertical_landing_speed)**2 + (self.p_dot_TO[0])**2 + (self.p_dot_TO[1])**2)
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

        self.theta_ld = apply_fit_result_inv(self.P_dot_l, beta,
                                             p00=0.05442, p10=-0.1719, p01=-0.04432,
                                             p20=-0.2315, p11=0.249, p02=0.007425)

        self.theta_to = apply_fit_result(self.theta_ld, self.P_dot_l,
                                         p00=0.0007468, p10=1.026, p01=-0.0004735,
                                         p20=-0.006102, p11=-0.002207, p02=6.335e-05)

        if e_beta_norm < self.tol:
            e_beta = np.array([1, 0, 0])
        else:
            e_beta = e_beta / e_beta_norm

        self.axis_angle_SP = np.array([e_beta[0], e_beta[1], e_beta[2], beta])  # this is not used

        self.p_dot_LD_delay = self.p_dot_LD
        self.p_dot_TO_delay = self.p_dot_TO
        self.p_dot_LD = - np.matmul(Rotation.from_rotvec(-e_beta * - self.theta_ld).as_matrix(), e_3_LD.T) * self.P_dot_l
        self.p_dot_TO = + np.matmul(Rotation.from_rotvec(-e_beta * + self.theta_to).as_matrix(), e_3_TK.T) * self.P_dot_l

        vel_plane_vector = np.cross(self.p_dot_TO_delay, - self.p_dot_LD)
        vel_plane_vector_norm = np.linalg.norm(vel_plane_vector)
        if vel_plane_vector_norm < self.tol:
            vel_plane_vector = np.array([1, 0, 0])
        else:
            vel_plane_vector = vel_plane_vector / vel_plane_vector_norm
        
        self.vel_plane_vertical_angle = math.atan2(np.linalg.norm(np.cross(self.z_axis_word, vel_plane_vector)), np.dot(self.z_axis_word, vel_plane_vector))

        VPCH = np.cross(vel_plane_vector,self.z_axis_word)
        VPCH_norm = np.linalg.norm(VPCH)
        if VPCH_norm < self.tol:
            VPCH = np.array([1, 0, 0])
        else:
            VPCH = VPCH / VPCH_norm
        rotvec_correct_1 = VPCH * (self.vel_plane_vertical_angle - math.pi/2)
        R1 = Rotation.from_rotvec(rotvec_correct_1).as_matrix()

        p_dot_TO_delay_correction_1 = np.matmul(R1, self.p_dot_TO_delay)
        p_dot_LD_correction_1 = np.matmul(R1, self.p_dot_LD)

        mean_vel_vector = p_dot_TO_delay_correction_1/np.linalg.norm(p_dot_TO_delay_correction_1) - p_dot_LD_correction_1/np.linalg.norm(p_dot_LD_correction_1)
        mean_vel_vector_norm = np.linalg.norm(mean_vel_vector)
        if mean_vel_vector_norm < self.tol:
            mean_vel_vector = np.array([0, 0, 1])
        else:
            mean_vel_vector = mean_vel_vector / mean_vel_vector_norm

        # print(mean_vel_vector)

        self.vel_inplane_angle = math.atan2(np.linalg.norm(np.cross(self.z_axis_word, mean_vel_vector)), np.dot(self.z_axis_word, mean_vel_vector))

        VVCH = np.cross(mean_vel_vector, self.z_axis_word)
        VVCH_norm = np.linalg.norm(VVCH)
        if VVCH_norm < self.tol:
            VVCH = np.array([1, 0, 0])
        else:
            VVCH = VVCH / VVCH_norm
        rotvec_correct_2 = VVCH * self.vel_inplane_angle
        R2 = Rotation.from_rotvec(rotvec_correct_2).as_matrix()

        correction_matrix = np.matmul(R2, R1)
        takeoff_velocity_estimated = np.matmul(correction_matrix, self.p_dot_TO)
        # print(Rotation.from_matrix(R1).as_rotvec(),Rotation.from_matrix(R2).as_rotvec())

        self.logging_data = [
            self.landing_qw, self.landing_qx, self.landing_qy, self.landing_qz,
            self.takeoff_qw, self.takeoff_qx, self.takeoff_qy, self.takeoff_qz,
            self.axis_angle_SP[0], self.axis_angle_SP[1], self.axis_angle_SP[2], self.axis_angle_SP[3],
            self.p_dot_LD[0], self.p_dot_LD[1], self.p_dot_LD[2],
            self.p_dot_TO[0], self.p_dot_TO[1], self.p_dot_TO[2],
            self.theta_to, self.theta_ld,
            self.jumping_height,
            self.vel_plane_vertical_angle, ]
        self.logging_data_simplified = [takeoff_velocity_estimated[0], takeoff_velocity_estimated[1], takeoff_velocity_estimated[2]]
        
        correction_rotvec = Rotation.from_matrix(correction_matrix).as_rotvec() * self.estimator_gain
        correction_quat = Rotation.from_rotvec(correction_rotvec).as_quat()
        takeoff_velocity_estimated = self.p_dot_TO
        
        return correction_quat, takeoff_velocity_estimated


class JumpingStateTrackerOnboard:
    def __init__(self, acc_z_up_limit=0.6, controller_engage_time=0.1, controller_engage_ratio=0.2,
                 controller_engage_time_auto=True, powered_climbing_end_timer=0.2):
        self.acc_z_up_limit = acc_z_up_limit
        self.acc_z_delay = 1
        self.jumping_state = 1
        self.jumping_state_old = 1
        self.takeoff_time = time.time()
        self.landing_time = time.time()
        self.predicted_max_altitude_abs_time = time.time()
        self._predicted_max_altitude_time_flag = False

        # modified by ding, self.aerial_time = 1.0
        self.aerial_time = 1

        self.init_flag = False

        # change the controller engage time depending on the jumping period
        self.controller_engage_time_auto = controller_engage_time_auto
        self.controller_engage_ratio = controller_engage_ratio
        self.controller_engage_abs_time = controller_engage_time
        self.controller_engage_timer = time.time()
        self.controller_engage_flag = False

        self.powered_climbing_end_flag = False
        self.powered_climbing_end_abs_time = time.time()
        self.powered_climbing_end_timer = powered_climbing_end_timer

        self.logging_list = [
            'JSTO_jumping_state', 'JSTO_aerial_time', 'JSTO_controller_engage_flag']
        self.logging_data = [0.0] * len(self.logging_list)

    def step(self, acc_z):
        self.jumping_state_old = self.jumping_state

        # falling to stance
        # if self.acc_z_delay < self.acc_z_up_limit < acc_z
        if self.acc_z_delay < self.acc_z_up_limit < acc_z or self.acc_z_delay > - self.acc_z_up_limit > acc_z:
            self.jumping_state = 2
            self.landing_time = time.time()
            self.aerial_time = self.landing_time - self.takeoff_time

            self.aerial_time = 0.6 # modified by ding

        # stance to climbing
        # if acc_z < self.acc_z_up_limit < self.acc_z_delay
        if acc_z < self.acc_z_up_limit < self.acc_z_delay or acc_z > - self.acc_z_up_limit > self.acc_z_delay:
            self.jumping_state = 3
            self.takeoff_time = time.time()
            self.predicted_max_altitude_abs_time = self.takeoff_time + self.aerial_time * 0.5
            self._predicted_max_altitude_time_flag = True

            if self.controller_engage_time_auto:
                self.controller_engage_abs_time = self.aerial_time * self.controller_engage_ratio
            self.controller_engage_timer = self.takeoff_time + self.controller_engage_abs_time
            self.controller_engage_flag = True

            self.powered_climbing_end_abs_time = self.takeoff_time + self.powered_climbing_end_timer
            self.powered_climbing_end_flag = True

        # climbing to falling (predicted)
        if time.time() > self.predicted_max_altitude_abs_time and self._predicted_max_altitude_time_flag:
            self.jumping_state = 1
            self._predicted_max_altitude_time_flag = False

        # controller engage
        if time.time() > self.controller_engage_timer and self.controller_engage_flag:
            self.controller_engage_flag = False

        if time.time() > self.powered_climbing_end_abs_time and self.powered_climbing_end_flag:
            self.powered_climbing_end_flag = False

        self.acc_z_delay = acc_z
        self.logging_data = [self.jumping_state, self.aerial_time, self.controller_engage_flag, ]

    def init(self, ):
        if not self.init_flag:
            self.jumping_state = 1
            self.jumping_state_old = 1
            self.aerial_time = 0.5
            self.init_flag = True


class LinearJumpingController:
    def __init__(self, g=9.0, velocity_limit_ratio=0.5, velocity_gain=0.3, k=2.0):
        # parameters
        self.g = g
        self.velocity_limit_ratio = velocity_limit_ratio
        self.velocity_gain = velocity_gain
        self.k = k
        self.max_drift_angle = 45 / 180 * math.pi

        # constants
        self.square_root_2 = math.sqrt(2)

        # variables
        self.desired_x = 0.0   # CoM location
        self.desired_y = 0.0
        # CoM jumping altitude
        self.jumping_altitude = 0.5

        self.landing_x_dot = 0.0  # landing state
        self.landing_y_dot = 0.0
        self.landing_z_dot = -2.0
        self.landing_x = 0.0
        self.landing_y = 0.0
        self.aerial_time = 1.0
        # self.landing_x_old = 0.0
        # self.landing_y_old = 0.0
        self.takeoff_x_dot = 0.0
        self.takeoff_y_dot = 0.0
        self.takeoff_z_dot = 2.0

    def set_reference(self, desired_x, desired_y, jumping_altitude, ):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.jumping_altitude = jumping_altitude

    def update_landing_state(self, landing_x_dot, landing_y_dot, landing_z_dot,
                             landing_x, landing_y,
                             aerial_time):
        self.landing_x_dot = landing_x_dot
        self.landing_y_dot = landing_y_dot
        self.landing_z_dot = landing_z_dot
        self.landing_x = landing_x
        self.landing_y = landing_y
        self.aerial_time = aerial_time  # xxx
        # self.landing_x_old = landing_x_old
        # self.landing_y_old = landing_y_old

    def jumping_planning(self, ):
        time_climb = math.sqrt(2 * self.jumping_altitude / self.g)
        time_fall = time_climb
        time_plan = time_climb + time_fall
        u_x = (self.desired_x - self.landing_x) / time_plan
        u_y = (self.desired_y - self.landing_y) / time_plan
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)

        temp = self.landing_x_dot * self.landing_x_dot + \
               self.landing_y_dot * self.landing_y_dot + \
               self.landing_z_dot * self.landing_z_dot
        if temp <= 0.5:
            predicted_takeoff_speed = math.sqrt(0.5)
        else:
            predicted_takeoff_speed = math.sqrt(temp)
        velocity_limit = predicted_takeoff_speed * self.velocity_limit_ratio
        if norm_v > velocity_limit:
            u_x = velocity_limit * u_x / norm_v
            u_y = velocity_limit * u_y / norm_v
        self.takeoff_x_dot = u_x
        self.takeoff_y_dot = u_y
        self.takeoff_z_dot = self.g * time_climb

    def inverse_jumping_model(self, yaw):
        V_landing = np.array([self.landing_x_dot, self.landing_y_dot, self.landing_z_dot])
        V_takeoff = np.array([self.takeoff_x_dot * self.velocity_gain, self.takeoff_y_dot * self.velocity_gain,
                              self.takeoff_z_dot])
        V_surface = np.cross(- V_landing, V_takeoff)
        V_surface_norm = np.linalg.norm(V_surface)
        if V_surface_norm < 0.0001:
            V_surface = np.array([1, 0, 0])
        else:
            V_surface = V_surface / V_surface_norm
        angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))
        desired_tilt_angle = angle_between_velocities / self.k
        desired_tilt_angle = saturation(desired_tilt_angle, self.max_drift_angle, 0)
        r = Rotation.from_rotvec(desired_tilt_angle * V_surface)
        Rotm = r.as_matrix()

        V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
        V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)

        # attitude regulator
        r = Rotation.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, V_landing_desired)

        self.roll = - math.asin(V_landing_desired_body[[1]])
        self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi

        return V_landing_desired


class JumpingHeightController:
    def __init__(self, leg_efficiency=0.8, g=9.81, t_p_low=0.04, t_p_high=0.3, thrust_gain=1):
        self.thrust_gain = thrust_gain
        self.t_p_high = t_p_high
        self.t_p_low = t_p_low
        self.g = g
        self.leg_efficiency = leg_efficiency
        pass

    def step(self, flight_time, desired_h):
        h = 0.5 * self.g * flight_time * flight_time / 4
        unpowered_h = self.leg_efficiency * h

        take_off_speed = math.sqrt(2 * self.g * h * self.leg_efficiency)

        if desired_h < h:
            powered_climbing_time = self.t_p_low
        else:
            powered_climbing_time = (desired_h - h)/(take_off_speed * self.thrust_gain)
            if powered_climbing_time > self.t_p_high:
                powered_climbing_time = powered_climbing_time
            if powered_climbing_time < self.t_p_low:
                powered_climbing_time = self.t_p_low
            if powered_climbing_time > flight_time * 0.4:
                powered_climbing_time = flight_time * 0.4

        # print(take_off_speed, h)
        return powered_climbing_time

