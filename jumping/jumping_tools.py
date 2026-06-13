# bsn functions and classes
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation


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
        self.tol = 1e-6

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
            self.P_dot_l = self.g * (flying_time / 2)

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

        self.axis_angle_SP = np.array([e_beta[0], e_beta[1], e_beta[2], beta])

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
        z_axis_word = np.array([0, 0, 1])
        self.vel_plane_vertical_angle = math.atan2(np.linalg.norm(np.cross(z_axis_word, vel_plane_vector)), np.dot(z_axis_word, vel_plane_vector))

        VPCH = np.cross(vel_plane_vector,z_axis_word)
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

        self.vel_inplane_angle = math.atan2(np.linalg.norm(np.cross(z_axis_word, mean_vel_vector)), np.dot(z_axis_word, mean_vel_vector))

        VVCH = np.cross(mean_vel_vector, z_axis_word)
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