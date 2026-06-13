import numpy as np
import math
from transforms3d.quaternions import quat2mat
from scipy.spatial.transform import Rotation as R


def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x


def get_hrotm(qw, qx, qy, qz, x, y, z):
    rotm = quat2mat([qw, qx, qy, qz])
    P = np.array([[x, y, z]]).T
    HB = np.array([[0, 0, 0, 1]])
    temp = np.concatenate((rotm, P), axis=1)
    hrotm = np.concatenate((temp, HB), axis=0)
    return hrotm


class JumpingController:
    def __init__(self, foot_offset=-0.15, jumping_threshold_foot=0.15, g=9.0, k=2.6, velocity_gain=0.89,
                 velocity_limit=1):
        self.velocity_limit = velocity_limit
        self.velocity_limit_backup = velocity_limit
        self.velocity_gain = velocity_gain
        self.velocity_gain_backup = self.velocity_gain
        self.K = k
        self.g = g
        self.foot_offset = foot_offset
        self.foot_tform_offset = get_hrotm(1, 0, 0, 0, 0, 0, foot_offset)
        self.jumping_threshold_foot = jumping_threshold_foot

        # state
        self.jumping_state = 0  # 1 for falling phase, 2 for jumping phase, 3 for falling phase, 0 for non-jumping mode
        self.jumping_state_old = 0
        self.foot_tform = get_hrotm(1, 0, 0, 0, 0, 0, 0)
        self.foot_height = 0

        self.max_height_before_next_jumping = 0
        self.max_height_before_next_jumping_abs_time = 0
        self.falling_time_estimated = 0
        self.landing_time_abs_estimated = 0
        self.landing_vz_estimated = 0
        self.max_height_before_next_jumping_x = 0
        self.max_height_before_next_jumping_y = 0

        self.landing_vx_estimated = 0
        self.landing_vy_estimated = 0
        self.predicted_landing_x = 0
        self.predicted_landing_y = 0

        self.climbing2falling_abs_time = 0
        self.climbing2falling_abs_time_old = 0
        self.jumping_peroid = 1

        self.desired_vx_after_jump = 0
        self.desired_vy_after_jump = 0
        self.desired_vz_after_jump = 0

        # desired jumping location
        self.desired_x = 0
        self.desired_y = 0

        self.roll = 0
        self.pitch = 0

    def update_desired_jumping_location(self, x, y):
        self.velocity_limit = self.velocity_limit_backup
        self.velocity_gain = self.velocity_gain_backup
        self.desired_x = x
        self.desired_y = y

    def update_desired_jumping_location_jump_left_and_right(self, distance):
        self.velocity_limit = 1.5
        self.velocity_gain = 1.15
        self.desired_x = 0
        if (self.jumping_state_old == 2 or self.jumping_state_old == 1) and self.jumping_state == 3:  # take off
            if self.desired_y == distance:
                self.desired_y = -distance
            elif self.desired_y == -distance:
                self.desired_y = distance
            else:
                self.desired_y = distance

    def step(self, hrotm, x_dot, y_dot, z_dot, yaw, abs_time):
        self.foot_tform = np.matmul(hrotm, self.foot_tform_offset)
        self.foot_height = self.foot_tform[2, 3]

        # update state and state_old
        self.jumping_state_old = self.jumping_state
        if self.foot_height < self.jumping_threshold_foot:  # in jumping phase
            self.jumping_state = 2
        elif z_dot >= 0:
            self.jumping_state = 3
        elif z_dot < 0:
            self.jumping_state = 1

        # find the highest jumping point
        if self.jumping_state_old == self.jumping_state:
            pass
        else:
            if self.jumping_state_old == 3 and self.jumping_state == 1:  # from climbing to falling
                self.climbing2falling_abs_time_old = self.climbing2falling_abs_time
                self.climbing2falling_abs_time = abs_time
                self.jumping_peroid = self.climbing2falling_abs_time - self.climbing2falling_abs_time_old

                self.max_height_before_next_jumping = hrotm[2, 3]
                self.max_height_before_next_jumping_abs_time = abs_time
                self.falling_time_estimated = math.sqrt(2.0 * (self.max_height_before_next_jumping
                                                               - self.jumping_threshold_foot + self.foot_offset) / self.g)
                self.landing_time_abs_estimated = self.max_height_before_next_jumping_abs_time \
                                                  + self.falling_time_estimated
                self.landing_vz_estimated = - math.sqrt(2.0 * self.g * (self.max_height_before_next_jumping
                                                                        - self.jumping_threshold_foot + self.foot_offset))
                self.landing_vx_estimated = x_dot
                self.landing_vy_estimated = y_dot

                self.max_height_before_next_jumping_x = hrotm[0, 3]
                self.max_height_before_next_jumping_y = hrotm[1, 3]

                self.predicted_landing_x = self.max_height_before_next_jumping_x + \
                                           self.landing_vx_estimated * self.falling_time_estimated
                self.predicted_landing_y = self.max_height_before_next_jumping_y + \
                                           self.landing_vy_estimated * self.falling_time_estimated

                error_x = self.desired_x - self.predicted_landing_x
                error_y = self.desired_y - self.predicted_landing_y

                self.desired_vx_after_jump = saturation((self.velocity_gain * error_x / self.jumping_peroid),
                                                        self.velocity_limit, -self.velocity_limit)
                self.desired_vy_after_jump = saturation((self.velocity_gain * error_y / self.jumping_peroid),
                                                        self.velocity_limit, -self.velocity_limit)

                temp = self.landing_vx_estimated * self.landing_vx_estimated + \
                       self.landing_vy_estimated * self.landing_vy_estimated + \
                       self.landing_vz_estimated * self.landing_vz_estimated - \
                       self.desired_vx_after_jump * self.desired_vx_after_jump - \
                       self.desired_vy_after_jump * self.desired_vy_after_jump
                if temp < 0:
                    temp = 0
                self.desired_vz_after_jump = math.sqrt(temp)

                V_landing = np.array([self.landing_vx_estimated, self.landing_vy_estimated, self.landing_vz_estimated])
                V_takeoff = np.array(
                    [self.desired_vx_after_jump, self.desired_vy_after_jump, self.desired_vz_after_jump])
                V_surface = np.cross(- V_landing, V_takeoff)
                V_surface_norm = np.linalg.norm(V_surface)
                V_surface = V_surface / V_surface_norm

                angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))

                desired_tilt_angle = angle_between_velocities / self.K

                desired_tilt_angle = saturation(desired_tilt_angle, 15 / 180 * math.pi, 0)
                r = R.from_rotvec(desired_tilt_angle * V_surface)
                Rotm = r.as_matrix()

                V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
                V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)

                r = R.from_rotvec(yaw * np.array([0, 0, -1]))
                Rotm = r.as_matrix()
                V_landing_desired_body = np.matmul(Rotm, V_landing_desired)

                self.roll = - math.asin(V_landing_desired_body[[1]])
                self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
                self.roll = self.roll * 180 / math.pi

            elif self.jumping_state_old == 1 and self.jumping_state == 2:
                self.max_height_before_next_jumping = 0
                self.falling_time_estimated = 0
                self.predicted_landing_x = 100  # for better show data
                self.predicted_landing_y = 100  # for better show data
                self.roll = 0
                self.pitch = 0
            elif (self.jumping_state_old == 2 or self.jumping_state_old == 1) and self.jumping_state == 3:  # take off
                self.max_height_before_next_jumping = 0
                self.falling_time_estimated = 0
                self.landing_vx_estimated = x_dot
                self.landing_vy_estimated = y_dot
                self.predicted_landing_x = 100  # for better show data
                self.predicted_landing_y = 100  # for better show data
                self.roll = 0
                self.pitch = 0

        return self.roll, self.pitch


class JumpingControllerHybrid:
    def __init__(self, foot_offset=-0.15, jumping_threshold_foot=0.15, g=9.0, k=2.6, velocity_gain=0.89,
                 velocity_limit=1, max_tilt_angle_from_velocity=15/180*math.pi):
        self.max_tilt_angle_from_velocity = max_tilt_angle_from_velocity
        self.velocity_limit = velocity_limit
        self.velocity_limit_backup = velocity_limit
        self.velocity_gain = velocity_gain
        self.velocity_gain_backup = self.velocity_gain
        self.K = k
        self.g = g
        self.foot_offset = foot_offset
        self.foot_tform_offset = get_hrotm(1, 0, 0, 0, 0, 0, foot_offset)
        self.jumping_threshold_foot = jumping_threshold_foot

        # state
        self.jumping_state = 0  # 1 for falling phase, 2 for jumping phase, 3 for falling phase, 0 for non-jumping mode
        self.jumping_state_old = 0
        self.foot_tform = get_hrotm(1, 0, 0, 0, 0, 0, 0)
        self.foot_height = 0

        self.max_height_before_next_jumping = 0
        self.max_height_before_next_jumping_abs_time = 0
        self.falling_time_estimated = 0
        self.landing_time_abs_estimated = 0
        self.landing_vz_estimated = 0
        self.max_height_before_next_jumping_x = 0
        self.max_height_before_next_jumping_y = 0

        self.landing_vx_estimated = 0
        self.landing_vy_estimated = 0
        self.predicted_landing_x = 0
        self.predicted_landing_y = 0

        self.climbing2falling_abs_time = 0
        self.climbing2falling_abs_time_old = 0
        self.jumping_peroid = 1
        self.jumping_peroid_old = 1

        self.desired_vx_after_jump = 0
        self.desired_vy_after_jump = 0
        self.desired_vz_after_jump = 0

        # desired jumping location
        self.desired_x = 0
        self.desired_y = 0

        self.roll = 0
        self.pitch = 0

        self.robot_mode = 0  # 0 for jumping, 1 for flying
        self.robot_mode_old = 0

        self.ignore_flag = False

    def update_desired_jumping_location(self, x, y):
        self.velocity_limit = self.velocity_limit_backup
        self.velocity_gain = self.velocity_gain_backup
        self.desired_x = x
        self.desired_y = y

    def update_desired_jumping_location_jump_left_and_right(self, distance):
        self.velocity_limit = 1.5
        self.velocity_gain = 1.15
        self.desired_x = 0
        if (self.jumping_state_old == 2 or self.jumping_state_old == 1) and self.jumping_state == 3:  # take off
            if self.desired_y == distance:
                self.desired_y = -distance
            elif self.desired_y == -distance:
                self.desired_y = distance
            else:
                self.desired_y = distance

    def _generate_control_input(self, yaw):
        error_x = self.desired_x - self.predicted_landing_x
        error_y = self.desired_y - self.predicted_landing_y

        self.desired_vx_after_jump = saturation((self.velocity_gain * error_x / self.jumping_peroid),
                                                self.velocity_limit, -self.velocity_limit)
        self.desired_vy_after_jump = saturation((self.velocity_gain * error_y / self.jumping_peroid),
                                                self.velocity_limit, -self.velocity_limit)
        temp = self.landing_vx_estimated * self.landing_vx_estimated + \
               self.landing_vy_estimated * self.landing_vy_estimated + \
               self.landing_vz_estimated * self.landing_vz_estimated - \
               self.desired_vx_after_jump * self.desired_vx_after_jump - \
               self.desired_vy_after_jump * self.desired_vy_after_jump
        if temp < 0:
            temp = 0
        self.desired_vz_after_jump = math.sqrt(temp)

        V_landing = np.array([self.landing_vx_estimated, self.landing_vy_estimated, self.landing_vz_estimated])
        V_takeoff = np.array(
            [self.desired_vx_after_jump, self.desired_vy_after_jump, self.desired_vz_after_jump])
        V_surface = np.cross(- V_landing, V_takeoff)
        V_surface_norm = np.linalg.norm(V_surface)
        V_surface = V_surface / V_surface_norm

        angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))

        desired_tilt_angle = angle_between_velocities / self.K

        desired_tilt_angle = saturation(desired_tilt_angle, self.max_tilt_angle_from_velocity, 0)
        r = R.from_rotvec(desired_tilt_angle * V_surface)
        Rotm = r.as_matrix()

        V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
        V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)

        r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, V_landing_desired)

        self.roll = - math.asin(V_landing_desired_body[[1]])
        self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi

    def step(self, hrotm, x_dot, y_dot, z_dot, yaw, abs_time, robot_mode):
        self.robot_mode_old = self.robot_mode
        self.robot_mode = robot_mode

        self.foot_tform = np.matmul(hrotm, self.foot_tform_offset)
        self.foot_height = self.foot_tform[2, 3]

        # update state and state_old
        self.jumping_state_old = self.jumping_state
        if self.foot_height < self.jumping_threshold_foot:  # in jumping phase
            self.jumping_state = 2  # jumping phase
            self.ignore_flag = False
        elif z_dot >= 0:
            self.jumping_state = 3  # climbing phase
        elif z_dot < 0:
            self.jumping_state = 1  # falling phase

        # main decision
        if self.ignore_flag:
            pass
        else:
            if self.robot_mode_old == 1 and self.robot_mode == 0:  # fly to jump transition
                # update
                self.falling_time_estimated = (z_dot + math.sqrt(z_dot * z_dot + 2 * self.g * (hrotm[2, 3]
                                                                   - self.jumping_threshold_foot + self.foot_offset))) / self.g
                self.landing_time_abs_estimated = abs_time + self.falling_time_estimated
                self.landing_vz_estimated = z_dot - self.g * self.falling_time_estimated
                self.landing_vx_estimated = x_dot
                self.landing_vy_estimated = y_dot
                self.predicted_landing_x = hrotm[0, 3] + x_dot * self.falling_time_estimated
                self.predicted_landing_y = hrotm[1, 3] + y_dot * self.falling_time_estimated
                self._generate_control_input(yaw)

                # reset jumping state to falling
                self.jumping_state_old = 1
                self.jumping_state = 1
                pass
                self.ignore_flag = True
                self.jumping_peroid = self.jumping_peroid_old
            elif self.robot_mode_old == 0 and self.robot_mode == 1:  # jump to fly transition
                pass
            elif self.robot_mode == 0:  # jump mode
                if self.jumping_state_old == 3 and self.jumping_state == 1:  # from climbing to falling
                    # update
                    self.climbing2falling_abs_time_old = self.climbing2falling_abs_time
                    self.climbing2falling_abs_time = abs_time
                    self.jumping_peroid_old = self.jumping_peroid
                    self.jumping_peroid = self.climbing2falling_abs_time - self.climbing2falling_abs_time_old

                    self.max_height_before_next_jumping = hrotm[2, 3]
                    self.max_height_before_next_jumping_abs_time = abs_time
                    self.falling_time_estimated = math.sqrt(2.0 * (hrotm[2, 3]
                                                                   - self.jumping_threshold_foot + self.foot_offset) / self.g)
                    self.landing_time_abs_estimated = abs_time + self.falling_time_estimated
                    self.landing_vz_estimated = - self.g * self.falling_time_estimated
                    self.landing_vx_estimated = x_dot
                    self.landing_vy_estimated = y_dot

                    self.predicted_landing_x = hrotm[0, 3] + self.landing_vx_estimated * self.falling_time_estimated
                    self.predicted_landing_y = hrotm[1, 3] + self.landing_vy_estimated * self.falling_time_estimated
                    self._generate_control_input(yaw)
                    pass
                elif self.jumping_state_old == 1 and self.jumping_state == 2:  # from falling to jumping
                    pass
                    self.roll = 0
                    self.pitch = 0
                elif (self.jumping_state_old == 2 or self.jumping_state_old == 1) and self.jumping_state == 3:  # take off
                    pass
                    self.roll = 0
                    self.pitch = 0

                # generate control input
            elif self.robot_mode == 1:  # fly mode
                self.roll = 0
                self.pitch = 0
                pass

        # if self.robot_mode == 0:
        #     # jumping mode
        #     # find the highest jumping point
        #     if self.jumping_state_old == self.jumping_state:
        #         pass
        #     else:
        #         if self.jumping_state_old == 3 and self.jumping_state == 1:  # from climbing to falling
        #             self.climbing2falling_abs_time_old = self.climbing2falling_abs_time
        #             self.climbing2falling_abs_time = abs_time
        #             self.jumping_peroid = self.climbing2falling_abs_time - self.climbing2falling_abs_time_old
        #
        #             self.max_height_before_next_jumping = hrotm[2, 3]
        #             self.max_height_before_next_jumping_abs_time = abs_time
        #             self.falling_time_estimated = math.sqrt(2.0 * (self.max_height_before_next_jumping
        #                                                            - self.jumping_threshold_foot + self.foot_offset) / self.g)
        #             self.landing_time_abs_estimated = self.max_height_before_next_jumping_abs_time \
        #                                               + self.falling_time_estimated
        #             self.landing_vz_estimated = - math.sqrt(2.0 * self.g * (self.max_height_before_next_jumping
        #                                                                     - self.jumping_threshold_foot + self.foot_offset))
        #             self.landing_vx_estimated = x_dot
        #             self.landing_vy_estimated = y_dot
        #
        #             self.max_height_before_next_jumping_x = hrotm[0, 3]
        #             self.max_height_before_next_jumping_y = hrotm[1, 3]
        #
        #             self.predicted_landing_x = self.max_height_before_next_jumping_x + \
        #                                        self.landing_vx_estimated * self.falling_time_estimated
        #             self.predicted_landing_y = self.max_height_before_next_jumping_y + \
        #                                        self.landing_vy_estimated * self.falling_time_estimated
        #
        #             error_x = self.desired_x - self.predicted_landing_x
        #             error_y = self.desired_y - self.predicted_landing_y
        #
        #             self.desired_vx_after_jump = saturation((self.velocity_gain * error_x / self.jumping_peroid),
        #                                                     self.velocity_limit, -self.velocity_limit)
        #             self.desired_vy_after_jump = saturation((self.velocity_gain * error_y / self.jumping_peroid),
        #                                                     self.velocity_limit, -self.velocity_limit)
        #
        #             temp = self.landing_vx_estimated * self.landing_vx_estimated + \
        #                    self.landing_vy_estimated * self.landing_vy_estimated + \
        #                    self.landing_vz_estimated * self.landing_vz_estimated - \
        #                    self.desired_vx_after_jump * self.desired_vx_after_jump - \
        #                    self.desired_vy_after_jump * self.desired_vy_after_jump
        #             if temp < 0:
        #                 temp = 0
        #             self.desired_vz_after_jump = math.sqrt(temp)
        #
        #             V_landing = np.array([self.landing_vx_estimated, self.landing_vy_estimated, self.landing_vz_estimated])
        #             V_takeoff = np.array(
        #                 [self.desired_vx_after_jump, self.desired_vy_after_jump, self.desired_vz_after_jump])
        #             V_surface = np.cross(- V_landing, V_takeoff)
        #             V_surface_norm = np.linalg.norm(V_surface)
        #             V_surface = V_surface / V_surface_norm
        #
        #             angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))
        #
        #             desired_tilt_angle = angle_between_velocities / self.K
        #
        #             desired_tilt_angle = saturation(desired_tilt_angle, self.max_tilt_angle_from_velocity, 0)
        #             r = R.from_rotvec(desired_tilt_angle * V_surface)
        #             Rotm = r.as_matrix()
        #
        #             V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
        #             V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)
        #
        #             r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        #             Rotm = r.as_matrix()
        #             V_landing_desired_body = np.matmul(Rotm, V_landing_desired)
        #
        #             self.roll = - math.asin(V_landing_desired_body[[1]])
        #             self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        #             self.roll = self.roll * 180 / math.pi
        #
        #         elif self.jumping_state_old == 1 and self.jumping_state == 2:
        #             self.max_height_before_next_jumping = 0
        #             self.falling_time_estimated = 0
        #             self.predicted_landing_x = 100  # for better show data
        #             self.predicted_landing_y = 100  # for better show data
        #             self.roll = 0
        #             self.pitch = 0
        #         elif (self.jumping_state_old == 2 or self.jumping_state_old == 1) and self.jumping_state == 3:  # take off
        #             self.max_height_before_next_jumping = 0
        #             self.falling_time_estimated = 0
        #             self.landing_vx_estimated = x_dot
        #             self.landing_vy_estimated = y_dot
        #             self.predicted_landing_x = 100  # for better show data
        #             self.predicted_landing_y = 100  # for better show data
        #             self.roll = 0
        #             self.pitch = 0

        return self.roll, self.pitch


class JumpingControllerSeparated:
    def __init__(self, foot_offset=-0.24, jumping_threshold_foot=0.1, g=8.3, k=2.48,
                 velocity_gain=0.8, velocity_limit=1.0, c_r=0.6, thrust_ratio=8.0, max_drift_angle=40):
        self.pitch_gain = 1
        self.velocity_limit = velocity_limit
        self.velocity_gain = velocity_gain
        self.K = k
        self.g = g
        self.foot_offset = foot_offset  # negative length of leg
        self.foot_tform_offset = get_hrotm(1, 0, 0, 0, 0, 0, foot_offset)
        self.jumping_threshold_foot = jumping_threshold_foot
        self.c_r = c_r
        self.thrust_ratio = thrust_ratio
        self.max_drift_angle = max_drift_angle

        self.pitch = 0
        self.roll = 0

        self.pitch_3d = 0
        self.roll_3d = 0

        self.foot_height = 0
        self.jumping_state_old = 0
        self.jumping_state = 0  # 1 for falling phase, 2 for jumping phase, 3 for falling phase, 0 for non-jumping mode
        # self.jumping_state_2 = 0
        # self.jumping_state_2_old = 0

        self.one_step_K_tuning_flag = False
        self.k_update = k
        self.one_step_max_drift_angle_tuning_flag = False
        self.max_drift_angle_update = max_drift_angle

        # bias correction matrix
        roll_bias = 0
        pitch_bias = 0
        Rotx = np.matrix([[1, 0, 0],
                          [0, math.cos(roll_bias), - math.sin(roll_bias)],
                          [0, math.sin(roll_bias), + math.cos(roll_bias)]])
        Roty = np.matrix([[+ math.cos(pitch_bias), 0, math.sin(pitch_bias)],
                          [0, 1, 0],
                          [- math.sin(pitch_bias), 0, math.cos(pitch_bias)]])
        self.bias_rotm = np.matmul(Roty, Rotx)

    def update_bias_rotm(self, roll_bias, pitch_bias):
        Rotx = np.matrix([[1, 0, 0],
                          [0, math.cos(roll_bias), - math.sin(roll_bias)],
                          [0, math.sin(roll_bias), + math.cos(roll_bias)]])
        Roty = np.matrix([[+ math.cos(pitch_bias), 0, math.sin(pitch_bias)],
                          [0, 1, 0],
                          [- math.sin(pitch_bias), 0, math.cos(pitch_bias)]])
        self.bias_rotm = np.matmul(Roty, Rotx)

    def landing_state_prediction(self, x_0, y_0, z_0, x_dot, y_dot, z_dot,
                                 A, B, C, x_s, y_s, z_s):

        x_s = x_s + A * - self.foot_offset
        y_s = y_s + B * - self.foot_offset
        z_s = z_s + C * - self.foot_offset

        A2 = A * A
        B2 = B * B
        C2 = C * C
        AC = A * C
        BC = B * C
        AB = A * B
        Cg = C * self.g
        C2g = C2 * self.g
        x_dot2 = x_dot * x_dot
        y_dot2 = y_dot * y_dot
        z_dot2 = z_dot * z_dot
        ACg = AC * self.g
        BCg = BC * self.g

        temp = A2*x_dot2 + B2*y_dot2 + C2*z_dot2 + 2.0*C2g*z_0 - 2.0*C2g*z_s + 2.0*ACg*x_0 - 2.0*ACg*x_s + 2.0*BCg*y_0 - 2.0*BCg*y_s + 2.0*AB*x_dot*y_dot + 2.0*AC*x_dot*z_dot + 2.0*BC*y_dot*z_dot
        if temp > 0:
            sqrtb24ac = math.sqrt(temp)

            landing_time_1 = (A*x_dot + B*y_dot + C*z_dot + sqrtb24ac) / Cg
            # landing_time_2 = (A*x_dot + B*y_dot + C*z_dot - sqrtb24ac)/Cg

            x_n = x_dot * landing_time_1 + x_0
            y_n = y_dot * landing_time_1 + y_0
            z_n = -0.5 * self.g * landing_time_1 * landing_time_1 + z_dot * landing_time_1 + z_0

            v_xn = x_dot
            v_yn = y_dot
            v_zn = z_dot - self.g * landing_time_1

            return landing_time_1, x_n, y_n, z_n, v_xn, v_yn, v_zn
        else:
            return -1, x_0, y_0, z_0, x_dot, y_dot, z_dot

    def jumping_planning(self, x_n, y_n, z_n, z_p, x_d, y_d, z_d):
        # z_p is desired jumping altitude
        square_root_2 = math.sqrt(2)
        time_climb = square_root_2 * math.sqrt(z_p / self.g)
        time_fall = square_root_2 * math.sqrt((2 * z_n - 2 * z_d + 2 * z_p) / (2 * self.g))
        time_plan = time_climb + time_fall
        u_x = (x_d - x_n) / time_plan
        u_y = (y_d - y_n) / time_plan
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)
        if norm_v > self.velocity_limit:
            u_x = self.velocity_limit * u_x / norm_v
            u_y = self.velocity_limit * u_y / norm_v
        u_z = self.g * time_climb

        return u_x, u_y, u_z

    def jumping_planning_velocity_generation(self, z_p, u_x, u_y):
        time_climb = math.sqrt(2) * math.sqrt(z_p / self.g)
        u_z = self.g * time_climb
        return u_x, u_y, u_z

    def jumping_altitude_control(self, u_x, u_y, u_z, z_n, v_xn, v_yn, v_zn):
        u_x2 = u_x * u_x
        u_y2 = u_y * u_y
        u_z2 = u_z * u_z
        z_off = z_n + \
                (u_z * ((u_x2 + u_y2 + u_z2) / 2 - (self.c_r * (v_xn * v_xn + v_yn * v_yn + v_zn * v_zn)) / 2)) / \
                (self.thrust_ratio * math.sqrt(u_x2 + u_y2 + u_z2))
        return z_off

    def inverse_jumping_model(self, v_xn, v_yn, v_zn, u_x, u_y, u_z, yaw):
        # inversie jumping model
        V_landing = np.array([v_xn, v_yn, v_zn])
        u_x = u_x * self.velocity_gain
        u_y = u_y * self.velocity_gain
        V_takeoff = np.array([u_x, u_y, u_z])
        V_surface = np.cross(- V_landing, V_takeoff)
        V_surface_norm = np.linalg.norm(V_surface)
        V_surface = V_surface / V_surface_norm

        angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))
        if self.one_step_K_tuning_flag:
            desired_tilt_angle = angle_between_velocities / self.k_update

            self.one_step_K_tuning_flag = False
            print('control: ', self.k_update, desired_tilt_angle )
        else:
            desired_tilt_angle = angle_between_velocities / self.K
        # print(desired_tilt_angle)
        if self.one_step_max_drift_angle_tuning_flag:
            desired_tilt_angle = saturation(desired_tilt_angle, self.max_drift_angle_update / 180 * math.pi, 0)
            self.one_step_max_drift_angle_tuning_flag = False
        else:
            desired_tilt_angle = saturation(desired_tilt_angle, self.max_drift_angle / 180 * math.pi, 0)
        r = R.from_rotvec(desired_tilt_angle * V_surface)
        Rotm = r.as_matrix()

        V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
        V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)

        V_landing_desired = np.matmul(self.bias_rotm, V_landing_desired)  # bias correction

        # attitude regulator
        r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, V_landing_desired)

        self.roll = - math.asin(V_landing_desired_body[[1]])
        self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi

    def inverse_jumping_model_3D(self, v_xn, v_yn, v_zn, u_x, u_y, u_z, yaw):
        V_landing = np.array([v_xn, v_yn, v_zn])
        u_x = u_x * self.velocity_gain
        u_y = u_y * self.velocity_gain
        V_takeoff = np.array([u_x, u_y, u_z])
        V_surface = np.cross(- V_landing, V_takeoff)
        V_surface_norm = np.linalg.norm(V_surface)
        V_surface = V_surface / V_surface_norm

        angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))

        sym_x = np.linalg.norm(V_landing)
        sym_z = angle_between_velocities * 180 / math.pi
        desired_tilt_angle = (102078514122806912*sym_x)/2503093013621569 - (1152921504606846976*((1516793839585586393547591649423839*sym_x*sym_x)/20769187434139310514121985316880384 + (4999336199642260733107015097027571*sym_x)/5192296858534827628530496329220096 - (7509279040864707*sym_z)/576460752303423488 + 17386743509150727952144082147562133/5192296858534827628530496329220096)**(1/2))/7509279040864707 + 2108269719205169920/7509279040864707
        desired_tilt_angle = desired_tilt_angle / 180 * math.pi
        # print(desired_tilt_angle)
        desired_tilt_angle = saturation(desired_tilt_angle, self.max_drift_angle / 180 * math.pi, 0)

        r = R.from_rotvec(desired_tilt_angle * V_surface)
        Rotm = r.as_matrix()

        V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
        V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)

        # attitude regulator
        r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, V_landing_desired)

        self.roll_3d = - math.asin(V_landing_desired_body[[1]])
        self.pitch_3d = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll_3d)) * 180 / math.pi
        self.roll_3d = self.roll_3d * 180 / math.pi

    def step(self, hrotm,  z_dot, ):
        foot_tform = np.matmul(hrotm, self.foot_tform_offset)
        self.foot_height = foot_tform[2, 3]

        self.jumping_state_old = self.jumping_state
        if self.foot_height < self.jumping_threshold_foot:  # in jumping phase
            self.jumping_state = 2  # jumping phase
            self.ignore_flag = False
        elif z_dot >= 0:
            self.jumping_state = 3  # climbing phase


        elif z_dot < 0:
            self.jumping_state = 1  # falling phase

    def step_onboard(self, z, z_dot, ):

        # foot_tform = np.matmul(hrotm, self.foot_tform_offset)
        self.foot_height = z + self.foot_offset

        self.jumping_state_old = self.jumping_state
        if self.foot_height < self.jumping_threshold_foot:  # in jumping phase
            self.jumping_state = 2  # jumping phase
            self.ignore_flag = False
        elif z_dot >= 0:
            self.jumping_state = 3  # climbing phase
        elif z_dot < 0:
            self.jumping_state = 1  # falling phase

    def one_step_K_tuning(self, k):
        self.one_step_K_tuning_flag = True
        self.k_update = k

    def one_step_max_drift_angle_tuning(self, max_drift_angle):
        self.one_step_max_drift_angle_tuning_flag = True
        self.max_drift_angle_update = max_drift_angle


def _vectors_angle(u, v):
    return np.arctan2(np.linalg.norm(np.cross(u,v)),np.dot(u,v))


def inverse_transformarion(landing_speed, theta_v):
    sym_x = landing_speed
    sym_z = theta_v
    theta_l = (41212670110314176 * sym_x) / 1230263675033725 - (576460752303423488 * (
                (974403269862014920054241043161349 * sym_x ** 2) / 20769187434139310514121985316880384 + (
                    8672019878050132728313746427250193 * sym_x) / 10384593717069655257060992658440192 - (
                            3690791025101175 * sym_z) / 288230376151711744 + 316666246617286875401830049979054579 / 83076749736557242056487941267521536) ** (1 / 2)) / 3690791025101175 + 375172374703519232 / 1230263675033725
    sym_y = theta_l
    takeoff_speed = (6705375265762309 * sym_x ** 2) / 288230376151711744 + (
            3543626747540235 * sym_x * sym_y) / 9223372036854775808 + (
                            7375759309974461 * sym_x) / 9007199254740992 + (
                            3713229464970937 * sym_y ** 2) / 73786976294838206464 - (
                            1805822083827939 * sym_y) / 2305843009213693952 + 2303148103199417 / 18014398509481984
    theta_t = (4094059185447651 * sym_y) / 4503599627370496 - (5617120889527667 * sym_x) / 4503599627370496 + (
            7280283997844195 * sym_x * sym_y) / 36028797018963968 + (
                      5588393766433539 * sym_x ** 2) / 18014398509481984 - (
                      5498328034157103 * sym_y ** 2) / 2305843009213693952 + 2292625160111515 / 2251799813685248

    return theta_l, takeoff_speed, theta_t


class JumpingControllerSolved:
    def __init__(self, foot_offset=-0.22, jumping_threshold_foot=0.1, g=8.3, ):
        self.g = g
        self.foot_offset = foot_offset  # negative length of leg
        self.l_0 = - foot_offset
        self.foot_tform_offset = get_hrotm(1, 0, 0, 0, 0, 0, foot_offset)
        self.jumping_threshold_foot = jumping_threshold_foot

        self.pitch = 0
        self.roll = 0

        self.foot_height = 0
        self.jumping_state_old = 0
        self.jumping_state = 0  # 1 for falling phase, 2 for jumping phase, 3 for falling phase, 0 for non-jumping mode

        self.lateral_error_imag = 0.0
        self.altitude_error = 0
        self.z_off = self.l_0
        self.theta_l = 0
        self.theta_v = 0
        self.theta_t = 0
        Rotm = np.array([[1, 0,  0],
                         [0,  1,  0],
                         [0,  0,  1]])
        self.z_bl = np.matmul(Rotm, np.array([0, 0, 1]).T)

        self.roll = 0
        self.pitch = 0

        # landing state prediction
        self.t_f = 0.1
        self.x_l_hat = 0
        self.y_l_hat = 0
        self.z_l_hat = 0
        self.x_l_dot_hat = 0
        self.y_l_dot_hat = 0
        self.z_l_dot_hat = 0

        self.takeoff_velocity = np.array([0, 0, 0])

    def landing_state_prediction(self, x_0, y_0, z_0, x_dot, y_dot, z_dot,):
        self.t_f = np.sqrt((2 * (z_0 - self.l_0)) / self.g)
        self.x_l_hat = x_dot * self.t_f + x_0
        self.y_l_hat = y_dot * self.t_f + y_0
        self.z_l_hat = self.l_0

        self.x_l_dot_hat = x_dot
        self.y_l_dot_hat = y_dot
        self.z_l_dot_hat = z_dot - self.g * self.t_f

    def lateral_error(self, n_z, x_l_d, y_l_d, z_d):

        # inverse jumping model
        landing_velocity = np.array([self.x_l_dot_hat, self.y_l_dot_hat, self.z_l_dot_hat])
        landing_location = np.array([self.x_l_hat, self.y_l_hat, self.z_l_hat])
        landing_location_desired = np.array([x_l_d, y_l_d, self.l_0])

        error_lateral = (landing_location_desired - landing_location)
        n_t = error_lateral + (np.array([0, 0, 1]) * n_z)
        v = np.cross(- landing_velocity, n_t)
        z_j = - landing_velocity
        x_j = np.cross(v, - landing_velocity)
        theta_v = _vectors_angle(z_j, n_t) * 180/ math.pi
        landing_speed = np.linalg.norm(landing_velocity)
        theta_l, takeoff_speed, theta_t = inverse_transformarion(landing_speed, theta_v)

        # trajectory planner
        self.takeoff_velocity = n_t / np.linalg.norm(n_t) * takeoff_speed
        vertical_takeoff_speed = self.takeoff_velocity[2]
        unpowered_altitude = 0.5 * (vertical_takeoff_speed ** 2) / self.g

        if unpowered_altitude >= z_d:
            t_unpowered = 2 * vertical_takeoff_speed / self.g
            lateral_distance_unpowered = t_unpowered * np.linalg.norm(self.takeoff_velocity[0:2])
            error_imag = lateral_distance_unpowered - np.linalg.norm(error_lateral)

            altitude_error = z_d - unpowered_altitude
            z_off = 0
        else:
            t_falling = np.sqrt(2 * z_d / self.g)
            t_p_result = (- vertical_takeoff_speed**2/(2*self.g) + z_d)/vertical_takeoff_speed
            t_up = vertical_takeoff_speed / self.g
            t_all = t_p_result + t_up + t_falling
            lateral_distance_powered = t_all * np.linalg.norm(self.takeoff_velocity[0:2])
            error_imag = lateral_distance_powered - np.linalg.norm(error_lateral)

            altitude_error = 0
            z_off = vertical_takeoff_speed * ((- vertical_takeoff_speed**2/(2*self.g) + z_d)/vertical_takeoff_speed)

        lateral_error_imag = abs(error_imag)
        landing_velocity_n_unit = -landing_velocity / np.linalg.norm(landing_velocity)
        # z_bl = axang2rotm([v;theta_l / 180 * pi]')*landing_velocity_n_unit;

        r = R.from_rotvec(theta_l/180*math.pi * (v/np.linalg.norm(v)))
        Rotm = r.as_matrix()

        z_bl = np.matmul(Rotm, np.array([landing_velocity_n_unit]).T)

        self.altitude_error = altitude_error
        self.z_off = z_off
        self.theta_l = theta_l
        self.theta_v = theta_v
        self.theta_t = theta_t
        self.z_bl = z_bl

        # return lateral_error_imag, altitude_error, z_off, theta_l, theta_v, theta_t, z_bl

        return lateral_error_imag

    def solve_result(self):
        return self.altitude_error, self.z_off, self.theta_l, self.theta_v, self.theta_t, self.z_bl

    def control_output(self, yaw):
        # attitude regulator
        r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, self.z_bl)

        self.roll = - math.asin(V_landing_desired_body[[1]])
        self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi


class JumpingStateTracker:
    def __init__(self, foot_offset=-0.3, jumping_threshold_foot=0.1):
        self.jumping_threshold_foot = jumping_threshold_foot
        self.foot_offset = foot_offset
        self.foot_tform_offset = get_hrotm(1, 0, 0, 0, 0, 0, foot_offset)
        self.foot_tform_offset_up = get_hrotm(1, 0, 0, 0, 0, 0, 0.31)
        self.foot_height = 0
        self.foot_height_up = 0
        self.jumping_state_old = 0
        self.jumping_state = 0
        # 1 for falling phase, 2 for jumping phase, 3 for climbing phase, 0 for flight mode

    def step(self, hrotm,  z_dot, ):
        foot_tform = np.matmul(hrotm, self.foot_tform_offset)
        foot_tform_up = np.matmul(hrotm, self.foot_tform_offset_up)
        self.foot_height = foot_tform[2, 3]
        self.foot_height_up = foot_tform_up[2, 3]
        # print('leg pos:',self.foot_height,'threshold:',self.jumping_threshold_foot,'vz:',z_dot)
        self.jumping_state_old = self.jumping_state
        if self.foot_height < self.jumping_threshold_foot or self.foot_height_up < self.jumping_threshold_foot:  # in jumping phase
            self.jumping_state = 2  # jumping phase
        # elif z_dot > 0.1 or self.jumping_state_old == 2:
        elif z_dot > 0.1:
            self.jumping_state = 3  # climbing phase
        elif z_dot < -0.1:
            self.jumping_state = 1  # falling phase
        # print('state:',self.jumping_state)
    # def step_down(self, hrotm,  z_dot, ):
    #     foot_tform = np.matmul(hrotm, - self.foot_tform_offset)
    #     self.foot_height = foot_tform[2, 3] - 0.7

    #     self.jumping_state_old = self.jumping_state
    #     if self.foot_height < self.jumping_threshold_foot:  # in jumping phase
    #         self.jumping_state = 2  # jumping phase
    #     elif z_dot >= 0:
    #         self.jumping_state = 3  # climbing phase
    #     elif z_dot < 0:
    #         self.jumping_state = 1  # falling phase
    
    def step_down(self, Z_f,  z_dot, ):
        self.foot_height = Z_f - 0.35

        self.jumping_state_old = self.jumping_state
        if self.foot_height < self.jumping_threshold_foot:  # in jumping phase
            self.jumping_state = 2  # jumping phase
        elif z_dot >= 0:
            self.jumping_state = 3  # climbing phase
        elif z_dot < 0:
            self.jumping_state = 1  # falling phase


class JumpingControllerSeparated2:
    def __init__(self, foot_offset=-0.24, jumping_threshold_foot=0.1, g=8.3, k=2.48,
                 velocity_gain=0.8, velocity_limit=1.0, c_r=0.6, thrust_ratio=8.0, max_drift_angle=40):
        self.g = g
        self.foot_offset = foot_offset  # negative length of leg
        self.landing_time = 0
        self.landing_x = 0
        self.landing_y = 0
        self.landing_z = 0
        self.landing_x_dot = 0
        self.landing_y_dot = 0
        self.landing_z_dot = 0

        self.desired_x = 0
        self.desired_y = 0
        self.desired_z = 0
        self.desired_jumping_altitude = 0

        self.velocity_limit = velocity_limit

        self.takeoff_x_dot = 0
        self.takeoff_y_dot = 0
        self.takeoff_z_dot = 1.5

        self.velocity_gain = velocity_gain
        self.K = k
        self.max_drift_angle = max_drift_angle
        self.roll = 0
        self.pitch = 0
        # self.pitch_gain = 1
        # self.velocity_limit = velocity_limit
        # self.velocity_gain = velocity_gain
        # self.K = k
        #
        # self.foot_offset = foot_offset  # negative length of leg
        # self.foot_tform_offset = get_hrotm(1, 0, 0, 0, 0, 0, foot_offset)
        # self.jumping_threshold_foot = jumping_threshold_foot
        # self.c_r = c_r
        # self.thrust_ratio = thrust_ratio
        # self.max_drift_angle = max_drift_angle
        #
        # self.pitch = 0
        # self.roll = 0
        #
        # self.pitch_3d = 0
        # self.roll_3d = 0
        #
        # self.foot_height = 0
        # self.jumping_state_old = 0
        # self.jumping_state = 0  # 1 for falling phase, 2 for jumping phase, 3 for falling phase, 0 for non-jumping mode
        # # self.jumping_state_2 = 0
        # # self.jumping_state_2_old = 0
        #
        # self.one_step_K_tuning_flag = False
        # self.k_update = k
        # self.one_step_max_drift_angle_tuning_flag = False
        # self.max_drift_angle_update = max_drift_angle
        #
        # # bias correction matrix
        # roll_bias = 0
        # pitch_bias = 0
        # Rotx = np.matrix([[1, 0, 0],
        #                   [0, math.cos(roll_bias), - math.sin(roll_bias)],
        #                   [0, math.sin(roll_bias), + math.cos(roll_bias)]])
        # Roty = np.matrix([[+ math.cos(pitch_bias), 0, math.sin(pitch_bias)],
        #                   [0, 1, 0],
        #                   [- math.sin(pitch_bias), 0, math.cos(pitch_bias)]])
        # self.bias_rotm = np.matmul(Roty, Rotx)

#  t_1 = z_dot/9.81

#         z_peak = z_dot * t_1 - 0.5*9.81*t_1**2

#         # y_peak = y_dot*t_1


#         # acc_x = -landing_att_x * acc
#         # acc_y = -landing_att_y * acc
#         # acc_z = 9.81+abs(landing_att_z * acc)

#         a = 1
#         d_e = desired_e - total_e

#         if d_e < 0:
#             s = 0
#         else:
#             s = d_e/a

#         if s > z_peak:
#             s = z_peak
    def landing_state_prediction_downwards(self, x_0, y_0, z_0, x_dot, y_dot, z_dot,landing_att_x,landing_att_y,landing_att_z,acc):
    
        t_1 = z_dot/9.81

        z_peak = z_dot * t_1 - 0.5*9.81*t_1**2

        # y_peak = y_dot*t_1


        # acc_x = -landing_att_x * acc
        # acc_y = -landing_att_y * acc
        # acc_z = 9.81+abs(landing_att_z * acc)

        
        t_2    = math.sqrt((2*z_peak)/9.81)

        vx_predict     = x_dot 
        vy_predict     = y_dot 

        y_predict = y_0 + y_dot * (t_1 + t_2)
        x_predict = x_0 + x_dot * (t_1 + t_2)

        self.landing_x = x_predict
        self.landing_y = y_predict
        self.landing_z = z_peak

        self.landing_x_dot = vx_predict
        self.landing_y_dot = vy_predict
        self.landing_z_dot = -9.81*t_2
        self.landing_time = t_1 + t_2







    def landing_state_prediction(self, x_0, y_0, z_0, x_dot, y_dot, z_dot,
                                 A, B, C, x_s, y_s, z_s):

        x_s = x_s + A * - self.foot_offset
        y_s = y_s + B * - self.foot_offset
        z_s = z_s + C * - self.foot_offset

        A2 = A * A
        B2 = B * B
        C2 = C * C
        AC = A * C
        BC = B * C
        AB = A * B
        Cg = C * self.g
        C2g = C2 * self.g
        x_dot2 = x_dot * x_dot
        y_dot2 = y_dot * y_dot
        z_dot2 = z_dot * z_dot
        ACg = AC * self.g
        BCg = BC * self.g

        temp = A2 * x_dot2 + B2 * y_dot2 + C2 * z_dot2 + 2.0 * C2g * z_0 - 2.0 * C2g * z_s + 2.0 * ACg * x_0 - 2.0 * ACg * x_s + 2.0 * BCg * y_0 - 2.0 * BCg * y_s + 2.0 * AB * x_dot * y_dot + 2.0 * AC * x_dot * z_dot + 2.0 * BC * y_dot * z_dot

        if temp > 0:
            sqrtb24ac = math.sqrt(temp)

            landing_time_1 = (A * x_dot + B * y_dot + C * z_dot + sqrtb24ac) / Cg

            self.landing_x = x_dot * landing_time_1 + x_0
            self.landing_y = y_dot * landing_time_1 + y_0
            self.landing_z = -0.5 * self.g * landing_time_1 * landing_time_1 + z_dot * landing_time_1 + z_0

            self.landing_x_dot = x_dot
            self.landing_y_dot = y_dot
            self.landing_z_dot = z_dot - self.g * landing_time_1
            self.landing_time = landing_time_1
        else:
            self.landing_x = x_0
            self.landing_y = y_0
            self.landing_z = z_0

            self.landing_x_dot = x_dot
            self.landing_y_dot = y_dot
            self.landing_z_dot = z_dot
            self.landing_time = -1

    def set_reference(self, desired_x, desired_y, desired_z, desired_jumping_altitude,):
        self.desired_x = desired_x
        self.desired_y = desired_y
        self.desired_z = desired_z
        self.desired_jumping_altitude = desired_jumping_altitude

    def jumping_planning(self, ):
        square_root_2 = math.sqrt(2)
        time_climb = square_root_2 * math.sqrt(self.desired_jumping_altitude / self.g)
        time_fall = square_root_2 * math.sqrt((2 * self.landing_z - 2 * self.desired_z + 2 * self.desired_jumping_altitude) / (2 * self.g))
        time_plan = time_climb + time_fall
        u_x = (self.desired_x - self.landing_x) / time_plan
        u_y = (self.desired_y - self.landing_y) / time_plan
        norm_v = math.sqrt(u_x * u_x + u_y * u_y)
        if norm_v > self.velocity_limit:
            u_x = self.velocity_limit * u_x / norm_v
            u_y = self.velocity_limit * u_y / norm_v

        self.takeoff_x_dot = u_x
        self.takeoff_y_dot = u_y
        self.takeoff_z_dot = self.g * time_climb

    def land_attitude_v_to_roll_pitch(self, landing_att_x, landing_att_y, landing_att_z, yaw):
        V_landing_desired = np.array([[landing_att_x],[landing_att_y],[landing_att_z]])
        r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, V_landing_desired)

        self.roll = - math.asin(V_landing_desired_body[[1]])
        self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi

    def inverse_jumping_model(self, yaw):
        # inversie jumping model
        V_landing = np.array([self.landing_x_dot, self.landing_y_dot, self.landing_z_dot])
        u_x = self.takeoff_x_dot * self.velocity_gain
        u_y = self.takeoff_y_dot * self.velocity_gain
        V_takeoff = np.array([u_x, u_y, self.takeoff_z_dot])
        V_surface = np.cross(- V_landing, V_takeoff)
        V_surface_norm = np.linalg.norm(V_surface)
        V_surface = V_surface / V_surface_norm

        angle_between_velocities = math.atan2(V_surface_norm, np.dot(- V_landing, V_takeoff))

        desired_tilt_angle = angle_between_velocities / self.K
        # print(desired_tilt_angle)
        desired_tilt_angle = saturation(desired_tilt_angle, self.max_drift_angle / 180 * math.pi, 0)
        r = R.from_rotvec(desired_tilt_angle * V_surface)
        Rotm = r.as_matrix()

        V_landing_desired = np.matmul(Rotm, np.array([-V_landing]).T)
        V_landing_desired = V_landing_desired / np.linalg.norm(V_landing)

        # V_landing_desired = np.matmul(self.bias_rotm, V_landing_desired)  # bias correction

        # attitude regulator
        r = R.from_rotvec(yaw * np.array([0, 0, -1]))
        Rotm = r.as_matrix()
        V_landing_desired_body = np.matmul(Rotm, V_landing_desired)
        print(V_landing_desired_body)
        self.roll = - math.asin(V_landing_desired_body[[1]])
        self.pitch = math.asin(V_landing_desired_body[[0]] / math.cos(self.roll)) * 180 / math.pi
        self.roll = self.roll * 180 / math.pi

        return V_landing_desired
