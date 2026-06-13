import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import RegularGridInterpolator



class JumpingHeightController2:
    def __init__(self, leg_efficiency=0.8, g=9.81, t_p_low=0.04, t_p_high=0.3, thrust_gain=1):
        self.thrust_gain = thrust_gain
        self.t_p_high = t_p_high
        self.t_p_low = t_p_low
        self.g = g
        self.leg_efficiency = leg_efficiency

    def update(self, h, desired_h, ):
        if h < 0:
            h = 0
        take_off_speed = math.sqrt(2 * self.g * h * self.leg_efficiency)
        if desired_h < h:
            powered_climbing_time = self.t_p_low
        else:
            if take_off_speed * self.thrust_gain <= 0:
                powered_climbing_time = self.t_p_low
            else:
                powered_climbing_time = (desired_h - h)/(take_off_speed * self.thrust_gain)
            if powered_climbing_time > self.t_p_high:
                powered_climbing_time = self.t_p_high
            if powered_climbing_time < self.t_p_low:
                powered_climbing_time = self.t_p_low
        return powered_climbing_time


class JumpingStateTrackerLog:
    def __init__(self, ):
        self.jumping_state = False  # aerial phase
        self.jumping_state_old = False

        self.takeoff_flag = False
        self.landing_flag = False

    def update(self, log_jumping_state):
        self.jumping_state_old = self.jumping_state
        self.jumping_state = log_jumping_state
        self.takeoff_flag = False
        self.landing_flag = False

        # aerial to stance
        if self.jumping_state and not self.jumping_state_old:
            self.jumping_state = True  # stance phase
            self.landing_flag = True

        # stance to aerial
        if self.jumping_state_old and not self.jumping_state:
            self.jumping_state = False  # aerial phase
            self.takeoff_flag = True

    def init(self, ):
        self.jumping_state = False
        self.jumping_state_old = False
        self.takeoff_flag = False
        self.landing_flag = False


# old method, to be removed
class JumpingStateTrackerOffBoard:
    def __init__(self, acc_z_up_limit=2.0, ):
        self._acc_z_up_limit = acc_z_up_limit
        self._acc_z_delay = acc_z_up_limit / 2
        self.jumping_state = False  # aerial phase
        self.jumping_state_old = False

        self.takeoff_flag = False
        self.landing_flag = False

    def update(self, acc_z):
        self.jumping_state_old = self.jumping_state
        self.takeoff_flag = False
        self.landing_flag = False

        # aerial to stance
        if self._acc_z_delay < self._acc_z_up_limit < acc_z:
            self.jumping_state = True  # stance phase
            self.landing_flag = True

        # stance to aerial
        if acc_z < self._acc_z_up_limit < self._acc_z_delay:
            self.jumping_state = False  # aerial phase
            self.takeoff_flag = True

        self._acc_z_delay = acc_z

    def init(self, ):
        self.jumping_state = False
        self.jumping_state_old = False
        self.takeoff_flag = False
        self.landing_flag = False


class JumpingHeightRecorder2:
    def __init__(self, ):
        self._h_min = 0
        self._h_max = 0.4
        self.jumping_height = self._h_max - self._h_min

    def update(self, h, jumping_state, landing_flag):
        if landing_flag:
            self.jumping_height = self._h_max - self._h_min

        if jumping_state:  # stance phase
            self._h_min = h
            self._h_max = h
        else:
            if h > self._h_max:
                self._h_max = h


class PoweredClimbingTimer2:
    def __init__(self, control_waitting_cycles=3):
        self.takeoff_timestamp = -1
        self.powered_climbing_on = False
        self.powered_climbing_on2off = False
        self.powered_climbing_on2off_delay = False
        self._waitting_cycles = control_waitting_cycles
        self._cycle_recorder = control_waitting_cycles
        self.control_flag = True
        self.control_flag_off2on = False

    def update(self, jumping_state, takeoff_flag, abs_time, power_time):
        self.powered_climbing_on2off_delay = self.powered_climbing_on2off
        self.powered_climbing_on2off = False
        self.control_flag_off2on = False

        if takeoff_flag:  # takeoff timestamp
            self.takeoff_timestamp = abs_time
            self.powered_climbing_on = True
            self._cycle_recorder = self._waitting_cycles
            self.control_flag = False

        elif jumping_state:  # stance phase
            self.powered_climbing_on = False
            self.control_flag = False

        elif not jumping_state:  # aerial phase
            if self.powered_climbing_on and (abs_time - self.takeoff_timestamp > power_time):
                self.powered_climbing_on = False
                self.powered_climbing_on2off = True

            if not self.powered_climbing_on:
                self._cycle_recorder = self._cycle_recorder - 1
                if self._cycle_recorder < 0:
                    self.control_flag = True
                    self.control_flag_off2on = True


class JumpingStateEstimatorLegMeasurement:
    def __init__(self, leg_length_0, P_LD, P_TO, max_contraction, min_contraction, tof_zero,
                 attitude_correction_gain=0.1, max_iteration=10):
        # constants
        self.max_contraction = max_contraction
        self.min_contraction = min_contraction
        self.tof_zero = tof_zero
        self.P_LD = P_LD
        self.P_TO = P_TO
        self.leg_length_0 = leg_length_0
        self.model_order = len(P_TO) - 1

        self.e_3_g = np.array([0, 0, 9.81])
        self.direction_error_limit = 0.5
        self.direction_gain = 0.2
        self.buffer_length = 500
        self.attitude_correction_gain = attitude_correction_gain
        self.max_iteration = max_iteration

        self.R_c_0 = np.eye(3)

        # variables
        self.buffer_Abi = np.zeros((3, self.buffer_length), dtype=float)
        self.buffer_t = np.zeros(self.buffer_length, dtype=float)
        self.buffer_R = np.stack([np.eye(3)] * self.buffer_length)
        self.buffer_index = -1

        self.delta_l_max_old = 0.1
        self.omega_mean_old = np.array([0, 0, 0])

        self.R_LD = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_LD_old = self.R_LD
        self.R_TO = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_TO_old = self.R_TO
        self.landing_location_prediction = np.array([0, 0, 0])
        self.current_position = np.array([0, 0, 0])
        self.current_velocity = np.array([0, 0, 3])
        self.height = 0.458
        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.correction_matrix = np.eye(3)
        self.quat_corr = Rotation.from_rotvec(
            Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
            canonical=False)

        self.logging_list = ['JSELM_vx', 'JSELM_vy', 'JSELM_vz', 'JSELM_px', 'JSELM_py', 'JSELM_pz', 'JSELM_iteration_count', 'JSELM_height']
        self.logging_data = [0.0] * len(self.logging_list)

    def update(self, jumping_state_1, takeoff_flag, landing_flag, t, Abi, quat, delta_l_max, omega_mean):

        iteration_count = 0

        if takeoff_flag:
            # update the takeoff attitude
            self.R_TO_old = self.R_TO
            self.R_TO = Rotation.from_quat(quat).as_matrix()
            landing_location_old = self.landing_location_prediction

            directional_error = 10
            R_c = self.R_c_0

            # useless
            position = np.array([0, 0, 0])
            v_TO = np.array([0, 0, 3])
            height = 0

            while directional_error > self.direction_error_limit:
                # _prediction_error
                z_b_LD_old = np.matmul(R_c, self.R_LD_old[:, 2])
                z_b_TO_old = np.matmul(R_c, self.R_TO_old[:, 2])
                z_b_LD = np.matmul(R_c, self.R_LD[:, 2])
                z_b_TO = np.matmul(R_c, self.R_TO[:, 2])

                # call model
                _, v_TO_old = self._leg_measure_model(False, self.omega_mean_old, self.delta_l_max_old, z_b_LD_old, z_b_TO_old, self.R_TO_old)

                # initial condition
                position = landing_location_old
                velocity = v_TO_old
                height = 0

                for i in range(1, self.buffer_index + 1):
                    dt = self.buffer_t[i] - self.buffer_t[i - 1]
                    acc_world = np.matmul(R_c, np.matmul(self.buffer_R[i, :, :], self.buffer_Abi[:, i])) - self.e_3_g
                    velocity = velocity + acc_world * dt
                    position = position + velocity * dt
                    if position[2] > height:
                        height = position[2]

                # call model
                v_LD_unit, v_TO = self._leg_measure_model(True, omega_mean, delta_l_max, z_b_LD, z_b_TO, self.R_TO)
                landing_speed = np.linalg.norm(velocity)
                if landing_speed == 0:
                    v_LD_unit_predicted = np.array([0, 0, -1])
                else:
                    v_LD_unit_predicted = velocity / landing_speed
                # print(v_LD_unit_predicted, v_LD_unit)
                directional_error = angle_between_vectors_degree(v_LD_unit_predicted, v_LD_unit)
                # print(directional_error)
                temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
                # R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)
                R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)

                iteration_count += 1
                if iteration_count > self.max_iteration:
                    break
            self.height = height
            self.landing_location_prediction = position
            self.landing_location_prediction[2] = 0  # reset ground altitude to zero
            self.takeoff_velocity_prediction = v_TO
            self.correction_matrix = R_c
            self.quat_corr = Rotation.from_rotvec(
                Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
                canonical=False)

            self.buffer_Abi[:, 0] = Abi  # 添加新元素
            self.buffer_t[0] = t
            self.buffer_R[0, :, :] = Rotation.from_quat(quat).as_matrix()
            self.buffer_index = 0
        else:
            if jumping_state_1:
                # stance
                pass
            else:
                self.buffer_index += 1
                self.buffer_Abi[:, self.buffer_index] = Abi
                self.buffer_t[self.buffer_index] = t
                self.buffer_R[self.buffer_index, :, :] = Rotation.from_quat(quat).as_matrix()

        if landing_flag:
            # update the landing attitude
            self.R_LD_old = self.R_LD
            self.R_LD = Rotation.from_quat(quat).as_matrix()

        # current state
        if not jumping_state_1:
            if takeoff_flag and t > 0:  # takeoff timestamp
                self.current_position = self.landing_location_prediction
                self.current_velocity = self.takeoff_velocity_prediction
            elif not jumping_state_1:  # after takeoff, aerial
                if self.buffer_index == 0:
                    # first time run the estimator
                    pass
                else:
                    dt = self.buffer_t[self.buffer_index] - self.buffer_t[self.buffer_index - 1]
                    acc_world = np.matmul(self.buffer_R[self.buffer_index, :, :], self.buffer_Abi[:, self.buffer_index]) - self.e_3_g
                    self.current_velocity = self.current_velocity + acc_world * dt
                    self.current_position = self.current_position + self.current_velocity * dt

                if self.current_velocity[2] > 0:
                    self.delta_l_max_old = delta_l_max
                    self.omega_mean_old = omega_mean

            else:
                # stance phase
                pass
        self.logging_data = [self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             iteration_count, self.height]

    def _leg_measure_model(self, v_LD_flag, mean_omega, delta_l_max, z_b_LD, z_b_TO, R_TO):
        mean_omega_world = np.matmul(R_TO, mean_omega)
        v_LD_unit = np.array([0, 0, -1])
        v_axial_TO = self._eval_poly(self.P_TO, float(self.tof_zero - delta_l_max)) * z_b_TO
        v_tangential_TO = np.cross(mean_omega_world/180*math.pi, (z_b_TO * self.leg_length_0))
        v_TO = v_tangential_TO + v_axial_TO
        if v_LD_flag:
            v_axial_LD = self._eval_poly(self.P_LD, float(self.tof_zero - delta_l_max)) * z_b_LD
            v_tangential_LD = np.cross(mean_omega_world/180*math.pi, (z_b_LD * self.leg_length_0))
            v_LD = v_tangential_LD + v_axial_LD
            v_LD_norm = np.linalg.norm(v_LD)
            if v_LD_norm == 0:
                v_LD_unit = np.array([0, 0, -1])
            else:
                v_LD_unit = v_LD / v_LD_norm
        return v_LD_unit, v_TO

    def _eval_poly(self, p, x):
        if x > self.max_contraction:
            x = self.max_contraction
        if x < self.min_contraction:
            x = self.min_contraction

        if self.model_order == 4:
            v = p[0]*(x**4) + p[1]*(x**3) + p[2]*(x**2) + p[3]*x + p[4]
        elif self.model_order == 3:
            v = p[0]*(x**3) + p[1]*(x**2) + p[2]*x + p[3]
        elif self.model_order == 2:
            v = p[0]*(x**2) + p[1]*x + p[2]
        else:  # elif self.model_order == 1:
            v = p[0]*x + p[1]
        return v


class JumpingStateEstimatorLegMeasurement2:
    def __init__(self, leg_length_0, P_LD, P_TO, max_contraction, min_contraction,
                 attitude_correction_gain=0.1, max_iteration=10, acc_bias=np.array([0, 0, 0]), decoupled_leg=False):
        # constants
        self.max_contraction = max_contraction
        self.min_contraction = min_contraction
        self.P_LD = P_LD
        self.P_TO = P_TO
        self.leg_length_0 = leg_length_0
        self.model_order = len(P_TO) - 1
        self.acc_bias = acc_bias

        self.decoupled_leg = decoupled_leg
        self.e_3_g = np.array([0, 0, 9.81])
        self.direction_error_limit = 0.5
        self.direction_gain = 0.2
        self.buffer_length = 500
        self.attitude_correction_gain = attitude_correction_gain
        self.max_iteration = max_iteration

        self.R_c_0 = np.eye(3)

        # variables
        self.buffer_Abi = np.zeros((3, self.buffer_length), dtype=float)
        self.buffer_t = np.zeros(self.buffer_length, dtype=float)
        self.buffer_R = np.stack([np.eye(3)] * self.buffer_length)
        self.buffer_index = -1

        self.delta_l_max_old = 0.1
        self.omega_mean_old = np.array([0, 0, 0])

        self.q_LDTO = np.array([1, 0, 0, 0])

        self.delta_l_aerial_mean = 200
        self.aerial_count = 1
        self.R_LD = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_LD_old = self.R_LD
        self.R_TO = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_TO_old = self.R_TO
        self.landing_location_prediction = np.array([0, 0, 0])
        self.current_position = np.array([0, 0, 0])
        self.current_velocity = np.array([0, 0, 3])
        self.height = 0.458
        self.height_no_thrust = self.height
        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.correction_matrix = np.eye(3)
        self.quat_corr = Rotation.from_rotvec(
            Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
            canonical=False)

        self.logging_list = ['JSELM_vx', 'JSELM_vy', 'JSELM_vz',
                             'JSELM_px', 'JSELM_py', 'JSELM_pz',
                             'JSELM_iteration_count', 'JSELM_height',
                             'JSELM_delta_l_aerial_mean']
        self.logging_data = [0.0] * len(self.logging_list)

    def update_DL(self, q_LDTO):
        self.q_LDTO = q_LDTO

    def update(self, jumping_state_1, takeoff_flag, landing_flag, t, Abi, quat, delta_l_max, omega_mean, delta_l, ):

        Abi = Abi - self.acc_bias

        iteration_count = 0

        if takeoff_flag:
            # update the takeoff attitude
            self.R_TO_old = self.R_TO
            if self.decoupled_leg:
                self.R_TO = Rotation.from_quat(self.q_LDTO).as_matrix()
            else:
                self.R_TO = Rotation.from_quat(quat).as_matrix()
            landing_location_old = self.landing_location_prediction

            directional_error = 10
            R_c = self.R_c_0

            # useless
            position = np.array([0, 0, 0])
            v_TO = np.array([0, 0, 3])
            height = 0

            while directional_error > self.direction_error_limit:
                # _prediction_error
                z_b_LD_old = np.matmul(R_c, self.R_LD_old[:, 2])
                z_b_TO_old = np.matmul(R_c, self.R_TO_old[:, 2])
                z_b_LD = np.matmul(R_c, self.R_LD[:, 2])
                z_b_TO = np.matmul(R_c, self.R_TO[:, 2])

                # call model
                _, v_TO_old = self._leg_measure_model(False, self.omega_mean_old, self.delta_l_max_old, z_b_LD_old, z_b_TO_old, self.R_TO_old)

                # initial condition
                position = landing_location_old
                velocity = v_TO_old
                height = 0

                for i in range(1, self.buffer_index + 1):
                    dt = self.buffer_t[i] - self.buffer_t[i - 1]
                    acc_world = np.matmul(R_c, np.matmul(self.buffer_R[i, :, :], self.buffer_Abi[:, i])) - self.e_3_g
                    velocity = velocity + acc_world * dt
                    position = position + velocity * dt
                    if position[2] > height:
                        height = position[2]

                # call model
                v_LD_unit, v_TO = self._leg_measure_model(True, omega_mean, delta_l_max, z_b_LD, z_b_TO, self.R_TO)
                landing_speed = np.linalg.norm(velocity)
                if landing_speed == 0:
                    v_LD_unit_predicted = np.array([0, 0, -1])
                else:
                    v_LD_unit_predicted = velocity / landing_speed
                # print(v_LD_unit_predicted, v_LD_unit)
                directional_error = angle_between_vectors_degree(v_LD_unit_predicted, v_LD_unit)
                # print(directional_error)
                temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
                # R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)
                R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)

                iteration_count += 1
                if iteration_count > self.max_iteration:
                    break
            self.height = height
            self.landing_location_prediction = position
            self.landing_location_prediction[2] = 0  # reset ground altitude to zero
            self.takeoff_velocity_prediction = v_TO
            # h = 0.5*v^2/g
            self.height_no_thrust = 0.5 * self.takeoff_velocity_prediction[2]**2 / 9.81
            self.correction_matrix = R_c
            self.quat_corr = Rotation.from_rotvec(
                Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
                canonical=False)

            self.buffer_Abi[:, 0] = Abi  # 添加新元素
            self.buffer_t[0] = t
            self.buffer_R[0, :, :] = Rotation.from_quat(quat).as_matrix()
            self.buffer_index = 0

            self.aerial_count = 1
            self.delta_l_aerial_mean = float(delta_l)
        else:
            if jumping_state_1:
                # stance phase
                pass
            else:
                # arial phase
                self.buffer_index += 1
                self.buffer_Abi[:, self.buffer_index] = Abi
                self.buffer_t[self.buffer_index] = t
                self.buffer_R[self.buffer_index, :, :] = Rotation.from_quat(quat).as_matrix()

                aerial_count_p1 = self.aerial_count + 1
                self.delta_l_aerial_mean = (self.aerial_count * self.delta_l_aerial_mean / aerial_count_p1) + (float(delta_l) / aerial_count_p1)
                self.aerial_count = aerial_count_p1
        if landing_flag:
            # update the landing attitude
            self.R_LD_old = self.R_LD
            if self.decoupled_leg:
                self.R_LD = Rotation.from_quat(self.q_LDTO).as_matrix()
            else:
                self.R_LD = Rotation.from_quat(quat).as_matrix()

        # current state
        if not jumping_state_1:
            if takeoff_flag and t > 0:  # takeoff timestamp
                self.current_position = self.landing_location_prediction
                self.current_velocity = self.takeoff_velocity_prediction
            elif not jumping_state_1:  # after takeoff, aerial
                if self.buffer_index == 0:
                    # first time run the estimator
                    pass
                else:
                    dt = self.buffer_t[self.buffer_index] - self.buffer_t[self.buffer_index - 1]
                    acc_world = np.matmul(self.buffer_R[self.buffer_index, :, :], self.buffer_Abi[:, self.buffer_index]) - self.e_3_g
                    self.current_velocity = self.current_velocity + acc_world * dt
                    self.current_position = self.current_position + self.current_velocity * dt

                if self.current_velocity[2] > 0:
                    self.delta_l_max_old = delta_l_max
                    self.omega_mean_old = omega_mean

            else:
                # stance phase
                pass
        self.logging_data = [self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             iteration_count, self.height,
                             self.delta_l_aerial_mean]

    def _leg_measure_model(self, v_LD_flag, mean_omega, delta_l_max, z_b_LD, z_b_TO, R_TO):
        mean_omega_world = np.matmul(R_TO, mean_omega)
        # correction_factor = 0.2143 * math.log(1 + abs(mean_omega[1])/300)
        # correction_factor = 0.17 * math.log(1 + abs(mean_omega[1]) / 550)
        correction_factor = 0
        v_LD_unit = np.array([0, 0, -1])
        v_axial_TO = self._eval_poly(self.P_TO, float(self.delta_l_aerial_mean - delta_l_max)) * z_b_TO
        v_tangential_TO = np.cross(mean_omega_world/180*math.pi, (z_b_TO * self.leg_length_0)) # manual tuning
        v_TO = v_tangential_TO + v_axial_TO
        if v_LD_flag:
            v_axial_LD = self._eval_poly(self.P_LD, float(self.delta_l_aerial_mean - delta_l_max)) * z_b_LD
            v_tangential_LD = np.cross(mean_omega_world/180*math.pi, (z_b_LD * self.leg_length_0))   # manual tuning
            v_LD = v_tangential_LD + v_axial_LD
            v_LD_norm = np.linalg.norm(v_LD)
            if v_LD_norm == 0:
                v_LD_unit = np.array([0, 0, -1])
            else:
                v_LD_unit = v_LD / v_LD_norm
        return v_LD_unit, v_TO

    def _eval_poly(self, p, x):
        if x > self.max_contraction:
            x = self.max_contraction
        if x < self.min_contraction:
            x = self.min_contraction

        if self.model_order == 4:
            v = p[0]*(x**4) + p[1]*(x**3) + p[2]*(x**2) + p[3]*x + p[4]
        elif self.model_order == 3:
            v = p[0]*(x**3) + p[1]*(x**2) + p[2]*x + p[3]
        elif self.model_order == 2:
            v = p[0]*(x**2) + p[1]*x + p[2]
        else:  # elif self.model_order == 1:
            v = p[0]*x + p[1]
        return v


class JumpingStateEstimatorLegMeasurement3:
    def __init__(self, stLUTX, stLUTY, stLUTvald, stLUTvato, stLUTvtan,
                 attitude_correction_gain=0.1, max_iteration=10, acc_bias=np.array([0, 0, 0]), decoupled_leg=False):
        # constants
        self.omega_max = stLUTX[-1]
        self.delta_l_lut_max = stLUTY[-1]
        self.delta_l_lut_min = stLUTY[1]
        self.stLUTvald = RegularGridInterpolator((stLUTX, stLUTY), stLUTvald, method='linear', bounds_error=False)
        self.stLUTvato = RegularGridInterpolator((stLUTX, stLUTY), stLUTvato, method='linear', bounds_error=False)
        self.stLUTvtan = RegularGridInterpolator((stLUTX, stLUTY), stLUTvtan, method='linear', bounds_error=False)

        self.acc_bias = acc_bias

        self.decoupled_leg = decoupled_leg
        self.e_3_g = np.array([0, 0, 9.81])
        self.direction_error_limit = 0.5
        self.direction_gain = 0.2
        self.buffer_length = 500
        self.attitude_correction_gain = attitude_correction_gain
        self.max_iteration = max_iteration

        self.R_c_0 = np.eye(3)

        # variables
        self.buffer_Abi = np.zeros((3, self.buffer_length), dtype=float)
        self.buffer_t = np.zeros(self.buffer_length, dtype=float)
        self.buffer_R = np.stack([np.eye(3)] * self.buffer_length)
        self.buffer_index = -1

        self.delta_l_max_old = 40
        self.omega_mean_old = np.array([0, 0, 0])

        self.q_LDTO = np.array([1, 0, 0, 0])

        self.delta_l_aerial_mean = 200
        self.aerial_count = 1
        self.R_LD = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_LD_old = self.R_LD
        self.R_TO = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_TO_old = self.R_TO
        self.landing_location_prediction = np.array([0, 0, 0])
        self.current_position = np.array([0, 0, 0])
        self.current_velocity = np.array([0, 0, 3])
        self.height = 0.458
        self.height_no_thrust = self.height
        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.correction_matrix = np.eye(3)
        self.quat_corr = Rotation.from_rotvec(
            Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
            canonical=False)

        self.logging_list = ['JSELM_vx', 'JSELM_vy', 'JSELM_vz',
                             'JSELM_px', 'JSELM_py', 'JSELM_pz',
                             'JSELM_iteration_count', 'JSELM_height',
                             'JSELM_delta_l_aerial_mean']
        self.logging_data = [0.0] * len(self.logging_list)

    def update_DL(self, q_LDTO):
        self.q_LDTO = q_LDTO

    def update(self, jumping_state_1, takeoff_flag, landing_flag, t, Abi, quat, delta_l_max, omega_mean, delta_l_aerial, ):

        Abi = Abi - self.acc_bias

        iteration_count = 0

        if takeoff_flag:
            # update the takeoff attitude
            self.R_TO_old = self.R_TO
            if self.decoupled_leg:
                self.R_TO = Rotation.from_quat(self.q_LDTO).as_matrix()
            else:
                self.R_TO = Rotation.from_quat(quat).as_matrix()
            landing_location_old = self.landing_location_prediction

            directional_error = 10
            R_c = self.R_c_0

            # useless
            position = np.array([0, 0, 0])
            v_TO = np.array([0, 0, 3])
            height = 0

            while directional_error > self.direction_error_limit:
                # _prediction_error
                z_b_LD_old = np.matmul(R_c, self.R_LD_old[:, 2])
                z_b_TO_old = np.matmul(R_c, self.R_TO_old[:, 2])
                z_b_LD = np.matmul(R_c, self.R_LD[:, 2])
                z_b_TO = np.matmul(R_c, self.R_TO[:, 2])

                # call model
                _, v_TO_old = self._leg_measure_model(False, self.omega_mean_old, (-float(self.delta_l_max_old)+self.delta_l_aerial_mean), z_b_LD_old, z_b_TO_old, self.R_TO_old)

                # initial condition
                position = landing_location_old
                velocity = v_TO_old
                height = 0

                for i in range(1, self.buffer_index + 1):
                    dt = self.buffer_t[i] - self.buffer_t[i - 1]
                    acc_world = np.matmul(R_c, np.matmul(self.buffer_R[i, :, :], self.buffer_Abi[:, i])) - self.e_3_g
                    velocity = velocity + acc_world * dt
                    position = position + velocity * dt
                    if position[2] > height:
                        height = position[2]

                # call model
                v_LD_unit, v_TO = self._leg_measure_model(True, omega_mean, (-float(delta_l_max)+self.delta_l_aerial_mean), z_b_LD, z_b_TO, self.R_TO)
                landing_speed = np.linalg.norm(velocity)
                if landing_speed == 0:
                    v_LD_unit_predicted = np.array([0, 0, -1])
                else:
                    v_LD_unit_predicted = velocity / landing_speed
                # print(v_LD_unit_predicted, v_LD_unit)
                directional_error = angle_between_vectors_degree(v_LD_unit_predicted, v_LD_unit)
                # print(directional_error)
                temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
                # R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)
                R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)

                iteration_count += 1
                if iteration_count > self.max_iteration:
                    break
            self.height = height
            self.landing_location_prediction = position
            self.landing_location_prediction[2] = 0  # reset ground altitude to zero
            self.takeoff_velocity_prediction = v_TO
            # h = 0.5*v^2/g
            self.height_no_thrust = 0.5 * self.takeoff_velocity_prediction[2]**2 / 9.81
            self.correction_matrix = R_c
            self.quat_corr = Rotation.from_rotvec(
                Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
                canonical=False)

            self.buffer_Abi[:, 0] = Abi  # 添加新元素
            self.buffer_t[0] = t
            self.buffer_R[0, :, :] = Rotation.from_quat(quat).as_matrix()
            self.buffer_index = 0

            self.aerial_count = 1
            self.delta_l_aerial_mean = float(delta_l_aerial)
        else:
            if jumping_state_1:
                # stance phase
                pass
            else:
                # arial phase
                self.buffer_index += 1
                self.buffer_Abi[:, self.buffer_index] = Abi
                self.buffer_t[self.buffer_index] = t
                self.buffer_R[self.buffer_index, :, :] = Rotation.from_quat(quat).as_matrix()

                aerial_count_p1 = self.aerial_count + 1
                self.delta_l_aerial_mean = (self.aerial_count * self.delta_l_aerial_mean / aerial_count_p1) + (float(delta_l_aerial) / aerial_count_p1)
                self.aerial_count = aerial_count_p1
        if landing_flag:
            # update the landing attitude
            self.R_LD_old = self.R_LD
            if self.decoupled_leg:
                self.R_LD = Rotation.from_quat(self.q_LDTO).as_matrix()
            else:
                self.R_LD = Rotation.from_quat(quat).as_matrix()

        # current state
        if not jumping_state_1:
            if takeoff_flag and t > 0:  # takeoff timestamp
                self.current_position = self.landing_location_prediction
                self.current_velocity = self.takeoff_velocity_prediction
            elif not jumping_state_1:  # after takeoff, aerial
                if self.buffer_index == 0:
                    # first time run the estimator
                    pass
                else:
                    dt = self.buffer_t[self.buffer_index] - self.buffer_t[self.buffer_index - 1]
                    acc_world = np.matmul(self.buffer_R[self.buffer_index, :, :], self.buffer_Abi[:, self.buffer_index]) - self.e_3_g
                    self.current_velocity = self.current_velocity + acc_world * dt
                    self.current_position = self.current_position + self.current_velocity * dt

                if self.current_velocity[2] > 0:
                    self.delta_l_max_old = delta_l_max
                    self.omega_mean_old = omega_mean

            else:
                # stance phase
                pass
        self.logging_data = [self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             iteration_count, self.height,
                             self.delta_l_aerial_mean]

    def _leg_measure_model(self, v_LD_flag, mean_omega, delta_l_max, z_b_LD, z_b_TO, R_TO):
        mean_omega = mean_omega /180*math.pi
        mean_omega_world = np.matmul(R_TO, mean_omega)

        mean_omega_norm = np.linalg.norm(mean_omega)
        if mean_omega_norm > self.omega_max:
            mean_omega_norm = self.omega_max
        if delta_l_max > self.delta_l_lut_max:
            delta_l_max = self.delta_l_lut_max
        if delta_l_max < self.delta_l_lut_min:
            delta_l_max = self.delta_l_lut_min

        A = np.array([mean_omega_norm, delta_l_max])

        v_LD_unit = np.array([0, 0, -1])
        v_axial_TO = self.stLUTvato(A) * z_b_TO
        v_tangential_TO_temp = np.cross(mean_omega_world, 10*z_b_TO)
        v_tangential_TO_temp_norm = np.linalg.norm(v_tangential_TO_temp)
        v_tangential_TO_norm = self.stLUTvtan(A)

        if v_tangential_TO_temp_norm == 0:
            v_tangential_TO = np.array([0, 0, 0])
        else:
            v_tangential_TO = (v_tangential_TO_temp/v_tangential_TO_temp_norm)*v_tangential_TO_norm

        v_TO = v_tangential_TO + v_axial_TO
        if v_LD_flag:

            v_axial_LD = self.stLUTvald(A) * z_b_LD
            v_tangential_LD_temp = np.cross(mean_omega_world, 10 * z_b_LD)
            v_tangential_LD_temp_norm = np.linalg.norm(v_tangential_LD_temp)
            v_tangential_LD_norm = v_tangential_TO_norm

            if v_tangential_LD_temp_norm == 0:
                v_tangential_LD = np.array([0, 0, 0])
            else:
                v_tangential_LD = (v_tangential_LD_temp / v_tangential_LD_temp_norm) * v_tangential_LD_norm

            v_LD = v_tangential_LD + v_axial_LD
            v_LD_norm = np.linalg.norm(v_LD)
            if v_LD_norm == 0:
                v_LD_unit = np.array([0, 0, -1])
            else:
                v_LD_unit = v_LD / v_LD_norm
        return v_LD_unit, v_TO


class JumpingStateEstimatorLegMeasurement3_stance_time:
    def __init__(self, stLUTX, stLUTY, stLUTvald, stLUTvato, stLUTvtan,
                 attitude_correction_gain=0.1, max_iteration=10, acc_bias=np.array([0, 0, 0]), ):
        # constants
        self.omega_max = stLUTX[-1]
        self.delta_l_lut_max = stLUTY[-1]
        self.delta_l_lut_min = stLUTY[1]
        self.stLUTvald = RegularGridInterpolator((stLUTX, stLUTY), stLUTvald, method='linear', bounds_error=False)
        self.stLUTvato = RegularGridInterpolator((stLUTX, stLUTY), stLUTvato, method='linear', bounds_error=False)
        self.stLUTvtan = RegularGridInterpolator((stLUTX, stLUTY), stLUTvtan, method='linear', bounds_error=False)

        self.acc_bias = acc_bias

        self.e_3_g = np.array([0, 0, 9.81])
        self.direction_error_limit = 0.5
        self.direction_gain = 0.2
        self.buffer_length = 500
        self.attitude_correction_gain = attitude_correction_gain
        self.max_iteration = max_iteration

        self.R_c_0 = np.eye(3)

        # variables
        self.buffer_Abi = np.zeros((3, self.buffer_length), dtype=float)
        self.buffer_t = np.zeros(self.buffer_length, dtype=float)
        self.buffer_R = np.stack([np.eye(3)] * self.buffer_length)
        self.buffer_index = -1

        self.omega_mean_old = np.array([0, 0, 0])
        self.stance_timex100_old = 60

        self.q_LDTO = np.array([1, 0, 0, 0])

        self.aerial_count = 1
        self.R_LD = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_LD_old = self.R_LD
        self.R_TO = Rotation.from_euler('zyx', [0, 0, 0], degrees=False).as_matrix()
        self.R_TO_old = self.R_TO
        self.landing_location_prediction = np.array([0, 0, 0])
        self.current_position = np.array([0, 0, 0])
        self.current_velocity = np.array([0, 0, 3])
        self.height = 0.458
        self.height_no_thrust = self.height
        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.correction_matrix = np.eye(3)
        self.quat_corr = Rotation.from_rotvec(
            Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
            canonical=False)

        self.logging_list = ['JSELM_vx', 'JSELM_vy', 'JSELM_vz',
                             'JSELM_px', 'JSELM_py', 'JSELM_pz',
                             'JSELM_iteration_count', 'JSELM_height',]
        self.logging_data = [0.0] * len(self.logging_list)

    def update_DL(self, q_LDTO):
        self.q_LDTO = q_LDTO

    def update(self, jumping_state_1, takeoff_flag, landing_flag, t, Abi, quat, omega_mean, stance_timex100):

        Abi = Abi - self.acc_bias

        iteration_count = 0

        if takeoff_flag:
            # update the takeoff attitude
            self.R_TO_old = self.R_TO

            self.R_TO = Rotation.from_quat(quat).as_matrix()
            landing_location_old = self.landing_location_prediction

            directional_error = 10
            R_c = self.R_c_0

            # useless
            position = np.array([0, 0, 0])
            v_TO = np.array([0, 0, 3])
            height = 0

            while directional_error > self.direction_error_limit:
                # _prediction_error
                z_b_LD_old = np.matmul(R_c, self.R_LD_old[:, 2])
                z_b_TO_old = np.matmul(R_c, self.R_TO_old[:, 2])
                z_b_LD = np.matmul(R_c, self.R_LD[:, 2])
                z_b_TO = np.matmul(R_c, self.R_TO[:, 2])

                # call model
                _, v_TO_old = self._leg_measure_model(False, self.omega_mean_old, self.stance_timex100_old, z_b_LD_old, z_b_TO_old, self.R_TO_old)

                # initial condition
                position = landing_location_old
                velocity = v_TO_old
                height = 0

                for i in range(1, self.buffer_index + 1):
                    dt = self.buffer_t[i] - self.buffer_t[i - 1]
                    acc_world = np.matmul(R_c, np.matmul(self.buffer_R[i, :, :], self.buffer_Abi[:, i])) - self.e_3_g
                    velocity = velocity + acc_world * dt
                    position = position + velocity * dt
                    if position[2] > height:
                        height = position[2]

                # call model
                v_LD_unit, v_TO = self._leg_measure_model(True, omega_mean, stance_timex100, z_b_LD, z_b_TO, self.R_TO)
                landing_speed = np.linalg.norm(velocity)
                if landing_speed == 0:
                    v_LD_unit_predicted = np.array([0, 0, -1])
                else:
                    v_LD_unit_predicted = velocity / landing_speed
                # print(v_LD_unit_predicted, v_LD_unit)
                directional_error = angle_between_vectors_degree(v_LD_unit_predicted, v_LD_unit)
                # print(directional_error)
                temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
                # R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)
                R_c = np.matmul(Rotation.from_rotvec(-self.direction_gain * temp_axis).as_matrix(), R_c)

                iteration_count += 1
                if iteration_count > self.max_iteration:
                    break
            self.height = height
            self.landing_location_prediction = position
            self.landing_location_prediction[2] = 0  # reset ground altitude to zero
            self.takeoff_velocity_prediction = v_TO
            # h = 0.5*v^2/g
            self.height_no_thrust = 0.5 * self.takeoff_velocity_prediction[2]**2 / 9.81
            self.correction_matrix = R_c
            self.quat_corr = Rotation.from_rotvec(
                Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(
                canonical=False)

            self.buffer_Abi[:, 0] = Abi  # 添加新元素
            self.buffer_t[0] = t
            self.buffer_R[0, :, :] = Rotation.from_quat(quat).as_matrix()
            self.buffer_index = 0

            self.aerial_count = 1

        else:
            if jumping_state_1:
                # stance phase
                pass
            else:
                # arial phase
                self.buffer_index += 1
                self.buffer_Abi[:, self.buffer_index] = Abi
                self.buffer_t[self.buffer_index] = t
                self.buffer_R[self.buffer_index, :, :] = Rotation.from_quat(quat).as_matrix()

                aerial_count_p1 = self.aerial_count + 1

                self.aerial_count = aerial_count_p1
        if landing_flag:
            # update the landing attitude
            self.R_LD_old = self.R_LD

            self.R_LD = Rotation.from_quat(quat).as_matrix()

        # current state
        if not jumping_state_1:
            if takeoff_flag and t > 0:  # takeoff timestamp
                self.current_position = self.landing_location_prediction
                self.current_velocity = self.takeoff_velocity_prediction
            elif not jumping_state_1:  # after takeoff, aerial
                if self.buffer_index == 0:
                    # first time run the estimator
                    pass
                else:
                    dt = self.buffer_t[self.buffer_index] - self.buffer_t[self.buffer_index - 1]
                    acc_world = np.matmul(self.buffer_R[self.buffer_index, :, :], self.buffer_Abi[:, self.buffer_index]) - self.e_3_g
                    self.current_velocity = self.current_velocity + acc_world * dt
                    self.current_position = self.current_position + self.current_velocity * dt

                if self.current_velocity[2] > 0:

                    self.omega_mean_old = omega_mean
                    self.stance_timex100_old = stance_timex100

            else:
                # stance phase
                pass
        self.logging_data = [self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             iteration_count, self.height,]

    def _leg_measure_model(self, v_LD_flag, mean_omega, delta_l_max, z_b_LD, z_b_TO, R_TO):
        mean_omega = mean_omega /180*math.pi
        mean_omega_world = np.matmul(R_TO, mean_omega)

        mean_omega_norm = np.linalg.norm(mean_omega)
        if mean_omega_norm > self.omega_max:
            mean_omega_norm = self.omega_max
        if delta_l_max > self.delta_l_lut_max:
            delta_l_max = self.delta_l_lut_max
        if delta_l_max < self.delta_l_lut_min:
            delta_l_max = self.delta_l_lut_min

        A = np.array([mean_omega_norm, delta_l_max])

        v_LD_unit = np.array([0, 0, -1])
        v_axial_TO = self.stLUTvato(A) * z_b_TO
        v_tangential_TO_temp = np.cross(mean_omega_world, 10*z_b_TO)
        v_tangential_TO_temp_norm = np.linalg.norm(v_tangential_TO_temp)
        v_tangential_TO_norm = self.stLUTvtan(A)

        if v_tangential_TO_temp_norm == 0:
            v_tangential_TO = np.array([0, 0, 0])
        else:
            v_tangential_TO = (v_tangential_TO_temp/v_tangential_TO_temp_norm)*v_tangential_TO_norm

        v_TO = v_tangential_TO + v_axial_TO
        if v_LD_flag:

            v_axial_LD = self.stLUTvald(A) * z_b_LD
            v_tangential_LD_temp = np.cross(mean_omega_world, 10 * z_b_LD)
            v_tangential_LD_temp_norm = np.linalg.norm(v_tangential_LD_temp)
            v_tangential_LD_norm = v_tangential_TO_norm

            if v_tangential_LD_temp_norm == 0:
                v_tangential_LD = np.array([0, 0, 0])
            else:
                v_tangential_LD = (v_tangential_LD_temp / v_tangential_LD_temp_norm) * v_tangential_LD_norm

            v_LD = v_tangential_LD + v_axial_LD
            v_LD_norm = np.linalg.norm(v_LD)
            if v_LD_norm == 0:
                v_LD_unit = np.array([0, 0, -1])
            else:
                v_LD_unit = v_LD / v_LD_norm
        return v_LD_unit, v_TO


def angle_between_vectors_degree(u, v):
    angle = math.atan2(np.linalg.norm(np.cross(u, v)), np.dot(u, v)) * 180 / math.pi
    return angle


class YawControl:
    def __init__(self, ):
        self.takeoff_heading_vector = np.array([1, 0])
        self.desired_heading_vector = np.array([1, 0])
        self.current_heading_vector = np.array([1, 0])
        self.rot90 = np.array([[0, -1],[1, 0]])
        self.current_yaw = 0
        self.desired_yaw = 0

    def takeoff_degree(self, desired_yaw):
        desired_yaw_rad = math.radians(desired_yaw)
        self.takeoff_heading_vector = np.array([math.cos(desired_yaw_rad), math.sin(desired_yaw_rad)])

    def takeoff_radians(self, desired_yaw):
        self.takeoff_heading_vector = np.array([math.cos(desired_yaw), math.sin(desired_yaw)])

    def update_ref_degree(self, desired_yaw):
        desired_yaw_rad = math.radians(desired_yaw)
        self.desired_heading_vector = np.array([math.cos(desired_yaw_rad), math.sin(desired_yaw_rad)])
        self._compute_current()

    def update_ref_radians(self, desired_yaw):
        self.desired_heading_vector = np.array([math.cos(desired_yaw), math.sin(desired_yaw)])
        self._compute_current()

    def _compute_current(self):
        self.current_heading_vector = self.takeoff_heading_vector + 0.5 * (- self.takeoff_heading_vector + self.desired_heading_vector)
        norm_current_heading_vector = np.linalg.norm(self.current_heading_vector)

        if norm_current_heading_vector == 0:
            self.current_heading_vector = np.matmul(self.rot90, self.current_heading_vector)
        else:
            self.current_heading_vector = self.current_heading_vector/norm_current_heading_vector

        self.current_yaw = math.atan2(self.current_heading_vector[1], self.current_heading_vector[0])
        # self.desired_yaw = math.atan2(self.desired_heading_vector[1], self.desired_heading_vector[0])





