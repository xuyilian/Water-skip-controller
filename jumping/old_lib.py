import numpy as np
import math
import time
from scipy.spatial.transform import Rotation
import threading
import socket
from tqdm import tqdm
import struct


def angle_between_vectors(u, v):
    angle = math.atan2(np.linalg.norm(np.cross(u, v)), (u * v).sum())
    return angle


def SO(omega):
    omega = omega.tolist()
    return np.array([[0, -omega[2], omega[1]],
                  [omega[2], 0, -omega[0]],
                  [-omega[1], omega[0], 0]])


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


# this is NOT incorrect!
class JumpingStateEstimator2Height_2:
    # using threading to reduce the computational time
    def __init__(self, k1, k2, leg_efficiency, g, direction_gain, altitude_gain, attitude_correction_gain,
                 landing_speed_old=4, max_iteration_count=50, acc_bias=np.array([0.1, 0.045, 0]), gyro_bais=np.array([0, 0, 0]),
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
        self.gyro_body_1 = [np.array([0, 0, 0])]
        self.quat_1 = [np.array([0, 0, 0, 1])]
        self.z_1 = [0]
        # buffer set 2
        self.time_course_2 = [0]
        self.acc_body_2 = [np.array([0, 0, 0])]
        self.gyro_body_2 = [np.array([0, 0, 0])]
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
        self.gyro_bais = gyro_bais

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

    def update(self, t, acc, quat, estimator_state, z, gyro):
        acc = acc - self.acc_bias
        gyro = gyro - self.gyro_bais
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
                self.gyro_body_1.append(gyro)
                self.quat_1.append(quat)
                self.z_1.append(z)
            else:
                self.time_course_2.append(t)
                self.acc_body_2.append(acc)
                self.gyro_body_2.append(gyro)
                self.quat_2.append(quat)
                self.z_2.append(z)
        else:
            # first time (this is the takeoff timestamp)
            self.aerial_start_flag = True

            # clean and init the arrays
            if self.buffer_flag:
                self.time_course_1 = [t]
                self.acc_body_1 = [acc]
                self.gyro_body_1 = [gyro]
                self.quat_1 = [quat]
                self.z_1 = [z]
            else:
                self.time_course_2 = [t]
                self.acc_body_2 = [acc]
                self.gyro_body_2 = [gyro]
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

                    R_corrected = np.matmul(R_c, R)
                    acc_world = np.matmul(R_corrected, self.acc_body_1[index]) - self.e_3 * self.g + np.matmul(np.matmul(R_corrected, SO(self.gyro_body_1[index])), np.matmul(R_corrected.T, velocity))
                    # acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    velocity_z = (self.z_1[index] - self.z_1[index-1]) / dt
                    velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    # position[2] = self.z_1[index]

                    position = position + velocity * dt
                    # position_z = self.z_1[index]
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

                    R_corrected = np.matmul(R_c, R)
                    acc_world = np.matmul(R_corrected, self.acc_body_2[index]) - self.e_3 * self.g + np.matmul(np.matmul(R_corrected, SO(self.gyro_body_2[index])), np.matmul(R_corrected.T, velocity))
                    # acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
                    velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * velocity[2]
                    # position[2] = self.z_2[index]
                    position = position + velocity * dt
                    # position_z = self.z_2[index]
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
                    R_corrected = np.matmul(R_c, R)
                    acc_world = np.matmul(R_corrected, self.acc_body_1[index]) - self.e_3 * self.g + np.matmul(np.matmul(R_corrected, SO(self.gyro_body_1[index])), np.matmul(R_corrected.T, velocity))
                    # acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    velocity_z = (self.z_1[index] - self.z_1[index - 1]) / dt
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

                    R_corrected = np.matmul(R_c, R)
                    acc_world = np.matmul(R_corrected, self.acc_body_2[index]) - self.e_3 * self.g + np.matmul(np.matmul(R_corrected, SO(self.gyro_body_2[index])), np.matmul(R_corrected.T, velocity))
                    # acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * dt

                    velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
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
                acc_world = np.matmul(R, self.acc_body_1[-1]) - self.e_3 * self.g + np.matmul(np.matmul(R, SO(self.gyro_body_1[-1])), np.matmul(R.T, self.current_velocity))
                # acc_world = np.matmul(R, self.acc_body_1[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * dt

                velocity_z = (self.z_1[-1] - self.z_1[-2]) / dt
                self.current_velocity[2] = self.complementary_filter_gain_z_measure * velocity_z + self.complementary_filter_gain_z_acc_measure * self.current_velocity[2]
                # self.current_position[2] = self.z_1[-1]
                self.current_position = self.current_position + self.current_velocity * dt
                position_z = self.z_1[-1]
                # self.current_position[2] = self.current_position[2] + (position_z - self.current_position[2]) * self.complementary_filter_gain_z_measure

            else:
                dt = self.time_course_2[-1] - self.time_course_2[-2]  #xxxxxxxx

                R = Rotation.from_quat(self.quat_2[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_2[-1]) - self.e_3 * self.g + np.matmul(np.matmul(R, SO(self.gyro_body_2[-1])), np.matmul(R.T, self.current_velocity))
                # acc_world = np.matmul(R, self.acc_body_2[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * dt

                velocity_z = (self.z_2[-1] - self.z_2[-2]) / dt
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


class JumpingStateEstimator2Height_3:
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
            self._landing(quat_leg)
            self.aerial_start_flag = False

        if self.iteration_done:
            self.iteration_done = False
            self.update_current_state()
        else:
            self.current_z_dot_old = self.current_velocity[2]
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

                    velocity_z = (self.z_1[index] - self.z_1[index-1]) / dt
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

                    velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
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

                    velocity_z = (self.z_1[index] - self.z_1[index - 1]) / dt
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

                    velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
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

                velocity_z = (self.z_1[-1] - self.z_1[-2]) / dt
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

                velocity_z = (self.z_2[-1] - self.z_2[-2]) / dt
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


class JumpingStateEstimator2Height:
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

                    velocity_z = (self.z_1[index] - self.z_1[index-1]) / dt
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

                    velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
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

                    velocity_z = (self.z_1[index] - self.z_1[index - 1]) / dt
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

                    velocity_z = (self.z_2[index] - self.z_2[index - 1]) / dt
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

                velocity_z = (self.z_1[-1] - self.z_1[-2]) / dt
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

                velocity_z = (self.z_2[-1] - self.z_2[-2]) / dt
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


class JumpingStateEstimator2:
    # using threading to reduce the computational time
    def __init__(self, k1, k2, leg_efficiency, g, direction_gain, altitude_gain, attitude_correction_gain,
                 landing_speed_old=4, max_iteration_count=50, acc_bias=np.array([0.1, 0.045, 0]),
                 direction_error_limit=0.2, altitude_error_limit=0.02):
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
        # buffer set 2
        self.time_course_2 = [0]
        self.acc_body_2 = [np.array([0, 0, 0])]
        self.quat_2 = [np.array([0, 0, 0, 1])]

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

    def update(self, t, acc, quat, estimator_state):
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
            else:
                self.time_course_2.append(t)
                self.acc_body_2.append(acc)
                self.quat_2.append(quat)
        else:
            # first time (this is the takeoff timestamp)
            self.aerial_start_flag = True

            # clean and init the arrays
            if self.buffer_flag:
                self.time_course_1 = [t]
                self.acc_body_1 = [acc]
                self.quat_1 = [quat]
            else:
                self.time_course_2 = [t]
                self.acc_body_2 = [acc]
                self.quat_2 = [quat]
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

        directional_error = 10
        altitude_error = 10
        height = 0

        landing_location_old = self.landing_location_prediction


        R_c = self.R_c_ini
        landing_speed_old = self.landing_speed_prediction
        self.landing_speed_old_original = landing_speed_old

        while directional_error > self.direction_error_limit or abs(altitude_error) > self.altitude_error_limit:
            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height = self._prediction_error(R_c, landing_speed_old, landing_location_old, buffer_flag)
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
        # print('xxx', self.altitude_error, self.directional_error)

    def _iteration_worker(self, ):
        buffer_flag = self.buffer_flag_worker
        landing_location_old = self.landing_location_old_worker

        directional_error = self.directional_error
        altitude_error = self.altitude_error
        R_c = self.correction_matrix
        landing_speed_old = self.landing_speed_old_prediction
        while True:
            time.sleep(0.001)

            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height = self._prediction_error(R_c, landing_speed_old, landing_location_old, buffer_flag)

            temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
            R_c = np.matmul(Rotation.from_rotvec(temp_axis * - self.direction_gain).as_matrix(), R_c)
            landing_speed_old = landing_speed_old - altitude_error * self.altitude_gain

            self.iteration_count += 1
            print(self.iteration_count)

            if (self.iteration_count > self.max_iteration_count) or (directional_error < self.direction_error_limit and abs(altitude_error) < self.altitude_error_limit):
                break
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
        # print('yyy', self.altitude_error, self.directional_error)

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
                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * (t - self.time_course_1[index - 1])
                    position = position + velocity * (t - self.time_course_1[index - 1])

                    if position[2] > height:
                        height = position[2]
                index += 1
        else:
            for t in self.time_course_2:

                R = Rotation.from_quat(self.quat_2[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * (t - self.time_course_2[index - 1])
                    position = position + velocity * (t - self.time_course_2[index - 1])

                    if position[2] > height:
                        height = position[2]
                index += 1

        self.lock.release()

        landing_velocity_predicted = velocity
        v_LD_unit_predicted = landing_velocity_predicted / np.linalg.norm(landing_velocity_predicted)

        altitude_error = position[2]
        directional_error = angle_between_vectors(v_LD_unit_predicted, v_LD_unit) * 180 / math.pi

        return directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height

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
                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_1[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * (t - self.time_course_1[index - 1])
                    position = position + velocity * (t - self.time_course_1[index - 1])

                    if position[2] > height:
                        height = position[2]
                index += 1
        else:
            for t in self.time_course_2:
                R = Rotation.from_quat(self.quat_2[index]).as_matrix()
                if index == 0:
                    pass
                else:
                    acc_world = np.matmul(R_c, np.matmul(R, self.acc_body_2[index])) - self.e_3 * self.g
                    velocity = velocity + acc_world * (t - self.time_course_2[index - 1])
                    position = position + velocity * (t - self.time_course_2[index - 1])

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
                R = Rotation.from_quat(self.quat_1[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_1[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * (self.time_course_1[-1] - self.time_course_1[-2])
                self.current_position = self.current_position + self.current_velocity * (self.time_course_1[-1] - self.time_course_1[-2])
            else:
                R = Rotation.from_quat(self.quat_2[-1]).as_matrix()
                acc_world = np.matmul(R, self.acc_body_2[-1]) - self.e_3 * self.g
                self.current_velocity = self.current_velocity + acc_world * (self.time_course_2[-1] - self.time_course_2[-2])
                self.current_position = self.current_position + self.current_velocity * (self.time_course_2[-1] - self.time_course_2[-2])
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


class JumpingStateEstimator:
    def __init__(self, k1, k2, leg_efficiency, g, direction_gain, altitude_gain, attitude_correction_gain,
                 landing_speed_old=4, max_iteration_count=50, acc_bias=np.array([0.1, 0.045, 0]),
                 direction_error_limit=0.2, altitude_error_limit=0.02):
        self.direction_error_limit = direction_error_limit
        self.altitude_error_limit = altitude_error_limit
        # stance model parameters
        self.iteration_count = 0
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

        self.time_course = [0]
        self.acc_body = [np.array([0, 0, 0])]
        self.quat = [np.array([0, 0, 0, 1])]

        self.R_TO = Rotation.from_quat(self.quat[0]).as_matrix()
        self.R_TO_old = self.R_TO
        self.R_LD = Rotation.from_quat(self.quat[0]).as_matrix()
        self.R_LD_old = self.R_LD

        self.correction_matrix = Rotation.from_quat(self.quat[0]).as_matrix()
        self.altitude_error = 0
        self.directional_error = 0

        # variables inside the loop
        self.R_c_ini = Rotation.from_quat(self.quat[0]).as_matrix()
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

        self.logging_list = ['JSE_time_cost',
                             'JSE_v_TO_x_predicted', 'JSE_v_TO_y_predicted', 'JSE_v_TO_z_predicted',
                             'JSE_v_LD_x_predicted', 'JSE_v_LD_y_predicted', 'JSE_v_LD_z_predicted',
                             'JSE_v_x', 'JSE_v_y', 'JSE_v_z',
                             'JSE_p_x', 'JSE_p_y', 'JSE_p_z',
                             'JSE_acc_x', 'JSE_acc_y', 'JSE_acc_z',]
        self.logging_data = [0.0] * len(self.logging_list)

    def init(self):
        self.time_course = [0]
        self.acc_body = [np.array([0, 0, 0])]
        self.quat = [np.array([0, 0, 0, 1])]

        self.R_TO = Rotation.from_quat(self.quat[0]).as_matrix()
        self.R_TO_old = self.R_TO
        self.R_LD = Rotation.from_quat(self.quat[0]).as_matrix()
        self.R_LD_old = self.R_LD

        self.correction_matrix = Rotation.from_quat(self.quat).as_matrix()
        self.altitude_error = 0
        self.directional_error = 0

        self.R_c_ini = Rotation.from_quat(self.quat[0]).as_matrix()
        self.landing_speed_prediction = 3

        self.landing_speed_old_prediction = 3
        self.landing_speed_old_original = 3

        self.takeoff_velocity_prediction = np.array([0, 0, 3])
        self.takeoff_velocity_original = np.array([0, 0, 3])
        self.landing_velocity_prediction = np.array([0, 0, -3])
        self.landing_location_prediction = np.array([0, 0, 0])

        self.quat_corr = np.array([0, 0, 0, 1])

        self.time_cost = 0
        self.height = 0.458

    def update(self, t, acc, quat, estimator_state):
        acc = acc - self.acc_bias
        start_time = time.time()
        # time = 0
        # acc = np.array([0, 0, 0])
        # quat = np.array([0, 0, 0, 1])

        if estimator_state == 21:
            self._takeoff(quat)

        if self.aerial_start_flag:
            # append
            self.time_course.append(t)
            self.acc_body.append(acc)
            self.quat.append(quat)
        else:
            # first time (this is the takeoff timestamp)
            self.aerial_start_flag = True

            # clean and init the arrays
            self.time_course = [t]
            self.acc_body = [acc]
            self.quat = [quat]

        if estimator_state == 12:
            self._landing(quat)
            self.aerial_start_flag = False

        self.current_state()

        self.time_cost = time.time() - start_time
        self.logging_data = [self.time_cost,
                             self.takeoff_velocity_prediction[0], self.takeoff_velocity_prediction[1], self.takeoff_velocity_prediction[2],
                             self.landing_velocity_prediction[0], self.landing_velocity_prediction[1], self.landing_velocity_prediction[2],
                             self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                             self.current_position[0], self.current_position[1], self.current_position[2],
                             acc[0], acc[1], acc[2], ]

    def _landing(self, quat):
        # quat = np.array([0, 0, 0, 1])
        self.R_LD_old = self.R_LD
        self.R_LD = Rotation.from_quat(quat).as_matrix()

    def _takeoff(self, quat):
        self.iteration_count = 0
        # update the takeoff attitude
        self.R_TO_old = self.R_TO
        self.R_TO = Rotation.from_quat(quat).as_matrix()

        position = np.array([0, 0, 0])
        velocity = np.array([0, 0, -4])
        v_TO_unit = np.array([0, 0, 1])

        directional_error = 10
        altitude_error = 10
        height = 0

        landing_location_old = self.landing_location_prediction

        R_c = self.R_c_ini
        landing_speed_old = self.landing_speed_prediction
        self.landing_speed_old_original = landing_speed_old

        while directional_error > self.direction_error_limit or abs(altitude_error) > self.altitude_error_limit:
            directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height = self._prediction_error(R_c, landing_speed_old, landing_location_old)
            temp_axis = np.cross(v_LD_unit_predicted, v_LD_unit)
            R_c = np.matmul(Rotation.from_rotvec(temp_axis * - self.direction_gain).as_matrix(), R_c)
            landing_speed_old = landing_speed_old - altitude_error * self.altitude_gain

            if self.iteration_count == 0:
                landing_speed_original = np.linalg.norm(velocity)
                takeoff_speed_original = math.sqrt(self.leg_efficiency * landing_speed_original * landing_speed_original)
                self.takeoff_velocity_original = v_TO_unit * takeoff_speed_original
            self.iteration_count += 1
            if self.iteration_count > self.max_iteration_count:
                break

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

        self.quat_corr = Rotation.from_rotvec(
            Rotation.from_matrix(self.correction_matrix).as_rotvec() * self.attitude_correction_gain).as_quat(canonical=False)

    def _prediction_error(self, R_c, landing_speed_old, landing_location_old):
        # landing_location_old = np.array([0, 0, 0])

        z_b_LD_old = np.matmul(R_c, np.matmul(self.R_LD_old, self.e_3))
        z_b_TO_old = np.matmul(R_c, np.matmul(self.R_TO_old, self.e_3))
        z_b_LD = np.matmul(R_c, np.matmul(self.R_LD, self.e_3))
        z_b_TO = np.matmul(R_c, np.matmul(self.R_TO, self.e_3))
        takeoff_speed_old = math.sqrt(self.leg_efficiency * landing_speed_old * landing_speed_old)
        v_TO_unit_old, v_LD_unit_old = self._compute_velocities(z_b_LD_old, z_b_TO_old)
        v_TO_unit, v_LD_unit = self._compute_velocities(z_b_LD, z_b_TO)
        takeoff_velocity_old = v_TO_unit_old * takeoff_speed_old

        position = landing_location_old
        velocity = takeoff_velocity_old
        # acc_world = np.matmul(R_c, np.matmul(Rotation.from_quat(self.quat[index]).as_matrix(), self.acc_body[index])) - self.e_3 * self.g
        height = 0
        index = 0
        for t in self.time_course:

            R = Rotation.from_quat(self.quat[index]).as_matrix()
            if index == 0:
                pass
            else:
                acc_world = np.matmul(R_c, np.matmul(R, self.acc_body[index])) - self.e_3 * self.g
                velocity = velocity + acc_world * (t - self.time_course[index-1])
                position = position + velocity * (t - self.time_course[index-1])

                if position[2] > height:
                    height = position[2]
            index += 1

        landing_velocity_predicted = velocity
        v_LD_unit_predicted = landing_velocity_predicted / np.linalg.norm(landing_velocity_predicted)

        altitude_error = position[2]
        directional_error = angle_between_vectors(v_LD_unit_predicted, v_LD_unit) * 180/math.pi

        return directional_error, altitude_error, v_LD_unit_predicted, v_LD_unit, position, velocity, v_TO_unit, height

    def current_state(self):
        # run this after self.update()
        if self.RD_aerial_start_flag.step(self.aerial_start_flag):  # takeoff timestamp
            self.current_position = self.landing_location_prediction
            self.current_velocity = self.takeoff_velocity_prediction
        elif self.aerial_start_flag:  # after takeoff
            R = Rotation.from_quat(self.quat[-1]).as_matrix()
            acc_world = np.matmul(R, self.acc_body[-1]) - self.e_3 * self.g
            self.current_velocity = self.current_velocity + acc_world * (self.time_course[-1] - self.time_course[-2])
            self.current_position = self.current_position + self.current_velocity * (self.time_course[-1] - self.time_course[-2])
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