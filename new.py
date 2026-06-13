import time
import math
import numpy as np
import scipy.io

from rislab_lib.Joystick.joystick_3 import PygameJoystick
from RisLib.cflog import LoggingCore
from RisLib import savemat

from Library.quat import quat_operation
from scipy.spatial.transform import Rotation

from jumping.jumping_model import RealTimeSleeper
from jumping.jumping_model import Differentiator3 as Differentiator
from jumping.jumping_model import saturation

from jumping.jumping_estimator_v2 import LinearJumpingControllerLeg
from jumping.jumping_estimator_v2 import DirectJumpingControllerLeg
from jumping.jumping_estimator_v2 import InPlaneJumpingModel
from jumping.jumping_estimator_v2 import InPlaneJumpingModelLUT
# from jumping.jumping_estimator_v2 import JumpingStateEstimator2TOF_leg as JumpingStateEstimator
from jumping.jumping_estimator_v2 import RiseDetect
from jumping.jumping_estimator_v2 import landing_state_prediction_2
from jumping.jumping_estimator_v2 import FlightController
from jumping.jumping_estimator_v2 import float_float_compress
from jumping.jumping_estimator_v2 import UdpRigidBodies3 as UdpRigidBodies
from jumping.jumping_estimator_v2 import DataProcessor2 as DataProcessor
from jumping.jumping_estimator_v2 import ParameterSetQueue

from jumping.jumping_estimator_v3 import JumpingHeightController2
from jumping.jumping_estimator_v3 import PoweredClimbingTimer2
from jumping.jumping_estimator_v3 import JumpingHeightRecorder2
from jumping.jumping_estimator_v3 import JumpingStateTrackerLog
from jumping.jumping_estimator_v3 import JumpingStateEstimatorLegMeasurement3_stance_time as JumpingStateEstimatorLegMeasurement


class RobotState:
    def __init__(self):
        self.p = np.array([0, 0, 0])
        self.p1 = np.array([0, 0, 0])
        self.robot_R = Rotation.from_quat([0, 0, 0, 1])
        self.robot_quat = self.robot_R.as_quat()


def roty(angle):
    CA = math.cos(angle)
    SA = math.sin(angle)
    return np.array([[CA,  0., SA], [0.,  1., 0.], [-SA, 0., CA]])


def rotx_2d(angle):
    CA = math.cos(angle)
    SA = math.sin(angle)
    return np.array([[CA, -SA], [SA, CA]])


def robot_stop():
    lc.cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.02)
    #lc.cf.param.set_value('sensfusion6.gc', '1')  # enable gravity compensation in estimator
    time.sleep(0.02)
    lc.stop()


def wait_for_start():
    print('Press cross button to continue ...')
    while not PJ.get_key('Cross'):
        PJ.step()
        time.sleep(0.1)



if __name__ == '__main__':

    operation_direction = 2

    extpose_flag = False
    extpose_waiting_cycles = 10

    RS_mocap = RobotState()

    udp_enable_flag = False
    rigid_body_index = 1
    sample_time = 0.01
    external_loop_freq = 1.0 / sample_time 

    hoppingcontrol = False

    PJ = PygameJoystick()
    wait_for_start()
    jumping_mode = False
    G = 9.81
    leg_efficiency = 1.00
    leg_length = 0.31
    RTS = RealTimeSleeper(sample_time)
    FC = FlightController(100, 100, 
                          100, 101, 
                          45000, 15000, 55000, 
                          0, 25)
    
    #FC = FlightController(10, 11, 
    #                      1, 11, 
    #                      0, 0, 0, 
    #                      0, 25)
    JSTO = JumpingStateTrackerLog()

    k1, k2 = 1.541, 2.377
    IPJM = InPlaneJumpingModel(k1, k2)
    
    DJC = DirectJumpingControllerLeg(model=IPJM, velocity_gain=0.1, velocity_limit_ratio=1, g=0.93)



    predicted_next_landing_x, predicted_next_landing_y = 0, 0
    predicted_next_landing_vz = -3.5
    predicted_next_landing_vz1 = -3.5
    desired_x, desired_y, desired_z = 0, 0, 0.35
    desired_x_dot, desired_y_dot, desired_z_dot = 0, 0, 0
    desired_yaw = 0

    desired_roll, desired_pitch = 0, 0

    ld_roll = 0

    controller_start_flag = False
    armed = False
    roll_state = 6.2
    pitch_state = 6.4
    cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 0, 0, 0, 0
    
    landing_att_x, landing_att_y, landing_att_z= 0, 0, 0
    takeoff_att_x, takeoff_att_y, takeoff_att_z= 0, 0, 0

    ke = 0.000607
    delta = 33
    fjump_value = 6
    ready_to_drop = False
    jumping_counter = 0

    RD_circle_button = RiseDetect()
    RD_triangle_button = RiseDetect()
    RD_option_button = RiseDetect()
    RD_option_up = RiseDetect()
    RD_option_down = RiseDetect()
    RD_option_left = RiseDetect()
    RD_option_right = RiseDetect()
    RD_right = RiseDetect()
    RD_right2 = RiseDetect()
    RD_left = RiseDetect()
    RD_rightstick_down = RiseDetect()
    RD_rightstick_up = RiseDetect()

    if udp_enable_flag:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        DP = DataProcessor(UDP.len_data)
        udp_time = 0
    else:
        # generate an empty DP object
        udp_time = 0
        DP = DataProcessor(14)

    Diff_X, Diff_Y, Diff_Z = Differentiator(diff_steps=2), Differentiator(diff_steps=2), Differentiator(diff_steps=2)
    # 注释掉基于位置微分的速度估计，直接使用动捕回传速度
    mocap_vx, mocap_vy, mocap_vz = 0.0, 0.0, 0.0
    

    if True:

        logging_list = {'stateEstimateZ.quat': 'uint32_t', 'pm.vbat': 'FP16',
                        #'stabilizer.roll':'FP16','stabilizer.pitch':'FP16','stabilizer.yaw':'FP16',
                        #'acc.x': 'FP16', 'acc.y': 'FP16', 'acc.z': 'FP16', 
                        #'motor.m1req':'FP16','motor.m2req':'FP16','motor.m3req':'FP16','motor.m4req':'FP16',
                        'foclog.angle': 'FP16', 'foclog.velo': 'FP16', #'foclog.foct': 'FP16',
                        'CurrentSense.current':'FP16','CurrentSense.power':'FP16',
                        #'lcontrol.e':'FP16', 'lcontrol.K':'FP16',
                        'stabilizer.phase':'uint8_t',#'stabilizer.r':'uint8_t',

                        }

        lc = LoggingCore('radio://0/16/2M', 10, logging_list)


        parameter_list = {'stabilizer.kpx': '32000',
                        'stabilizer.kpy': '30000',
                        'stabilizer.kpz': '0',
                        #'stabilizer.kpz': '2000',

                        'stabilizer.kplz': '3500',
                        'stabilizer.kdlz': '28',
                        'stabilizer.kilz': '0.2',
                        'stabilizer.kdx': '50',
                        'stabilizer.kdy': '50',
                        'stabilizer.kdz': '0',
                        #'stabilizer.kdz': '15',
                        'stabilizer.qyo1': '0',
                        'powerDist.idleThrust': '0',
                        #'stabilizer.fdamp': '0.75',
                        #'stabilizer.fdamp2': '16',
                        #'stabilizer.fdamp1': '0',
                        #'stabilizer.fstop': '0.8',
                        #'stabilizer.fjump': '4.3',
                        'stabilizer.JSTOz': '2.0',
                        'stabilizer.fvelth': '-60.0',
                        #'stabilizer.fdelth': '20.0',
                
                        #'foc.y':'0.000607',
                        'stabilizer.exfreq': f'{external_loop_freq}',
                        'powerDist.minThrust':'1500',
                        'foc.p':'60',
                        #'foc.d':'0.02',
                        #'foc.i':'50',
                        #"foc.e": "1",
                        }

        lc.pre_set_parameter(parameter_list)
        #PSQ = ParameterSetQueue(lc)

    if True:
        saver = savemat.DataSaver(
            'Abs_time', *tuple(lc.temp_keys), 'udp_time',
            *tuple(DP.logging_list), 

            'desired_x', 'desired_y', 'desired_z', 'desired_yaw',
            'robot_roll', 'robot_pitch', 'robot_yaw', 
            #'mocap_roll','mocap_pitch','mocap_yaw',

            'b1_vx', 'b1_vy', 'b1_vz',
            'b1_x', 'b1_y', 'b1_z',
            'cmd_roll', 'cmd_pitch', 'cmd_yaw', 'cmd_thrust',

            'prevz','prex','prey',
            'lattx','latty','lattz',
            'tattx','tatty','tattz',
        )

    lc.cf.commander.send_setpoint(0, 0, 0, 0)

    RTS.init()
    start_time = RTS.loop_start_time
    Abs_time = 0
    loop_flag = 0
    state = 0
    state_old = 0
    R1_key = False
    dir_state = 0  # 0:none, 1:Up, 2:Down, 3:Left, 4:Right
    # --- Landing->Takeoff alignment gating ---
    landing_hold_zero_cmd = False      # landing 阶段低高度时强制 0cmd
    takeoff_align_active = False       # 速度归零后开始对齐起跳姿态
    stabilizer_r_sent = False          # 防止重复发送 stabilizer.r
    foc_k_sent = False
    VEL_ZERO_EPS = 0.03                # m/s，速度接近 0 的阈值

    while lc.is_connected:

        loop_flag += 1
        Abs_time = RTS.loop_start_time - start_time

        PJ.step()

        r1 = PJ.get_key('R1')
        l1 = PJ.get_key('L1')

        r1_rise = RD_right.step(r1)
        l1_rise = RD_left.step(l1)

        if r1_rise and l1:
            lc.cf.param.set_value("stabilizer.p", "3")
            print('landing set')

        elif l1_rise:
            lc.cf.param.set_value("stabilizer.p", "2")

        elif r1_rise:
            lc.cf.param.set_value("stabilizer.p", "1")

        if udp_enable_flag:
            data_raw, udp_time = UDP.get_data()
            data = DP.process_data(data_raw)
            robot_R = Rotation.from_quat([data[rigid_body_index]['qx'], data[rigid_body_index]['qy'], data[rigid_body_index]['qz'], data[rigid_body_index]['qw'], ])
            robot_zb = robot_R.as_matrix()[:,2]
            mocap_yaw, mocap_pitch, mocap_roll = robot_R.as_euler('ZYX', degrees=True)
            # robot_R = Rotation.from_matrix(np.matmul(robot_R.as_matrix(), correction_matrix_mocap))  # attitude correction, this is correct
            pos_raw = np.array([
            data[rigid_body_index]['x'],
            data[rigid_body_index]['y'],
            data[rigid_body_index]['z']
            ])
            MAX_JUMP = 0.2
            alpha = 0.2

            if not hasattr(RS_mocap, 'p_filt'):
                RS_mocap.p_filt = pos_raw.copy()

            jump = np.linalg.norm(pos_raw - RS_mocap.p_filt)
            is_outlier = jump > MAX_JUMP or np.isnan(pos_raw).any()

            if not is_outlier:
                RS_mocap.p_filt = alpha * pos_raw + (1 - alpha) * RS_mocap.p_filt

            Diff_X.step(RS_mocap.p_filt[0], udp_time)
            Diff_Y.step(RS_mocap.p_filt[1], udp_time)
            Diff_Z.step(RS_mocap.p_filt[2], udp_time)

            RS_mocap.p = RS_mocap.p_filt
            RS_mocap.p1 = np.array([
                Diff_X.data_rate,
                Diff_Y.data_rate,
                Diff_Z.data_rate
            ])
            #mocap_vx, mocap_vy, mocap_vz = data[rigid_body_index]['vx'], data[rigid_body_index]['vy'], data[rigid_body_index]['vz']
            #RS_mocap.p = np.array([data[rigid_body_index]['x'], data[rigid_body_index]['y'], data[rigid_body_index]['z']])
            #RS_mocap.p1 = np.array([mocap_vx, mocap_vy, mocap_vz])


         
        if True:
            logged_data = lc.get_logged_data()
            q_imu = quat_operation.quaternion_decompress(logged_data['stateEstimateZ.quat'])
            robot_R_imu = Rotation.from_quat(q_imu)
            robot_yaw, robot_pitch, robot_roll = robot_R_imu.as_euler('ZYX', degrees=False)

            state = logged_data['stabilizer.phase']



            #acc = np.array([logged_data['acc.x'], logged_data['acc.y'], logged_data['acc.z']]) * G  # unit: m/s^2

        if True:    
            JSL_x, JSL_y = operation_direction * PJ.get_key('LeftStickY'), operation_direction * PJ.get_key('LeftStickX')

            if abs(JSL_x) < 0.1:
                 JSL_x = 0
            if abs(JSL_y) < 0.1:
                 JSL_y = 0


            if RD_option_up.step(PJ.get_key('Up')):
                ld_roll = 180
                print('180')
            elif RD_option_down.step(PJ.get_key('Down')):
                ld_roll = 0
                print('0')
            elif RD_option_left.step(PJ.get_key('Left')):
                ld_roll = -90
                print('-90')
            elif RD_option_right.step(PJ.get_key('Right')):
                ld_roll = 90
                print('90')


            desired_velocity_world = 0.05*np.matmul(rotx_2d(desired_yaw), np.array([JSL_x, JSL_y]))

            # position
            desired_x_dot = desired_velocity_world[0]
            desired_x = desired_x_dot * sample_time + desired_x
            desired_y_dot = desired_velocity_world[1]
            desired_y = desired_y_dot * sample_time + desired_y

            if udp_enable_flag:
                desired_x = saturation(desired_x, 1.5, -1.5)
                desired_y = saturation(desired_y, 1.5, -1.5)


            JSR_y_raw = PJ.get_key('RightStickY')

            # 右摇杆向上 +0.1，向下 -0.1（边沿触发）
            if RD_rightstick_up.step(JSR_y_raw < -0.8):
                fjump_value += 0.5
                print(f"fjump -> {fjump_value}")

            if RD_rightstick_down.step(JSR_y_raw > 0.8):
                fjump_value -= 0.5
                print(f"fjump -> {fjump_value}")

        
            
            if extpose_flag and udp_enable_flag:
                if RTS.loop_flag % extpose_waiting_cycles == 2:
                    RS_mocap.robot_quat = RS_mocap.robot_R.as_quat()
                    lc.cf.extpos.send_extpose(0, 0, 0,
                                          RS_mocap.robot_quat[0], RS_mocap.robot_quat[1],
                                          RS_mocap.robot_quat[2], RS_mocap.robot_quat[3], )
            
            if RD_circle_button.step(PJ.get_key('Circle')):
                if controller_start_flag:
                    #controller_start_flag = False

                    print('=> Hopping stop')
                    hoppingcontrol = False
                    lc.cf.param.set_value("stabilizer.p", "4")
                    #if armed:
                    #    lc.cf.platform.send_arming_request(False)
                    #    armed = False
                else:
                
                    print('=> controller start')


                    lc.cf.param.set_value("foc.e", "1")
                    #lc.cf.param.set_value("stabilizer.p", "4")
                    #hoppingcontrol = True
                    print('=> Hopping start')
                    #lc.cf.param.set_value("foc.j", "1")
                    lc.cf.param.set_value("foc.t", "16")
                    lc.cf.param.set_value("foc.j", "1")
                    #lc.cf.param.set_value("foc.l", "1")
                    #lc.cf.param.set_value("foc.t", "0.6")
                    #cf.param.set_value("stabilizer.finit", "1")

                    controller_start_flag = True

                    if not armed:

                        lc.cf.platform.send_arming_request(True)
                        armed = True


            if RD_option_button.step(PJ.get_key('Option')):
                if jumping_mode:
                    jumping_mode = False
                    print('=> flight mode')
                    ready_to_drop = False
                #PSQ.enqueue('sensfusion6.gc', '1')  # enable gravity compensation in estimator
                else:
                    jumping_mode = True
                    print('=> jumping mode')
                    ready_to_drop = True
            
            if controller_start_flag and udp_enable_flag and not jumping_mode:
                FC.update(desired_x, desired_y, desired_z,
                      desired_x_dot, desired_y_dot, desired_z_dot, desired_yaw,
                      data[rigid_body_index]['x'], data[rigid_body_index]['y'], data[rigid_body_index]['z'],
                      Diff_X.data_rate, Diff_Y.data_rate, Diff_Z.data_rate,
                      robot_yaw)
                
                if RD_triangle_button.step(PJ.get_key('Triangle')):
                    lc.cf.param.set_value("stabilizer.r", "1")
                    print('[Triangle] jump')

            # ========== jumping controller update ==========
            if controller_start_flag and jumping_mode and udp_enable_flag:
                if ready_to_drop:
                    jumping_counter = 0
                    JSTO.init()
                    DJC.init()
                    # IMPORTANT: only run init once when entering jumping_mode
                    ready_to_drop = False
                    

                # Triangle: trigger jump (only once on rising edge)
                if RD_triangle_button.step(PJ.get_key('Triangle')):
                    lc.cf.param.set_value("stabilizer.r", "1")
                    print('[Triangle] jump')

                current_position = np.array([
                    data[rigid_body_index]['x'],
                    data[rigid_body_index]['y'],
                    data[rigid_body_index]['z']
                ])

                # --- Safety: XY越界自动停机 ---
                #if abs(current_position[0]) > 2.0 or abs(current_position[1]) > 2.0:
                #    lc.cf.commander.send_setpoint(0, 0, 0, 0)
                #    lc.cf.param.set_value("foc.e", "0")
                #    print('stop')
                #    hoppingcontrol = False
                #    controller_start_flag = False

                #    continue
                # use differentiated mocap velocity
                current_velocity = RS_mocap.p1

                


                #FC.update(desired_x, desired_y, desired_z,
                #      desired_x_dot, desired_y_dot, desired_z_dot, desired_yaw,
                #      data[rigid_body_index]['x'], data[rigid_body_index]['y'], data[rigid_body_index]['z'],
                #      Diff_X.data_rate, Diff_Y.data_rate, Diff_Z.data_rate,
                #      robot_yaw)
                
                DJC.set_reference(desired_x, desired_y, 0.4)
                if state == 3:
                    DJC.update_zbw_sym(robot_roll, robot_pitch, robot_yaw)

                #if state == 5:

                    #if state_old != 5:
                    #    lc.cf.param.set_value("foc.t", f"{fjump_value}")

                    #predicted_next_landing_vz, predicted_next_landing_x, predicted_next_landing_y, flight_time = \
                    #landing_state_prediction_2(current_position, current_velocity)

                    #DJC.update_landing_state(
                    #    Diff_X.data_rate,
                    #    Diff_Y.data_rate,
                    #    predicted_next_landing_vz,
                    #    predicted_next_landing_x,
                    #    predicted_next_landing_y,
                    #)

                #DJC.jumping_planning()

                DJC.inverse_jumping_model(robot_yaw)

                DJC.lyaw1 = math.degrees(desired_yaw)

                state_old = state   

            elif controller_start_flag and jumping_mode and (not udp_enable_flag):
                # No mocap: do NOT run DJC (avoid using data[]). Still allow Triangle to trigger firmware jump.
                if ready_to_drop:
                    jumping_counter = 0
                    JSTO.init()
                    # keep flags consistent
                    ready_to_drop = False

                if RD_triangle_button.step(PJ.get_key('Triangle')):
                    lc.cf.param.set_value("stabilizer.r", "1")
                    print('[Triangle] jump (no mocap)')


            else:
                # keep JSTO running
                JSTO.update(False)


            if controller_start_flag:
                if jumping_mode:
                    # jumping mode
                    if udp_enable_flag:
                        # === mocap ON: use DJC outputs ===
                        if state == 3:
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = ld_roll, 0, 0, 1000
                        elif (state == 5 )and hoppingcontrol:
                            #cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = DJC.roll+6, DJC.pitch-6, DJC.yaw, 1000
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 0, 0, 0, 1000
                            #cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = FC.roll_flight+4, FC.pitch_flight-4, FC.yaw_flight, 1000
                        elif (state == 5 )and (not hoppingcontrol):
                            #cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = DJC.lroll, DJC.lpitch+3, DJC.lyaw, 1000
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = DJC.lroll1, DJC.lpitch1-3, DJC.lyaw1, 1000
                        else:
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 0, 0, 0, 0
                    else:
                        if state == 1 or state == 2:
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 0, 0, 0, 1000
                        elif state == 3:
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = ld_roll, 0, 0, 1000
                        elif state == 5:
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 0, 0, 0, 000
                        else:
                            cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 0, 0, 0, 0
                else:
                    # flight mode (no position control currently)
                    if hoppingcontrol:
                        cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = FC.roll_flight, FC.pitch_flight, FC.yaw_flight, FC.thrust_flight
                    else:
                        cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = 2, 0, 0, 1000    

            if armed:
                lc.cf.commander.send_setpoint(cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust)
                #lc.cf.commander.send_setpoint(cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust)


            
            




        if True:
            saver.add_elements(
                Abs_time, *tuple(logged_data.values()), udp_time,
                *tuple(DP.logging_data), 

                desired_x, desired_y, desired_z, desired_yaw,

                robot_roll,robot_pitch,robot_yaw,
                #mocap_roll,mocap_pitch,mocap_yaw,
                
                Diff_X.data_rate, Diff_Y.data_rate, Diff_Z.data_rate,
                RS_mocap.p[0],RS_mocap.p[1],RS_mocap.p[2],
                cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust,

                predicted_next_landing_vz,predicted_next_landing_x,predicted_next_landing_y,
                landing_att_x,landing_att_y,landing_att_z,
                takeoff_att_x, takeoff_att_y, takeoff_att_z,
                )
        
        if PJ.get_key('Square'):
            lc.cf.param.set_value("foc.t", "0")
            lc.cf.param.set_value("foc.e", "0")
            if armed:
                lc.cf.platform.send_arming_request(False)
                armed = False
            robot_stop()
            break

        #PSQ.auto_set()
        RTS.sleep()
    PJ.quit()
    saver.save2mat('DataExchange/')
    if udp_enable_flag:
        UDP.stop_thread()