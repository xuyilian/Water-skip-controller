import time
import math
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from rislab_lib.Joystick.joystick_3 import PygameJoystick
from RisLib import savemat

from scipy.spatial.transform import Rotation

from jumping.jumping_model import RealTimeSleeper
from jumping.jumping_model import Differentiator3 as Differentiator
from jumping.jumping_model import saturation

from jumping.jumping_estimator_v2 import RiseDetect
from jumping.jumping_estimator_v2 import UdpRigidBodies3 as UdpRigidBodies
from jumping.jumping_estimator_v2 import DataProcessor2 as DataProcessor


class RobotState:
    def __init__(self):
        self.p = np.array([0.0, 0.0, 0.0])
        self.p1 = np.array([0.0, 0.0, 0.0])
        self.robot_R = Rotation.from_quat([0, 0, 0, 1])
        self.robot_quat = self.robot_R.as_quat()


def rotx_2d(angle_rad: float):
    CA = math.cos(angle_rad)
    SA = math.sin(angle_rad)
    return np.array([[CA, -SA], [SA, CA]])


def wait_for_start(PJ: PygameJoystick):
    print('Press cross button to continue ...')
    while not PJ.get_key('Cross'):
        PJ.step()
        time.sleep(0.1)


if __name__ == '__main__':

    # ---------------- user config (same style as new.py) ----------------
    operation_direction = 1

    uri = 'radio://0/71/2M'
    udp_enable_flag = True
    rigid_body_index = 3

    # external pose feed
    extpose_flag = True
    extpose_waiting_cycles = 10

    # loop
    sample_time = 0.01  # 50 Hz to reduce radio load
    external_loop_freq = 1.0 / sample_time

    # command limits
    VXY_MAX = 5
    VZ_MAX = 5
    YAWRATE_MAX = 120.0  # deg/s

    DEADZONE = 0.1

    # position outer-loop gains: v = K * (p_des - p)
    KP_XY = 1.0
    KP_Z = 80.0

    # initial targets
    desired_x, desired_y, desired_z = 0.0, 0.0, 1
    desired_yaw = 0.0  # rad

    # ---------------- init: joystick ----------------
    PJ = PygameJoystick()
    wait_for_start(PJ)

    # edge detectors (reference style)
    RD_circle_button = RiseDetect()
    RD_square_button = RiseDetect()

    # ---------------- init: mocap/udp ----------------
    RS_mocap = RobotState()

    if udp_enable_flag:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        DP = DataProcessor(UDP.len_data)
        udp_time = 0.0
    else:
        UDP = None
        DP = DataProcessor(14)
        udp_time = 0.0

    Diff_X = Differentiator(diff_steps=2)
    Diff_Y = Differentiator(diff_steps=2)
    Diff_Z = Differentiator(diff_steps=2)

    # ---------------- init: CF link (no LoggingCore) ----------------
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')

    # Parameters to set after connecting (tune here)
    # NOTE: values are strings as required by cf.param.set_value
    parameter_list = {
        # ----- Position loop (posCtlPid) PD -----
        'posCtlPid.xKp': '30',
        'posCtlPid.xKd': '0',
        'posCtlPid.yKp': '30',
        'posCtlPid.yKd': '0',
        'posCtlPid.zKp': '2',
        'posCtlPid.zKd': '0',

        # ----- Velocity loop (velCtlPid) PD -----
        #'velCtlPid.vxKp': '25',
        #'velCtlPid.vxKd': '0',
        #'velCtlPid.vyKp': '25',
        #'velCtlPid.vyKd': '0',
        #'velCtlPid.vzKp': '25',
        #'velCtlPid.vzKd': '0',

        # ----- Thrust constants (posCtlPid) -----
        # Base thrust and minimum thrust (uint16)
        'posCtlPid.thrustBase': '40000',
        'posCtlPid.thrustMin': '10000',

        # ----- Attitude loop (pid_attitude) PD -----
        'pid_attitude.roll_kp': '6',
        'pid_attitude.roll_kd': '0',
        'pid_attitude.pitch_kp': '6',
        'pid_attitude.pitch_kd': '0',
        'pid_attitude.yaw_kp': '6',
        'pid_attitude.yaw_kd': '0.35',

        'stabilizer.estimator': '2',
    }

    # ---------------- init: saver ----------------
    saver = savemat.DataSaver(
        'Abs_time', 'udp_time',
        *tuple(DP.logging_list),

        'desired_x', 'desired_y', 'desired_z', 'desired_yaw',
        'mocap_x', 'mocap_y', 'mocap_z',
        'mocap_vx', 'mocap_vy', 'mocap_vz',

        'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_yawrate',
        'ls_x', 'ls_y', 'rs_x', 'rs_y',
    )

    # ---------------- timing ----------------
    RTS = RealTimeSleeper(sample_time)
    RTS.init()
    start_time = RTS.loop_start_time

    # ---------------- state ----------------
    armed = False
    enabled_send = False

    print('Connecting to', uri)

    try:
        with SyncCrazyflie(uri, cf=cf) as scf:
            cf = scf.cf

            # set parameters (if any)
            if parameter_list:
                print('setting Crazyflie parameters...')
                for k, v in parameter_list.items():
                    try:
                        cf.param.set_value(k, str(v))
                    except Exception as e:
                        print(f'  [param skip] {k}={v} ({e})')
                print('parameters set')

            # ensure motors are not running at start
            try:
                cf.commander.send_stop_setpoint()
                time.sleep(0.05)
            except Exception:
                pass

            while True:
                Abs_time = RTS.loop_start_time - start_time

                PJ.step()

                # ---- buttons ----
                circle = bool(PJ.get_key('Circle'))
                square = bool(PJ.get_key('Square'))
                circle_rise = RD_circle_button.step(circle)
                square_rise = RD_square_button.step(square)

                # Circle rising edge: arm once
                if circle_rise and (not armed):
                    try:
                        cf.platform.send_arming_request(True)
                        armed = True
                        enabled_send = True
                        print('Armed')
                    except Exception as e:
                        print(f'Arming request failed: {e}')

                # Square: disarm + stop + exit
                if square_rise:
                    if armed:
                        try:
                            cf.platform.send_arming_request(False)
                            armed = False
                            print('Disarmed')
                        except Exception as e:
                            print(f'Disarm request failed: {e}')

                    try:
                        cf.commander.send_stop_setpoint()
                        time.sleep(0.05)
                    except Exception:
                        pass
                    break

                #enabled_send = circle_rise and armed

                # ---- mocap ----
                mocap_x = mocap_y = mocap_z = 0.0
                mocap_vx = mocap_vy = mocap_vz = 0.0

                if udp_enable_flag:
                    data_raw, udp_time = UDP.get_data()
                    data = DP.process_data(data_raw)

                    # validate rigid body index
                    if rigid_body_index not in data:
                        if not hasattr(RTS, 'printed_rb_indices'):
                            RTS.printed_rb_indices = True
                            try:
                                print(f"UDP rigid body indices available: {list(data.keys())}")
                            except Exception:
                                print('UDP rigid body indices available: <unknown>')
                        RTS.sleep()
                        continue

                    mocap_x = float(data[rigid_body_index]['x'])
                    mocap_y = float(data[rigid_body_index]['y'])
                    mocap_z = float(data[rigid_body_index]['z'])

                    # update velocity estimate from differentiated position
                    Diff_X.step(mocap_x, udp_time)
                    Diff_Y.step(mocap_y, udp_time)
                    Diff_Z.step(mocap_z, udp_time)
                    mocap_vx, mocap_vy, mocap_vz = Diff_X.data_rate, Diff_Y.data_rate, Diff_Z.data_rate

                    # update mocap quaternion if available
                    try:
                        RS_mocap.robot_R = Rotation.from_quat([
                            data[rigid_body_index]['qx'],
                            data[rigid_body_index]['qy'],
                            data[rigid_body_index]['qz'],
                            data[rigid_body_index]['qw'],
                        ])
                        RS_mocap.robot_quat = RS_mocap.robot_R.as_quat()
                    except Exception:
                        pass

                    # extpose feed (position + quaternion)
                    if extpose_flag:
                        if not hasattr(RTS, 'loop_flag'):
                            RTS.loop_flag = 0
                        RTS.loop_flag += 1
                        if RTS.loop_flag % extpose_waiting_cycles == 2:
                            cf.extpos.send_extpose(
                                mocap_x, mocap_y, mocap_z,
                                RS_mocap.robot_quat[0], RS_mocap.robot_quat[1],
                                RS_mocap.robot_quat[2], RS_mocap.robot_quat[3],
                            )

                # ---- joystick -> desired target update (same idea as new.py) ----
                JSL_x = operation_direction * float(PJ.get_key('LeftStickY'))
                JSL_y = operation_direction * float(PJ.get_key('LeftStickX'))

                if abs(JSL_x) < DEADZONE:
                    JSL_x = 0.0
                if abs(JSL_y) < DEADZONE:
                    JSL_y = 0.0

                desired_velocity_world = 0.05 * np.matmul(rotx_2d(desired_yaw), np.array([JSL_x, JSL_y]))
                desired_x = saturation(desired_x + desired_velocity_world[0] * sample_time, 1.5, -1.5)
                desired_y = saturation(desired_y + desired_velocity_world[1] * sample_time, 1.5, -1.5)

                rs_y = float(PJ.get_key('RightStickY'))
                if abs(rs_y) < DEADZONE:
                    rs_y = 0.0
                desired_z = saturation(desired_z + (-rs_y) * sample_time, 1.7, 0.0)

                rs_x = float(PJ.get_key('RightStickX'))
                if abs(rs_x) < DEADZONE:
                    rs_x = 0.0
                cmd_yawrate = saturation(rs_x * YAWRATE_MAX, YAWRATE_MAX, -YAWRATE_MAX)

                # ---- position error -> velocity command ----
                err_x = desired_x - mocap_x
                err_y = desired_y - mocap_y
                err_z = desired_z - mocap_z

                cmd_vx = saturation(KP_XY * err_x, VXY_MAX, -VXY_MAX)
                cmd_vy = saturation(KP_XY * err_y, VXY_MAX, -VXY_MAX)
                cmd_vz = saturation(KP_Z * err_z, VZ_MAX, -VZ_MAX)

                # ---- send ----
                #if enabled_send:
                #    cf.commander.send_velocity_world_setpoint(cmd_vx, cmd_vy, cmd_vz, cmd_yawrate)
                #else:
                #    cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)

                if enabled_send:
                    cf.commander.send_position_setpoint(desired_x, desired_y, desired_z, 0)
                else:
                    cf.commander.send_position_setpoint(0.0, 0.0, 0.0, 0.0)    

                saver.add_elements(
                    Abs_time, float(udp_time),
                    *tuple(DP.logging_data),

                    desired_x, desired_y, desired_z, desired_yaw,
                    mocap_x, mocap_y, mocap_z,
                    float(mocap_vx), float(mocap_vy), float(mocap_vz),

                    cmd_vx, cmd_vy, cmd_vz, cmd_yawrate,
                    float(JSL_x), float(JSL_y), float(rs_x), float(rs_y),
                )

                RTS.sleep()

    finally:
        # robust cleanup to avoid segfault/leaked semaphores
        try:
            cf.commander.send_stop_setpoint()
            time.sleep(0.05)
        except Exception:
            pass

        if udp_enable_flag and UDP is not None:
            try:
                UDP.stop_thread()
                time.sleep(0.05)
            except Exception:
                pass

        try:
            PJ.quit()
        except Exception:
            pass

        try:
            saver.save2mat('DataExchange/')
        except Exception as e:
            print(f'Data save failed: {e}')

        print('Stopped.')