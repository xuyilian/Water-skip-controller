import time
import math
import numpy as np

from rislab_lib.Joystick.joystick_3 import PygameJoystick
from RisLib.cflog import LoggingCore
from RisLib import savemat

from Library.quat import quat_operation
from scipy.spatial.transform import Rotation

from jumping.jumping_model import RealTimeSleeper
from jumping.jumping_model import Differentiator3 as Differentiator
from jumping.jumping_model import saturation
from jumping.jumping_estimator_v2 import FlightController
from jumping.jumping_estimator_v2 import RiseDetect
from jumping.jumping_estimator_v2 import UdpRigidBodies3 as UdpRigidBodies
from jumping.jumping_estimator_v2 import RealTimeProcessor, saturation_fcn, circular_saturation_fcn
from Library.control import GeoController



def rotx_2d(angle):
    CA = math.cos(angle)
    SA = math.sin(angle)
    return np.array([[CA, -SA], [SA, CA]])


# ---- helpers for GeoController ----
def circular_saturation(u_x: float, u_y: float, limit: float):
    n = math.hypot(u_x, u_y)
    if n <= limit or n < 1e-9:
        return u_x, u_y
    s = limit / n
    return u_x * s, u_y * s


def robot_stop(lc: LoggingCore):
    # stop motors safely
    lc.cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.05)
    lc.stop()


def wait_for_start(PJ: PygameJoystick):
    print('Press cross button to continue ...')
    while not PJ.get_key('Cross'):
        PJ.step()
        time.sleep(0.1)


if __name__ == '__main__':

    # ---------------- user config ----------------
    uri = 'radio://0/71/2M'

    operation_direction = 1

    udp_enable_flag = True
    rigid_body_index = 1


    sample_time = 0.01  # 50 Hz
    G = 9.81

    # joystick scaling
    DEADZONE_L = 0.3
    DEADZONE_R = 0.2

    # desired limits
    XY_LIMIT = 1.5
    Z_MIN, Z_MAX = 0.3, 1.4

    # yaw control
    yaw_rate_max = 1000.0  # rad/s

    BODY_BYTES = 20   # 14 or 20 (预留20字节版本)

    extpose_flag = True
    extpose_waiting_cycles = 10

    # ---------------- init ----------------

    PJ = PygameJoystick()
    wait_for_start(PJ)

    RD_circle = RiseDetect()
    RD_triangle = RiseDetect()

    RTS = RealTimeSleeper(sample_time)

    # GeoController (replace FC)
    # Parameters follow the user's working configuration; tune as needed.
    bi_controller_1 = GeoController.Controller(
        sample_time,
        0, 
        0/100, 0/100, 0/100, 50, 0,
        9.35, 0.56, 7.80,
        15000, 1000, 8000, 30000
    )

    # FlightController (FC)
    FC = FlightController(25, 23, 25, 10, 15000, 8000, 30000, 0, 25)

# control mode: 'FC' or 'BI'
    control_mode = 'FC'
# in FC mode: if True, use BI to override roll/pitch (outer-loop attitude override)
    attitude_override = False

# z velocity estimate (RTP doesn't provide Z_F_d in your current log)
    prev_z_f = 0.0

    # YawController
    yaw_controller = GeoController.YawController(sample_time, 0, 10, 5)

    # extra differentiators for R13/R23 and yaw
    # Removed Diff_R13, Diff_R23, prev_yaw as per instructions

    # UDP mocap
    if udp_enable_flag:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        udp_time = 0.0
    else:
        UDP = None
        udp_time = 0.0

    sample_rate_rt = 1.0 / sample_time
    dp1 = RealTimeProcessor(5, 18, 'lowpass', 'cheby2', 85, sample_rate_rt, 0)  # take off
    dp2 = RealTimeProcessor(4, 16, 'lowpass', 'cheby2', 58, sample_rate_rt, 0)

    # Crazyflie logging
    logging_list = {
        'stateEstimateZ.quat': 'uint32_t',
        #'controller.ctr_roll': 'int16_t',
        #'controller.ctr_pitch': 'int16_t',
        #'controller.ctr_yaw': 'int16_t',
        'pm.vbat': 'FP16',
    }

    lc = LoggingCore(uri, 10, logging_list)


    # ---------------- data saver ----------------
    saver = savemat.DataSaver(
        'Abs_time', *tuple(lc.temp_keys), 'udp_time',*tuple(dp2.logging_list),
        'desired_x', 'desired_y', 'desired_z', 'desired_yaw',
        'mocap_x', 'mocap_y', 'mocap_z',
        'mocap_vx', 'mocap_vy', 'mocap_vz',
        'cmd_roll', 'cmd_pitch', 'cmd_yaw', 'cmd_thrust',
        'imu_roll_deg', 'imu_pitch_deg', 'imu_yaw_deg',
        'mocap_roll_deg', 'mocap_pitch_deg', 'mocap_yaw_deg',
        'bi_roll_flight', 'bi_pitch_flight',
        'U_X', 'U_Y', 'U_Z', 'U_yaw',
        'R13', 'R23', 'R13_d', 'R23_d',
        'mocap_yawrate_dps',
    )

    RTS.init()
    start_time = RTS.loop_start_time

    # desired references
    desired_x, desired_y, desired_z = 0.0, 0.0, 0.6
    desired_x_dot, desired_y_dot, desired_z_dot = 0.0, 0.0, 0.0
    desired_yaw = 0.0

    cmd_yawrate_dps = 0.0
    mocap_yawrate_dps = 0.0

    controllerEnable = False

    cmd_roll = cmd_pitch = cmd_yaw = 0.0
    cmd_thrust = 0

    bi_roll_flight = 0.0
    bi_pitch_flight = 0.0

    prev_udp_time = None

    loop_flag = 0

    while lc.is_connected:
        loop_flag += 1
        Abs_time = RTS.loop_start_time - start_time

        PJ.step()

        # toggle controller with Circle
        if RD_circle.step(PJ.get_key('Circle')):
            controllerEnable = not controllerEnable
            if controllerEnable:
                print('controllerEnable -> True')
            else:
                print('controllerEnable -> False')

# Triang
        if RD_triangle.step(PJ.get_key('Triangle')):
            control_mode = 'BI' if control_mode == 'FC' else 'FC'
            print(f"control_mode -> {control_mode}")

            try:
                lc.cf.param.set_value('stabilizer.b', '1' if control_mode == 'BI' else '0')
            except Exception as e:
                print(f"[WARN] set stabilizer.b1 failed: {e}")

        # emergency stop
        if PJ.get_key('Square'):
            robot_stop(lc)
            break

        # ---- mocap (UDP) -> RealTimeProcessor ----
        if udp_enable_flag:
            data_raw, udp_time = UDP.get_data()

            dp1.step(data_raw, True, body_index=rigid_body_index,abstime=udp_time)
            dp2.step(data_raw, True, body_index=rigid_body_index,abstime=udp_time)

            # warmup: use dp1 first, then dp2
            dp_used = dp2 if loop_flag >= 800 else dp1
            angle_yaw_rad = math.radians(dp_used.angleY)
            # mocap Euler angles from mocap quaternion (dp_used.QX/QY/QZ/QW), degrees
            try:
                mocap_yaw_deg, mocap_pitch_deg, mocap_roll_deg = Rotation.from_quat([
                float(dp_used.QX), float(dp_used.QY), float(dp_used.QZ), float(dp_used.QW)
                ]).as_euler('ZYX', degrees=True)
            except Exception:
                mocap_roll_deg = 0.0
                mocap_pitch_deg = 0.0
                mocap_yaw_deg = 0.0
        else:
            RTS.sleep()
            continue

        # ---- CF attitude (yaw) from onboard estimator ----
        logged_data = lc.get_logged_data()
        q_imu = quat_operation.quaternion_decompress(logged_data['stateEstimateZ.quat'])
        robot_R_imu = Rotation.from_quat(q_imu)
        robot_euler_imu = robot_R_imu.as_euler('ZYX', degrees=False)
        robot_yaw = robot_euler_imu[0]

        imu_yaw_rad, imu_pitch_rad, imu_roll_rad = robot_euler_imu  # yaw,pitch,roll (rad)
        imu_roll_deg  = math.degrees(imu_roll_rad)
        imu_pitch_deg = math.degrees(imu_pitch_rad)
        imu_yaw_deg   = math.degrees(imu_yaw_rad)

        if extpose_flag and udp_enable_flag:
            if loop_flag % extpose_waiting_cycles == 2:
                try:
                    lc.cf.extpos.send_extpose(
                float(dp_used.X_F), float(dp_used.Y_F), float(dp_used.Z_F),
                float(dp_used.QX), float(dp_used.QY), float(dp_used.QZ), float(dp_used.QW)
                    )
                except Exception as e:
            # avoid spamming
                    if loop_flag % (extpose_waiting_cycles * 20) == 2:
                        print(f"[WARN] extpose send failed: {e}")
                
        # ---- references from joystick ----
        # yaw (right stick X -> yaw rate)
        JSR_x = float(PJ.get_key('RightStickX'))
        if abs(JSR_x) < 0.1:
            JSR_x = 0.0

        desired_yaw_rate = -yaw_rate_max * JSR_x      # deg/s
        desired_yaw = desired_yaw + desired_yaw_rate * sample_time  # deg

# wrap to [-180, 180)
        if desired_yaw >= 180.0:
            desired_yaw -= 360.0
        elif desired_yaw < -180.0:
            desired_yaw += 360.0

        # XY velocity input (left stick)
        JSL_x = operation_direction * float(PJ.get_key('LeftStickY'))
        JSL_y = operation_direction * float(PJ.get_key('LeftStickX'))
        if abs(JSL_x) < DEADZONE_L:
            JSL_x = 0.0
        if abs(JSL_y) < DEADZONE_L:
            JSL_y = 0.0

        desired_velocity_world = 0.5 * np.matmul(rotx_2d(desired_yaw), np.array([JSL_x, JSL_y]))

        desired_x_dot = desired_velocity_world[0]
        desired_y_dot = desired_velocity_world[1]
        desired_x = saturation(desired_x + desired_x_dot * sample_time, XY_LIMIT, -XY_LIMIT)
        desired_y = saturation(desired_y + desired_y_dot * sample_time, XY_LIMIT, -XY_LIMIT)

        # Z target (right stick Y)
        JSR_y = float(PJ.get_key('RightStickY'))
        if abs(JSR_y) < DEADZONE_R:
            JSR_y = 0.0

        thrust_manual = saturation(round(-JSR_y * 30000), 30000, 0)
        if thrust_manual > 4000 and loop_flag % 100 == 50:
            print('thrust (manual): ', thrust_manual)    
        desired_z = saturation(desired_z - 0.2 * JSR_y * sample_time, Z_MAX, Z_MIN)

        # ---- desired setpoints into controller ----
        # GeoController uses its internal Desired_X/Y/Z; keep it synced
        if controllerEnable:
            bi_controller_1.update_desired_position(desired_x, desired_y, desired_z)
            FC.update(
            desired_x, desired_y, desired_z,
            desired_x_dot, desired_y_dot, desired_z_dot, desired_yaw,
            float(dp_used.X_F), float(dp_used.Y_F), float(dp_used.Z_F),
            float(dp_used.X_F_d), float(dp_used.Y_F_d), float(dp_used.Z_F_d),
            float(angle_yaw_rad),
        )
            yaw_controller.update_desired_state(
                0,
                saturation(
                    yaw_controller.desired_yaw_rate + (-desired_yaw_rate) * sample_time * 20.0,
                    -180, 180
                )
            )
        else:
            bi_controller_1.update_desired_position(0.0, 0.0, 0.6)
            yaw_controller.update_desired_state(0.0, 0.0)

        
        # ---- control ----
        if controllerEnable and udp_enable_flag:

            dt = sample_time
            if prev_udp_time is not None:
                dt = max(1e-4, udp_time - prev_udp_time)
            prev_udp_time = udp_time

            U_X, U_Y, U_Z = bi_controller_1.update_error(dp_used, dt)

            U_X, U_Y = circular_saturation(U_X, U_Y, 300.0)
            U_yaw = yaw_controller.update_error(dp_used)

            bi_pitch_flight = -(U_X * math.cos(angle_yaw_rad) + U_Y * math.sin(angle_yaw_rad))
            bi_roll_flight = -(U_Y * math.cos(angle_yaw_rad) - U_X * math.sin(angle_yaw_rad))

            if control_mode == 'FC':
                cmd_roll = FC.roll_flight
                cmd_pitch = FC.pitch_flight
                cmd_yaw = desired_yaw          # yawrate
                cmd_thrust = round(FC.thrust_flight)
                #cmd_thrust = 1000
            else:
                cmd_roll = float(bi_roll_flight)
                cmd_pitch = float(bi_pitch_flight)
                #cmd_yaw = float(U_yaw)
                cmd_yaw = -desired_yaw_rate/100
                #cmd_thrust = U_Z
                cmd_thrust = thrust_manual

            R13 = float(dp_used.R13_F)
            R23 = float(dp_used.R23_F)
            R13_d = float(dp_used.R13_F_d)
            R23_d = float(dp_used.R23_F_d)
            mocap_yawrate_dps = float(dp_used.angleY_d)   # deg/s
        else:
            U_X = U_Y = 0.0
            U_Z = 0.0
            U_yaw = 0.0
            R13 = R23 = 0.0
            R13_d = R23_d = 0.0
            cmd_roll = 0.0
            cmd_pitch = 0.0
            cmd_yaw = 0.0
            cmd_thrust = 0

        lc.cf.commander.send_setpoint(cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust)

        # ---- save ----
        saver.add_elements(
            Abs_time, *tuple(logged_data.values()), float(udp_time),*tuple(dp_used.logging_data),
            desired_x, desired_y, desired_z, desired_yaw,
            float(dp_used.X_F), float(dp_used.Y_F), float(dp_used.Z_F),
            float(dp_used.X_F_d), float(dp_used.Y_F_d), 0.0,
            float(cmd_roll), float(cmd_pitch), float(cmd_yaw), int(cmd_thrust),
            float(imu_roll_deg), float(imu_pitch_deg), float(imu_yaw_deg),
            float(mocap_roll_deg), float(mocap_pitch_deg), float(mocap_yaw_deg),
            float(bi_roll_flight), float(bi_pitch_flight),
            float(U_X), float(U_Y), float(U_Z), float(U_yaw),
            float(R13), float(R23), float(R13_d), float(R23_d),
            float(mocap_yawrate_dps),
        )

        # Removed PSQ.auto_set() as per instructions
        RTS.sleep()

    PJ.quit()
    saver.save2mat('DataExchange/')
    if udp_enable_flag and UDP is not None:
        UDP.stop_thread()