import atexit
import os
import time
import numpy as np

from rislab_lib.Joystick.joystick_3 import PygameJoystick
from RisLib.cflog import LoggingCore
from RisLib import savemat
from scipy.spatial.transform import Rotation
from jumping.jumping_model import RealTimeSleeper
from jumping.jumping_model import saturation
from jumping.jumping_estimator_v2 import RiseDetect
from jumping.jumping_estimator_v2 import UdpRigidBodies3 as UdpRigidBodies
from jumping.jumping_estimator_v2 import RealTimeProcessor
from jumping.jumping_estimator_v2 import BiController
from jumping.jumping_estimator_v2 import JoyBiController
from jumping.jumping_estimator_v2 import FittedBiController
from jumping.jumping_estimator_v2 import RProjectionLmsEstimator


def robot_stop(lc: LoggingCore):
    # stop motors safely
    try:
        lc.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.05)
    except Exception as e:
        print(f'Warning: failed to send stop command: {e}')

    try:
        lc.cf.close_link()
        time.sleep(0.1)
    except Exception as e:
        print(f'Warning: failed to close Crazyflie link: {e}')


def wait_for_start(PJ: PygameJoystick):
    print('Press cross button to continue ...')
    while not PJ.get_key('Cross'):
        PJ.step()
        time.sleep(0.1)


def height_pd_thrust(
        desired_z,
        z,
        z_dot,
        z_error_i,
        dt,
        thrust_base,
        thrust_min,
        thrust_max,
        z_kp,
        z_ki,
        z_kd,
        z_int_limit):
    if dt <= 1e-6:
        dt = 1e-6

    z_error = desired_z - z
    z_error_i = saturation(
        z_error_i + z_error * dt,
        z_int_limit,
        -z_int_limit,
    )
    z_error_d = -z_dot

    thrust_pid = (
        thrust_base
        + z_kp * z_error
        + z_ki * z_error_i
        + z_kd * z_error_d
    )

    thrust = int(round(saturation(thrust_pid, thrust_max, thrust_min)))
    return thrust, z_error_i, z_error, z_error_d


if __name__ == '__main__':

    # ---------------- user config ----------------
    uri = 'radio://0/70/2M'

    sample_time = 0.01
    cf_log_period_ms = 10
    save_data_enable = True

    udp_enable_flag = True
    rigid_body_index = 1

    # joystick scaling
    DEADZONE_L = 0.2
    DEADZONE_R = 0.2

    # left stick -> BI U_X / U_Y command scale
    # In firmware BI mode:
    #   positive cmd_roll  pushes quadrotor to -X axis
    #   positive cmd_pitch pushes quadrotor to -Y axis

    # manual thrust limit
    THRUST_MAX = 25000
    THRUST_MIN = 0

    # manual mode thrust profile:
    # after enabling controller in manual mode:
    #   0-7 s: ramp thrust from 10000 to base thrust
    #   >7 s: right stick adjusts desired_z, height PD outputs thrust
    MANUAL_BASE_THRUST = 15000
    MANUAL_RAMP_TIME = 7.0
    MANUAL_THRUST_MIN = 10000
    MANUAL_JOYSTICK_RP_SCALE = 5

    # right stick Y integrates desired_z over time
    DESIRED_Z_RATE = 0.2      # m/s when RightStickY is fully pushed
    DESIRED_Z_MIN = 0.1
    DESIRED_Z_MAX = 1.5

    # Shared height PD parameters for JBI / BI / FBI.
    HEIGHT_THRUST_BASE = 28000
    HEIGHT_THRUST_MIN = 3000
    HEIGHT_THRUST_MAX = 60000
    HEIGHT_Z_KP = 13000.0
    HEIGHT_Z_KI = 0.0
    HEIGHT_Z_KD = 5000.0
    HEIGHT_Z_INT_LIMIT = 1.0

    CONTROL_MODES = ('manual', 'JBI', 'BI', 'FBI')
    CLOSED_LOOP_MODES = ('JBI', 'BI', 'FBI')

    # Online LMS low-frequency estimator for R13/R23
    # Model:
    #   R13 = A13 + B13*sin(phase) + C13*cos(phase)
    #   R23 = A23 + B23*sin(phase) + C23*cos(phase)
    # phase is integrated from a fixed yawrate to extract low-frequency R13/R23
    R_LMS_MU = 0.5
    R_LMS_INPUT_ALPHA = None
    R_LMS_YAW_RATE_ALPHA = 1.0
    R_LMS_START_YAWRATE_DEG = 500.0
    R_LMS_FIXED_YAWRATE_DEG = -2100.0
    R_LMS_YAWRATE_SPIKE_WINDOW = 0
    R_LMS_YAWRATE_JUMP_LIMIT = 5200.0
    R_LMS_YAWRATE_ABS_LIMIT = 9000.0

    # realtime first-order low-pass filter for derivative signals
    # smaller alpha -> stronger filtering, larger delay
    DERIV_FILTER_ALPHA = 0.5
    MOCAP_Z_FILTER_ALPHA = 0.3

    # ---------------- init ----------------
    PJ = PygameJoystick()
    wait_for_start(PJ)

    RD_circle = RiseDetect()
    RD_triangle = RiseDetect()

    RTS = RealTimeSleeper(sample_time)

    BI = BiController(
        r13_r23_kp=-2.5,
        xy_cmd_limit=120,
        thrust_base=HEIGHT_THRUST_BASE,
        thrust_min=HEIGHT_THRUST_MIN,
        thrust_max=HEIGHT_THRUST_MAX,
        z_kp=HEIGHT_Z_KP,
        z_ki=HEIGHT_Z_KI,
        z_kd=HEIGHT_Z_KD,
        z_int_limit=HEIGHT_Z_INT_LIMIT,
        xy_kp=0.1,
        xy_kd=0.15)

    JBI = JoyBiController(
        r13_r23_kp=10.5,
        xy_cmd_limit=120.0,
        thrust_base=HEIGHT_THRUST_BASE,
        thrust_min=HEIGHT_THRUST_MIN,
        thrust_max=HEIGHT_THRUST_MAX,
        z_kp=HEIGHT_Z_KP,
        z_ki=HEIGHT_Z_KI,
        z_kd=HEIGHT_Z_KD,
        z_int_limit=HEIGHT_Z_INT_LIMIT,
        joy_r13_gain=0.3,
        joy_r23_gain=0.3,
        joy_deadzone=0.05,
        desired_r_limit=0.50,
    )

    FBI = FittedBiController(
        wn_xy=0.1,
        cmd_limit=1000.0,
        thrust_base=HEIGHT_THRUST_BASE,
        thrust_min=HEIGHT_THRUST_MIN,
        thrust_max=HEIGHT_THRUST_MAX,
        z_kp=HEIGHT_Z_KP,
        z_ki=HEIGHT_Z_KI,
        z_kd=HEIGHT_Z_KD,
        z_int_limit=HEIGHT_Z_INT_LIMIT,
        g=9.81
    )

    R_lms = RProjectionLmsEstimator(
        mu=R_LMS_MU,
        alpha=1.0,
        yaw_rate_alpha=R_LMS_YAW_RATE_ALPHA,
        input_alpha=R_LMS_INPUT_ALPHA,
        lms_start_yawrate_deg=R_LMS_START_YAWRATE_DEG,
        yawrate_spike_window=R_LMS_YAWRATE_SPIKE_WINDOW,
        yawrate_jump_limit=R_LMS_YAWRATE_JUMP_LIMIT,
        yawrate_abs_limit=R_LMS_YAWRATE_ABS_LIMIT,
        use_output_smoothing=False,
        fixed_yawrate_deg=R_LMS_FIXED_YAWRATE_DEG,
    )

    XY_lms = RProjectionLmsEstimator(
        mu=0.3,
        alpha=0.3,
        yaw_rate_alpha=0.3,
    )

    # UDP mocap, used for raw x/y/z, quaternion, and velocity input.
    if udp_enable_flag:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        udp_time = 0.0
    else:
        UDP = None
        udp_time = 0.0

    # RealTimeProcessor is only used to unpack mocap packet fields here.
    sample_rate_rt = 1.0 / sample_time
    dp_mocap = RealTimeProcessor(sample_rate_rt)

    # Keep Crazyflie onboard logging disabled to reduce radio traffic.
    logging_list = {}
    lc = LoggingCore(uri, cf_log_period_ms, logging_list)

    # wait until Crazyflie connection is ready
    connect_wait_start = time.time()
    while not lc.is_connected and time.time() - connect_wait_start < 10.0:
        time.sleep(0.05)

    if not lc.is_connected:
        print('Crazyflie connection failed or timed out.')

        if udp_enable_flag and UDP is not None:
            UDP.stop_thread()

        PJ.quit()
        raise SystemExit

    print('Crazyflie connected, entering main loop ...')

    # Set motor idle thrust before entering control loop
    try:
        lc.cf.param.set_value('powerDist.idleThrust', '6000')
        time.sleep(0.1)
        print('powerDist.idleThrust -> 6000')
    except Exception as e:
        print(f'[WARN] failed to set powerDist.idleThrust: {e}')

    # ---------------- data saver ----------------
    saver = None
    if save_data_enable:
        saver = savemat.DataSaver(
            'Abs_time', 'udp_time',
            'mocap_x_raw', 'mocap_y_raw', 'mocap_z_raw',
            'R13', 'R23',
            'mocap_roll_deg', 'mocap_pitch_deg', 'mocap_yaw_deg',
            'mocap_yawrate_deg', 'mocap_yawrate_deg_filt',
            'mocap_x_filt', 'mocap_y_filt', 'mocap_z_filt',
            'mocap_vx_filt', 'mocap_vy_filt', 'mocap_vz_filt',
            'R13_filt', 'R23_filt',
            'R13_d', 'R23_d', 'R13_d_filt', 'R23_d_filt',
            'desired_x', 'desired_y', 'desired_z',
            'cmd_roll', 'cmd_pitch', 'yaw_deg', 'cmd_yaw', 'cmd_thrust',
        )

    _save_done = False

    def save_flight_data():
        global _save_done
        if (
                _save_done
                or saver is None
                or len(saver.varList[0]) == 0):
            return
        save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'DataExchange')
        os.makedirs(save_dir, exist_ok=True)
        saver.save2mat(save_dir + os.sep)
        _save_done = True

    atexit.register(save_flight_data)

    lc.cf.commander.send_setpoint(0, 0, 0, 0)

    RTS.init()
    start_time = RTS.loop_start_time

    controllerEnable = False
    manual_start_time = None
    manual_stage = 'off'
    manual_z_error_i = 0.0
    armed = False

    # control_mode cycle with Triangle:
    #   manual -> JBI -> BI -> FBI -> manual
    control_mode = 'manual'

    desired_x = 0.0
    desired_y = 0.0
    desired_z = 0.5

    R13_filt = 0.0
    R23_filt = 0.0
    R13_filt_prev = 0.0
    R23_filt_prev = 0.0
    R13_d = 0.0
    R23_d = 0.0
    R13_d_filt = 0.0
    R23_d_filt = 0.0
    R_deriv_initialized = False
    yawrate_deg = 0.0
    yawrate_deg_filt = 0.0
    mocap_x_filt = 0.0
    mocap_y_filt = 0.0
    mocap_z_filt = 0.0

    mocap_x_filt_prev = 0.0
    mocap_y_filt_prev = 0.0
    mocap_z_filt_prev = 0.0
    mocap_z_prev = 0.0
    mocap_filt_vel_initialized = False
    mocap_vx_filt = 0.0
    mocap_vy_filt = 0.0
    mocap_vz_filt = 0.0

    cmd_roll = cmd_pitch = yaw_deg = cmd_yaw = 0.0
    cmd_thrust = 0

    loop_flag = 0
    last_loop_abs_time = None

    while lc.is_connected:
        loop_flag += 1
        Abs_time = RTS.loop_start_time - start_time
        if last_loop_abs_time is None:
            loop_dt = sample_time
        else:
            loop_dt = max(1e-6, Abs_time - last_loop_abs_time)
        last_loop_abs_time = Abs_time

        PJ.step()

        # toggle controller with Circle
        if RD_circle.step(PJ.get_key('Circle')):
            controllerEnable = not controllerEnable
            if controllerEnable:
                print('controllerEnable -> True')

                # Reset filters/controllers when starting a new run
                R_lms.reset()
                XY_lms.reset()
                R_deriv_initialized = False
                mocap_filt_vel_initialized = False
                R13_d = 0.0
                R23_d = 0.0
                R13_d_filt = 0.0
                R23_d_filt = 0.0
                mocap_vx = 0.0
                mocap_vy = 0.0
                mocap_vx_filt = 0.0
                mocap_vy_filt = 0.0
                mocap_vz_filt = 0.0
                manual_z_error_i = 0.0

                if not armed:
                    lc.cf.platform.send_arming_request(True)
                    armed = True

                if control_mode == 'manual':
                    manual_start_time = Abs_time
                    manual_stage = 'ramp'
                    print(
                        f'[manual] ramp stage: {MANUAL_THRUST_MIN} -> {MANUAL_BASE_THRUST} '
                        f'in {MANUAL_RAMP_TIME:.1f}s'
                    )
            else:
                print('controllerEnable -> False')

                # Reset filters/controllers when stopping, so next run starts clean
                R_lms.reset()
                XY_lms.reset()
                R_deriv_initialized = False
                mocap_filt_vel_initialized = False
                R13_d = 0.0
                R23_d = 0.0
                R13_d_filt = 0.0
                R23_d_filt = 0.0
                mocap_vx = 0.0
                mocap_vy = 0.0
                mocap_vx_filt = 0.0
                mocap_vy_filt = 0.0
                mocap_vz_filt = 0.0
                manual_z_error_i = 0.0

                if armed:
                    lc.cf.platform.send_arming_request(False)
                    armed = False
                manual_start_time = None
                manual_stage = 'off'
                manual_z_error_i = 0.0
                print('[manual] off stage: thrust = 0')

        # cycle manual / JBI / BI / FBI mode with Triangle
        if RD_triangle.step(PJ.get_key('Triangle')):
            control_mode = CONTROL_MODES[
                (CONTROL_MODES.index(control_mode) + 1) % len(CONTROL_MODES)
            ]
            print(f'control_mode -> {control_mode}')

            if controllerEnable and control_mode == 'manual':
                manual_start_time = Abs_time
                manual_stage = 'ramp'
                manual_z_error_i = 0.0
                print(
                    f'[manual] ramp stage: {MANUAL_THRUST_MIN} -> {MANUAL_BASE_THRUST} '
                    f'in {MANUAL_RAMP_TIME:.1f}s'
                )
            else:
                if manual_stage != 'off':
                    print('[manual] off stage: thrust = 0')
                manual_start_time = None
                manual_stage = 'off'
                manual_z_error_i = 0.0

        # emergency stop
        if PJ.get_key('Square'):
            R_lms.reset()
            XY_lms.reset()
            R_deriv_initialized = False
            mocap_filt_vel_initialized = False
            R13_d = 0.0
            R23_d = 0.0
            R13_d_filt = 0.0
            R23_d_filt = 0.0
            mocap_vx = 0.0
            mocap_vy = 0.0
            mocap_vx_filt = 0.0
            mocap_vy_filt = 0.0
            mocap_vz_filt = 0.0
            manual_z_error_i = 0.0
            if armed:
                lc.cf.platform.send_arming_request(False)
                armed = False
            robot_stop(lc)
            break

        # ---- mocap input ----
        if udp_enable_flag:
            data_raw, udp_time = UDP.get_data()
            dp_mocap.step(data_raw, body_index=rigid_body_index, abstime=udp_time)

            mocap_x_raw = float(dp_mocap.X)
            mocap_y_raw = float(dp_mocap.Y)
            mocap_z_raw = float(dp_mocap.Z)
            mocap_qx_raw = float(dp_mocap.QX)
            mocap_qy_raw = float(dp_mocap.QY)
            mocap_qz_raw = float(dp_mocap.QZ)
            mocap_qw_raw = float(dp_mocap.QW)
            mocap_vz = dp_mocap.VZ
            mocap_vx = dp_mocap.VX
            mocap_vy = dp_mocap.VY
        else:
            mocap_x_raw = 0.0
            mocap_y_raw = 0.0
            mocap_z_raw = 0.0
            mocap_qx_raw = 0.0
            mocap_qy_raw = 0.0
            mocap_qz_raw = 0.0
            mocap_qw_raw = 1.0
            mocap_vx = 0.0
            mocap_vy = 0.0
            mocap_vz = 0.0

        # ---- quaternion to Euler angle, degree ----
        try:
            mocap_rotation = Rotation.from_quat([
                mocap_qx_raw,
                mocap_qy_raw,
                mocap_qz_raw,
                mocap_qw_raw,
            ])
            mocap_yaw_deg, mocap_pitch_deg, mocap_roll_deg = mocap_rotation.as_euler('ZYX', degrees=True)
            R = mocap_rotation.as_matrix()
        except Exception:
            mocap_roll_deg = 0.0
            mocap_pitch_deg = 0.0
            mocap_yaw_deg = 0.0
            R = np.eye(3)

        R13 = R[0, 2]   # body z-axis projection on world X
        R23 = R[1, 2]   # body z-axis projection on world Y

        R13_filt, R23_filt = R_lms.update(
            R13=R13,
            R23=R23,
            yaw_deg=mocap_yaw_deg,
            dt=loop_dt,
        )
        yawrate_deg = R_lms.yawrate_deg
        yawrate_deg_filt = R_lms.yawrate_deg_filt

        # ---- online finite difference and filtering for R13_filt/R23_filt ----
        if not R_deriv_initialized:
            R13_d = 0.0
            R23_d = 0.0
            R13_d_filt = 0.0
            R23_d_filt = 0.0
            R13_filt_prev = R13_filt
            R23_filt_prev = R23_filt
            R_deriv_initialized = True
        else:
            R13_d = (R13_filt - R13_filt_prev) / sample_time
            R23_d = (R23_filt - R23_filt_prev) / sample_time
            R13_d_filt = R13_d_filt + DERIV_FILTER_ALPHA * (R13_d - R13_d_filt)
            R23_d_filt = R23_d_filt + DERIV_FILTER_ALPHA * (R23_d - R23_d_filt)
            R13_filt_prev = R13_filt
            R23_filt_prev = R23_filt

        # Use the same online LMS low-frequency estimator for mocap x/y
        mocap_x_filt, mocap_y_filt = XY_lms.update(
            R13=mocap_x_raw,
            R23=mocap_y_raw,
            yaw_deg=mocap_yaw_deg,
            dt=sample_time,
        )

        # ---- online finite difference from filtered mocap position ----
        if not mocap_filt_vel_initialized:
            mocap_vx = 0.0
            mocap_vy = 0.0
            mocap_vz = 0.0
            mocap_vx_filt = 0.0
            mocap_vy_filt = 0.0
            mocap_vz_filt = 0.0
            mocap_z_filt = mocap_z_raw

            mocap_x_filt_prev = mocap_x_filt
            mocap_y_filt_prev = mocap_y_filt
            mocap_z_filt_prev = mocap_z_filt
            mocap_z_prev = mocap_z_raw

            mocap_filt_vel_initialized = True
        else:
            mocap_z_filt = mocap_z_filt + MOCAP_Z_FILTER_ALPHA * (mocap_z_raw - mocap_z_filt)

            mocap_vx = (mocap_x_filt - mocap_x_filt_prev) / sample_time
            mocap_vy = (mocap_y_filt - mocap_y_filt_prev) / sample_time
            mocap_vz = (mocap_z_raw - mocap_z_prev) / sample_time
            mocap_vz_from_filt = (mocap_z_filt - mocap_z_filt_prev) / sample_time
            mocap_vx_filt = mocap_vx_filt + DERIV_FILTER_ALPHA * (mocap_vx - mocap_vx_filt)
            mocap_vy_filt = mocap_vy_filt + DERIV_FILTER_ALPHA * (mocap_vy - mocap_vy_filt)
            mocap_vz_filt = mocap_vz_filt + DERIV_FILTER_ALPHA * (mocap_vz_from_filt - mocap_vz_filt)

            mocap_x_filt_prev = mocap_x_filt
            mocap_y_filt_prev = mocap_y_filt
            mocap_z_filt_prev = mocap_z_filt
            mocap_z_prev = mocap_z_raw

        # left stick X/Y for BI manual horizontal command
        JSL_x = float(PJ.get_key('LeftStickX'))
        JSL_y = float(PJ.get_key('LeftStickY'))
        if abs(JSL_x) < DEADZONE_L:
            JSL_x = 0.0
        if abs(JSL_y) < DEADZONE_L:
            JSL_y = 0.0

        # Z target / manual thrust from right stick Y
        JSR_y = float(PJ.get_key('RightStickY'))
        if abs(JSR_y) < DEADZONE_R:
            JSR_y = 0.0

        thrust_manual_direct = saturation(round(-JSR_y * THRUST_MAX), THRUST_MAX, THRUST_MIN)

        # Integrate desired_z from right stick Y over time.
        # Push RightStickY up/down to increase/decrease desired_z.
        # The sign matches manual thrust convention:
        #   negative JSR_y -> increase desired_z
        manual_joystick_z_pd = (
            controllerEnable
            and control_mode == 'manual'
            and manual_stage == 'joystick'
        )
        if controllerEnable and (control_mode in CLOSED_LOOP_MODES or manual_joystick_z_pd):
            desired_z = saturation(
                desired_z - JSR_y * DESIRED_Z_RATE * sample_time,
                DESIRED_Z_MAX,
                DESIRED_Z_MIN,
            )

        if controllerEnable and control_mode == 'manual':
            if manual_start_time is None:
                manual_start_time = Abs_time

            manual_elapsed_time = Abs_time - manual_start_time

            if manual_elapsed_time < MANUAL_RAMP_TIME:
                if manual_stage != 'ramp':
                    manual_stage = 'ramp'
                    print(
                        f'[manual] ramp stage: {MANUAL_THRUST_MIN} -> {MANUAL_BASE_THRUST} '
                        f'in {MANUAL_RAMP_TIME:.1f}s'
                    )
                thrust_manual = int(round((MANUAL_BASE_THRUST - MANUAL_THRUST_MIN) * manual_elapsed_time / MANUAL_RAMP_TIME + MANUAL_THRUST_MIN))
            else:
                if manual_stage != 'joystick':
                    manual_stage = 'joystick'
                    manual_z_error_i = 0.0
                    print(
                        '[manual] joystick stage: right stick controls desired_z, '
                        f'z PD controls thrust, keeping desired_z = {desired_z:.2f} m'
                    )
                thrust_manual, manual_z_error_i, _, _ = height_pd_thrust(
                    desired_z,
                    mocap_z_raw,
                    mocap_vz,
                    manual_z_error_i,
                    sample_time,
                    HEIGHT_THRUST_BASE,
                    HEIGHT_THRUST_MIN,
                    HEIGHT_THRUST_MAX,
                    HEIGHT_Z_KP,
                    HEIGHT_Z_KI,
                    HEIGHT_Z_KD,
                    HEIGHT_Z_INT_LIMIT,
                )
        else:
            manual_elapsed_time = 0.0
            thrust_manual = thrust_manual_direct
            if control_mode != 'manual' and manual_stage != 'off':
                manual_stage = 'off'
                manual_z_error_i = 0.0

        if thrust_manual > 8000 and loop_flag % 100 == 50:
            print('thrust (manual): ', thrust_manual)

        active_controller = None
        if controllerEnable and control_mode in CLOSED_LOOP_MODES:
            if control_mode == 'JBI':
                JBI.update(JSL_x, -JSL_y, desired_z, R13_filt, R23_filt,
                           mocap_z_raw, mocap_vz, mocap_yaw_deg, sample_time)
                active_controller = JBI
            elif control_mode == 'BI':
                BI.update(desired_x, desired_y, desired_z, R13_filt, R23_filt,
                          mocap_x_filt, mocap_y_filt, mocap_z_raw, mocap_vx_filt, mocap_vy_filt, mocap_vz,
                          mocap_yaw_deg, sample_time)
                active_controller = BI
            else:
                FBI.update(desired_x, desired_y, desired_z,
                           R13_filt, R23_filt, R13_d_filt, R23_d_filt,
                           mocap_x_filt, mocap_y_filt, mocap_z_raw, mocap_vx_filt, mocap_vy_filt, mocap_vz,
                           mocap_yaw_deg, sample_time)
                active_controller = FBI
        # ---- command output ----
        if controllerEnable:

            if control_mode == 'manual':
                cmd_thrust = thrust_manual
                #cmd_thrust = 26000
                # Manual mode:
                # In waterrevolve firmware, roll/pitch are still mixed into motors.
                cmd_roll = JSL_x * MANUAL_JOYSTICK_RP_SCALE
                cmd_roll = 0
                cmd_pitch = -JSL_y * MANUAL_JOYSTICK_RP_SCALE
                yaw_deg = mocap_yaw_deg
                cmd_yaw = 0.0

            elif control_mode in CLOSED_LOOP_MODES:
                # First-order P controller:
                # desired_x tracks R13, desired_y tracks R23.
                # cmd_roll is U_X, cmd_pitch is U_Y for firmware BI mode.
                cmd_yaw = 0.0

                cmd_thrust = active_controller.thrust_flight
                cmd_roll = active_controller.roll_flight
                cmd_pitch = active_controller.pitch_flight
                yaw_deg = mocap_yaw_deg

                if loop_flag % 100 == 50:
                    print(f'[{control_mode}] thrust: {cmd_thrust}')

        else:
            cmd_roll = 0.0
            cmd_pitch = 0.0
            yaw_deg = mocap_yaw_deg
            cmd_yaw = 0.0
            cmd_thrust = 0

        yaw_send_deg = ((mocap_yaw_deg + 180 + 180) % 360) - 180

        lc.cf.commander.send_setpoint_revolving(
            cmd_roll,
            cmd_pitch,
            yaw_send_deg,
            cmd_thrust,
        )

        if saver is not None:
            saver.add_elements(
                Abs_time, float(udp_time),
                float(mocap_x_raw), float(mocap_y_raw), float(mocap_z_raw),
                float(R13), float(R23),
                float(mocap_roll_deg), float(mocap_pitch_deg), float(mocap_yaw_deg),
                float(yawrate_deg), float(yawrate_deg_filt),
                float(mocap_x_filt), float(mocap_y_filt), float(mocap_z_filt),
                float(mocap_vx_filt), float(mocap_vy_filt), float(mocap_vz_filt),
                float(R13_filt), float(R23_filt),
                float(R13_d), float(R23_d), float(R13_d_filt), float(R23_d_filt),
                float(desired_x), float(desired_y), float(desired_z),
                float(cmd_roll), float(cmd_pitch), float(yaw_deg), float(cmd_yaw), int(cmd_thrust),
            )

        RTS.sleep()

    try:
        lc.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.05)
    except Exception as e:
        print(f'Warning: failed to send stop command: {e}')

    try:
        lc.cf.close_link()
        time.sleep(0.1)
    except Exception as e:
        print(f'Warning: failed to close Crazyflie link: {e}')

    if udp_enable_flag and UDP is not None:
        UDP.stop_thread()

    PJ.quit()
    save_flight_data()
