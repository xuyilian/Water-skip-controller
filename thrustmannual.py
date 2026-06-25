import time
import math
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


if __name__ == '__main__':

    # ---------------- user config ----------------
    uri = 'radio://0/99/2M'

    sample_time = 0.01

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
    THRUST_MAX = 30000
    THRUST_MIN = 0

    # manual mode thrust profile:
    # after enabling controller in manual mode:
    #   0-10 s: ramp thrust from 0 to 25000
    #   10-13 s: hold thrust at 25000
    #   >13 s: right stick adjusts thrust around 25000
    MANUAL_BASE_THRUST = 30000
    MANUAL_RAMP_TIME = 7.0
    MANUAL_HOLD_TIME = 3.0
    MANUAL_JOYSTICK_DELTA = 2000
    MANUAL_THRUST_MIN = 24000
    MANUAL_THRUST_MAX = 30000

    POSITION_GAIN = 0.3

    # right stick Y integrates desired_z over time
    DESIRED_Z_RATE = 0.2      # m/s when RightStickY is fully pushed
    DESIRED_Z_MIN = 0.1
    DESIRED_Z_MAX = 1.5

    # BI mode step-thrust test triggered by R1
    # When enabled:
    #   pulse:   cmd_thrust = BI_STEP_THRUST for BI_STEP_DURATION seconds
    #   recover: cmd_thrust = BI.thrust_flight until z reaches desired_z again
    #   repeat automatically
    BI_STEP_THRUST = 30000
    BI_STEP_DURATION = 0.1
    BI_STEP_ACC_Z_RISE_THRESHOLD = 0.1
    BI_RECOVER_DURATION = 2

    # Online LMS low-frequency estimator for R13/R23
    # Model:
    #   R13 = A13 + B13*sin(phase) + C13*cos(phase)
    #   R23 = A23 + B23*sin(phase) + C23*cos(phase)
    # phase is integrated from f0 = yawrate_deg / 360
    R_LMS_MU = 0.5
    R_LMS_ALPHA = 0.5
    YAW_RATE_ALPHA = 0.3

    # BI self-spin / motor-delay compensation.
    # send_setpoint_revolving(..., yaw_torque, thrust) maps yaw_torque to
    # firmware setpoint.attitude.yaw, then firmware multiplies it by 20000.
    BI_YAW_TORQUE_CMD = 0.3
    BI_MOTOR_RESPONSE_DELAY_S = 0.025
    BI_USE_DYNAMIC_YAW_DELAY_DEG = True
    BI_FIXED_YAW_DELAY_DEG = 0.6
    BI_YAW_DELAY_SIGN = 1.0
    BI_YAW_DELAY_DEG_LIMIT = 10.0

    # realtime first-order low-pass filter for derivative signals
    # smaller alpha -> stronger filtering, larger delay
    DERIV_FILTER_ALPHA = 0.5

    # ---------------- init ----------------
    PJ = PygameJoystick()
    wait_for_start(PJ)

    RD_circle = RiseDetect()
    RD_triangle = RiseDetect()
    RD_R1 = RiseDetect()

    RTS = RealTimeSleeper(sample_time)

    BI = BiController(
        r13_r23_kp=-15.0,
        xy_cmd_limit=120,
        thrust_base=30000,
        thrust_min=3000,
        thrust_max=65000,
        z_kp=10000,
        z_ki=0,
        z_kd=5000,
        z_int_limit=1,
        xy_kp=0.2,
        xy_kd=0.15)

    JBI = JoyBiController(
        r13_r23_kp=-6.0,
        xy_cmd_limit=120.0,
        thrust_base=49000,
        thrust_min=3000,
        thrust_max=60000,
        z_kp=3000.0,
        z_ki=0.0,
        z_kd=55000.0,
        z_int_limit=1.0,
        joy_r13_gain=0.3,
        joy_r23_gain=0.3,
        joy_deadzone=0.05,
        desired_r_limit=0.50,
    )
    
    FBI = FittedBiController(
        wn_xy=0.1,
        cmd_limit=1000.0,
        thrust_base=49000,
        thrust_min=3000,
        thrust_max=56000,
        z_kp=8000.0,
        z_ki=0,
        z_kd=5000.0,
        z_int_limit=1,
        g=9.81
    )

    R_lms = RProjectionLmsEstimator(
    mu=0.3,
    alpha=0.3,
    yaw_rate_alpha=0.3,)

    XY_lms = RProjectionLmsEstimator(
    mu=0.3,
    alpha=0.3,
    yaw_rate_alpha=0.3,)

    # UDP mocap, only used for raw x/y/z and quaternion logging
    if udp_enable_flag:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        udp_time = 0.0
    else:
        UDP = None
        udp_time = 0.0

    # RealTimeProcessor is only used to unpack mocap packet fields here.
    # Do not use filtered outputs such as X_F/Y_F/Z_F for saving.
    sample_rate_rt = 1.0 / sample_time
    dp_mocap = RealTimeProcessor(sample_rate_rt)

    # Crazyflie logging
    logging_list = {
        'pm.vbat': 'FP16',
        'acc.x': 'FP16',
        'acc.y': 'FP16',
        'acc.z': 'FP16',
    }

    lc = LoggingCore(uri, 10, logging_list)

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

    # ---------------- data saver ----------------

    # wait until Crazyflie connection is ready

    # ---------------- data saver ----------------
    saver = savemat.DataSaver(
    'Abs_time', *tuple(lc.temp_keys), 'udp_time',
    'mocap_x_raw', 'mocap_y_raw', 'mocap_z_raw',
    'mocap_x_lms_raw', 'mocap_y_lms_raw', 'mocap_x_filt', 'mocap_y_filt',
    'mocap_qx_raw', 'mocap_qy_raw', 'mocap_qz_raw', 'mocap_qw_raw',
    'mocap_roll_deg', 'mocap_pitch_deg', 'mocap_yaw_deg',
    'R13', 'R23', 'R33', 'R13_lms_raw', 'R23_lms_raw', 'R13_filt', 'R23_filt',
    'R13_d', 'R23_d', 'R13_d_filt', 'R23_d_filt',
    'mocap_vx', 'mocap_vy', 'mocap_vz', 'mocap_vx_filt', 'mocap_vy_filt',
    'mocap_yawrate_deg', 'mocap_yawrate_deg_filt', 'f0_yaw',
    'desired_x', 'desired_y',
    'cmd_roll', 'cmd_pitch', 'cmd_yaw', 'cmd_thrust',
    'BI_yaw_delay_deg', 'BI_yaw_torque_cmd',
    'right_stick_y', 'controllerEnable',
    )

    lc.cf.commander.send_setpoint(0, 0, 0, 0)

    RTS.init()
    start_time = RTS.loop_start_time

    controllerEnable = False
    manual_start_time = None
    manual_stage = 'off'
    armed = False

    # control_mode:
    #   manual -> stabilizer.b = 0
    #   BI     -> stabilizer.b = 1
    control_mode = 'manual'

    desired_x = 0.0
    desired_y = 0.0
    desired_z = 1

    desired_R13 = desired_R23 = 0.0

    R13_filt = 0.0
    R23_filt = 0.0
    R13_filt_prev = 0.0
    R23_filt_prev = 0.0
    R13_d = 0.0
    R23_d = 0.0
    R13_d_filt = 0.0
    R23_d_filt = 0.0
    R_deriv_initialized = False
    R13_low_raw = 0.0
    R23_low_raw = 0.0
    yawrate_deg = 0.0
    yawrate_deg_filt = 0.0
    f0_yaw = 0.0
    
    mocap_x_low_raw = 0.0
    mocap_y_low_raw = 0.0
    mocap_x_filt = 0.0
    mocap_y_filt = 0.0

    mocap_x_filt_prev = 0.0
    mocap_y_filt_prev = 0.0
    mocap_z_prev = 0.0
    mocap_filt_vel_initialized = False
    mocap_vx_filt = 0.0
    mocap_vy_filt = 0.0

    cmd_roll = cmd_pitch = cmd_yaw = 0.0
    cmd_thrust = 0
    bi_yaw_delay_deg = 0.0
    bi_yaw_torque_cmd = 0.0

    loop_flag = 0

    bi_step_enable = False
    bi_step_phase = 'idle'       # 'idle', 'pulse', or 'recover'
    bi_step_start_time = None

    acc_z_raw = 0.0
    acc_z_prev = 0.0
    acc_z_initialized = False

    while lc.is_connected:
        loop_flag += 1
        Abs_time = RTS.loop_start_time - start_time

        PJ.step()

        logged_data = lc.get_logged_data()
        acc_z_raw = float(logged_data.get('acc.z', 0.0))
        if not acc_z_initialized:
            acc_z_prev = acc_z_raw
            acc_z_initialized = True

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

                if not armed:
                        lc.cf.platform.send_arming_request(True)
                        armed = True

                if control_mode == 'manual':
                    manual_start_time = Abs_time
                    manual_stage = 'ramp'
                    print(f'[manual] ramp stage: 0 -> {MANUAL_BASE_THRUST} in {MANUAL_RAMP_TIME:.1f}s')
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

                if armed:
                        lc.cf.platform.send_arming_request(False)
                        armed = False
                manual_start_time = None
                manual_stage = 'off'
                print('[manual] off stage: thrust = 0')

                # toggle manual / BI mode with Triangle
        if RD_triangle.step(PJ.get_key('Triangle')):
            if control_mode == 'manual':
                control_mode = 'BI'
            else:
                control_mode = 'manual'

            try:
                lc.cf.param.set_value(
                    'stabilizer.b',
                    '1' if control_mode == 'BI' else '0'
                )
                print(
                    f'control_mode -> {control_mode}, '
                    f'stabilizer.b -> {1 if control_mode == "BI" else 0}'
                )
                if controllerEnable and control_mode == 'manual':
                    manual_start_time = Abs_time
                    manual_stage = 'ramp'
                    print(f'[manual] ramp stage: 0 -> {MANUAL_BASE_THRUST} in {MANUAL_RAMP_TIME:.1f}s')
                elif control_mode != 'manual':
                    bi_step_enable = False
                    bi_step_phase = 'idle'
                    bi_step_start_time = None
                    bi_step_left_target = False

                    manual_start_time = None
                    manual_stage = 'off'
                    print('[manual] off stage: thrust = 0')
            except Exception as e:
                print(f'[WARN] failed to set stabilizer.b: {e}')

        # toggle BI step-thrust cycle with R1
        if RD_R1.step(PJ.get_key('R1')):
            bi_step_enable = not bi_step_enable

            if bi_step_enable:
                bi_step_phase = 'pulse'
                bi_step_start_time = Abs_time
                acc_z_prev = acc_z_raw
                print('[BI step] enabled: thrust pulse -> 30000')
            else:
                bi_step_phase = 'idle'
                bi_step_start_time = None
                bi_step_left_target = False
                print('[BI step] disabled')        
                
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
            bi_step_enable = False
            bi_step_phase = 'idle'
            bi_step_start_time = None
            bi_step_left_target = False
            if armed:
                        lc.cf.platform.send_arming_request(False)
                        armed = False
            robot_stop(lc)
            break

        # ---- raw mocap logging only, no filtering ----
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
            mocap_vz = 0.0

        # ---- quaternion to Euler angle, degree ----
        try:
            mocap_yaw_deg, mocap_pitch_deg, mocap_roll_deg = Rotation.from_quat([
                mocap_qx_raw,
                mocap_qy_raw,
                mocap_qz_raw,
                mocap_qw_raw,
            ]).as_euler('ZYX', degrees=True)
        except Exception:
            mocap_roll_deg = 0.0
            mocap_pitch_deg = 0.0
            mocap_yaw_deg = 0.0    

        R = Rotation.from_quat([mocap_qx_raw, mocap_qy_raw, mocap_qz_raw, mocap_qw_raw]).as_matrix()

        R13 = R[0, 2]   # body z-axis projection on world X
        R23 = R[1, 2]   # body z-axis projection on world Y
        R33 = R[2, 2]   # body z-axis projection on world Z

        R13_filt, R23_filt = R_lms.update(
            R13=R13,
            R23=R23,
            yaw_deg=mocap_yaw_deg,
            dt=sample_time,
        )

        R13_low_raw = R_lms.R13_low_raw
        R23_low_raw = R_lms.R23_low_raw
        yawrate_deg = R_lms.yawrate_deg
        yawrate_deg_filt = R_lms.yawrate_deg_filt
        f0_yaw = R_lms.f0_yaw

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

                # ---- online finite difference from filtered mocap x/y ----
        if not mocap_filt_vel_initialized:
            mocap_vx = 0.0
            mocap_vy = 0.0
            mocap_vx_filt = 0.0
            mocap_vy_filt = 0.0

            mocap_x_filt_prev = mocap_x_filt
            mocap_y_filt_prev = mocap_y_filt

            mocap_filt_vel_initialized = True
        else:
            mocap_vx = (mocap_x_filt - mocap_x_filt_prev) / sample_time
            mocap_vy = (mocap_y_filt - mocap_y_filt_prev) / sample_time
            mocap_vz = (mocap_z_raw - mocap_z_prev) / sample_time
            mocap_vx_filt = mocap_vx_filt + DERIV_FILTER_ALPHA * (mocap_vx - mocap_vx_filt)
            mocap_vy_filt = mocap_vy_filt + DERIV_FILTER_ALPHA * (mocap_vy - mocap_vy_filt)

            mocap_x_filt_prev = mocap_x_filt
            mocap_y_filt_prev = mocap_y_filt
            mocap_z_prev = mocap_z_raw

        mocap_x_low_raw = XY_lms.R13_low_raw
        mocap_y_low_raw = XY_lms.R23_low_raw

        # left stick X/Y for BI manual horizontal command
        JSL_x = float(PJ.get_key('LeftStickX'))
        JSL_y = float(PJ.get_key('LeftStickY'))
        if abs(JSL_x) < DEADZONE_L:
            JSL_x = 0.0
        if abs(JSL_y) < DEADZONE_L:
            JSL_y = 0.0

        #desired_x = JSL_x * POSITION_GAIN
        #desired_y = JSL_y * POSITION_GAIN

        # Z target / manual thrust from right stick Y
        JSR_y = float(PJ.get_key('RightStickY'))
        if abs(JSR_y) < DEADZONE_R:
            JSR_y = 0.0

        thrust_manual_direct = saturation(round(-JSR_y * THRUST_MAX), THRUST_MAX, THRUST_MIN)

        # Integrate desired_z from right stick Y over time.
        # Push RightStickY up/down to increase/decrease desired_z.
        # The sign matches manual thrust convention:
        #   negative JSR_y -> increase desired_z
        if controllerEnable and control_mode == 'BI':
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
                    print(f'[manual] ramp stage: 0 -> {MANUAL_BASE_THRUST} in {MANUAL_RAMP_TIME:.1f}s')
                thrust_manual = int(round((MANUAL_BASE_THRUST - MANUAL_THRUST_MIN) * manual_elapsed_time / MANUAL_RAMP_TIME + MANUAL_THRUST_MIN))
            elif manual_elapsed_time < MANUAL_RAMP_TIME + MANUAL_HOLD_TIME:
                if manual_stage != 'hold':
                    manual_stage = 'hold'
                    print(f'[manual] hold stage: thrust = {MANUAL_BASE_THRUST} for {MANUAL_HOLD_TIME:.1f}s')
                thrust_manual = MANUAL_BASE_THRUST
            else:
                if manual_stage != 'joystick':
                    manual_stage = 'joystick'
                    print(f'[manual] joystick stage: thrust = {MANUAL_BASE_THRUST} +/- {MANUAL_JOYSTICK_DELTA}')
                thrust_manual = int(saturation(
                    round(MANUAL_BASE_THRUST - JSR_y * MANUAL_JOYSTICK_DELTA),
                    MANUAL_THRUST_MAX,
                    MANUAL_THRUST_MIN,
                ))
        else:
            manual_elapsed_time = 0.0
            thrust_manual = thrust_manual_direct
            if control_mode != 'manual' and manual_stage != 'off':
                manual_stage = 'off'

        if thrust_manual > 8000 and loop_flag % 100 == 50:
            print('thrust (manual): ', thrust_manual)

        BI.update(desired_x, desired_y, desired_z, R13_filt, R23_filt,
                  mocap_x_filt, mocap_y_filt, mocap_z_raw, mocap_vx_filt, mocap_vy_filt, mocap_vz, 
                  mocap_yaw_deg, sample_time)    

        JBI.update(JSL_x, -JSL_y, desired_z, R13_filt, R23_filt,
            mocap_z_raw, mocap_vz, mocap_yaw_deg, sample_time)

        FBI.update(desired_x, desired_y, desired_z,
                   R13_filt, R23_filt, R13_d_filt, R23_d_filt,
                    mocap_x_filt, mocap_y_filt, mocap_z_raw, mocap_vx_filt, mocap_vy_filt, mocap_vz,
                    mocap_yaw_deg, sample_time) 
        # ---- command output ----
        if controllerEnable:

            if control_mode == 'manual':
                cmd_thrust = thrust_manual
                #cmd_thrust = 26000
                # Manual mode:
                # firmware stabilizer.b = 0
                # only thrust is used
                #cmd_roll = desired_x*1000
                #cmd_pitch = desired_y*1000
                cmd_roll = JSL_x * 5
                cmd_pitch = -JSL_y * 5
                cmd_yaw = mocap_yaw_deg - 90
                bi_yaw_delay_deg = 0.0
                bi_yaw_torque_cmd = 0.0


            else:
                # First-order P controller:
                # desired_x tracks R13, desired_y tracks R23.
                # cmd_roll is U_X, cmd_pitch is U_Y for firmware BI mode.
                cmd_thrust = BI.thrust_flight
                cmd_roll = BI.roll_flight
                cmd_pitch = BI.pitch_flight
                if BI_USE_DYNAMIC_YAW_DELAY_DEG:
                    bi_yaw_delay_deg = BI_YAW_DELAY_SIGN * saturation(
                        yawrate_deg_filt * BI_MOTOR_RESPONSE_DELAY_S,
                        BI_YAW_DELAY_DEG_LIMIT,
                        -BI_YAW_DELAY_DEG_LIMIT,
                    )
                else:
                    bi_yaw_delay_deg = BI_YAW_DELAY_SIGN * saturation(
                        BI_FIXED_YAW_DELAY_DEG,
                        BI_YAW_DELAY_DEG_LIMIT,
                        -BI_YAW_DELAY_DEG_LIMIT,
                    )

                cmd_yaw = BI.yaw_flight + bi_yaw_delay_deg
                bi_yaw_torque_cmd = BI_YAW_TORQUE_CMD

                #cmd_thrust = JBI.thrust_flight
                #cmd_roll   = JBI.roll_flight
                #cmd_pitch  = JBI.pitch_flight
                #cmd_yaw    = JBI.yaw_flight

                #cmd_thrust = FBI.thrust_flight
                #cmd_roll = FBI.roll_flight
                #cmd_pitch = FBI.pitch_flight
                #cmd_yaw = FBI.yaw_flight - 90

                if bi_step_enable:
                    if bi_step_phase == 'idle':
                        bi_step_phase = 'pulse'
                        bi_step_start_time = Abs_time

                    if bi_step_phase == 'pulse':
                        cmd_thrust = BI_STEP_THRUST

                        if bi_step_start_time is None:
                            bi_step_start_time = Abs_time
                            acc_z_prev = acc_z_raw

                        # Only use raw acc.z rising detection.
                        acc_z_rising = (acc_z_raw - acc_z_prev) > BI_STEP_ACC_Z_RISE_THRESHOLD

                        if acc_z_rising:
                            bi_step_phase = 'recover'
                            bi_step_start_time = Abs_time
                            print('[BI step] recover: raw acc.z rising')

                        acc_z_prev = acc_z_raw

                    elif bi_step_phase == 'recover':
                        cmd_thrust = BI.thrust_flight

                        if bi_step_start_time is None:
                            bi_step_start_time = Abs_time

                        if Abs_time - bi_step_start_time >= BI_RECOVER_DURATION:
                            bi_step_phase = 'pulse'
                            bi_step_start_time = Abs_time
                            cmd_thrust = BI_STEP_THRUST
                            acc_z_prev = acc_z_raw
                            print('[BI step] pulse: thrust -> 30000 for 0.5 s')
                print('thrust : ', cmd_thrust)

        else:
            cmd_roll = 0.0
            cmd_pitch = 0.0
            cmd_yaw = 0.0
            cmd_thrust = 0
            bi_yaw_delay_deg = 0.0
            bi_yaw_torque_cmd = 0.0

        lc.cf.commander.send_setpoint_revolving(
            cmd_roll,
            cmd_pitch,
            cmd_yaw,
            bi_yaw_torque_cmd,
            cmd_thrust,
        )


        logged_data = lc.get_logged_data()

        # ---- save ----
        saver.add_elements(
        Abs_time, *tuple(logged_data.values()), float(udp_time),
        float(mocap_x_raw), float(mocap_y_raw), float(mocap_z_raw),
        float(mocap_x_low_raw), float(mocap_y_low_raw), float(mocap_x_filt), float(mocap_y_filt),
        float(mocap_qx_raw), float(mocap_qy_raw), float(mocap_qz_raw), float(mocap_qw_raw),
        float(mocap_roll_deg), float(mocap_pitch_deg), float(mocap_yaw_deg),
        float(R13), float(R23), float(R33), float(R13_low_raw), float(R23_low_raw), float(R13_filt), float(R23_filt),
        float(R13_d), float(R23_d), float(R13_d_filt), float(R23_d_filt),
        float(mocap_vx), float(mocap_vy), float(mocap_vz), float(mocap_vx_filt), float(mocap_vy_filt),
        float(yawrate_deg), float(yawrate_deg_filt), float(f0_yaw),
        float(desired_x), float(desired_y),
        float(cmd_roll), float(cmd_pitch), float(cmd_yaw), int(cmd_thrust),
        float(bi_yaw_delay_deg), float(bi_yaw_torque_cmd),
        float(JSR_y), bool(controllerEnable),
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
    saver.save2mat('DataExchange/')
