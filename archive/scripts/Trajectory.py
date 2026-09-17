# Trajectory tracking
import time
import threading
import math
from rislab_lib.Joystick.joystick_3 import PygameJoystick

from Library.SaveMAT import savemat
from Library.CF import CommandSend
from Library.UdpReceive import UdpReceive2
from Library import DataProcessing2 as DataProcessor
from Library.control import GeoController


class RiseDetect:
    def __init__(self):
        self._prev = False

    def step(self, cur: bool) -> bool:
        rise = (cur and not self._prev)
        self._prev = cur
        return rise


def wait_for_start(PJ: PygameJoystick):
    print('Press cross button to continue ...')
    while not PJ.get_key('Cross'):
        PJ.step()
        time.sleep(0.1)


if __name__ == '__main__':

    Trajectory_on = False
    bi_flag = -1
    controllerEnable = False
    armed = False
    done = False
    pad_speed = 1
    PJ = PygameJoystick()

    wait_for_start(PJ)

    # Rising-edge detectors (same style as reference)
    RD_cross_button = RiseDetect()
    RD_circle_button = RiseDetect()
    RD_square_button = RiseDetect()
    RD_triangle_button = RiseDetect()
    RD_option_button = RiseDetect()

    # ------ Data to be Saved ------
    saver_1 = savemat.DataSaver('Data_time',
                                'Data_Position_X',
                                'Data_Position_Y',
                                'Data_Position_Z',
                                'Data_Position_X_F',
                                'Data_Position_Y_F',
                                'Data_Position_Z_F',
                                'Data_Desired_X',
                                'Data_Desired_Y',
                                'Data_Desired_Z',
                                'Data_Angle_Yaw',
                                'Data_Quaternion_qw',
                                'Data_Quaternion_qx',
                                'Data_Quaternion_qy',
                                'Data_Quaternion_qz',
                                'Data_R31',
                                'Data_R32',
                                'Data_R31_F',
                                'Data_R32_F',
                                'Data_R31_F_d',
                                'Data_R32_F_d',
                                'Data_U_X',
                                'Data_U_Y',
                                'Data_U_Z',
                                'Data_Angle_Yaw_d',
                                'Data_Desired_X_d',
                                'Data_Desired_Y_d',
                                'Data_Position_X_d',
                                'Data_Position_Y_d', )
    # ------ connect to CFs ------
    URI1 = 'radio://0/80/2M'
    cf_1 = CommandSend.CmdSend(URI1)
    command_send_ready_1 = threading.Event()
    cf_1.thread_start(command_send_ready_1)

    # -------- udp Receiver init ------
    udp_receiver = UdpReceive2.UdpRigidBodies()
    sample_rate = udp_receiver.get_sample_rate()
    sample_time = 1 / sample_rate
    # -------- controller init --------
    bi_controller_1 = GeoController.Controller(sample_time, 
                                               165 / 53, 
                                               5, 45, 45, 55, 5, 
                                               9.3548, 18, 350 / 53, 
                                               10000,
                                               1000, 9000, 35000)  # for upper robot
    # -------- yaw controller init --------
    # YawController is defined in Library/control/GeoController.py
    # These gains are conservative defaults; tune as needed.
    yaw_controller = GeoController.YawController(sample_time, 0, 10, 5)
    desired_yaw = 0.0
    desired_yaw_rate = 0.0
    # set different desired position to avoid collision
    # bi_controller_1.update_desired_position(0, 0, 0.6)
    bi_controller_1.load_trajectory()

    # data processor
    dp1 = DataProcessor.RealTimeProcessor(5, [18], 'lowpass', 'cheby2', 85, sample_rate, 165 / 53)  # for take off
    dp2 = DataProcessor.RealTimeProcessor(4, [16], 'lowpass', 'cheby2', 58, sample_rate, 165 / 53)

    # ------ udp thread start ------
    udp_ready = threading.Event()
    controller_ready = threading.Event()
    udp_receiver.start_thread(udp_ready, controller_ready)

    Controller_Start_Time = time.time()
    while_flag = 0

    while not done:
        AbsTime = time.time() - Controller_Start_Time

        PJ.step()
        # Left stick: X/Y -> x/y command
        d_padX = float(PJ.get_key('LeftStickX'))
        d_padY = float(PJ.get_key('LeftStickY'))

        # Right stick: X -> yawrate command, Y -> z command
        yawrate_cmd = float(PJ.get_key('RightStickX'))
        z_cmd = float(PJ.get_key('RightStickY'))

        # Deadzone
        if abs(d_padX) < 0.1:
            d_padX = 0.0
        if abs(d_padY) < 0.1:
            d_padY = 0.0
        if abs(yawrate_cmd) < 0.1:
            yawrate_cmd = 0.0
        if abs(z_cmd) < 0.1:
            z_cmd = 0.0
        d_padZ = -z_cmd

        yaw_rate_gain = 20.0    # scaling for stick input (dimensionless)

        
        # ----- set desired position ------
        if controllerEnable:
            if not Trajectory_on:
                bi_controller_1.update_desired_position(
                    DataProcessor.saturation_fcn(
                        bi_controller_1.Desired_X + d_padX * sample_time * pad_speed,
                        [-1.5, 1.5]),
                    DataProcessor.saturation_fcn(
                        bi_controller_1.Desired_Y + d_padY * sample_time * pad_speed,
                        [-1.5, 1.5]),
                    DataProcessor.saturation_fcn(
                        bi_controller_1.Desired_Z + d_padZ * sample_time * pad_speed,
                        [0, 1.7]), )
                # desired yaw is kept at 0 deg; desired yaw rate is driven by right-stick X
                yaw_controller.update_desired_state(
                    0,
                    DataProcessor.saturation_fcn(
                        yaw_controller.desired_yaw_rate + yawrate_cmd * sample_time * yaw_rate_gain,
                        [-180, 180]
                    )
                )
            else:
                bi_controller_1.update_desired_position_trajectory()

        else:
            bi_controller_1.update_desired_position(0, 0, 0.3)
            yaw_controller.update_desired_state(0,0)

        # ---- receive pose and process ----
        udp_ready.wait()
        udp_ready.clear()
        data = udp_receiver.get_data_from_thread()  # data contain two rigid bodies
        controller_ready.set()
        # ----------- data processor -----------
        dp2.step(data[0:14], True)
        dp1.step(data[0:14], True)
        # dp2.step(data[14:28], True)
        # ----------- Controller -----------
        dp_used = dp2 if bi_flag >= 800 else dp1
        U_X, U_Y, U_Z = bi_controller_1.update_error(dp_used)  # upper robot
        U_X, U_Y = DataProcessor.circular_saturation_fcn(U_X, U_Y, 300, )
        U_yaw = yaw_controller.update_error(dp_used)
        circle_pressed = bool(PJ.get_key('Circle'))

        pitch_flight = (U_X * math.cos(dp_used.angleY) + U_Y * math.sin(dp_used.angleY))
        roll_flight  = -(U_Y * math.cos(dp_used.angleY) - U_X * math.sin(dp_used.angleY))

        
        # ----------- send commands (Circle-hold arming gate) -----------
        if not circle_pressed:
            # Not holding Circle: keep outputs at zero (safe)
            cf_1.send(0, 0, 0, 0, dp_used.angleY)
            command_send_ready_1.set()
        else:
            # Holding Circle: arm once, then allow normal command sending
            if not armed:
                try:
                    cf_1.cf.platform.send_arming_request(True)
                    armed = True
                    print('Armed')
                except Exception as e:
                    print(f'Arming request failed: {e}')

            if controllerEnable:
                bi_flag = bi_flag + 1
                if bi_flag >= 200:
                    cf_1.send(roll_flight, pitch_flight, round(U_yaw), round(U_Z))
                else:   # open-looped flight
                    cf_1.send(0, 0, round(U_yaw), round(U_Z))
            else:
                cf_1.send(0, 0, round(U_yaw), round(U_Z))
            command_send_ready_1.set()
        # ------ tails ------
        if while_flag % 100 == 0:
            print(bi_controller_1.trajectory_flag)
            # print("Desired position, X,Y,Z: ",
            #       '%.2f' % bi_controller_1.Desired_X,
            #       '%.2f' % bi_controller_1.Desired_Y,
            #       '%.2f' % bi_controller_1.Desired_Z, )
        while_flag = while_flag + 1

        # --- buttons (reference-style: named keys + rising-edge) ---
        cross_rise = RD_cross_button.step(bool(PJ.get_key('Cross')))
        circle_rise = RD_circle_button.step(bool(PJ.get_key('Circle')))
        square_rise = RD_square_button.step(bool(PJ.get_key('Square')))
        triangle_rise = RD_triangle_button.step(bool(PJ.get_key('Triangle')))
        option_rise = RD_option_button.step(bool(PJ.get_key('Option')))

        # Square: emergency stop (also disarm)
        if square_rise:
            done = True
            # send a safe zero-thrust command
            cf_1.send(0, 0, 0, 0, dp_used.angleY)
            command_send_ready_1.set()

            # cancel arming / disarm
            if armed:
                try:
                    cf_1.cf.platform.send_arming_request(False)
                    armed = False
                    print('Disarmed')
                except Exception as e:
                    print(f'Disarm request failed: {e}')

        # Circle: toggle trajectory mode
        if triangle_rise:
            Trajectory_on = not Trajectory_on
            print(f"Trajectory_on -> {Trajectory_on}")

        # Cross: enable controller (once)
        if (not controllerEnable) and cross_rise:
            bi_controller_1.integrator_enable()
            controllerEnable = True
            print('Controller enabled')
        # ------ save data ------
        if controllerEnable:
            saver_1.add_elements(AbsTime,
                                 dp2.X,
                                 dp2.Y,
                                 dp2.Z,
                                 dp2.X_F,
                                 dp2.Y_F,
                                 dp2.Z_F,
                                 bi_controller_1.Desired_X,
                                 bi_controller_1.Desired_Y,
                                 bi_controller_1.Desired_Z,
                                 dp2.angleY,
                                 dp2.QW,
                                 dp2.QX,
                                 dp2.QY,
                                 dp2.QZ,
                                 dp2.R13,
                                 dp2.R23,
                                 dp2.R13_F,
                                 dp2.R23_F,
                                 dp2.R13_F_d,
                                 dp2.R23_F_d,
                                 round(U_X),
                                 round(U_Y),
                                 round(U_Z),
                                 dp2.angleY_d,
                                 bi_controller_1.desired_x_d,
                                 bi_controller_1.desired_y_d,
                                 dp2.X_F_d,
                                 dp2.Y_F_d, )
    PJ.quit()
    udp_receiver.stop_thread(controller_ready)
    cf_1.thread_stop(command_send_ready_1)
    print('system stop')
    saver_1.add_info({'Data_sample_rate': udp_receiver.get_sample_rate()})
    saver_1.save2mat('DataExchange/')
