# new Crazyflie framework

from RisLib import UdpReceiver
from RisLib import savemat
from RisLib.cflog import LoggingCore

import time
import os
import serial
from transforms3d.quaternions import quat2mat

import numpy as np


class LaunchPlatform:
    def __init__(self,):
        devs_name = os.listdir('/dev')
        self.target = 'none'
        for dev in devs_name:
            if 'cu.usbmodem' in dev:
                print('arduino detected: ' + dev)
                self.target = dev
                break
        if self.target == 'none':
            print('error: no arduino detected!')
        else:
            self.arduino = serial.Serial(port=('/dev/'+self.target), baudrate=115200, timeout=.1)
            time.sleep(5)
            print('connected to arduino')

    def write_read(self, x):
        if self.target == 'none':
            print('error: no device')
        else:
            self.arduino.write(bytes(x, 'utf-8'))
        return 1


def get_hrotm(qw, qx, qy, qz, x, y, z):
    rotm = quat2mat([qw, qx, qy, qz])
    P = np.array([[x, y, z]]).T
    HB = np.array([[0, 0, 0, 1]])
    temp = np.concatenate((rotm, P), axis=1)
    hrotm = np.concatenate((temp, HB), axis=0)
    return hrotm


if __name__ == '__main__':
    # launch platform
    LP = LaunchPlatform()

    # Initialize UDP
    UDP = UdpReceiver.UdpRigidBodies()
    UDP.start_thread()
    DP = UdpReceiver.DataProcessor(UDP.num_bodies, UDP.get_sample_rate())
    # print(DP.save_list_name)

    # Initialize Crazyflie
    uri = 'radio://0/80/2M'

    logging_list = {
                    'acc.x': 'float',
                    'acc.y': 'float', }
    lc = LoggingCore(uri, 10, logging_list)

    parameter_list = {'motorPowerSet.enable': '0',
                      'motorPowerSet.m1': '0'}
    lc.pre_set_parameter(parameter_list)

    # ------ customized code start ------

    pass

    # ------ customized code end ------

    # Initialize mat saver
    saver = savemat.DataSaver('Abs_time', 'thrust', *tuple(lc.temp_keys), *tuple(DP.save_list_name), )
    lc.cf.commander.send_setpoint(0, 0, 0, 0)
    start_time = time.time()
    abs_time = 0
    flag = 0

    while lc.is_connected:
        abs_time = time.time() - start_time
        flag = flag + 1

        data_raw = UDP.get_data()
        data, save_list_data = DP.process_data(data_raw)

        logged_data = lc.get_logged_data()
        # print(logged_data)

        # ------ customized code start ------

        U_thrust = 0

        U_x = 0
        U_y = 0

        if U_thrust < 0:
            U_thrust = 0

        if flag % 10 == 0:
            print(U_thrust)

        lc.cf.commander.send_setpoint(0, 0, 0, U_thrust)

        # ------ customized code end ------

        if abs_time > 10:
            lc.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)
            lc.stop()
            break

        saver.add_elements(abs_time, U_thrust, *tuple(logged_data.values()), *tuple(save_list_data))
        time.sleep(0.01)


    saver.save2mat('DataExchange/')
    UDP.stop_thread()


