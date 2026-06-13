import threading
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import numpy as np


def saturation(x, max_x, min_x):
    if x > max_x:
        return max_x
    elif x < min_x:
        return min_x
    else:
        return x


class SwarmCore(object):
    def __init__(self, uri_dict=None, logging_dict=None, logging_sample_time=10 ):
        if uri_dict is None:
            URI1 = 'radio://0/28/2M'
            URI2 = 'radio://0/21/2M'
            uri_dict = {URI1: [0], URI2: [1]}
        # main variables
        self.uris = set(list(uri_dict.keys()))
        self.seq_args = uri_dict
        self.done = False
        self.rpyts = np.zeros([len(self.seq_args), 4])

        self.sample_time = logging_sample_time
        self._logging_dict = logging_dict

        self._logging_output = {}
        for i in logging_dict:
            self._logging_output[i] = logging_dict[i]  # element by elemnet copying

        for i in self._logging_output:
            self._logging_output[i] = 0.0
        print(self._logging_output)

        time.sleep(0.2)

        # temp_dict = {}
        # for i in logging_dict:
        #     temp_dict[i] = logging_dict[i]  # element by elemnet copying
        # self.temp_keys = list(temp_dict)
        # for key in self.temp_keys:
        #     self.temp_keys[self.temp_keys.index(key)] = key.replace(".", "_")
        # time.sleep(0.1)

    def get_logged_data(self):
        return self._logging_output

    def start_swarm(self):
        threading.Thread(target=self._swarm_robot).start()

    def _swarm_robot(self):
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(self.uris, factory=factory) as swarm:
            # print(1)
            swarm.parallel(self._run_sequence, args_dict=self.seq_args)

    def _run_sequence(self, scf, sequence):

        cflib.crtp.init_drivers()
        lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        lg_stab.add_variable('pm.vbatMV', 'uint16_t')
        # set parameter

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        with SyncLogger(scf, lg_stab) as logger:
            for log_entry in logger:
                roll = self.rpyts[sequence, 0]
                pitch = self.rpyts[sequence, 1]
                yaw = self.rpyts[sequence, 2]
                thrust = saturation(round(self.rpyts[sequence, 3]), 65534, 0)

                # print(log_entry)


                scf.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
                if self.done:
                    break

        scf.cf.commander.send_setpoint(0, 0, 0, 0)

    def stop(self):
        self.done = True

    def set_cmd(self, rpyts):
        self.rpyts = rpyts


class Run(object):
    def __init__(self, parameter_list):
        # URI1 = 'radio://0/86/2M'
        # URI2 = 'radio://0/84/2M'
        URI1 = 'radio://0/28/2M'
        URI2 = 'radio://0/21/2M'
        self.thrust = 0
        self.u_x = 0
        self.u_y = 0
        self.us_x = 0
        self.us_y = 0
        self.yaw = 0
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0
        self.thrust_test = 0
        self.uris = {URI1, URI2}
        self.seq_args = {URI1: [1], URI2: [2]}
        self.done = False
        self.parameter_flag = False
        self.parameter_list = parameter_list

    def start_swarm(self):
        threading.Thread(target=self.swarm_robot).start()

    def set_data(self, u_x, u_y, thrust, yaw, e, us_x, us_y):

        self.u_x = u_x
        self.u_y = u_y
        self.us_x = us_x
        self.us_y = us_y
        self.thrust = thrust
        self.yaw = yaw
        self.thrust_test = round(saturation(e, 63000, 0))

    def swarm_robot(self):
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(self.uris, factory=factory) as swarm:
            # print(1)
            swarm.parallel(self._run_sequence, args_dict=self.seq_args)

    def set_param(self):
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(self.uris, factory=factory) as swarm:
            # print(1)
            swarm.parallel(self._parameter_send)

    def _parameter_send(self, scf):
        # set parameter
        for key in self.parameter_list:
            scf.cf.param.set_value(key, self.parameter_list[key])
            time.sleep(0.1)
        print('parameters set',)

    def _run_sequence(self, scf, sequence):
        # set parameter

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        # if sequence == 1:
        #     scf.cf.param.set_value('stabilizer.mode', '0')
        # if sequence == 2:
        #     scf.cf.param.set_value('stabilizer.mode', '0')

        while not self.done:

            if sequence == 1:
                # scf.cf.commander.send_setpoint(self.u_x, self.u_y, self.yaw, self.thrust)
                scf.cf.commander.send_setpoint(self.u_x, self.u_y, self.yaw, self.thrust)

            elif sequence == 2:
                scf.cf.commander.send_setpoint(self.us_x, -self.us_y, 0, 0)
                # scf.cf.commander.send_setpoint(self.us_y, -self.us_x, 0, 0)

            # scf.cf.commander.send_setpoint(0, 0, 0, thrust_flight)
            time.sleep(0.01)

        scf.cf.commander.send_setpoint(0, 0, 0, 0)

    def stop(self):
        self.done = True