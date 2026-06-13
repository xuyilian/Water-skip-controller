import logging
import cflib.crtp
import threading
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie


def motor_enable(cf):
    cf.commander.send_setpoint(0, 0, 0, 0)


class CmdSend(object):
    def __init__(self, uri):
        self.cf_done = False
        self.U_X = 0
        self.U_Y = 0
        self.U_yaw = 0      # used as tau_z in send_setpoint_revolving
        self.U_Z = 0
        self.AngleY = 0
        self.uri = uri
        self.special_send = False
        self.special_send_vx = 0
        self.special_send_vy = 0
        self.special_send_vz = 0
        self.special_send_yawrate = 0

    def thread_start(self, command_send_ready):
        controller_thread1 = threading.Thread(target=self.thread_worker,args=( command_send_ready,))
        controller_thread1.start()
        self.send(0, 0, 0, 0, 0)

    def thread_worker(self, command_send_ready):
        # -----Crazyflie init
        logging.basicConfig(level=logging.DEBUG,
                            format='(%(threadName)-10s) %(message)s', )
        # URI = 'radio://0/80/2M'
        # Only output errors from the logging framework
        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers(enable_debug_driver=False)
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            motor_enable(cf)
            print('CrazyFlie: ', self.uri, ' connected')
            command_send_ready.set()
            while not self.cf_done:
                command_send_ready.wait()
                # Custom 5-value packet: (tau_x, tau_y, tau_z, yaw, thrust)
                cf.commander.send_setpoint(float(self.U_X), float(self.U_Y), float(self.U_yaw),
                                                     int(max(0, min(65535, int(self.U_Z)))))
                if self.special_send:
                    # send other commands
                    cf.commander.send_velocity_world_setpoint(self.special_send_vx,self.special_send_vy,
                                                              self.special_send_vz,self.special_send_yawrate)
                    self.special_send = False
                command_send_ready.clear()

    def send(self, U_X, U_Y, U_yaw, U_Z, AngleY):
        self.U_X = U_X
        self.U_Y = U_Y
        self.U_yaw = U_yaw
        self.U_Z = U_Z
        self.AngleY = AngleY

    def thread_stop(self, command_send_ready):
        command_send_ready.set()
        self.cf_done = True
        print('CrazyFlie: ', self.uri, ' disconnected')

    def send_type_2(self, vx, vy, vz, yawrate):
        self.special_send = True
        self.special_send_vx = vx
        self.special_send_vy = vy
        self.special_send_vz = vz
        self.special_send_yawrate = yawrate


class CmdSendQuad(object):  # for a standard crazyflie robot
    def __init__(self, uri):
        self.cf_done = False
        self.U_roll = 0
        self.U_pitch = 0
        self.U_yaw = 0
        self.U_thrust = 0

        self.uri = uri

    def thread_start(self, command_send_ready):
        controller_thread1 = threading.Thread(target=self.thread_worker, args=(command_send_ready,))
        controller_thread1.start()

    def send(self, roll, pitch, yaw, thrust, ):
        self.U_roll = roll
        self.U_pitch = pitch
        self.U_yaw = yaw
        self.U_thrust = thrust

    def thread_stop(self, command_send_ready):
        command_send_ready.set()
        self.cf_done = True
        print('CrazyFlie: ', self.uri, ' disconnected')

    def thread_worker(self, command_send_ready):
        # -----Crazyflie init
        logging.basicConfig(level=logging.DEBUG,
                            format='(%(threadName)-10s) %(message)s', )
        # URI = 'radio://0/80/2M'
        # Only output errors from the logging framework
        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers(enable_debug_driver=False)
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            motor_enable(cf)
            print('CrazyFlie: ', self.uri, ' connected')
            command_send_ready.set()
            cf.commander.send_setpoint(0, 0, 0, 0)
            while not self.cf_done:
                command_send_ready.wait()
                cf.commander.send_setpoint(self.U_roll, self.U_pitch, self.U_yaw, self.U_thrust)
                command_send_ready.clear()
