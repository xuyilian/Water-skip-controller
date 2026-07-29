# =============================================================================
# revolvingdario.py -- water-hop flight controller
#
# Joystick-teleoperated Crazyflie flight script for the water-skipping MAV.
# Supports several vehicles/boards (see MACHINES) and drives them through a
# hop experiment cycle -- manual takeoff, closed-loop position hold, and a
# DROP mode that releases the vehicle into the water hop -- while logging
# telemetry to a .mat file for offline analysis (see analyse.m).
#
# File map (top to bottom):
#   1. Imports
#   2. Helpers: robot_stop(), SetpointSender (radio-sender thread)
#   3. Per-machine hardware config: MACHINES
#   4. Mocap rigid-body selection: select_rigid_body()
#   5. Crazyflie selection: select_machine()
#   6. Shared height-PID helper: height_pd_thrust()
#   7. Main script (under `if __name__ == '__main__':`):
#        7a. Startup selection (machine, drone body, pool body)
#        7b. Configuration constants (joystick scaling, mode gains, etc.)
#        7c. Init (controllers, estimators, mocap receiver, logging, connect)
#        7d. Runtime state (mutated by the main loop)
#        7e. Main control loop
#        7f. Shutdown / cleanup
# =============================================================================

import atexit
import os
import threading
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


# =============================================================================
# Helpers: motor stop + radio setpoint sender thread
# =============================================================================

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


class SetpointSender(threading.Thread):
    """Push revolving setpoints to the radio in a dedicated thread.

    The control loop calls update() (non-blocking) with the newest setpoint;
    this thread transmits only the most recent value. If the radio link
    back-pressures (cflib's out_queue has size 1 and blocks on put), only
    this thread waits -- the control loop keeps running at full rate and any
    intermediate setpoints are simply superseded by the latest one.
    """

    def __init__(self, cf, period=0.005):
        super().__init__(name='SetpointSenderThread', daemon=True)
        self._cf = cf
        self._period = period
        self._lock = threading.Lock()
        self._setpoint = (0.0, 0.0, 0.0, 0)   # roll, pitch, yawdeg, thrust
        self._active = False                  # don't send until first update
        self._stop_evt = threading.Event()

    def update(self, roll, pitch, yawdeg, thrust):
        """Store the latest setpoint (called from the control loop)."""
        with self._lock:
            self._setpoint = (float(roll), float(pitch),
                              float(yawdeg), int(thrust))
            self._active = True

    def run(self):
        while not self._stop_evt.is_set():
            t0 = time.time()
            if self._active:
                with self._lock:
                    roll, pitch, yawdeg, thrust = self._setpoint
                try:
                    self._cf.commander.send_setpoint_revolving(
                        roll, pitch, yawdeg, thrust)
                except Exception as e:
                    print(f'[sender] send failed: {e}')
            # pace the loop; radio throughput further limits the real rate
            dt = self._period - (time.time() - t0)
            if dt > 0:
                self._stop_evt.wait(dt)

    def stop(self):
        self._stop_evt.set()


# =============================================================================
# Per-machine hardware configuration
# =============================================================================
# Each MACHINES entry bundles ALL settings specific to one physical vehicle:
# radio channel, thrust trim, yaw mount offset, BI/height gains, and (for the
# tangential-spin board) mixer/spin-thrust params. Pick one at startup with
# select_machine() (Option to toggle, Cross to confirm).
MACHINES = [
    {
        'name': 'cf2',
        'radio_channel': 60,          # -> radio://0/<ch>/2M
        'thrust_base': 35000,         # HEIGHT_THRUST_BASE (JBI/BI/FBI)
        'manual_base_thrust': 35000,  # MANUAL_BASE_THRUST (manual mode)
        'yaw_offset_deg': 50 ,       # added to mocap yaw before send
        'fixed_yawrate_deg': -4800.0, # R_LMS_FIXED_YAWRATE_DEG (LMS estimator)
        'bi_r13_r23_kp': -25.5,        # BI controller r13_r23_kp
        'bi_xy_kp': 0.35,              # BI controller xy_kp
        'bi_xy_kd': 0.6,             # BI controller xy_kd
        'height_z_kp': 23000.0,        # height PID (JBI/BI/FBI/manual joystick stage)
        'height_z_ki': 0.0,
        'height_z_kd': 10000.0,
        'height_z_int_limit': 1.0,
        'height_thrust_min': 3000,
        'height_thrust_max': 60000,
    },
    {
        'name': 'cf2bl',
        'radio_channel': 80,          # TODO: set actual channel for machine 2
        'thrust_base': 18000,         # TODO: calibrate for machine 2
        'manual_base_thrust': 18000,  # TODO: calibrate for machine 2
        'yaw_offset_deg': 180.0,      # TODO: calibrate yaw mount for machine 2
        'fixed_yawrate_deg': -2100.0, # TODO: calibrate for machine 2
        'bi_r13_r23_kp': -2.5,        # TODO: calibrate for machine 2
        'bi_xy_kp': 0.15,              # TODO: calibrate for machine 2
        'bi_xy_kd': 0.15,             # TODO: calibrate for machine 2
        'height_z_kp': 23000.0,        # TODO: calibrate for machine 2
        'height_z_ki': 0.0,            # TODO: calibrate for machine 2
        'height_z_kd': 10000.0,        # TODO: calibrate for machine 2
        'height_z_int_limit': 1.0,     # TODO: calibrate for machine 2
        'height_thrust_min': 3000,     # TODO: calibrate for machine 2
        'height_thrust_max': 60000,    # TODO: calibrate for machine 2
    },
    {
        # Tangential-spin board on the Crazyflie 2.1 BRUSHLESS platform: 2
        # vertical lift rotors (motors 1 & 3) + 2 tangential yaw-spin rotors
        # (motors 2 & 4, fixed thrust, no lift/attitude authority) -- motor
        # numbering confirmed via cfclient's motor-power-set-index test,
        # 2026-07-23. See power_distribution_quadrotor.c
        # (powerDistributionTangentialSpin) for the firmware-side mixer --
        # unchanged by the brushless switch, it applies to PLATFORM_CF21BL
        # exactly the same as PLATFORM_CF2.
        #
        # IMPORTANT: this board runs DIFFERENT FIRMWARE. Build with
        # `make cf21bl_defconfig && make`, which produces build/cf21bl.bin
        # (NOT cf2.bin) -- flash that file, not the cf2 build.
        #
        # Every value below is a first-guess placeholder and needs FULL
        # recalibration on the bench: brushless motors on DSHOT have a
        # fundamentally different thrust curve than the old brushed/
        # OneShot125 motors these numbers were guessed from -- do not trust
        # any of them as a starting altitude, ramp up from a low value.
        'name': 'cf21blspin',
        'radio_channel': 80,          # confirmed -- this board is on channel 80
        'thrust_base': 23000,         # TODO: recalibrate from scratch -- brushless thrust curve, not brushed
        'manual_base_thrust': 23000,  # TODO: recalibrate from scratch, same reason
        'yaw_offset_deg': 30,         # TODO: recalibrate on bench
        'fixed_yawrate_deg': -6000.0, # TODO: recalibrate -- spin comes from dedicated tangential thrust, not prop reaction torque
        'bi_r13_r23_kp': -25.5,       # TODO: recalibrate
        'bi_xy_kp': 0.35,             # TODO: recalibrate
        'bi_xy_kd': 0.6,              # TODO: recalibrate
        'mixer_type': 1,              # selects powerDistributionTangentialSpin in firmware
        'spin_thrust': 30000,         # TODO: bench-tune from scratch -- fixed thrust for motors 2 & 4, brushless DSHOT curve
        'drop_spin_thrust': 30000,    # TODO: tune separately from spin_thrust -- motors 2/4 thrust during DROP mode only
        'motor_thrust_ceiling': 60000, # per-motor max thrust, this board
        'idle_thrust': 7000,          # TODO: bench-tune -- DSHOT/brushless idle floor differs from the 6000 brushed default sent elsewhere
        # Height PID: copied from cf2 as a STARTING POINT only. This board has
        # only 2 lift motors (half of cf2's 4) on a different motor
        # technology (brushless/DSHOT vs brushed/OneShot125), so its thrust-
        # per-height-error response has no reason to match cf2's -- treat
        # these as unverified and retune from scratch on the bench.
        'height_z_kp': 24000.0,        # TODO: recalibrate from scratch
        'height_z_ki': 0.0,            # TODO: recalibrate from scratch
        'height_z_kd': 10000.0,        # TODO: recalibrate from scratch
        'height_z_int_limit': 1.0,     # TODO: recalibrate from scratch
        'height_thrust_min': 3000,     # TODO: recalibrate from scratch
        'height_thrust_max': 60000,    # TODO: recalibrate from scratch
    },
    {
        # Tangential-spin board on the Crazyflie BOLT platform: same physical
        # motors/props/hydrofoils/frame as cf21blspin above (2 vertical lift
        # rotors m1/m3 + 2 tangential yaw-spin rotors m2/m4, fixed thrust) --
        # only the flight-controller board changed. Reason: the CF2.1
        # Brushless board (cf21blspin) is CONFIRMED BROKEN hardware (never a
        # firmware/software issue) and was replaced with this Bolt board.
        #
        # M1 history, 2026-07: this exact Bolt board's M1 signal line
        # wouldn't drive its motor (self-test flutter worked, but neither
        # cfclient's motor-power-set-index test nor Python could spin it).
        # Root cause found and fixed: a burnt resistor on M1's signal path.
        # M1 is now confirmed working -- this entry assumes that fix is in
        # place. If M1 misbehaves again, suspect the resistor/solder joint
        # again before suspecting firmware.
        #
        # IMPORTANT: this board runs DIFFERENT FIRMWARE (same mixer source,
        # different platform target). Build with `make bolt_defconfig &&
        # make`, which produces build/bolt.bin (NOT cf2.bin/cf21bl.bin) --
        # flash that file for this board.
        #
        # Values below are copied from cf21blspin as a first-guess starting
        # point (same motors/geometry) -- cf21blspin itself was NEVER
        # flight-validated (its board was broken), so treat every number here
        # as an untested placeholder needing full bench recalibration, same
        # caution as cf21blspin. Also bench-re-verify the m1/m3 vs m2/m4
        # motor-role mapping and corner positions on THIS physical board
        # before flying -- the wiring harness moved to a new connector set.
        'name': 'bolt',
        'radio_channel': 70,           # custom channel for this board
        'radio_address': 'E7E7E7E7E7', # note: this is actually the Crazyflie DEFAULT address
        'thrust_base': 18000,          # TODO: recalibrate from scratch
        'manual_base_thrust': 18000,   # TODO: recalibrate from scratch
        'yaw_offset_deg': 80,          # TODO: recalibrate on bench
        'fixed_yawrate_deg': -8700.0,  # TODO: recalibrate
        'bi_r13_r23_kp': -25.5,        # TODO: recalibrate
        'bi_xy_kp': 0.2,              # TODO: recalibrate
        'bi_xy_kd': 0.3,               # TODO: recalibrate
        'mixer_type': 1,               # selects powerDistributionTangentialSpin in firmware
        'spin_thrust': 40000,          # TODO: bench-tune from scratch
        'drop_spin_thrust': 40000,     # TODO: tune separately from spin_thrust -- motors 2/4 thrust during DROP mode only
        'motor_thrust_ceiling': 65535, # per-motor max thrust, this board
        'idle_thrust': 6000,           # TODO: bench-tune
        'height_z_kp': 20000.0,        # TODO: recalibrate from scratch
        'height_z_ki': 0.0,            # TODO: recalibrate from scratch
        'height_z_kd': 13000.0,        # TODO: recalibrate from scratch
        'height_z_int_limit': 1.0,     # TODO: recalibrate from scratch
        'height_thrust_min': 3000,     # TODO: recalibrate from scratch
        'height_thrust_max': 65535,    # TODO: recalibrate from scratch
    },
]


# =============================================================================
# Mocap rigid-body selection
# =============================================================================
# The mocap volume can hold several rigid bodies at once (parallel experiments,
# plus the pool). body_index is 1-based: index k reads the k-th rigid body in
# the stream. Selectors are ALWAYS shown so you pick both the DRONE body and
# the POOL body every run. The packet size gives a hint of how many bodies are
# currently streaming (14 bytes/body), but Motive only streams a body while it
# is tracked, so we always offer at least RIGID_BODY_CHOICES options.
RIGID_BODY_CHOICES = 4


def select_rigid_body(PJ: PygameJoystick, indices, label='rigid body'):
    """Pick a mocap rigid body index (1-based).

    Several experiments share the mocap volume, so the same menu is used to
    pick both the drone body and the pool body. Press Option to cycle, Cross
    to confirm.
    """
    # Drain any Cross still held from the previous menu so it doesn't
    # instantly confirm this one.
    while True:
        PJ.step()
        if not PJ.get_key('Cross'):
            break
        time.sleep(0.05)

    i = 0
    prev_option = False
    print(f'Select {label}: Option = toggle, Cross = confirm')
    print(f'  -> {label} index {indices[i]}')
    while True:
        PJ.step()
        option = bool(PJ.get_key('Option'))
        if option and not prev_option:
            i = (i + 1) % len(indices)
            print(f'  -> {label} index {indices[i]}')
        prev_option = option
        if PJ.get_key('Cross'):
            break
        time.sleep(0.1)
    idx = indices[i]
    print(f'Selected {label} index {idx}')
    return idx


# =============================================================================
# Crazyflie machine selection
# =============================================================================

def select_machine(PJ: PygameJoystick, machines):
    """Pick which Crazyflie to fly.

    Press Option to cycle through the machines, Cross to confirm and start.
    Returns the selected machine dict.
    """
    idx = 0
    prev_option = False
    print('Select machine: Option = toggle, Cross = confirm')
    m = machines[idx]
    print(f'  -> {m["name"]} (ch {m["radio_channel"]})')
    while True:
        PJ.step()
        option = bool(PJ.get_key('Option'))
        if option and not prev_option:
            idx = (idx + 1) % len(machines)
            m = machines[idx]
            print(f'  -> {m["name"]} (ch {m["radio_channel"]})')
        prev_option = option
        if PJ.get_key('Cross'):
            break
        time.sleep(0.1)
    m = machines[idx]
    print(
        f'Selected {m["name"]}: ch {m["radio_channel"]}, '
        f'thrust_base {m["thrust_base"]}, '
        f'manual_base {m["manual_base_thrust"]}, '
        f'yaw_offset {m["yaw_offset_deg"]}, '
        f'fixed_yawrate {m["fixed_yawrate_deg"]}'
    )
    return m


# =============================================================================
# Shared height-PID helper (used by manual mode's joystick stage)
# =============================================================================

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


# =============================================================================
# Main script
# =============================================================================
if __name__ == '__main__':

    # -------------------------------------------------------------------
    # 7a. Startup selection: machine, then drone rigid body, then pool
    #     rigid body (all via the same joystick Option/Cross menu).
    # -------------------------------------------------------------------

    # ---------------- machine selection ----------------
    # Initialize joystick and pick which Crazyflie to fly.
    # Option toggles between machines, Cross confirms and starts.
    PJ = PygameJoystick()
    machine = select_machine(PJ, MACHINES)

    # Pick which mocap rigid bodies to use this run: first the DRONE body, then
    # the POOL body (its origin = pool centre, used by AUTOBI). Done here
    # (before the UDP receiver starts) so the toggles always appear at boot,
    # even if the mocap stream isn't up yet. Offers indices 1..RIGID_BODY_CHOICES.
    rigid_body_index = select_rigid_body(
        PJ, list(range(1, RIGID_BODY_CHOICES + 1)), label='DRONE rigid body')
    pool_rigid_body_index = select_rigid_body(
        PJ, list(range(1, RIGID_BODY_CHOICES + 1)), label='POOL rigid body')
    if pool_rigid_body_index == rigid_body_index:
        print(f'[WARN] pool body index equals the drone body index '
              f'({rigid_body_index}) — AUTOBI would chase the drone itself. '
              f'Fix in Motive or reboot and pick different indices.')

    # -------------------------------------------------------------------
    # 7b. Configuration constants. Everything from here down to the
    #     "init" section below is pure config -- no hardware/radio side
    #     effects yet.
    # -------------------------------------------------------------------

    # ---------------- user config ----------------
    uri = f'radio://0/{machine["radio_channel"]}/2M'
    # Bolt board runs a custom radio address (not the Crazyflie default) --
    # append it only if the machine defines one.
    if machine.get('radio_address'):
        uri += f'/{machine["radio_address"]}'

    # per-machine yaw offset (deg) added to mocap yaw before sending
    YAW_OFFSET_DEG = machine['yaw_offset_deg']

    sample_time = 0.01
    cf_log_period_ms = 10
    save_data_enable = True

    udp_enable_flag = True
    # rigid_body_index chosen above via select_rigid_body()

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
    MANUAL_BASE_THRUST = machine['manual_base_thrust']
    MANUAL_RAMP_TIME = 10.0
    MANUAL_THRUST_MIN = 7000
    MANUAL_JOYSTICK_RP_SCALE = 15

    # right stick Y integrates desired_z over time
    DESIRED_Z_RATE = 0.2      # m/s when RightStickY is fully pushed
    DESIRED_Z_MIN = 0.1
    DESIRED_Z_MAX = 1.5

    # BI mode: left stick integrates the horizontal setpoint over time.
    # desired_x/desired_y accumulate joystick * rate * dt each loop
    # (same velocity-style nudge as desired_z), clamped to +/- limit.
    DESIRED_XY_RATE = 0.5     # m/s when LeftStick is fully pushed
    DESIRED_XY_LIMIT = 1.5    # m, saturation bound on desired_x/desired_y

    # Height PD parameters (JBI / BI / FBI / manual joystick stage) -- per-
    # machine, since lift-motor count/technology differs across boards (e.g.
    # cf21blspin's 2 brushless lift motors vs cf2's 4 brushed ones).
    HEIGHT_THRUST_BASE = machine['thrust_base']
    HEIGHT_THRUST_MIN = machine['height_thrust_min']
    HEIGHT_THRUST_MAX = machine['height_thrust_max']
    HEIGHT_Z_KP = machine['height_z_kp']
    HEIGHT_Z_KI = machine['height_z_ki']
    HEIGHT_Z_KD = machine['height_z_kd']
    HEIGHT_Z_INT_LIMIT = machine['height_z_int_limit']

    # BI controller gains (per-machine)
    BI_R13_R23_KP = machine['bi_r13_r23_kp']
    BI_XY_KP = machine['bi_xy_kp']
    BI_XY_KD = machine['bi_xy_kd']

    # DROP / hop mode: cut the LIFT motors (m1/m3) to a low thrust so the MAV
    # falls into the hop, while the TANGENTIAL spin motors (m2/m4, tangential-
    # spin boards only) keep spinning at their own separately-tuned rate so
    # the vehicle doesn't shed yaw rate during the freefall. Two independent
    # knobs:
    #   - DROP_THRUST (below): sent as cmd_thrust every loop in DROP -> drives
    #     m1/m3 (and, on the legacy mixer, all 4 motors -- there's no
    #     separate tangential pair there, so DROP_THRUST alone applies).
    #   - machine['drop_spin_thrust']: pushed to firmware ONCE, on entering
    #     DROP (see the Triangle handler below), as a fresh powerDist.spinThrust
    #     value -- independent of the normal flight spin_thrust. Restored to
    #     the flight value if the cycle ever comes back around to 'manual'.
    #     Only meaningful on tangential-spin boards (mixer_type 1); ignored
    #     otherwise.
    DROP_THRUST = 8000

    # Hop experiment cycle: manual -> JBI -> BI -> AUTOBI -> DROP -> manual.
    #   Circle spins up (manual ramp),
    #   Triangle #1 -> JBI    (fly up manually; desired_z reset to
    #                          JBI_INITIAL_HEIGHT on entry),
    #   Triangle #2 -> BI     (hold a fixed (x, y) setpoint, reset to
    #                          BI_INITIAL_X/Y on entry; right-stick-Z as usual;
    #                          left stick still nudges x/y from there),
    #   Triangle #3 -> AUTOBI (auto-locate the pool centre in X/Y only -- Z is
    #                          untouched, left wherever BI/manual left it, same
    #                          as legacy behavior; left stick can still nudge
    #                          x/y off the found centre, same mechanism as BI),
    #   Triangle #4 -> DROP   (release into the water hop; spin motors switch
    #                          to drop_spin_thrust, lift motors cut to DROP_THRUST).
    # FBI is left OUT of the cycle; its controller code is kept but stale.
    CONTROL_MODES = ('manual', 'JBI', 'BI', 'AUTOBI', 'DROP')
    # Closed-loop (controller-driven) modes. AUTOBI reuses the BI controller
    # (XY position -> tilt, height PD for Z). FBI stays listed so its existing
    # code remains valid. DROP is open-loop (fixed low thrust), so it is NOT here.
    CLOSED_LOOP_MODES = ('JBI', 'BI', 'AUTOBI', 'FBI')

    # (AUTOBI's pool-centre auto-locate behavior is implemented where its
    # state variables are declared, in the runtime-state section below --
    # see "AUTOBI pool-centre auto-locate state.")

    # JBI: desired_z is reset to this the instant JBI is entered (Triangle
    # #1), so every run starts each JBI stage from the same known altitude
    # instead of whatever desired_z happened to be left at previously.
    JBI_INITIAL_HEIGHT = 1

    # BI: desired_x/desired_y are reset to this fixed (x, y) the instant BI is
    # entered (Triangle #2) -- "a set coord" to hold before handing off to
    # AUTOBI. Still nudgeable afterward via the left stick, same as always.
    BI_INITIAL_X = 0.0
    BI_INITIAL_Y = 0.0

    # AUTOBI now does real pool-centre auto-locate (no fixed test target) --
    # left as a toggle in case open-space bench testing without the pool is
    # ever needed again. Z is unaffected either way -- it stays whatever it
    # was left at entering AUTOBI (BI's height or manual/right-stick), and
    # stays right-stick adjustable, matching the legacy behavior.
    AUTOBI_USE_FIXED_TEST_TARGET = False
    AUTOBI_TEST_X = 0.0
    AUTOBI_TEST_Y = 0.0

    # Online LMS low-frequency estimator for R13/R23
    # Model:
    #   R13 = A13 + B13*sin(phase) + C13*cos(phase)
    #   R23 = A23 + B23*sin(phase) + C23*cos(phase)
    # phase is integrated from a fixed yawrate to extract low-frequency R13/R23
    R_LMS_MU = 0.5
    R_LMS_INPUT_ALPHA = None
    R_LMS_YAW_RATE_ALPHA = 1.0
    R_LMS_START_YAWRATE_DEG = 500.0
    R_LMS_FIXED_YAWRATE_DEG = machine['fixed_yawrate_deg']
    R_LMS_YAWRATE_SPIKE_WINDOW = 0
    R_LMS_YAWRATE_JUMP_LIMIT = 5200.0
    R_LMS_YAWRATE_ABS_LIMIT = 9000.0

    # realtime first-order low-pass filter for derivative signals
    # smaller alpha -> stronger filtering, larger delay
    DERIV_FILTER_ALPHA = 0.5
    MOCAP_Z_FILTER_ALPHA = 0.3

    # -------------------------------------------------------------------
    # 7c. Init: edge detectors, controllers, estimators, mocap receiver,
    #     onboard logging, Crazyflie connection, power-distribution params,
    #     and the flight-data saver.
    # -------------------------------------------------------------------

    # ---------------- init ----------------
    # -- edge detectors & timing --
    RD_circle = RiseDetect()
    RD_triangle = RiseDetect()

    RTS = RealTimeSleeper(sample_time)

    # -- attitude/position controllers (BI / JBI / FBI) --
    BI = BiController(
        r13_r23_kp=BI_R13_R23_KP,
        xy_cmd_limit=120,
        thrust_base=HEIGHT_THRUST_BASE,
        thrust_min=HEIGHT_THRUST_MIN,
        thrust_max=HEIGHT_THRUST_MAX,
        z_kp=HEIGHT_Z_KP,
        z_ki=HEIGHT_Z_KI,
        z_kd=HEIGHT_Z_KD,
        z_int_limit=HEIGHT_Z_INT_LIMIT,
        xy_kp=BI_XY_KP,
        xy_kd=BI_XY_KD)

    JBI = JoyBiController(
        r13_r23_kp=20,
        xy_cmd_limit=120.0,
        thrust_base=HEIGHT_THRUST_BASE,
        thrust_min=HEIGHT_THRUST_MIN,
        thrust_max=HEIGHT_THRUST_MAX,
        z_kp=HEIGHT_Z_KP,
        z_ki=HEIGHT_Z_KI,
        z_kd=HEIGHT_Z_KD,
        z_int_limit=HEIGHT_Z_INT_LIMIT,
        joy_r13_gain=0.8,
        joy_r23_gain=0.8,
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

    # -- R13/R23 and XY position LMS estimators --
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

    # -- mocap UDP receiver & per-body unpackers --
    # UDP mocap, used for raw x/y/z, quaternion, and velocity input.
    if udp_enable_flag:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        udp_time = 0.0

        # Info only: how many bodies the stream currently carries (14 bytes each).
        # The body was already chosen above; warn if it isn't streaming yet.
        num_bodies = max(1, UDP.len_data // 14)
        print(f'Mocap stream currently carries {num_bodies} rigid body(ies).')
        if rigid_body_index > num_bodies:
            print(f'[WARN] selected rigid body {rigid_body_index} is not in the stream yet '
                  f'({num_bodies} present) — mocap will read zeros until it is tracked.')
        # AUTOBI needs the pool rigid body (its origin = pool centre).
        if pool_rigid_body_index == rigid_body_index:
            print(f'[WARN] pool body index ({pool_rigid_body_index}) equals the drone '
                  f'body index — pick a DIFFERENT rigid body for AUTOBI.')
        elif pool_rigid_body_index > num_bodies:
            print(f'[WARN] pool body {pool_rigid_body_index} is not in the stream yet '
                  f'({num_bodies} present) — AUTOBI will hold position until it is tracked.')
    else:
        UDP = None
        udp_time = 0.0
        rigid_body_index = 1

    # RealTimeProcessor is only used to unpack mocap packet fields here.
    sample_rate_rt = 1.0 / sample_time
    dp_mocap = RealTimeProcessor(sample_rate_rt)
    # Second unpacker for the pool rigid body (its origin = pool centre), used
    # by AUTOBI to auto-locate the vehicle over the 50x50 cm pool.
    dp_pool = RealTimeProcessor(sample_rate_rt)

    # -- onboard logging + Crazyflie connection --
    # Log battery voltage so we can watch for voltage sag under the revolving
    # load (fast-spinning props draw a lot of current). One float at ~10 Hz
    # keeps the added radio traffic negligible.
    logging_list = {'pm.vbat': 'float'}
    lc = LoggingCore(uri, 100, logging_list)

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

    # -- power distribution parameters (idleThrust / mixerType / spinThrust / thrustCap) --
    # Set motor idle thrust before entering control loop. Per-machine so the
    # brushed (cf2/cf2bl, default 6000) and brushless/DSHOT (cf21blspin,
    # needs its own bench-tuned value) boards don't share a floor that only
    # makes sense for one motor technology.
    try:
        idle_thrust = machine.get('idle_thrust', 6000)
        lc.cf.param.set_value('powerDist.idleThrust', str(idle_thrust))
        time.sleep(0.1)
        print(f'powerDist.idleThrust -> {idle_thrust}')
    except Exception as e:
        print(f'[WARN] failed to set powerDist.idleThrust: {e}')

    # Select the firmware motor mixer for this machine. powerDist.mixerType is
    # NOT persistent (resets to 0/legacy on every boot), so it must be set
    # explicitly every connection -- this is what lets one firmware image
    # (cf2.bin) serve both the legacy 4-motor boards and the tangential-spin
    # board, switched purely by which MACHINES entry you pick at startup.
    try:
        mixer_type = machine.get('mixer_type', 0)
        lc.cf.param.set_value('powerDist.mixerType', str(mixer_type))
        time.sleep(0.1)
        print(f'powerDist.mixerType -> {mixer_type}')

        if mixer_type == 1:
            spin_thrust = machine['spin_thrust']
            lc.cf.param.set_value('powerDist.spinThrust', str(spin_thrust))
            time.sleep(0.1)
            print(f'powerDist.spinThrust -> {spin_thrust}')

        thrust_cap = machine.get('motor_thrust_ceiling', 65535)
        lc.cf.param.set_value('powerDist.thrustCap', str(thrust_cap))
        time.sleep(0.1)
        print(f'powerDist.thrustCap -> {thrust_cap}')
    except Exception as e:
        print(f'[WARN] failed to set power distribution params: {e}')

    # -- flight-data logging to .mat --
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
            'pm_vbat',
            'pool_x', 'pool_y', 'autobi_desired_x', 'autobi_desired_y',
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

    # -- radio setpoint sender thread --
    lc.cf.commander.send_setpoint(0, 0, 0, 0)

    # Decouple radio sending from the control loop: this thread always
    # transmits the latest setpoint, so radio back-pressure never stalls
    # the loop (which would trigger "loop frequency lower than expected").
    sender = SetpointSender(lc.cf, period=sample_time)
    sender.start()

    RTS.init()
    start_time = RTS.loop_start_time

    # -------------------------------------------------------------------
    # 7d. Runtime state -- everything below is mutated by the main loop.
    # -------------------------------------------------------------------

    controllerEnable = False
    manual_start_time = None
    manual_stage = 'off'
    manual_z_error_i = 0.0
    armed = False

    # control_mode cycle with Triangle:
    #   manual -> JBI -> BI -> AUTOBI -> DROP -> manual
    control_mode = 'manual'

    desired_x = 0.0
    desired_y = -0.3
    desired_z = 1

    # AUTOBI pool-centre auto-locate state.
    # The 50x50 cm water pool is defined in Motive as a single rigid body
    # whose origin is at the pool centre (4 corner markers), streamed at
    # pool_rigid_body_index (chosen at boot, right after the drone body --
    # see the startup-selection section above; they must be DIFFERENT
    # indices). AUTOBI captures this centre once on entry and holds it (see
    # the main loop's AUTOBI branch) so a momentary marker dropout can't jump
    # the target to (0,0).
    pool_x = 0.0            # live pool-centre X from the pool rigid body
    pool_y = 0.0            # live pool-centre Y
    pool_valid = False      # True once a non-trivial pool sample has been seen
    autobi_desired_x = 0.0  # captured pool-centre target held during AUTOBI
    autobi_desired_y = 0.0
    autobi_captured = False

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

    # =====================================================================
    # 7e. MAIN CONTROL LOOP
    # =====================================================================
    while lc.is_connected:
        loop_flag += 1
        Abs_time = RTS.loop_start_time - start_time
        if last_loop_abs_time is None:
            loop_dt = sample_time
        else:
            loop_dt = max(1e-6, Abs_time - last_loop_abs_time)
        last_loop_abs_time = Abs_time

        PJ.step()

        # ---- toggle controller (Circle) ----
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

        # ---- cycle control mode (Triangle): manual -> JBI -> BI -> AUTOBI -> DROP ----
        if RD_triangle.step(PJ.get_key('Triangle')):
            control_mode = CONTROL_MODES[
                (CONTROL_MODES.index(control_mode) + 1) % len(CONTROL_MODES)
            ]
            print(f'control_mode -> {control_mode}')

            if control_mode == 'JBI':
                desired_z = JBI_INITIAL_HEIGHT
                print(f'[JBI] desired_z reset to {JBI_INITIAL_HEIGHT:.2f} m')
            elif control_mode == 'BI':
                desired_x = BI_INITIAL_X
                desired_y = BI_INITIAL_Y
                print(f'[BI] desired_x/y reset to ({BI_INITIAL_X:.2f}, {BI_INITIAL_Y:.2f}) m')
            elif control_mode == 'DROP':
                drop_spin = machine.get('drop_spin_thrust')
                if drop_spin is not None and machine.get('mixer_type', 0) == 1:
                    try:
                        lc.cf.param.set_value('powerDist.spinThrust', str(drop_spin))
                        print(f'[DROP] powerDist.spinThrust -> {drop_spin}')
                    except Exception as e:
                        print(f'[WARN] failed to set DROP spinThrust: {e}')

            if controllerEnable and control_mode == 'manual':
                manual_start_time = Abs_time
                manual_stage = 'ramp'
                manual_z_error_i = 0.0
                print(
                    f'[manual] ramp stage: {MANUAL_THRUST_MIN} -> {MANUAL_BASE_THRUST} '
                    f'in {MANUAL_RAMP_TIME:.1f}s'
                )
                # Cycled all the way back around from DROP -- restore the
                # normal flight spin thrust (only matters if it was ever
                # switched away, i.e. the run went through DROP already).
                if machine.get('mixer_type', 0) == 1:
                    try:
                        lc.cf.param.set_value('powerDist.spinThrust', str(machine['spin_thrust']))
                        print(f"powerDist.spinThrust -> {machine['spin_thrust']} (restored)")
                    except Exception as e:
                        print(f'[WARN] failed to restore spinThrust: {e}')
            else:
                if manual_stage != 'off':
                    print('[manual] off stage: thrust = 0')
                manual_start_time = None
                manual_stage = 'off'
                manual_z_error_i = 0.0

        # ---- emergency stop (Square) ----
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

            # Pool rigid body (its origin = pool centre) for AUTOBI. If the pool
            # index is out of range, step() leaves the last values; treat an
            # exact (0,0) as "no pool seen yet".
            dp_pool.step(data_raw, body_index=pool_rigid_body_index, abstime=udp_time)
            pool_x = float(dp_pool.X)
            pool_y = float(dp_pool.Y)
            if pool_x != 0.0 or pool_y != 0.0:
                pool_valid = True
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

        # BI mode: integrate horizontal setpoint from the left stick over time.
        # Push LeftStick X/Y to walk desired_x/desired_y; release to hold.
        # Y is negated to match JBI's and manual mode's LeftStick/RightStick-Y
        # convention (JBI.update() passes -JSL_y; manual's cmd_pitch uses
        # -JSL_y) -- without this, BI's up/down feel is reversed relative to
        # every other mode. X needs no negation; it already matches everywhere.
        if controllerEnable and control_mode == 'BI':
            desired_x = saturation(
                desired_x + JSL_x * DESIRED_XY_RATE * sample_time,
                DESIRED_XY_LIMIT,
                -DESIRED_XY_LIMIT,
            )
            desired_y = saturation(
                desired_y - JSL_y * DESIRED_XY_RATE * sample_time,
                DESIRED_XY_LIMIT,
                -DESIRED_XY_LIMIT,
            )
        # AUTOBI: same left-stick nudge mechanism as BI (same rate, same
        # limit, same BiController gains -- AUTOBI is just BI with an
        # auto-located starting target), applied on top of the captured pool
        # centre so you can trim off the auto-locate result. Gated on
        # autobi_captured so there's nothing to nudge before the pool is found
        # (see the AUTOBI dispatch below, which owns the capture itself).
        elif controllerEnable and control_mode == 'AUTOBI' and autobi_captured:
            autobi_desired_x = saturation(
                autobi_desired_x + JSL_x * DESIRED_XY_RATE * sample_time,
                DESIRED_XY_LIMIT,
                -DESIRED_XY_LIMIT,
            )
            autobi_desired_y = saturation(
                autobi_desired_y - JSL_y * DESIRED_XY_RATE * sample_time,
                DESIRED_XY_LIMIT,
                -DESIRED_XY_LIMIT,
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

        # AUTOBI holds a captured pool-centre target; clear it whenever we are
        # not in AUTOBI so it re-captures fresh on the next entry.
        if control_mode != 'AUTOBI':
            autobi_captured = False

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
            elif control_mode == 'AUTOBI':
                if AUTOBI_USE_FIXED_TEST_TARGET:
                    # Fixed test setpoint (see TEMPORARY TEST OVERRIDE above) --
                    # pool-centre logic below is bypassed, not removed.
                    autobi_desired_x = AUTOBI_TEST_X
                    autobi_desired_y = AUTOBI_TEST_Y
                else:
                    # Auto-locate to the pool centre in X/Y (Z stays on the right
                    # stick via the shared height PD). Capture the pool centre once
                    # on entry and hold it, so a momentary marker dropout can't jump
                    # the target to (0,0).
                    if not autobi_captured:
                        if pool_valid:
                            autobi_desired_x = pool_x
                            autobi_desired_y = pool_y
                            autobi_captured = True
                            print(f'[AUTOBI] pool centre captured: '
                                  f'x={autobi_desired_x:.3f}, y={autobi_desired_y:.3f}')
                        else:
                            # No pool sample yet: hold current position rather than
                            # driving to the origin. Retry capture next loop.
                            autobi_desired_x = mocap_x_filt
                            autobi_desired_y = mocap_y_filt
                            if loop_flag % 100 == 50:
                                print(f'[AUTOBI] waiting for pool body '
                                      f'{pool_rigid_body_index} — holding position')
                BI.update(autobi_desired_x, autobi_desired_y, desired_z,
                          R13_filt, R23_filt,
                          mocap_x_filt, mocap_y_filt, mocap_z_raw,
                          mocap_vx_filt, mocap_vy_filt, mocap_vz,
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
                #cmd_roll = 0
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

            elif control_mode == 'DROP':
                # Water-hop drop: no attitude/height control. cmd_thrust drives
                # the lift motors (m1/m3, and all 4 on the legacy mixer) down
                # to DROP_THRUST so the MAV falls. On tangential-spin boards,
                # the tangential motors (m2/m4) don't read cmd_thrust at all --
                # they run at whatever powerDist.spinThrust currently is,
                # which was switched to machine['drop_spin_thrust'] the instant
                # DROP was entered (see the Triangle handler above). Stays
                # armed so the motors don't stop.
                cmd_thrust = DROP_THRUST
                cmd_roll = 0.0
                cmd_pitch = 0.0
                yaw_deg = mocap_yaw_deg
                cmd_yaw = 0.0

                if loop_flag % 100 == 50:
                    print(f'[DROP] lift thrust: {cmd_thrust}')

        else:
            cmd_roll = 0.0
            cmd_pitch = 0.0
            yaw_deg = mocap_yaw_deg
            cmd_yaw = 0.0
            cmd_thrust = 0

        yaw_send_deg = ((mocap_yaw_deg + YAW_OFFSET_DEG + 180) % 360) - 180

        # Hand the latest setpoint to the sender thread (non-blocking).
        sender.update(cmd_roll, cmd_pitch, yaw_send_deg, cmd_thrust)

        # Latest logged battery voltage (held between ~10 Hz log updates).
        # Low-battery warning fires at 3.2 V on the CF2 firmware.
        pm_vbat = float(lc.get_logged_data().get('pm.vbat', 0.0))
        if loop_flag % 100 == 50:
            warn = ' <-- LOW' if 0.0 < pm_vbat < 3.2 else ''
            print(f'[battery] vbat = {pm_vbat:.2f} V{warn}')

        # ---- flight-data logging ----
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
                float(pm_vbat),
                float(pool_x), float(pool_y),
                float(autobi_desired_x), float(autobi_desired_y),
            )

        RTS.sleep()
    # ===================== end of main control loop =====================

    # -------------------------------------------------------------------
    # 7f. Shutdown / cleanup
    # -------------------------------------------------------------------

    # Stop the sender thread before issuing the final motor-stop command,
    # so it can't keep pushing revolving setpoints afterwards.
    sender.stop()
    sender.join(timeout=1.0)

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
