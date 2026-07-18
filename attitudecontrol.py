# =====================================================================
#  attitudecontrol.py
#  A beginner-friendly drone "attitude" (orientation) controller.
#  Standalone learning scratch for a NORMAL quadrotor.
# =====================================================================
#
#  THE BIG PICTURE (read this first):
#
#        gamepad sticks                 drone sensors (over radio)
#             |                                |
#             v                                v
#    "I WANT to tilt 10 deg"        "I AM tilted 4 deg, turning 2 deg/s"
#             |                                |
#             +---------------+----------------+
#                             v
#                     PD CONTROLLER
#                (the math that decides the fix)
#                             |
#                             v
#              "push the motors THIS much" --radio--> drone
#
#  This whole picture repeats about 100 times every second, forever,
#  until you press the STOP button on the gamepad.
#
# ---------------------------------------------------------------------
#  SURVIVAL GUIDE TO READING PYTHON (the structure stuff):
#
#   * INDENTATION (the blank space at the start of a line) is how Python
#     groups lines. Lines indented the same amount, under a heading line,
#     all "belong together" as one block.
#   * A line ending in ':'  opens a block. The indented lines below it are
#     what runs "inside" that block.
#   * '#'  starts a comment. Python ignores everything after it -- comments
#     are notes for humans (like this whole header).
#   * 'def name(args):'  defines a FUNCTION = a named, reusable chunk of code
#     you "call" later by writing name(...).
#   * 'if condition:'     runs its block ONLY when the condition is true.
#   * 'while condition:'  REPEATS its block over and over while true (a loop).
#   * '='  means "take the value on the RIGHT and store it in the name on the
#     LEFT".  So  x = 5  means "let x be 5".
#   * 'try: ... except: ...'  means "attempt this; if it errors, do that
#     instead of crashing the whole program".
# =====================================================================


# ----- imports: borrow code other people already wrote, so we can use it -----
import time                                   # time.sleep() (pausing) + timestamps
import math                                   # math helpers (barely used here)
import numpy as np                            # number-crunching library ("np" = its nickname)

from scipy.spatial.transform import Rotation  # converts orientation formats (quaternion <-> angles)

from rislab_lib.Joystick.joystick_3 import PygameJoystick   # reads the gamepad
from RisLib.cflog import LoggingCore                          # opens radio link + streams drone data
from RisLib import savemat                                   # saves our data to a .mat file
from Library.quat import quat_operation                      # unpacks the drone's packed orientation
from jumping.jumping_model import RealTimeSleeper            # keeps the loop at a steady rate
from jumping.jumping_model import saturation                 # clamps a number between a min and a max
from jumping.jumping_estimator_v2 import RiseDetect          # detects a button's "just pressed" moment


# =====================================================================
#  FUNCTION 1: robot_stop  -- the safe shutdown / kill switch
# =====================================================================
def robot_stop(lc):
    # 'lc' is our live connection to the drone. Whoever calls this hands it in.
    # This function turns the motors off and hangs up the radio.

    # --- step A: command the motors OFF ---
    try:                                              # try this; don't crash if it fails
        lc.cf.commander.send_setpoint(0, 0, 0, 0)     # roll0 pitch0 yaw0 thrust0  ->  motors off
        time.sleep(0.05)                              # wait 0.05s so the message actually goes out
    except Exception as e:                            # if anything above broke...
        print(f'Warning: failed to send stop command: {e}')   # ...print why ({e} drops the error in)

    # --- step B: hang up the radio link ---
    try:
        lc.cf.close_link()                            # disconnect from the drone
        time.sleep(0.1)
    except Exception as e:
        print(f'Warning: failed to close Crazyflie link: {e}')


# =====================================================================
#  FUNCTION 2: wait_for_start  -- pause until the pilot says "go"
# =====================================================================
def wait_for_start(PJ):
    # 'PJ' is the gamepad reader. This stops the program here until you press
    # the Cross button, so nothing starts by surprise.
    print('Press cross button to continue ...')
    while not PJ.get_key('Cross'):     # "while the Cross button is NOT pressed, keep looping"
        PJ.step()                      # refresh the gamepad readings
        time.sleep(0.1)                # tiny pause so we don't spin the CPU at full speed
    # (once Cross IS pressed, the while condition becomes false and we leave the loop)


# =====================================================================
#  THE MAIN PROGRAM
#  The line below means "only run this if THIS file is the one being
#  started directly" (a standard Python guard). Everything indented under
#  it is the actual program.
# =====================================================================
if __name__ == '__main__':

    # =================================================================
    #  SECTION 1: CONFIG -- the knobs you tune. Just settings for now;
    #  none of this talks to the drone yet.
    # =================================================================

    uri = 'radio://0/70/2M'      # the drone's radio address: dongle 0, channel 70, 2 Mbit

    sample_time = 0.01           # seconds per loop pass. 0.01 = 100 passes per second (100 Hz)

    # A "deadzone": sticks never rest at exactly 0, so we treat tiny values as 0.
    DEADZONE_L = 0.2             # left stick
    DEADZONE_R = 0.2             # right stick

    # Thrust = overall throttle. The drone wants an integer roughly 0..65535;
    # we cap it low for bench safety.
    THRUST_MAX = 30000
    THRUST_MIN = 0

    # ----- PD gains for ROLL & PITCH (the tilt axes) -----
    # These are NEGATIVE because, on THIS hardware, a positive command tilts the
    # opposite way from our angle sign. Flipping the sign makes the controller
    # CORRECT errors instead of making them worse. (Found by bench testing.)
    MAX_TILT_DEG  = 15.0         # full left-stick push asks for this much tilt
    ATT_KP        = -0.5  # P gain: command per 1 degree of tilt error
    ATT_KD        = -0.03  # D gain: command per 1 deg/s of measured turn rate
    ATT_CMD_LIMIT = 120.0        # never let the roll/pitch command exceed +/- this

    # ----- controller for YAW (heading / nose direction) -----
    # On a normal drone the right stick X asks for a TURN RATE (deg/s), not a fixed
    # heading. Stick centred -> target rate 0 -> the controller fights any drift =
    # it holds the current heading. Push the stick -> it turns.
    YAW_RATE_STICK = 120.0       # full right-stick X  ->  this many deg/s of turn
    YAW_KP         = 0     # yaw command per 1 deg/s of turn-rate error
    YAW_CMD_LIMIT  = 0.5         # never let the yaw command exceed +/- this
    #   (the firmware multiplies this yaw number a lot internally, so keep it SMALL.
    #    If yaw pushes the WRONG way and spins up, flip YAW_KP to a negative number.)

    # =================================================================
    #  SECTION 2: SET-UP -- connect to the gamepad, the drone, and logging.
    # =================================================================

    PJ = PygameJoystick()        # create the gamepad reader
    wait_for_start(PJ)           # PAUSE here until you press Cross (see function above)

    RD_circle = RiseDetect()     # detects the single moment the Circle button is pressed

    RTS = RealTimeSleeper(sample_time)   # the loop's metronome (keeps it at 100 Hz)

    # Which drone variables to stream back to us over the radio.
    # Left side = firmware's name for the value. Right side = its data type.
    logging_list = {
        'stateEstimateZ.quat': 'uint32_t',   # the drone's orientation guess (a packed quaternion)
        'gyro.x': 'FP16',                     # how fast it's rolling   (deg/s)
        'gyro.y': 'FP16',                     # how fast it's pitching  (deg/s)
        'gyro.z': 'FP16',                     # how fast it's yawing    (deg/s)
        'pm.vbat': 'FP16',                    # battery voltage
    }

    lc = LoggingCore(uri, 10, logging_list)  # open the radio link & start streaming those values

    # Wait up to 10 seconds for the link to actually connect.
    connect_wait_start = time.time()                                   # remember "now"
    while not lc.is_connected and time.time() - connect_wait_start < 10.0:
        time.sleep(0.05)                                               # keep checking every 0.05s

    if not lc.is_connected:                      # if we still aren't connected after 10s...
        print('Crazyflie connection failed or timed out.')
        PJ.quit()                                # release the gamepad
        raise SystemExit                         # ...and stop the whole program

    print('Crazyflie connected, entering main loop ...')

    # Tell the motors to idle gently when armed (faster response than dead-stopped).
    try:
        lc.cf.param.set_value('powerDist.idleThrust', '6000')
        time.sleep(0.1)
        print('powerDist.idleThrust -> 6000')
    except Exception as e:
        print(f'[WARN] failed to set powerDist.idleThrust: {e}')

    # ----- the data saver -----
    # Records one ROW of numbers per loop, then writes them to a file at the end.
    # The NAMES here must match, IN ORDER, the values we save at the bottom of the
    # loop. (*tuple(lc.temp_keys) auto-expands to the streamed variables.)
    saver = savemat.DataSaver(
        'Abs_time', *tuple(lc.temp_keys),
        'cf_roll_deg', 'cf_pitch_deg', 'cf_yaw_deg',          # measured angles
        'des_roll_deg', 'des_pitch_deg', 'des_yaw_rate',      # what we asked for
        'cmd_roll', 'cmd_pitch', 'cmd_yaw', 'cmd_thrust',     # what we sent
        'right_stick_y', 'controllerEnable',
    )

    lc.cf.commander.send_setpoint(0, 0, 0, 0)   # one "all zero" command for a known safe start

    RTS.init()                          # start the metronome
    start_time = RTS.loop_start_time    # remember when the loop began (for timestamps)

    # ----- variables the loop will keep updating -----
    controllerEnable = False            # is the controller actively flying? (toggled by Circle)
    armed = False                       # have we told the drone it's allowed to spin motors?

    cmd_roll = cmd_pitch = cmd_yaw = 0.0   # current commands, start at 0
    cmd_thrust = 0
    yawdeg = 0.0                        # yaw-ANGLE channel (we leave this 0; we use rate instead)

    loop_flag = 0                       # counts loops; used to print only occasionally

    # =================================================================
    #  SECTION 3: THE CONTROL LOOP
    #  Repeats ~100x/sec while connected. Each pass does the full
    #  "read -> think -> act -> record" cycle from the big picture above.
    # =================================================================
    while lc.is_connected:

        loop_flag += 1                              # add 1 to the loop counter (+= means "add to")
        Abs_time = RTS.loop_start_time - start_time # seconds since the loop began (a timestamp)

        PJ.step()                                   # refresh the gamepad readings for this pass

        # -------------------------------------------------------------
        #  3a. BUTTONS
        # -------------------------------------------------------------

        # Circle = turn the controller ON/OFF. RD_circle.step(...) is true ONLY on
        # the press itself, so one press flips it once (not every loop it's held).
        if RD_circle.step(PJ.get_key('Circle')):
            controllerEnable = not controllerEnable    # 'not' flips True<->False
            if controllerEnable:                       # if we just turned it ON...
                print('controllerEnable -> True')
                if not armed:                          # ...and motors aren't armed yet...
                    lc.cf.platform.send_arming_request(True)   # arm them
                    armed = True
            else:                                      # otherwise we just turned it OFF...
                print('controllerEnable -> False')
                if armed:                              # ...so disarm the motors
                    lc.cf.platform.send_arming_request(False)
                    armed = False

        # Square = EMERGENCY STOP. Disarm, kill motors, close link, leave the loop.
        if PJ.get_key('Square'):
            if armed:
                lc.cf.platform.send_arming_request(False)
                armed = False
            robot_stop(lc)                             # the kill-switch function from the top
            break                                      # 'break' = jump out of the while loop now

        # -------------------------------------------------------------
        #  3b. READ THE STICKS  (what the pilot is asking for)
        #      Stick values run about -1.0 .. +1.0
        # -------------------------------------------------------------

        # LEFT stick -> desired tilt (roll = left/right, pitch = forward/back)
        JSL_x = float(PJ.get_key('LeftStickX'))        # left stick, sideways
        JSL_y = float(PJ.get_key('LeftStickY'))        # left stick, up/down
        if abs(JSL_x) < DEADZONE_L:                    # 'abs' = ignore the sign (size only)
            JSL_x = 0.0                                # inside the deadzone -> treat as 0
        if abs(JSL_y) < DEADZONE_L:
            JSL_y = 0.0

        # RIGHT stick, up/down -> thrust (throttle)
        JSR_y = float(PJ.get_key('RightStickY'))
        if abs(JSR_y) < DEADZONE_R:
            JSR_y = 0.0

        # RIGHT stick, sideways -> desired yaw turn rate
        JSR_x = float(PJ.get_key('RightStickX'))
        if abs(JSR_x) < DEADZONE_R:
            JSR_x = 0.0

        # Turn the right-stick-Y into a thrust number, clamped to [MIN .. MAX].
        # (minus sign: pushing the stick UP reads negative but should mean MORE thrust)
        thrust_manual = saturation(round(-JSR_y * THRUST_MAX), THRUST_MAX, THRUST_MIN)

        # Print thrust about once a second so the console isn't flooded.
        # 'loop_flag % 100 == 50' is true once every 100 loops (% = remainder).
        if thrust_manual > 8000 and loop_flag % 100 == 50:
            print('thrust (manual): ', thrust_manual)

        # -------------------------------------------------------------
        #  3c. READ THE DRONE'S SENSORS  (what it's actually doing)
        # -------------------------------------------------------------

        logged_data = lc.get_logged_data()             # latest values streamed from the drone

        # The drone sends its orientation as a PACKED quaternion (4 numbers squished
        # into 1 to save radio space). We unpack it, then convert to friendly angles.
        # Wrapped in try/except so a bad packet gives 0s instead of crashing.
        try:
            q_imu = quat_operation.quaternion_decompress(logged_data['stateEstimateZ.quat'])  # -> [x,y,z,w]
            cf_yaw_deg, cf_pitch_deg, cf_roll_deg = Rotation.from_quat(q_imu).as_euler('ZYX', degrees=True)
        except Exception:
            cf_roll_deg = 0.0                          # fallback if data was missing/garbled
            cf_pitch_deg = 0.0
            cf_yaw_deg = 0.0

        # Gyro = how fast the body is turning RIGHT NOW, in deg/s. These are the
        # "I'm already moving" signals the D term and the yaw controller use.
        gyro_x = float(logged_data.get('gyro.x', 0.0)) # roll rate
        gyro_y = float(logged_data.get('gyro.y', 0.0)) # pitch rate
        gyro_z = float(logged_data.get('gyro.z', 0.0)) # yaw rate

        # -------------------------------------------------------------
        #  3d. TURN STICK INPUT INTO TARGETS
        # -------------------------------------------------------------
        des_roll_deg  = JSL_x * MAX_TILT_DEG           # e.g. full stick -> +/-15 deg
        des_pitch_deg = -JSL_y * MAX_TILT_DEG
        des_yaw_rate  = JSR_x * YAW_RATE_STICK         # desired turn rate, deg/s (0 = hold heading)

        # -------------------------------------------------------------
        #  3e. THE PD CONTROLLER (the actual "brain")
        #
        #   For each axis:
        #
        #     desired ---(subtract)--->  error  --[ x Kp ]--+
        #     measured -/                                    +--> command
        #                                gyro    --[ x Kd ]--+
        #
        #     error = "how far from where I want to be"  -> P pushes toward target
        #     gyro  = "how fast I'm already turning"     -> D acts like a brake
        #     command = Kp*error  -  Kd*gyro
        # -------------------------------------------------------------

        # ROLL and PITCH: full PD (position error + rate damping)
        roll_pd  = ATT_KP * (des_roll_deg  - cf_roll_deg ) + ATT_KD * gyro_x
        pitch_pd = ATT_KP * (des_pitch_deg - cf_pitch_deg) + ATT_KD * gyro_y
        roll_pd  = saturation(roll_pd,  ATT_CMD_LIMIT, -ATT_CMD_LIMIT)   # clamp so it can't blow up
        pitch_pd = saturation(pitch_pd, ATT_CMD_LIMIT, -ATT_CMD_LIMIT)

        # YAW: we only have a RATE to work with (gyro_z), so this is a P controller
        # on turn-rate error. Stick centred -> des_yaw_rate 0 -> it cancels any
        # drift = holds heading. THIS is the yaw correction.
        yaw_pd = YAW_KP * (des_yaw_rate - gyro_z)
        yaw_pd = saturation(yaw_pd, YAW_CMD_LIMIT, -YAW_CMD_LIMIT)

        # LIVE YAW DEBUG: while the controller is on, print the yaw numbers a few
        # times per second. Twist the drone by hand and watch: if 'measured'
        # changes and 'cmd' reacts, the yaw controller IS working. If 'cmd' stays
        # ~0 while you twist, something upstream is wrong (sensor/sign/channel).
        if controllerEnable and loop_flag % 25 == 0:
            print(f'yaw  want={des_yaw_rate:7.1f} deg/s   measured={gyro_z:7.1f} deg/s   cmd={yaw_pd:+.3f}')

        # -------------------------------------------------------------
        #  3f. DECIDE WHAT TO SEND
        #      Only command real values when the controller is ON;
        #      otherwise hold everything at 0 so the drone stays inert.
        # -------------------------------------------------------------
        if controllerEnable:
            cmd_thrust = thrust_manual
            cmd_roll   = roll_pd
            cmd_pitch  = pitch_pd
            cmd_yaw    = yaw_pd        # <-- yaw correction flows into the yaw channel
            yawdeg     = 0.0           # we do NOT command a fixed yaw angle
        else:
            cmd_roll   = 0.0
            cmd_pitch  = 0.0
            cmd_yaw    = 0.0
            cmd_thrust = 0
            yawdeg     = 0.0

        # -------------------------------------------------------------
        #  3g. SEND THE COMMAND TO THE DRONE
        #
        #  The function's slots are:
        #     send_setpoint_revolving(roll, pitch, yaw_angle, yaw_torque, thrust)
        #
        #  NOTE the deliberate SWAP: we put cmd_pitch in the roll slot and cmd_roll
        #  in the pitch slot. On this drone our angle naming and the firmware's
        #  input axes don't line up, so the sticks felt swapped on the bench --
        #  crossing them here makes it respond the intuitive way.
        #
        #  yawdeg (yaw ANGLE) stays 0; cmd_yaw carries our yaw correction (a torque).
        # -------------------------------------------------------------
        lc.cf.commander.send_setpoint_revolving(
            cmd_pitch,     # -> roll slot
            cmd_roll,      # -> pitch slot
            
            yawdeg,        # -> yaw angle (unused, 0)
            cmd_yaw,       # -> yaw torque (our yaw correction)
            cmd_thrust,    # -> thrust
        )

        # -------------------------------------------------------------
        #  3h. RECORD THIS LOOP'S DATA (one row). Order MUST match the
        #      column names given to DataSaver earlier.
        # -------------------------------------------------------------
        saver.add_elements(
            Abs_time, *tuple(logged_data.values()),
            float(cf_roll_deg), float(cf_pitch_deg), float(cf_yaw_deg),
            float(des_roll_deg), float(des_pitch_deg), float(des_yaw_rate),
            float(cmd_roll), float(cmd_pitch), float(cmd_yaw), int(cmd_thrust),
            float(JSR_y), bool(controllerEnable),
        )

        RTS.sleep()    # pause just enough so this pass took ~sample_time (keeps us at 100 Hz)

    # =================================================================
    #  SECTION 4: SHUTDOWN
    #  We reach here after the emergency-stop 'break'. Best-effort stop
    #  the motors, close the link, and write all the recorded data out.
    # =================================================================
    try:
        lc.cf.commander.send_setpoint(0, 0, 0, 0)   # motors off
        time.sleep(0.05)
    except Exception as e:
        print(f'Warning: failed to send stop command: {e}')

    try:
        lc.cf.close_link()                          # hang up the radio
        time.sleep(0.1)
    except Exception as e:
        print(f'Warning: failed to close Crazyflie link: {e}')

    PJ.quit()                                       # release the gamepad
    saver.save2mat('DataExchange/')                 # write all rows to a .mat file
