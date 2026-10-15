# Extracted from the patched cflib in the `cf` conda env, 2026-10-15.
# Source file: cflib/crazyflie/commander.py
# This is the ONLY cflib file that was modified.

# Type constants as they stand in the patched file:
TYPE_STOP = 0
TYPE_VELOCITY_WORLD_LEGACY = 1
TYPE_ZDISTANCE_LEGACY = 2
TYPE_HOVER_LEGACY = 5
TYPE_FULL_STATE = 6
TYPE_POSITION = 7
TYPE_VELOCITY_WORLD = 8
TYPE_ZDISTANCE = 9
TYPE_HOVER = 10
TYPE_MANUAL = 11
TYPE_REVOLVING = 12
TYPE_META_COMMAND_NOTIFY_SETPOINT_STOP = 0

# The added method, to be pasted into class Commander:

    def send_setpoint_revolving(self, roll, pitch, yawdeg, thrust):
        """\
        Send a custom compressed 4-value revolving setpoint.

        Payload:
        - u_x: world-frame X command
        - u_y: world-frame Y command
        - yaw: mocap yaw angle in degrees
        - thrust: integer thrust command, 0 to 65535

        Firmware must parse TYPE_REVOLVING with payload format:
            <BhhhH

        Scaling:
        - roll/pitch: int16 = value * 10, resolution 0.1
        - yawdeg: int16 = value * 100, resolution 0.01 deg
        """
        if thrust > 0xFFFF or thrust < 0:
            raise ValueError('Thrust must be between 0 and 0xFFFF')

        def _to_i16(value, scale, name):
            value_i16 = int(round(float(value) * scale))
            if value_i16 > 32767 or value_i16 < -32768:
                raise ValueError(
                    f'{name}={value} is out of compressed int16 range '
                    f'[{(-32768 / scale):.3f}, {(32767 / scale):.3f}]'
                )
            return value_i16

        roll_i16 = _to_i16(roll, 10.0, 'roll')
        pitch_i16 = _to_i16(pitch, 10.0, 'pitch')
        yawdeg_i16 = _to_i16(yawdeg, 100.0, 'yawdeg')

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.channel = SET_SETPOINT_CHANNEL

        # Payload: [type_id][u_x_i16][u_y_i16][yawdeg_i16][thrust_u16]
        pk.data = struct.pack(
            '<BhhhH',
            TYPE_REVOLVING,
            roll_i16,
            pitch_i16,
            yawdeg_i16,
            int(thrust)
        )

        self._cf.send_packet(pk)

