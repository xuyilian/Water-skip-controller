#!/usr/bin/env python3
"""Re-apply the hand-made `send_setpoint_revolving` patch to an installed cflib.

Why this file exists
--------------------
The flight stack depends on a custom CRTP setpoint that is NOT part of upstream
cflib. On the old Windows machine it was edited directly into site-packages, so
it was invisible to git and was lost on any `pip install --upgrade cflib`, on a
fresh env, or -- as actually happened -- on migrating to a new machine.

Keep this script in the repo. It is the only durable copy of the patch.

Pinned against cflib==0.1.32 (the version flown on the Windows machine).

Usage:
    python tools/cflib_revolving_patch.py          # apply
    python tools/cflib_revolving_patch.py --check  # report status only
"""
import argparse
import shutil
import sys
from pathlib import Path

EXPECTED_CFLIB = "0.1.32"

TYPE_ANCHOR = "TYPE_MANUAL = 11"
TYPE_BLOCK = """
# Custom type for the revolving setpoint. MUST match the firmware's
# packetType_e enum (revolvingType). Stock enum ends at manualType=11, so the
# custom build's next value is 12 -- confirm against your firmware if unsure.
TYPE_REVOLVING = 12"""

METHOD_BLOCK = '''
    def send_setpoint_revolving(self, roll, pitch, yawdeg, thrust):
        """\\
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
'''


def locate():
    try:
        from cflib.crazyflie import commander
    except ImportError:
        sys.exit("cflib is not installed in this interpreter.\n"
                 f"  pip install cflib=={EXPECTED_CFLIB}")
    return Path(commander.__file__)


def installed_version():
    try:
        from importlib.metadata import version
        return version("cflib")
    except Exception:
        return "unknown"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="report patch status without modifying anything")
    args = ap.parse_args()

    path = locate()
    ver = installed_version()
    src = path.read_text(encoding="utf-8")
    patched = "send_setpoint_revolving" in src

    print(f"cflib {ver} at {path}")
    if ver != EXPECTED_CFLIB:
        print(f"  ! expected {EXPECTED_CFLIB}; flight behaviour is only "
              f"verified on that version")

    if args.check:
        print("  patch: PRESENT" if patched else "  patch: MISSING")
        return 0 if patched else 1

    if patched:
        print("  patch already present -- nothing to do")
        return 0

    if TYPE_ANCHOR not in src:
        sys.exit(f"  could not find anchor {TYPE_ANCHOR!r}; cflib layout "
                 f"changed. Apply the patch by hand.")

    backup = path.with_suffix(".py.orig")
    if not backup.exists():
        shutil.copy2(path, backup)
        print(f"  backed up stock file -> {backup.name}")

    src = src.replace(TYPE_ANCHOR, TYPE_ANCHOR + TYPE_BLOCK, 1)
    if not src.endswith("\n"):
        src += "\n"
    src += METHOD_BLOCK
    path.write_text(src, encoding="utf-8")
    print("  patch applied")

    # Verify it actually imports and the method is bound.
    import subprocess
    r = subprocess.run(
        [sys.executable, "-c",
         "from cflib.crazyflie.commander import Commander;"
         "assert hasattr(Commander,'send_setpoint_revolving');"
         "print('  verified: Commander.send_setpoint_revolving present')"],
        capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip())
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
