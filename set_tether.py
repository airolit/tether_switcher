from __future__ import print_function

import time, struct, os

# Import mavutil
from pymavlink import mavutil

header = """
   ▄████████  ▄█     ▄████████  ▄██████▄   ▄█        ▄█      ███
  ███    ███ ███    ███    ███ ███    ███ ███       ███  ▀█████████▄
  ███    ███ ███▌   ███    ███ ███    ███ ███       ███▌    ▀███▀▀██
  ███    ███ ███▌  ▄███▄▄▄▄██▀ ███    ███ ███       ███▌     ███   ▀
▀███████████ ███▌ ▀▀███▀▀▀▀▀   ███    ███ ███       ███▌     ███
  ███    ███ ███  ▀███████████ ███    ███ ███       ███      ███
  ███    ███ ███    ███    ███ ███    ███ ███▌    ▄ ███      ███
  ███    █▀  █▀     ███    ███  ▀██████▀  █████▄▄██ █▀      ▄████▀
                    ███    ███            ▀
    ███        ▄████████     ███        ▄█    █▄       ▄████████    ▄████████
▀█████████▄   ███    ███ ▀█████████▄   ███    ███     ███    ███   ███    ███
   ▀███▀▀██   ███    █▀     ▀███▀▀██   ███    ███     ███    █▀    ███    ███
    ███   ▀  ▄███▄▄▄         ███   ▀  ▄███▄▄▄▄███▄▄  ▄███▄▄▄      ▄███▄▄▄▄██▀
    ███     ▀▀███▀▀▀         ███     ▀▀███▀▀▀▀███▀  ▀▀███▀▀▀     ▀▀███▀▀▀▀▀
    ███       ███    █▄      ███       ███    ███     ███    █▄  ▀███████████
    ███       ███    ███     ███       ███    ███     ███    ███   ███    ███
   ▄████▀     ██████████    ▄████▀     ███    █▀      ██████████   ███    ███
                                                                   ███    ███
   ▄████████  ▄█     █▄   ▄█      ███      ▄████████    ▄█    █▄       ▄████████    ▄████████
  ███    ███ ███     ███ ███  ▀█████████▄ ███    ███   ███    ███     ███    ███   ███    ███
  ███    █▀  ███     ███ ███▌    ▀███▀▀██ ███    █▀    ███    ███     ███    █▀    ███    ███
  ███        ███     ███ ███▌     ███   ▀ ███         ▄███▄▄▄▄███▄▄  ▄███▄▄▄      ▄███▄▄▄▄██▀
▀███████████ ███     ███ ███▌     ███     ███        ▀▀███▀▀▀▀███▀  ▀▀███▀▀▀     ▀▀███▀▀▀▀▀
         ███ ███     ███ ███      ███     ███    █▄    ███    ███     ███    █▄  ▀███████████
   ▄█    ███ ███ ▄█▄ ███ ███      ███     ███    ███   ███    ███     ███    ███   ███    ███
 ▄████████▀   ▀███▀███▀  █▀      ▄████▀   ████████▀    ███    █▀      ██████████   ███    ███
                                                                                   ███    ███                                                                                                                                           ░
"""

def mavset(mav, name, value, retries=3, parm_type=None):
    '''set a parameter on a mavlink connection'''
    print("Updating parameter: " + name)
    got_ack = False

    if parm_type is not None and parm_type != mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        # need to encode as a float for sending
        if parm_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            vstr = struct.pack(">xxxB", int(value))
        elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            vstr = struct.pack(">xxxb", int(value))
        elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            vstr = struct.pack(">xxH", int(value))
        elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            vstr = struct.pack(">xxh", int(value))
        elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            vstr = struct.pack(">I", int(value))
        elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            vstr = struct.pack(">i", int(value))
        else:
            print("can't send %s of type %u" % (name, parm_type))
            return False
        vfloat, = struct.unpack(">f", vstr)
    else:
        vfloat = float(value)

    while retries > 0 and not got_ack:
        retries -= 1
        mav.param_set_send(name.upper(), vfloat, parm_type=parm_type)
        tstart = time.time()
        while time.time() - tstart < 1:
            ack = mav.recv_match(type='PARAM_VALUE', blocking=False)
            if ack is None:
                time.sleep(0.1)
                continue
            if str(name).upper() == str(ack.param_id).upper():
                got_ack = True
                break
    if not got_ack:
        print("timeout setting %s to %f" % (name, vfloat))
        return False
    return True

print(header)
print("Connecting to drone..")
try:
    master = mavutil.mavlink_connection("udpin:10.100.0.2:14550")
except Exception as e:
    input("Failed to connect to drone!\nMake sure that connection is established between drone and PC.\nPress enter to exit..")
    exit()

# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Set parameters
mavset(master, 'BAT_CRIT_THR', 0.500000000000000000, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
mavset(master, 'BAT_EMERGEN_THR', 0.449999988079071045, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
mavset(master, 'BAT_LOW_THR', 0.600000023841857910, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

mavset(master, 'COM_DL_LOSS_T', 5, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
mavset(master, 'GF_MAX_VER_DIST', 45.000000000000000000, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
mavset(master, 'MNT_MODE_IN', 0, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
mavset(master, 'NAV_DLL_ACT', 3, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
mavset(master, 'NAV_RCL_ACT', 0, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
mavset(master, 'RTL_RETURN_ALT', 0.000000000000000000, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

# Reboot before launching QGC
print("Rebooting drone")
master.reboot_autopilot()
time.sleep(2)
master.close()

print("Launcing AGC in tether mode..")
# Set a fake home for tether
os.environ["HOME"] = "/home/gs32/tether_home"
os.system("/home/gs32/AGC/agroundcontrol-start.sh")
