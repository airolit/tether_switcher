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
   ▄████████    ▄████████    ▄████████    ▄████████         ▄████████  ▄█       ▄██   ▄    ▄█  ███▄▄▄▄      ▄██████▄
  ███    ███   ███    ███   ███    ███   ███    ███        ███    ███ ███       ███   ██▄ ███  ███▀▀▀██▄   ███    ███
  ███    █▀    ███    ███   ███    █▀    ███    █▀         ███    █▀  ███       ███▄▄▄███ ███▌ ███   ███   ███    █▀
 ▄███▄▄▄      ▄███▄▄▄▄██▀  ▄███▄▄▄      ▄███▄▄▄           ▄███▄▄▄     ███       ▀▀▀▀▀▀███ ███▌ ███   ███  ▄███
▀▀███▀▀▀     ▀▀███▀▀▀▀▀   ▀▀███▀▀▀     ▀▀███▀▀▀          ▀▀███▀▀▀     ███       ▄██   ███ ███▌ ███   ███ ▀▀███ ████▄
  ███        ▀███████████   ███    █▄    ███    █▄         ███        ███       ███   ███ ███  ███   ███   ███    ███
  ███          ███    ███   ███    ███   ███    ███        ███        ███▌    ▄ ███   ███ ███  ███   ███   ███    ███
  ███          ███    ███   ██████████   ██████████        ███        █████▄▄██  ▀█████▀  █▀    ▀█   █▀    ████████▀
               ███    ███                                             ▀
   ▄████████  ▄█     █▄   ▄█      ███      ▄████████    ▄█    █▄       ▄████████    ▄████████
  ███    ███ ███     ███ ███  ▀█████████▄ ███    ███   ███    ███     ███    ███   ███    ███
  ███    █▀  ███     ███ ███▌    ▀███▀▀██ ███    █▀    ███    ███     ███    █▀    ███    ███
  ███        ███     ███ ███▌     ███   ▀ ███         ▄███▄▄▄▄███▄▄  ▄███▄▄▄      ▄███▄▄▄▄██▀
▀███████████ ███     ███ ███▌     ███     ███        ▀▀███▀▀▀▀███▀  ▀▀███▀▀▀     ▀▀███▀▀▀▀▀
         ███ ███     ███ ███      ███     ███    █▄    ███    ███     ███    █▄  ▀███████████
   ▄█    ███ ███ ▄█▄ ███ ███      ███     ███    ███   ███    ███     ███    ███   ███    ███
 ▄████████▀   ▀███▀███▀  █▀      ▄████▀   ████████▀    ███    █▀      ██████████   ███    ███
                                                                                   ███    ███
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

def mav_verify(mav, name, value, parm_type=None):
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

    mav.mav.param_request_read_send(mav.target_system, mav.target_component, name, -1)
    message = mav.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    if (vfloat == message['param_value']):
        return True
    else:
        return False

def set_and_verify_parameter(mav, name, value, retries=5, parm_type=None):
    time.sleep(0.5)
    success = False
    while retries > 0 and not success:
        retries -= 1
        mavset(mav, name, value, parm_type=parm_type)
        success = mav_verify(mav, bytes(name, 'utf-8'), value, parm_type=parm_type)
        if not success:
            print("Parameter not verified, resending")
            time.sleep(1)
    if (retries == 0):
        input("Failed to set parameter, restart the program again")
        exit()

print(header)
print("Connecting to drone..")
try:
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
except Exception as e:
    input("Failed to connect to drone!\nMake sure that connection is established between drone and PC.\nPress enter to exit..")
    exit()

# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Set parameters
set_and_verify_parameter(master, 'BAT_CRIT_THR', 0.300000011920928955, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
set_and_verify_parameter(master, 'BAT_EMERGEN_THR', 0.100000001490116119, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
set_and_verify_parameter(master, 'BAT_LOW_THR', 0.400000005960464478, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

set_and_verify_parameter(master, 'COM_DL_LOSS_T', 10, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
set_and_verify_parameter(master, 'GF_MAX_VER_DIST', 0.000000000000000000, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
set_and_verify_parameter(master, 'MNT_MODE_IN', 1, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
set_and_verify_parameter(master, 'NAV_DLL_ACT', 2, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
set_and_verify_parameter(master, 'NAV_RCL_ACT', 2, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
set_and_verify_parameter(master, 'RTL_RETURN_ALT', 50.000000000000000000, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

# Reboot before launching QGC
print("Rebooting drone")
master.reboot_autopilot()
time.sleep(2)
master.close()

print("Launcing AGC in tether mode..")
# Set a fake home for tether
os.environ["HOME"] = "/home/gs32/freeflying_home"
os.system("/home/gs32/AGC/agroundcontrol-start.sh")
