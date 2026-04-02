import hid
import struct


# ================== SpaceMouse setup ==================

def setup_spacemouse(vendor_id=0x256f, product_id=0xc62e):
    """
    Initialize the SpaceMouse device.
    """
    device = hid.Device(vid=vendor_id, pid=product_id)
    # device.open(vendor_id, product_id)
    # device.set_nonblocking(True)
    return device


def read_spacemouse(dev, state):
    """
    Read SpaceMouse data and update the state dictionary.
    """
    data = dev.read(64, timeout=10)
    if not data:
        return

    report_id = data[0]

    if report_id == 0x01 and len(data) >= 13:  # translation/rotation report
        vals = struct.unpack("<hhhhhh", bytes(data[1:13]))
        state["x"]  =  vals[0]
        state["y"]  = -vals[1]  # flip axis if needed
        state["z"]  = -vals[2]
        state["rx"] =  vals[3]
        state["ry"] =  vals[4]
        state["rz"] = -vals[5]
    elif report_id == 0x03 and len(data) >= 3:  # button report
        state["buttons"] = struct.unpack("<H", bytes(data[1:3]))[0]

    flush_hid(dev)

# def read_dualshock_4(device, state: dict[str, float]):
#     data = device.read(64)
#     if not data:
#         print("no data")
#         return
#
#     report_id = data[0]
#     print(report_id)
#
#     if report_id == 0x01 and len(data) >= 13:  # translation/rotation report
#         vals = struct.unpack("<hhhhhh", bytes(data[1:13]))
#         print(vals)
#         state["x"]  =  vals[0]
#         state["y"]  = -vals[1]  # flip axis if needed
#         state["z"]  = -vals[2]
#         state["rx"] =  vals[3]
#         state["ry"] =  vals[4]
#         state["rz"] =  vals[5]
#     elif report_id == 0x03 and len(data) >= 3:  # button report
#         state["buttons"] = struct.unpack("<H", bytes(data[1:3]))[0]
#
#     flush_hid(device)


def flush_hid(device):
    """This will flush the hid buffer of an item, if there is a delay blocking it.

    Args:
        device (_type_): _description_
    """
    while True:
        data = device.read(64, timeout=10)
        if not data:
            break
        elif len(data) == 0:
            break


def flush_hid_preserve_buttons(device, state):
    """
    Drain HID buffer but keep the most recent button state.
    """
    last_buttons = state["buttons"]

    while True:
        data = device.read(64)
        if not data:
            break

        report_id = data[0]

        if report_id == 0x03 and len(data) >= 3:
            last_buttons = struct.unpack("<H", bytes(data[1:3]))[0]

    state["buttons"] = last_buttons

