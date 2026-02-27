import os
import can
import gpiod
import time

INTERFACE = "socketcan"
CHANNEL = "can0"
BIT_RATE = 1000000


def reset_network():
    os.system("sudo ifconfig can0 down")
    os.system("sudo ip link set can0 up type can bitrate 1000000")
    os.system("sudo ip link set can0 up")


def check_gpio():
    print(f"is gpiochip device chip0 = {gpiod.is_gpiochip_device('/dev/gpiochip0')}")
    print(f"is gpiochip device chip1 = {gpiod.is_gpiochip_device('/dev/gpiochip1')}")


def can_tx(bus: can.bus.BusABC, id: int):
    # bus = can.Bus(CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=True)
    # for i in range(10):
    msg = can.Message(arbitration_id=id, data=[id, 5], is_extended_id=False)
    bus.send(msg)


def can_rx(bus: can.bus.BusABC):
    rec = bus.recv()
    if rec is not None:
        print(rec.data)
    else:
        print("rec was none")


def can_rx_alt(bus: can.bus.BusABC):
    rec = bus._recv_internal(2)
    print(rec)


def can_rx_loop(bus: can.bus.BusABC):
    rec = bus.recv()
    while rec != None:
        print(rec.data.decode())
        rec = bus.recv()


def main():
    reset_network()
    can.util.set_logging_level("info")
    bus = can.Bus(CHANNEL, INTERFACE, bitrate=BIT_RATE, receive_own_messages=True)

    while True:
        can_tx(bus, 3)
        can_rx(bus)
        time.sleep(1)
    # can_rx_loop(bus)

    # i = ""
    # while i != "q":
    #     i = input("r(read) or w(write)?\n> ")
    #     match(i):
    #         case "r":
    #             can_rx(bus)
    #             can_rx_alt(bus)
    #         case "w":
    #             can_tx(bus, 10)
    #         case "q":
    #             print("quitting")
    #         case _:
    #             print("problem")
    bus.shutdown()


if __name__ == "__main__":
    main()
