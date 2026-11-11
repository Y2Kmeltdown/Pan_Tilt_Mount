"""
Minimal Pelco-D example for the RS485 pan/tilt head (manual: "Pelco-D / Pelco-P adaptive").
Chain: PC (COM3) -> USB-serial adapter -> TTL-to-RS485 converter (auto direction) -> pan/tilt A/B
Requires: pip install pyserial

Pelco-D frame: 0xFF | address | cmd1 | cmd2 | data1 (pan speed) | data2 (tilt speed) | checksum
checksum = (address + cmd1 + cmd2 + data1 + data2) & 0xFF   (sync byte not included)
"""
import time
import serial  # pyserial

PORT = "COM6"
BAUD = 9600     # manual lists 1200/2400/4800/9600 - change if the head does not respond
ADDRESS = 57     # manual lists 0~64 - must match the head's address

RIGHT, LEFT, UP, DOWN = 0x02, 0x04, 0x08, 0x10


def pelco_d(address, cmd1, cmd2, data1=0, data2=0):
    body = [address, cmd1, cmd2, data1, data2]
    return bytes([0xFF] + body + [sum(body) & 0xFF])


def send(ser, frame):
    print("TX:", frame.hex(" ").upper())
    ser.write(frame)
    ser.flush()


def move(ser, direction, pan_speed=0x20, tilt_speed=0x20):
    send(ser, pelco_d(ADDRESS, 0x00, direction, pan_speed, tilt_speed))


def stop(ser):
    send(ser, pelco_d(ADDRESS, 0x00, 0x00, 0x00, 0x00))


def goto_preset(ser, n):
    send(ser, pelco_d(ADDRESS, 0x00, 0x07, 0x00, n))


def set_preset(ser, n):
    send(ser, pelco_d(ADDRESS, 0x00, 0x03, 0x00, n))


def main():
    with serial.Serial(PORT, BAUD, bytesize=8, parity=serial.PARITY_NONE,
                       stopbits=1, timeout=1) as ser:
        try:
            for name, d in (("right", RIGHT), ("left", LEFT), ("up", UP), ("down", DOWN)):
                print("Moving", name)
                move(ser, d)
                time.sleep(1.0)   # short timed move
                stop(ser)
                time.sleep(0.5)
        finally:
            stop(ser)             # always stop, even on Ctrl+C or error


if __name__ == "__main__":
    main()