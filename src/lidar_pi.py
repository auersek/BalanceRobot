# ------------------------
# Part 1 (Raspberry Pi): lidar_send.py
# ------------------------

import serial
import socket
import struct
from enum import Enum

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 230400
UDP_IP = "172.20.10.2"  # <-- replace with your Mac's IP address
UDP_PORT = 5005

PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

State = Enum("State", ["SYNC0", "SYNC1", "SYNC2", "LOCKED"])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)

state = State.SYNC0
data = b''

while True:
    if state == State.SYNC0:
        if lidar_serial.read() == b'\x54':
            data = b'\x54'
            state = State.SYNC1
    elif state == State.SYNC1:
        if lidar_serial.read() == b'\x2C':
            data += b'\x2C'
            state = State.SYNC2
        else:
            state = State.SYNC0
    elif state == State.SYNC2:
        data += lidar_serial.read(PACKET_LENGTH - 2)
        if len(data) == PACKET_LENGTH:
            sock.sendto(data, (UDP_IP, UDP_PORT))
        state = State.SYNC0

