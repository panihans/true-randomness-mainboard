import struct
import ctypes
from dataclasses import dataclass

import serial


@dataclass
class Command:
    motor1: int = 0
    motor2: int = 0
    motor3: int = 0
    thrower: int = 0
    led: int = 0
    delimiter: int = 0xBAD
    format = 'hhhhhh'

    def pack(self):
        return struct.pack(
            self.format,
            self.motor1,
            self.motor2,
            self.motor3,
            self.thrower,
            self.led,
            self.delimiter)

    def unpack(self, packed):
        unpacked = struct.unpack(self.format, packed)
        self.motor1 = unpacked[0]
        self.motor2 = unpacked[1]
        self.motor3 = unpacked[2]
        self.thrower = unpacked[3]
        self.led = unpacked[4]


def main():
    c = Command()
    c.motor1 = 50
    s = serial.Serial('COM4')
    s.write(c.pack())
    out = s.read(12)
    f = Command()
    f.unpack(out)
    k = out
    j = 1


if __name__ == '__main__':
    main()
