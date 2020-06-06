from typing import Union
from time import sleep

from spidev import SpiDev
import RPi.GPIO as GPIO

from .rc522_register import RC522Register
from .rc522 import RC522

class RC522Spi(RC522):

    def __init__(self, dev: SpiDev, reset_pin: None):
        super().__init__()
        self.dev = dev
        self.reset_pin = reset_pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        GPIO.output(self.reset_pin, 1)
        sleep(0.05)  # 50 ms
        self.reset(hard=True)
        self.init()

    def _encode_register(self, register: RC522Register, write: bool=False) -> int:
        result = (register << 1)
        if write == False:
            result |= (1 << 7)
        return result

    def register_write(self, register: RC522Register, data: bytes) -> None:
        encoded = self._encode_register(register, write=True)
        print("Write {:x} -> {:x}".format(register, encoded), data)
        buffer = [encoded, *data]
        self.dev.xfer2(buffer)

    def register_read(self, register: RC522Register, count: int=1) -> Union[int, bytes]:
        result = bytearray()
        for i in range(count):
            buffer = [0x00, 0x00]
            buffer[0] = self._encode_register(register)
            values = self.dev.xfer2(buffer)
            result.append(values[1])
        if len(result) == 1:
            return result[0]
        return bytes(result)

    def reset(self, hard: bool=False) -> None:
        if hard and self.reset_pin:
            # hard reset via reset pin
            GPIO.output(self.reset_pin, 0)
            sleep(0.05)  # 50 ms
            GPIO.output(self.reset_pin, 1)
        else:
            # soft reset
            super().reset(hard=False)