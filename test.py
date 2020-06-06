from traceback import print_last

from spidev import SpiDev
import RPi.GPIO as GPIO

from rfid_rc522 import RC522Spi

try:
    spi = SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 100000
    
    rfid = RC522Spi(spi, 22)

    assert rfid.calculate_crc(b'123456789') == b'\xbf\x05'

    if rfid.selftest():
       print("selftest passed!")
    else:
       print("selftest FAILED!")
except Exception as e:
    print_last()
finally:
    GPIO.cleanup()
