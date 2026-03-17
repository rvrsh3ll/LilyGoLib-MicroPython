from machine import I2C, Pin
import utime
from st25r3916 import create_nfc, nfc_probe, nfc_init, nfc_scan_once

i2c = I2C(0, scl=Pin(2), sda=Pin(3), freq=400000)

nfc = create_nfc(i2c)

print("probe =", nfc_probe(i2c, nfc))

nfc_init(i2c, nfc)

while True:
    info, err = nfc_scan_once(i2c, nfc)
    if info:
        print("Find card UID =", info["uid_hex"])
    else:
        # Turn on during debugging
        # print("No card:", err)
        pass
    utime.sleep_ms(300)