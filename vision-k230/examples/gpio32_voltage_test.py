"""Slow GPIO32 voltage test for a multimeter.

Measure K230 IO32 relative to K230 GND.  The voltage should alternate between
about 0 V and 3.3 V every two seconds.
"""

import time
from machine import Pin


pin = Pin(32, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
level = 0

print("[GPIO32] voltage test: IO32 toggles every 2 seconds")

while True:
    pin.value(level)
    print("[GPIO32] level={}".format(level))
    level ^= 1
    time.sleep_ms(2000)
