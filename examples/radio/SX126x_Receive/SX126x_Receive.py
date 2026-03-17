from machine import SPI, Pin, I2C
import sys
import lcd_bus
import st7796
import lvgl as lv
import ustruct as struct
import utime

i2c = I2C(0, scl=Pin(2), sda=Pin(3), freq=400000)
lv.init()
try:
    spi_bus = SPI.Bus(host=1, mosi=34, miso=33, sck=35)
    display_bus = lcd_bus.SPIBus(
        spi_bus=spi_bus,
        dc=37,
        cs=38,
        freq=80000000
    )

    display = st7796.ST7796(
        data_bus=display_bus,
        display_width=320,
        display_height=480,
        reset_state=st7796.STATE_LOW,
        color_space=lv.COLOR_FORMAT.RGB565,
        color_byte_order=st7796.BYTE_ORDER_RGB,
        rgb565_byte_swap=True
    )

    display.set_power(True)
    display.init()
    display.set_rotation(lv.DISPLAY_ROTATION._90)
except:
    pass

backlight_pin = Pin(42, Pin.OUT)
backlight_pin.value(1)

scrn = lv.screen_active()
scrn.set_style_bg_color(lv.color_hex(0x000000), 0)

label = lv.label(scrn)
label.set_text('NRF24L01 Receiving Client') 
label.center()

import task_handler
task_handler.TaskHandler(33)

from nrf24_pager_common import (
    radio_board_prepare,
    create_radio,
    radio_probe,
    radio_init_rx,
    radio_recv,
)

print("I2C scan:", i2c.scan())

radio_board_prepare(i2c)

nrf = create_radio()
status, aw, cfg = radio_probe(i2c, nrf)

print("STATUS = 0x%02X" % status)
print("SETUP_AW = 0x%02X" % aw)
print("CONFIG = 0x%02X" % cfg)

if aw != 0x03:
    raise OSError("nRF24L01+ Hardware not responding")

radio_init_rx(i2c, nrf)

print("NRF24 recv init OK")

while True:
    buf = radio_recv(nrf)

    if buf is not None:
        try:
            data, extra = struct.unpack("ii", buf)
            print("recv:", data, extra)
        except Exception as e:
            print("recv raw:", buf, "err:", e)

    utime.sleep_ms(10)
