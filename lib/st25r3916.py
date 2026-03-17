# nfc.py
from machine import Pin, SoftSPI
import utime

# =========================
# Board pins
# =========================
PIN_SPI_SCK  = 35
PIN_SPI_MOSI = 34
PIN_SPI_MISO = 33

PIN_DISPLAY_CS = 38
PIN_LORA_CS    = 36
PIN_NFC_CS     = 39
PIN_NFC_IRQ    = 5

# =========================
# GPIO0~7 -> Port0 bit0~7
# GPIO8~15 -> Port1 bit0~7
# 所以 GPIO5 = Port0 bit5
# =========================
XL9555_ADDR       = 0x20
XL_REG_OUTPUT_P0  = 0x02
XL_REG_OUTPUT_P1  = 0x03
XL_REG_CONFIG_P0  = 0x06
XL_REG_CONFIG_P1  = 0x07
XL_GPIO5_BIT      = 5

_display_cs = None
_lora_cs = None
_nfc_cs = None

# =========================
# ST25R3916 SPI opcodes
# =========================
ST25R_SPI_READ    = 0x40
ST25R_SPI_FIFO_W  = 0x80
ST25R_SPI_FIFO_R  = 0x9F

# =========================
# ST25R3916 registers (Space A)
# =========================
REG_IO_CONF1      = 0x00
REG_IO_CONF2      = 0x01
REG_OP_CTRL       = 0x02
REG_MODE_DEF      = 0x03
REG_BIT_RATE      = 0x04
REG_ISO14443A     = 0x05

REG_MASK_MAIN_IRQ = 0x16
REG_MASK_TIMER_IRQ= 0x17
REG_MASK_ERR_IRQ  = 0x18
REG_MASK_PT_IRQ   = 0x19

REG_MAIN_IRQ      = 0x1A
REG_TIMER_IRQ     = 0x1B
REG_ERR_IRQ       = 0x1C
REG_PT_IRQ        = 0x1D

REG_FIFO_STATUS1  = 0x1E
REG_FIFO_STATUS2  = 0x1F
REG_COLLISION     = 0x20

REG_NUM_TX1       = 0x22
REG_NUM_TX2       = 0x23

# =========================
# Direct commands
# =========================
CMD_SET_DEFAULT   = 0xC1
CMD_STOP          = 0xC3
CMD_TX_CRC        = 0xC4
CMD_TX_NOCRC      = 0xC5
CMD_TX_REQA       = 0xC6
CMD_TX_WUPA       = 0xC7
CMD_RESET_RX_GAIN = 0xD5
CMD_CLEAR_FIFO    = 0xDB
CMD_START_NRT     = 0xE3
CMD_STOP_NRT      = 0xE8

# =========================
# IRQ bits
# =========================
IRQ_MASK_OSC      = 0x80
IRQ_MASK_WL       = 0x40
IRQ_MASK_RXS      = 0x20
IRQ_MASK_RXE      = 0x10
IRQ_MASK_TXE      = 0x08
IRQ_MASK_COL      = 0x04

IRQ_MASK_DCT      = 0x80
IRQ_MASK_NRE      = 0x40

IRQ_MASK_CRC_ERR  = 0x80
IRQ_MASK_PAR_ERR  = 0x40
IRQ_MASK_ERR2     = 0x20
IRQ_MASK_ERR1     = 0x10


def deselect_other_spi_devices():
    global _display_cs, _lora_cs, _nfc_cs

    if _display_cs is None:
        _display_cs = Pin(PIN_DISPLAY_CS, Pin.OUT, value=1)
    if _lora_cs is None:
        _lora_cs = Pin(PIN_LORA_CS, Pin.OUT, value=1)
    if _nfc_cs is None:
        _nfc_cs = Pin(PIN_NFC_CS, Pin.OUT, value=1)

    _display_cs.value(1)
    _lora_cs.value(1)
    _nfc_cs.value(1)


def xl_read8(i2c, reg):
    return i2c.readfrom_mem(XL9555_ADDR, reg, 1)[0]


def xl_write8(i2c, reg, value):
    i2c.writeto_mem(XL9555_ADDR, reg, bytes([value & 0xFF]))


def xl_gpio5_prepare(i2c):
    cfg = xl_read8(i2c, XL_REG_CONFIG_P0)
    cfg &= ~(1 << XL_GPIO5_BIT)
    xl_write8(i2c, XL_REG_CONFIG_P0, cfg)


def xl_gpio5_set(i2c, level):
    outv = xl_read8(i2c, XL_REG_OUTPUT_P0)
    if level:
        outv |= (1 << XL_GPIO5_BIT)
    else:
        outv &= ~(1 << XL_GPIO5_BIT)
    xl_write8(i2c, XL_REG_OUTPUT_P0, outv)


def nfc_board_prepare(i2c):
    xl_gpio5_prepare(i2c)
    xl_gpio5_set(i2c, 1)
    deselect_other_spi_devices()


def create_softspi():
    return SoftSPI(
        baudrate=1000000,
        polarity=0,
        phase=0,
        sck=Pin(PIN_SPI_SCK),
        mosi=Pin(PIN_SPI_MOSI),
        miso=Pin(PIN_SPI_MISO)
    )


def crc_a(data):
    crc = 0x6363
    for byte in data:
        byte ^= crc & 0x00FF
        byte ^= (byte << 4) & 0xFF
        crc = ((crc >> 8) ^
               (byte << 8) ^
               (byte << 3) ^
               (byte >> 4)) & 0xFFFF
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class ST25R3916:
    def __init__(self, spi, cs, irq=None):
        self.spi = spi
        self.cs = cs
        self.irq = irq
        self.cs.init(Pin.OUT, value=1)
        if self.irq is not None:
            self.irq.init(Pin.IN)

    def _xfer(self, tx):
        txb = bytearray(tx)
        rxb = bytearray(len(txb))
        deselect_other_spi_devices()
        self.cs(0)
        self.spi.write_readinto(txb, rxb)
        self.cs(1)
        return rxb

    def reg_read(self, reg):
        rx = self._xfer(bytes([ST25R_SPI_READ | (reg & 0x3F), 0x00]))
        return rx[1]

    def reg_write(self, reg, value):
        self._xfer(bytes([reg & 0x3F, value & 0xFF]))

    def fifo_write(self, data):
        tx = bytearray(1 + len(data))
        tx[0] = ST25R_SPI_FIFO_W
        tx[1:] = data
        self._xfer(tx)

    def fifo_read(self, length):
        tx = bytearray(1 + length)
        tx[0] = ST25R_SPI_FIFO_R
        rx = self._xfer(tx)
        return bytes(rx[1:])

    def cmd(self, command):
        self._xfer(bytes([command & 0xFF]))

    def read_irqs(self):
        main_  = self.reg_read(REG_MAIN_IRQ)
        timer_ = self.reg_read(REG_TIMER_IRQ)
        err_   = self.reg_read(REG_ERR_IRQ)
        pt_    = self.reg_read(REG_PT_IRQ)
        return main_, timer_, err_, pt_

    def fifo_len(self):
        s1 = self.reg_read(REG_FIFO_STATUS1)
        s2 = self.reg_read(REG_FIFO_STATUS2)
        return s1 | ((s2 & 0xC0) << 2)

    def clear_all(self):
        self.cmd(CMD_STOP)
        self.read_irqs()

    def wait_irq(self, main_mask=0, timer_mask=0, err_mask=0, timeout_ms=50):
        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < timeout_ms:
            m, t, e, p = self.read_irqs()
            if (m & main_mask) or (t & timer_mask) or (e & err_mask):
                return m, t, e, p
            utime.sleep_ms(1)
        return 0, 0, 0, 0

    def reset_chip(self):
        self.cmd(CMD_SET_DEFAULT)
        utime.sleep_ms(2)

    def power_up_and_wait_osc(self):
        self.reg_write(REG_OP_CTRL, 0x80)
        self.wait_irq(main_mask=IRQ_MASK_OSC, timeout_ms=20)

    def init_iso14443a_reader(self):
        self.reset_chip()
        self.power_up_and_wait_osc()

        self.reg_write(REG_MODE_DEF, 0x08)
        self.reg_write(REG_BIT_RATE, 0x00)
        self.reg_write(REG_ISO14443A, 0x00)

        self.reg_write(REG_MASK_MAIN_IRQ,  0x00)
        self.reg_write(REG_MASK_TIMER_IRQ, 0x00)
        self.reg_write(REG_MASK_ERR_IRQ,   0x00)
        self.reg_write(REG_MASK_PT_IRQ,    0x00)

        self.reg_write(REG_OP_CTRL, 0xC8)
        utime.sleep_ms(5)

        self.cmd(CMD_RESET_RX_GAIN)
        self.clear_all()

    def transceive_bytes(self, tx_data, with_crc=True, antcl=False, timeout_ms=50):
        self.clear_all()
        self.cmd(CMD_CLEAR_FIFO)

        n = len(tx_data)
        self.reg_write(REG_NUM_TX1, (n >> 5) & 0xFF)
        self.reg_write(REG_NUM_TX2, ((n & 0x1F) << 3) | 0x00)

        isoa = self.reg_read(REG_ISO14443A)
        if antcl:
            isoa |= 0x01
        else:
            isoa &= 0xFE
        self.reg_write(REG_ISO14443A, isoa)

        self.fifo_write(tx_data)

        if with_crc:
            self.cmd(CMD_TX_CRC)
        else:
            self.cmd(CMD_TX_NOCRC)

        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < timeout_ms:
            m, t, e, p = self.read_irqs()

            if e & (IRQ_MASK_ERR1 | IRQ_MASK_ERR2 | IRQ_MASK_PAR_ERR | IRQ_MASK_CRC_ERR):
                return None, ("ERR", m, t, e)

            if m & IRQ_MASK_COL:
                coll = self.reg_read(REG_COLLISION)
                return None, ("COLLISION", coll)

            if t & IRQ_MASK_NRE:
                return None, ("TIMEOUT",)

            if m & IRQ_MASK_RXE:
                ln = self.fifo_len()
                data = self.fifo_read(ln) if ln > 0 else b""
                return data, None

            utime.sleep_ms(1)

        return None, ("TIMEOUT",)

    def request_a(self, wakeup=False):
        self.clear_all()

        isoa = self.reg_read(REG_ISO14443A)
        isoa &= 0xFE
        self.reg_write(REG_ISO14443A, isoa)

        if wakeup:
            self.cmd(CMD_TX_WUPA)
        else:
            self.cmd(CMD_TX_REQA)

        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < 30:
            m, t, e, p = self.read_irqs()

            if e & (IRQ_MASK_ERR1 | IRQ_MASK_ERR2 | IRQ_MASK_PAR_ERR):
                return None, ("ERR", m, t, e)

            if m & IRQ_MASK_RXE:
                ln = self.fifo_len()
                data = self.fifo_read(ln) if ln > 0 else b""
                if len(data) >= 2:
                    return data[:2], None
                return data, None

            utime.sleep_ms(1)

        return None, ("TIMEOUT",)

    def anticoll_select_once(self, cascade_level):
        if cascade_level == 1:
            sel = 0x93
        elif cascade_level == 2:
            sel = 0x95
        else:
            sel = 0x97

        resp, err = self.transceive_bytes(bytes([sel, 0x20]), with_crc=False, antcl=True, timeout_ms=50)
        if err or not resp or len(resp) < 5:
            return None, None, err if err else ("NO_ATICOLL",)

        uid_cln = resp[:5]

        frame = bytes([sel, 0x70]) + uid_cln
        sak_resp, err = self.transceive_bytes(frame, with_crc=True, antcl=False, timeout_ms=50)
        if err or not sak_resp or len(sak_resp) < 1:
            return None, None, err if err else ("NO_SAK",)

        sak = sak_resp[0]

        if uid_cln[0] == 0x88:
            uid_part = uid_cln[1:4]
        else:
            uid_part = uid_cln[0:4]

        return uid_part, sak, (sak & 0x04) != 0

    def read_uid(self):
        atqa, err = self.request_a(False)
        if err:
            return None, ("REQA_FAIL", err)

        uid = bytearray()

        part, sak, err = self.anticoll_select_once(1)
        if err:
            return None, ("CL1_FAIL", err)
        uid.extend(part)

        if sak & 0x04:
            part, sak, err = self.anticoll_select_once(2)
            if err:
                return None, ("CL2_FAIL", err)
            uid.extend(part)

            if sak & 0x04:
                part, sak, err = self.anticoll_select_once(3)
                if err:
                    return None, ("CL3_FAIL", err)
                uid.extend(part)

        return {
            "atqa": atqa,
            "uid": bytes(uid),
            "uid_hex": "".join("{:02X}".format(x) for x in uid),
        }, None


def create_nfc(i2c):
    nfc_board_prepare(i2c)
    spi = create_softspi()
    nfc = ST25R3916(
        spi,
        Pin(PIN_NFC_CS, Pin.OUT, value=1),
        Pin(PIN_NFC_IRQ, Pin.IN)
    )
    return nfc


def nfc_probe(i2c, nfc):
    nfc_board_prepare(i2c)
    nfc.reset_chip()
    nfc.power_up_and_wait_osc()

    op = nfc.reg_read(REG_OP_CTRL)
    mode = nfc.reg_read(REG_MODE_DEF)
    br = nfc.reg_read(REG_BIT_RATE)
    isoa = nfc.reg_read(REG_ISO14443A)
    return op, mode, br, isoa


def nfc_init(i2c, nfc):
    nfc_board_prepare(i2c)
    nfc.init_iso14443a_reader()


def nfc_scan_once(i2c, nfc):
    nfc_board_prepare(i2c)
    info, err = nfc.read_uid()
    return info, err


def nfc_loop(i2c, nfc, interval_ms=300):
    last_uid = None
    while True:
        info, err = nfc_scan_once(i2c, nfc)
        if info:
            if info["uid_hex"] != last_uid:
                print("ATQA =", ["0x%02X" % b for b in info["atqa"]])
                print("UID  =", info["uid_hex"])
                print("-" * 30)
                last_uid = info["uid_hex"]
        utime.sleep_ms(interval_ms)