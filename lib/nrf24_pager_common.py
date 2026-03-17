from machine import Pin, SoftSPI
import utime

# =========================
# Board pins
# =========================
PIN_SPI_SCK  = 35
PIN_SPI_MOSI = 34
PIN_SPI_MISO = 33

PIN_NRF_CE  = 43
PIN_NRF_CSN = 44

PIN_DISPLAY_CS = 38
PIN_LORA_CS    = 36
PIN_NFC_CS     = 39

# XL9555: GPIO9 = Port1 bit1
XL9555_ADDR = 0x20
XL_REG_OUTPUT_P1 = 0x03
XL_REG_CONFIG_P1 = 0x07
XL_GPIO9_BIT = 1

# =========================
# nRF24L01 registers/commands
# =========================
CONFIG      = 0x00
EN_AA       = 0x01
EN_RXADDR   = 0x02
SETUP_AW    = 0x03
SETUP_RETR  = 0x04
RF_CH       = 0x05
RF_SETUP    = 0x06
STATUS      = 0x07
RX_ADDR_P0  = 0x0A
TX_ADDR     = 0x10
RX_PW_P0    = 0x11
FIFO_STATUS = 0x17
DYNPD       = 0x1C

W_REGISTER   = 0x20
R_RX_PAYLOAD = 0x61
W_TX_PAYLOAD = 0xA0
FLUSH_TX     = 0xE1
FLUSH_RX     = 0xE2
NOP          = 0xFF

RX_DR    = 0x40
TX_DS    = 0x20
MAX_RT   = 0x10
RX_EMPTY = 0x01

PWR_UP  = 0x02
PRIM_RX = 0x01
EN_CRC  = 0x08
CRCO    = 0x04

ADDR = b"\xf0\xf0\xf0\xf0\xe1"
PAYLOAD_SIZE = 8

_display_cs = None
_lora_cs = None
_nfc_cs = None


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


def xl_gpio9_prepare(i2c):
    cfg = xl_read8(i2c, XL_REG_CONFIG_P1)
    cfg &= ~(1 << XL_GPIO9_BIT)   # GPIO9 -> output
    xl_write8(i2c, XL_REG_CONFIG_P1, cfg)


def xl_set_tx(i2c):
    outv = xl_read8(i2c, XL_REG_OUTPUT_P1)
    outv |= (1 << XL_GPIO9_BIT)   # HIGH -> TX
    xl_write8(i2c, XL_REG_OUTPUT_P1, outv)


def xl_set_rx(i2c):
    outv = xl_read8(i2c, XL_REG_OUTPUT_P1)
    outv &= ~(1 << XL_GPIO9_BIT)  # LOW -> RX
    xl_write8(i2c, XL_REG_OUTPUT_P1, outv)


def radio_board_prepare(i2c):
    xl_gpio9_prepare(i2c)
    deselect_other_spi_devices()


def create_softspi():
    return SoftSPI(
        baudrate=100000,
        polarity=0,
        phase=0,
        sck=Pin(PIN_SPI_SCK),
        mosi=Pin(PIN_SPI_MOSI),
        miso=Pin(PIN_SPI_MISO)
    )


class NRF24Mini:
    def __init__(self, spi, csn, ce):
        self.spi = spi
        self.csn = csn
        self.ce = ce
        self.csn.init(Pin.OUT, value=1)
        self.ce.init(Pin.OUT, value=0)

    def _xfer(self, tx):
        txb = bytearray(tx)
        rxb = bytearray(len(txb))
        deselect_other_spi_devices()
        self.csn(0)
        self.spi.write_readinto(txb, rxb)
        self.csn(1)
        return rxb

    def cmd(self, c):
        return self._xfer(bytes([c]))[0]

    def reg_read(self, reg):
        rx = self._xfer(bytes([reg, NOP]))
        return rx[1]

    def reg_write(self, reg, value):
        rx = self._xfer(bytes([W_REGISTER | reg, value]))
        return rx[0]

    def reg_write_bytes(self, reg, data):
        tx = bytearray(1 + len(data))
        tx[0] = W_REGISTER | reg
        tx[1:] = data
        rx = self._xfer(tx)
        return rx[0]

    def write_payload(self, payload):
        tx = bytearray(1 + len(payload))
        tx[0] = W_TX_PAYLOAD
        tx[1:] = payload
        rx = self._xfer(tx)
        return rx[0]

    def read_payload(self, length):
        tx = bytearray(length + 1)
        tx[0] = R_RX_PAYLOAD
        rx = self._xfer(tx)
        return bytes(rx[1:])

    def flush_tx(self):
        self.cmd(FLUSH_TX)

    def flush_rx(self):
        self.cmd(FLUSH_RX)

    def clear_irq(self):
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

    def power_up_tx(self):
        self.ce(0)
        self.reg_write(CONFIG, EN_CRC | CRCO | PWR_UP)
        utime.sleep_ms(2)

    def power_up_rx(self):
        self.ce(0)
        self.reg_write(CONFIG, EN_CRC | CRCO | PWR_UP | PRIM_RX)
        utime.sleep_ms(2)
        self.ce(1)
        utime.sleep_us(150)

    def pulse_ce(self):
        self.ce(1)
        utime.sleep_us(20)
        self.ce(0)

    def wait_tx_done(self, timeout_ms=200):
        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < timeout_ms:
            st = self.reg_read(STATUS)
            if st & (TX_DS | MAX_RT):
                return st
        return None

    def any(self):
        return not (self.reg_read(FIFO_STATUS) & RX_EMPTY)


def create_radio():
    spi = create_softspi()
    nrf = NRF24Mini(
        spi,
        Pin(PIN_NRF_CSN, Pin.OUT, value=1),
        Pin(PIN_NRF_CE, Pin.OUT, value=0)
    )
    return nrf


def radio_probe(i2c, nrf):
    # 已验证：探测时先切到 TX
    xl_set_tx(i2c)
    utime.sleep_ms(1)

    status = nrf.cmd(NOP)
    nrf.reg_write(SETUP_AW, 0x03)
    aw = nrf.reg_read(SETUP_AW)
    cfg = nrf.reg_read(CONFIG)
    return status, aw, cfg


def radio_common_config(nrf):
    nrf.reg_write(SETUP_AW, 0x03)
    nrf.reg_write(EN_AA, 0x00)         # no ACK
    nrf.reg_write(EN_RXADDR, 0x01)     # pipe0
    nrf.reg_write(SETUP_RETR, 0x00)
    nrf.reg_write(RF_CH, 46)
    nrf.reg_write(RF_SETUP, 0x26)      # 250kbps, 0dBm
    nrf.reg_write(DYNPD, 0x00)
    nrf.reg_write_bytes(TX_ADDR, ADDR)
    nrf.reg_write_bytes(RX_ADDR_P0, ADDR)
    nrf.reg_write(RX_PW_P0, PAYLOAD_SIZE)
    nrf.flush_tx()
    nrf.flush_rx()
    nrf.clear_irq()


def radio_init_tx(i2c, nrf):
    xl_set_tx(i2c)
    utime.sleep_ms(1)
    radio_common_config(nrf)
    nrf.power_up_tx()


def radio_init_rx(i2c, nrf):
    # 先切 TX 配好寄存器，再切 RX
    xl_set_tx(i2c)
    utime.sleep_ms(1)
    radio_common_config(nrf)

    xl_set_rx(i2c)
    utime.sleep_ms(1)
    nrf.power_up_rx()


def radio_send(i2c, nrf, payload):
    xl_set_tx(i2c)
    utime.sleep_ms(1)

    nrf.flush_tx()
    nrf.clear_irq()
    nrf.write_payload(payload)
    nrf.pulse_ce()

    return nrf.wait_tx_done(200)


def radio_recv(nrf):
    if nrf.any():
        buf = nrf.read_payload(PAYLOAD_SIZE)
        nrf.clear_irq()
        return buf
    return None