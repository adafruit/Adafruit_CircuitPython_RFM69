# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries # pylint: disable=too-many-lines
# SPDX-License-Identifier: MIT

"""
`adafruit_rfm69`
====================================================

CircuitPython RFM69 packet radio module. This supports basic RadioHead-compatible sending and
receiving of packets with RFM69 series radios (433/915Mhz).

.. warning:: This is NOT for LoRa radios!

.. note:: For reliable transmissions, use aio_send_with_ack and aio_recv_with_ack.
    With ACKs enabled, observed effective throughput of 14-28kbps with bitrates of 38-250kbps!
    Tested on ESP32-S3

* Author(s): Tony DiCola, Jerry Needell, Matthew Tai

Implementation Notes
--------------------

**Hardware:**

* Adafruit `RFM69HCW Transceiver Radio Breakout - 868 or 915 MHz - RadioFruit
  <https://www.adafruit.com/product/3070>`_ (Product ID: 3070)

* Adafruit `RFM69HCW Transceiver Radio Breakout - 433 MHz - RadioFruit
  <https://www.adafruit.com/product/3071>`_ (Product ID: 3071)

* Adafruit `Feather M0 RFM69HCW Packet Radio - 868 or 915 MHz - RadioFruit
  <https://www.adafruit.com/product/3176>`_ (Product ID: 3176)

* Adafruit `Feather M0 RFM69HCW Packet Radio - 433 MHz - RadioFruit
  <https://www.adafruit.com/product/3177>`_ (Product ID: 3177)

* Adafruit `Radio FeatherWing - RFM69HCW 900MHz - RadioFruit
  <https://www.adafruit.com/product/3229>`_ (Product ID: 3229)

* Adafruit `Radio FeatherWing - RFM69HCW 433MHz - RadioFruit
  <https://www.adafruit.com/product/3230>`_ (Product ID: 3230)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's asyncio library: https://github.com/adafruit/Adafruit_CircuitPython_asyncio

"""
import random
import time
import asyncio
import countio
import adafruit_bus_device.spi_device as spidev

from adafruit_ticks import ticks_ms, ticks_add, ticks_less
from micropython import const
from digitalio import DigitalInOut

# Try/Except everything used for typing
try:
    from typing import Optional, Type
    from circuitpython_typing import WriteableBuffer, ReadableBuffer
    from busio import SPI
    from microcontroller import Pin
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_RFM69.git"


# Internal constants:
_REG_FIFO = const(0x00)
_REG_OP_MODE = const(0x01)
_REG_DATA_MOD = const(0x02)
_REG_BITRATE_MSB = const(0x03)
_REG_BITRATE_LSB = const(0x04)
_REG_FDEV_MSB = const(0x05)
_REG_FDEV_LSB = const(0x06)
_REG_FRF_MSB = const(0x07)
_REG_FRF_MID = const(0x08)
_REG_FRF_LSB = const(0x09)
_REG_VERSION = const(0x10)
_REG_PA_LEVEL = const(0x11)
_REG_RX_BW = const(0x19)
_REG_AFC_BW = const(0x1A)
_REG_RSSI_VALUE = const(0x24)
_REG_DIO_MAPPING1 = const(0x25)
_REG_IRQ_FLAGS1 = const(0x27)
_REG_IRQ_FLAGS2 = const(0x28)
_REG_PREAMBLE_MSB = const(0x2C)
_REG_PREAMBLE_LSB = const(0x2D)
_REG_SYNC_CONFIG = const(0x2E)
_REG_SYNC_VALUE1 = const(0x2F)
_REG_PACKET_CONFIG1 = const(0x37)
_REG_FIFO_THRESH = const(0x3C)
_REG_PACKET_CONFIG2 = const(0x3D)
_REG_AES_KEY1 = const(0x3E)
_REG_TEMP1 = const(0x4E)
_REG_TEMP2 = const(0x4F)
_REG_TEST_PA1 = const(0x5A)
_REG_TEST_PA2 = const(0x5C)
_REG_TEST_DAGC = const(0x6F)

_TEST_PA1_NORMAL = const(0x55)
_TEST_PA1_BOOST = const(0x5D)
_TEST_PA2_NORMAL = const(0x70)
_TEST_PA2_BOOST = const(0x7C)

_REG_DIOMAPPING1_RX_PAYLOAD_READY = const(0b01)
_REG_DIOMAPPING1_TX_PACKET_SENT = const(0b00)
_REG_IRQ_FLAGS1_MODE_READY = const(0x80)

_REG_TEMP1_TEMP_MEAS_START = const(0x80)
_REG_TEMP1_TEMP_MEAS_RUNNING = const(0x40)

# The crystal oscillator frequency and frequency synthesizer step size.
# See the datasheet for details of this calculation.
_FXOSC = 32000000.0
_FSTEP = _FXOSC / 524288

# RadioHead specific compatibility constants.
RH_BROADCAST_ADDRESS = const(0xFF)

# RegDataModul - Defaults for using RadioHead GFSK modem configs (see RadioHead)
# 0b_00_____ = DATAMODE_PACKET
# 0b___00___ = MODULATIONTYPE_FSK
# 0b______01 = MODULATIONSHAPING_FSK_BT1_0
_RH_DATAMODUL_GFSK = const(0b00000001)

# RegDataModul - Defaults for using RadioHead GFSK modem configs (see RadioHead)
# 0b1_______ = PACKETFORMAT_VARIABLE
# 0b_10_____ = DCFREE_WHITENING
# 0b___1____ = CRC_ON
# 0b____0___ = CRC_AUTO_CLEAR_OFF
# 0b_____00_ = ADDRESSFILTERING_NONE
_RH_PACKETCONFIG1_WHITE = const(0b11010000)

#  bitrate: (frequency_deviation, RxBW,   AfcBW)
# GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
# AFC BW == RX BW == 2 x bit rate
RH_BITRATE_2000 = const(2000)
RH_BITRATE_55555 = const(55555)
RH_BITRATE_2400 = const(2400)
RH_BITRATE_4800 = const(4800)
RH_BITRATE_9600 = const(9600)
RH_BITRATE_19200 = const(19200)
RH_BITRATE_38400 = const(38400)
RH_BITRATE_57600 = const(57600)
RH_BITRATE_125000 = const(125000)
RH_BITRATE_250000 = const(250000)

_RH_BITRATE_TO_CONFIG_MAP = {
    RH_BITRATE_2000: (5000, 0xF4, 0xF5),  # GFSK_Rb2Fd5
    RH_BITRATE_55555: (50000, 0x42, 0x42),  # GFSK_Rb55555Fd50
    RH_BITRATE_2400: (4800, 0xF4, 0xF4),  # GFSK_Rb2_4Fd4_8
    RH_BITRATE_4800: (9600, 0xF4, 0xF4),  # GFSK_Rb4_8Fd9_6
    RH_BITRATE_9600: (19200, 0xF4, 0xF4),  # GFSK_Rb9_6Fd19_2
    RH_BITRATE_19200: (38400, 0xF3, 0xF3),  # GFSK_Rb19_2Fd38_4
    RH_BITRATE_38400: (76800, 0xF2, 0xF2),  # GFSK_Rb38_4Fd76_8
    RH_BITRATE_57600: (120000, 0xE2, 0xE2),  # GFSK_Rb57_6Fd120
    RH_BITRATE_125000: (125000, 0xE1, 0xE1),  # GFSK_Rb125Fd125
    RH_BITRATE_250000: (250000, 0xE0, 0xE0),  # GFSK_Rb250Fd250
}

# User facing constants:
SLEEP_MODE = const(0b000)
STANDBY_MODE = const(0b001)
FS_MODE = const(0b010)
TX_MODE = const(0b011)
RX_MODE = const(0b100)


class RHPacket:  # pylint: disable=too-few-public-methods
    """
    RadioHead Packet as returned by aio_recv
    """

    def __init__(self, payload: ReadableBuffer, rssi: int, time_: float):
        self.dest = payload[0]
        self.src = payload[1]
        self.sequence_number = payload[2]
        self.flags = payload[3]

        self.data = payload[4:]
        self.rssi = rssi
        self.time = time_  # Time in seconds


class _Register:  # pylint: disable=too-few-public-methods
    address = None
    value = None

    def __init__(self, value):
        self.value = value


class _Bits:  # pylint: disable=too-few-public-methods
    def __init__(self, *, offset: int = 0, bits: int = 1) -> None:
        assert 0 <= offset <= 7
        assert 1 <= bits <= 8
        assert (offset + bits) <= 8
        self._mask = 0
        for _ in range(bits):
            self._mask <<= 1
            self._mask |= 1
        self._mask <<= offset
        self._offset = offset

    def __get__(self, obj: Optional["_Register"], objtype: Type["_Register"]):
        return (obj.value & self._mask) >> self._offset

    def __set__(self, obj: Optional["_Register"], val: int) -> None:
        reg_value = obj.value
        reg_value &= ~self._mask
        reg_value |= (val & 0xFF) << self._offset
        obj.value = reg_value


class _RegPaLevel(_Register):  # pylint: disable=too-few-public-methods
    address = _REG_PA_LEVEL
    pa_0_on = _Bits(offset=7)
    pa_1_on = _Bits(offset=6)
    pa_2_on = _Bits(offset=5)
    output_power = _Bits(offset=0, bits=5)


class _RegDioMapping1(_Register):  # pylint: disable=too-few-public-methods
    address = _REG_DIO_MAPPING1
    dio_0_mapping = _Bits(offset=6, bits=2)
    dio_1_mapping = _Bits(offset=4, bits=2)
    dio_2_mapping = _Bits(offset=2, bits=2)
    dio_3_mapping = _Bits(offset=0, bits=2)


class _RegIrqFlags2(_Register):  # pylint: disable=too-few-public-methods
    address = _REG_IRQ_FLAGS2
    fifo_full = _Bits(offset=7)
    fifo_not_empty = _Bits(offset=6)
    fifo_level = _Bits(offset=5)
    fifo_overrun = _Bits(offset=4)
    packet_sent = _Bits(offset=3)
    payload_ready = _Bits(offset=2)
    crc_ok = _Bits(offset=1)


class _RegSyncConfig(_Register):  # pylint: disable=too-few-public-methods
    address = _REG_SYNC_CONFIG
    sync_on = _Bits(offset=7)
    sync_size = _Bits(offset=3, bits=3)


class _RegPacketConfig2(_Register):  # pylint: disable=too-few-public-methods
    address = _REG_PACKET_CONFIG2
    inter_packet_rx_delay = _Bits(offset=4, bits=3)
    restart_rx = _Bits(offset=2)
    auto_rx_restart_on = _Bits(offset=1)
    aes_on = _Bits(offset=0)


_RH_DEFAULT_SYNC_WORD = b"\x2D\xD4"
_RH_DEFAULT_PREAMBLE_LENGTH = const(4)
_RH_DEFAULT_BITRATE = RH_BITRATE_250000
_RH_DEFAULT_TX_POWER = const(13)
_RH_MIN_FIFO_LENGTH = const(5)

_RH_RELIABLE_DGRAM_TIMEOUT = 0.200  # in seconds
_RH_RELIABLE_DGRAM_RETRIES = const(3)

# The acknowledgement bit in the FLAGS
# The top 4 bits of the flags are reserved for RadioHead. The lower 4 bits are reserved
# for application layer use.
_RH_RELIABLE_DGRAM_PACKET_FLAGS_ACK = const(0x80)
_RH_RELIABLE_DGRAM_PACKET_FLAGS_RETRY = const(0x40)
_RH_ENABLE_EXPLICIT_RETRY_DEDUP = False


def aio_to_blocking(in_fxn):
    """Convenience decorator to turn aio methods into blocking calls"""

    def wrapper(self, *args, **kwargs):
        return asyncio.run(in_fxn(self, *args, **kwargs))

    return wrapper


class RFM69:  # pylint: disable=too-many-instance-attributes,too-many-public-methods
    """Interface to a RFM69 series packet radio.  Allows simple sending and
    receiving of wireless data at supported frequencies of the radio
    (433/915mhz).

    :param spidev.SPIDevice spi: The SPIDevice build using RFM69.spi_device(spi, cs, baudrate)
    :param ~microcontroller.Pin irq: A Pin object connected to the chip's DIO0/board's IRQ
        line.
    :param ~microcontroller.Pin reset: A Pin object connected to the chip's RST/reset
        line.
    :param int frequency: The center frequency to configure for radio transmission and reception.
        Must be a frequency supported by your hardware (i.e. either 433 or 915mhz).
    :param int bitrate: Set radio's bitrate to RH_BITRATE_XXX.  Updates Freq Deviation, Afc Bw,
        and Rx Bw to match RadioHead's GFSK settings.  Set to lower bitrates to increase range.
        Default is 250000.
    :param bool high_power: Indicate if the chip is a high power variant that supports boosted
        transmission power.  The default is True as it supports the common RFM69HCW modules sold by
        Adafruit.

    This library is a Python port of the RadioHead Arduino library.
    For compatibility, this library sets the radio modulation to match RadioHead's default of GFSK
    encoding with configurable bitrates from 2-250kbit/s.  Advanced RadioHead features like
    address/node specific packets or "reliable datagram" delivery are supported but are subject to
    missed packets.  In "reliable datagram" delivery, the sender can see if packets are not ACKed.
    """

    # Global buffer for SPI commands.
    _BUFFER = bytearray(4)

    def __init__(  # pylint: disable=too-many-arguments
        self,
        spi_device: spidev.SPIDevice,
        irq: Pin,
        reset: Pin,
        frequency: int,
        bitrate: int = RH_BITRATE_250000,
        *,
        high_power: bool = True,
        supports_interrupts: bool = False,
    ) -> None:
        ##########
        # Publically accessible members
        ##########
        self.address = RH_BROADCAST_ADDRESS
        self.tx_success = 0
        self.rx_success = 0

        self.rx_packet = None
        self.send_retransmissions = 0
        self.set_op_mode_timeout = 1.000  # in seconds

        # Supports Interrupts
        #     CircuitPython: False
        #     MicroPython:   True* (cannot use self.handle_interrupt as-is
        #                               due to ISR accessing SPI)
        #     RPi:           True  (not tested)
        #
        # NOTE: When setting supports_interrupts = True, you MUST setup your own
        #          ISRs and callbacks... self.handle_interrupt *may* be used
        #          on RaspberryPi but this is untested
        self.platform_supports_interrupts = supports_interrupts
        self.rx_polling_interval = 0.001  # in seconds
        self.tx_polling_interval = 0.001  # in seconds

        ##########
        # Private members used in RadioHead
        ##########
        self._promiscuous = False
        self._last_sequence_number = 0
        self._seen_ids = bytearray(256)

        ##########
        # Private members used in RFM69
        ##########
        self._is_high_power = high_power
        self._tx_power = _RH_DEFAULT_TX_POWER
        self._mode = None

        self._device = spi_device

        # Check the version of the chip.
        self._validate_chip_version()

        # Setup reset as a digital output that's low.
        self._irq_counter = countio.Counter(irq, edge=countio.Edge.RISE)

        self._reset_digital_pin = DigitalInOut(reset)
        self._reset_digital_pin.switch_to_output(value=False)
        self._reset()  # Reset the chip.

        self.set_mode_standby()  # Enter standby state.

        ##########
        # Configure the Radio in a similar way to the RadioHead RFM69 library.
        # Assumes GFSK, excluded OOK and FSK for simplicity
        ##########
        # Set FIFO TX condition to not empty and the default FIFO threshold to 15.
        self._spi_write_u8(_REG_FIFO_THRESH, 0b10001111)
        # Configure low beta off.
        self._spi_write_u8(_REG_TEST_DAGC, 0x30)

        # Set the syncronization word.
        self.set_sync_word(_RH_DEFAULT_SYNC_WORD)
        self.set_modem_config(bitrate)
        self.set_preamble_length(
            _RH_DEFAULT_PREAMBLE_LENGTH
        )  # Set the preamble length.
        self.set_frequency_mhz(frequency)  # Set frequency.
        self.set_encryption_key(None)  # Set encryption key.

        # Set transmit power to 13 dBm, a safe value any module supports.
        self.set_tx_power(_RH_DEFAULT_TX_POWER)

    def __del__(self):
        self._irq_counter.deinit()

    #####
    # Begin functions specific to CircuitPython
    #####
    @classmethod
    def spi_device(
        cls, spi: SPI, cs: Pin, baudrate: int = 2000000  # pylint: disable=invalid-name
    ) -> spidev.SPIDevice:
        """Builds a SPI Device with SPI baudrate defauled to 2mhz
        SPI Device is a required argument for RFM69()

        Device supports SPI mode 0 (polarity & phase = 0) up to a max of 10mhz.
        Ensure SCK, MOSI, and MISO are connected.
        """
        return spidev.SPIDevice(
            spi, DigitalInOut(cs), baudrate=baudrate, polarity=0, phase=0
        )

    def _spi_read_into(
        self, address: int, buf: WriteableBuffer, length: Optional[int] = None
    ) -> None:
        # Read a number of bytes from the specified address into the provided
        # buffer.  If length is not specified (the default) the entire buffer
        # will be filled.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = address & 0x7F  # Strip out top bit to set 0
            # value (read).
            device.write(self._BUFFER, end=1)
            device.readinto(buf, end=length)

    def _spi_read_u8(self, address: int) -> int:
        # Read a single byte from the provided address and return it.
        self._spi_read_into(address, self._BUFFER, length=1)
        return self._BUFFER[0]

    def _spi_read_register(self, register_cls: _Register) -> _Register:
        value = self._spi_read_u8(register_cls.address)
        return register_cls(value)

    def _spi_write_from(
        self, address: int, buf: ReadableBuffer, length: Optional[int] = None
    ) -> None:
        # Write a number of bytes to the provided address and taken from the
        # provided buffer.  If no length is specified (the default) the entire
        # buffer is written.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            device.write(self._BUFFER, end=1)
            device.write(buf, end=length)  # send data

    def _spi_write_u8(self, address: int, val: int) -> None:
        # Write a byte register to the chip.  Specify the 7-bit address and the
        # 8-bit value to write to that address.
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            self._BUFFER[1] = val & 0xFF
            device.write(self._BUFFER, end=2)

    def _spi_write_register(self, register_obj: _Register) -> None:
        address = register_obj.address
        value = register_obj.value
        self._spi_write_u8(address, value)

    #####
    # Begin RadioHead ported functions from RHGenericDriver
    #     Public : Python ports of RadioHead functions
    #     Private: Python helper/convenice functions that do NOT exist in RadioHead
    #####
    async def wait_for_packet_sent(self):  # pylint: disable=missing-function-docstring
        counter = self._irq_counter
        send_polling = self.tx_polling_interval

        # Wait for the TX mode to change, should happen in aio_send (no timeout parameter)
        is_tx_mode = bool(self._mode == TX_MODE)
        is_packet_sent = bool(counter.count)
        while is_tx_mode and not is_packet_sent:
            await asyncio.sleep(send_polling)
            is_tx_mode = bool(self._mode == TX_MODE)
            is_packet_sent = bool(counter.count)

        return is_packet_sent

    async def wait_for_payload_ready(
        self, timeout=1.0
    ):  # pylint: disable=missing-function-docstring
        end_time = ticks_add(ticks_ms(), int(timeout * 1000))

        counter = self._irq_counter
        recv_polling = self.rx_polling_interval

        is_rx_mode = bool(self._mode == RX_MODE)
        is_payload_ready = bool(counter.count)
        is_time_remaining = ticks_less(ticks_ms(), end_time)
        while is_rx_mode and not is_payload_ready and is_time_remaining:
            await asyncio.sleep(recv_polling)
            is_rx_mode = bool(self._mode == RX_MODE)
            is_payload_ready = bool(counter.count)
            is_time_remaining = ticks_less(ticks_ms(), end_time)

        return is_payload_ready

    #####
    # Begin RadioHead ported functions from RH_RF69
    #     Public : Python ports of RadioHead functions
    #     Private: Python helper/convenice functions that do NOT exist in RadioHead
    #####
    def _validate_chip_version(self):
        version = self._spi_read_u8(_REG_VERSION)
        if version != 0x24:
            raise RuntimeError("Invalid RFM69 version, check wiring!")

    def handle_interrupt(
        self, is_packet_sent=None, is_payload_ready=None
    ):  # pylint: disable=missing-function-docstring
        """Interrupt handler when IRQ goes high.

        Checks for PacketSent or PayloadReady events
        """
        if is_packet_sent is None and is_payload_ready is None:
            reg_irq_flags2 = self._spi_read_register(_RegIrqFlags2)
            is_packet_sent = bool(reg_irq_flags2.packet_sent)
            is_payload_ready = bool(reg_irq_flags2.payload_ready)

        if self._mode == TX_MODE and is_packet_sent:
            self.set_mode_standby()
            self.tx_success += 1
        elif self._mode == RX_MODE and is_payload_ready:
            # Log the RSSI and reception time in uController time
            recv_rssi = self.get_rssi()
            recv_time = ticks_ms() / 1000.0

            # Stop RX so we can read 1 packet and not worry about receiving
            #    yet another packet
            self.set_mode_standby()

            self.read_fifo(recv_rssi, recv_time)

    def read_fifo(self, recv_rssi: float, recv_time: float) -> bytearray:
        """
        Internally used function to read the RFM69 FIFO via SPI
        Applies RadioHead address filtering
        """
        # Read the length of the FIFO.
        fifo_length = self._spi_read_u8(_REG_FIFO)

        # Handle if the received packet is too small to include the 4 byte
        # RadioHead header and at least one byte of data --reject this packet and ignore it.
        if (
            fifo_length < _RH_MIN_FIFO_LENGTH
        ):  # read and clear the FIFO if anything in it
            return

        fifo_data = bytearray(fifo_length)
        self._spi_read_into(_REG_FIFO, fifo_data, fifo_length)

        # Filter packets not explicitly destined for me
        packet_dest = fifo_data[0]
        is_for_me = bool(packet_dest == self.address) or bool(
            packet_dest == RH_BROADCAST_ADDRESS
        )
        if self._promiscuous or is_for_me:
            self.rx_packet = RHPacket(fifo_data, recv_rssi, recv_time)
            self.rx_success += 1

    def get_temperature(self) -> float:
        """The internal temperature of the chip in degrees Celsius. Be warned this is not
        calibrated or very accurate.

        .. warning:: Reading this will STOP any receiving/sending that might be happening!
        """
        # Start a measurement then poll the measurement finished bit.
        self._spi_write_u8(_REG_TEMP1, _REG_TEMP1_TEMP_MEAS_START)

        temp_running = True
        while temp_running:
            temp_value = self._spi_read_u8(_REG_TEMP1)
            temp_running = bool(temp_value & _REG_TEMP1_TEMP_MEAS_RUNNING)

        # Grab the temperature value and convert it to Celsius.
        # This uses the same observed value formula from the Radiohead library.
        temp = self._spi_read_u8(_REG_TEMP2)
        return 166.0 - temp

    def get_frequency_mhz(self) -> float:
        """The frequency of the radio in Megahertz. (i.e. 433 vs. 915 mhz)!"""
        # FRF register is computed from the frequency following the datasheet.
        # See section 6.2 and FRF register description.
        # Read bytes of FRF register and assemble into a 24-bit unsigned value.
        msb = self._spi_read_u8(_REG_FRF_MSB)
        mid = self._spi_read_u8(_REG_FRF_MID)
        lsb = self._spi_read_u8(_REG_FRF_LSB)
        frf = ((msb << 16) | (mid << 8) | lsb) & 0xFFFFFF
        frequency = (frf * _FSTEP) / 1000000.0
        return frequency

    def set_frequency_mhz(self, val: float) -> None:
        """The frequency of the radio in Megahertz. Only the allowed values for your radio must be
        specified (i.e. 433 vs. 915 mhz)!
        """
        assert 290 <= val <= 1020
        # Calculate FRF register 24-bit value using section 6.2 of the datasheet.
        frf = int((val * 1000000.0) / _FSTEP) & 0xFFFFFF
        # Extract byte values and update registers.
        msb = frf >> 16
        mid = (frf >> 8) & 0xFF
        lsb = frf & 0xFF
        self._spi_write_u8(_REG_FRF_MSB, msb)
        self._spi_write_u8(_REG_FRF_MID, mid)
        self._spi_write_u8(_REG_FRF_LSB, lsb)

    def get_rssi(self) -> float:
        """The received strength indicator (in dBm).
        May be inaccuate if not read immediatey. last_rssi contains the value read immediately
        receipt of the last packet.
        """
        # Read RSSI register and convert to value using formula in datasheet.
        return -self._spi_read_u8(_REG_RSSI_VALUE) / 2.0

    def get_op_mode(self) -> int:
        """The operation mode value."""
        op_mode = self._spi_read_u8(_REG_OP_MODE)
        return (op_mode >> 2) & 0b111

    def set_op_mode(self, val: int) -> None:
        """The operation mode value.  Unless you're manually controlling the chip you shouldn't
        change the operation_mode with this property as other side-effects are required for
        changing logical modes--use :py:func:`set_mode_standby`, :py:func:`set_mode_sleep`,
        :py:func:`set_mode_tx`, :py:func:`set_mode_rx` insteads.
        """
        assert 0 <= val <= 4

        # Set the mode bits inside the operation mode register.
        op_mode = self._spi_read_u8(_REG_OP_MODE)
        op_mode &= 0b11100011
        op_mode |= val << 2
        self._spi_write_u8(_REG_OP_MODE, op_mode)

        # Locally cache the mode
        self._mode = val

        # Blocking call until ModeReady
        # TODO: Monitor IRQs via DIO5 pin if available...
        #       For now fallback to polling the register
        end_time = ticks_add(ticks_ms(), int(self.set_op_mode_timeout * 1000))
        is_time_remaining = True
        while is_time_remaining:
            irq_flags1 = self._spi_read_u8(_REG_IRQ_FLAGS1)
            # check if ModeReady
            if bool(irq_flags1 & _REG_IRQ_FLAGS1_MODE_READY):
                return

            is_time_remaining = ticks_less(ticks_ms(), end_time)

        raise TimeoutError("Operation Mode failed to set.")

    def _set_boost(self, pa1_setting: int, pa2_setting: int) -> None:
        """Set preamp boost if needed."""
        if self._tx_power >= 18:
            self._spi_write_u8(_REG_TEST_PA1, pa1_setting)
            self._spi_write_u8(_REG_TEST_PA2, pa2_setting)

    def set_mode_standby(self) -> bool:
        """Enter standby mode (switching off high power amplifiers if necessary)."""
        # Like RadioHead library, turn off high power boost if enabled.
        if self._mode == STANDBY_MODE:
            return False

        self._set_boost(_TEST_PA1_NORMAL, _TEST_PA2_NORMAL)
        self.set_op_mode(STANDBY_MODE)
        return True

    def set_mode_sleep(self) -> None:
        """Enter sleep mode."""
        if self._mode == SLEEP_MODE:
            return False

        self.set_op_mode(SLEEP_MODE)
        return True

    def set_mode_rx(self) -> None:
        """Listen for packets to be received by the chip.  Use :py:func:`receive` to listen, wait
        and retrieve packets as they're available.
        """
        if self._mode == RX_MODE:
            return False

        # Like RadioHead library, turn off high power boost if enabled.
        self._set_boost(_TEST_PA1_NORMAL, _TEST_PA2_NORMAL)

        # Enable payload ready interrupt for D0 line.
        reg_dio_mapping1 = self._spi_read_register(_RegDioMapping1)
        reg_dio_mapping1.dio_0_mapping = _REG_DIOMAPPING1_RX_PAYLOAD_READY
        self._spi_write_register(reg_dio_mapping1)

        # Reset the IRQ counter
        self._irq_counter.reset()

        # Enter RX mode (will clear FIFO!).
        self.set_op_mode(RX_MODE)
        return True

    def set_mode_tx(self) -> None:
        """Transmit a packet which is queued in the FIFO.  This is a low level function for
        entering transmit mode and more.  For generating and transmitting a packet of data use
        :py:func:`send` instead.
        """
        if self._mode == TX_MODE:
            return False

        # Like RadioHead library, turn on high power boost if enabled.
        self._set_boost(_TEST_PA1_BOOST, _TEST_PA2_BOOST)

        # Enable "packet sent" interrupt for IRQ line.
        reg_dio_mapping1 = self._spi_read_register(_RegDioMapping1)
        reg_dio_mapping1.dio_0_mapping = _REG_DIOMAPPING1_TX_PACKET_SENT
        self._spi_write_register(reg_dio_mapping1)

        # Reset the IRQ counter
        self._irq_counter.reset()

        # Enter TX mode (will clear FIFO!).
        self.set_op_mode(TX_MODE)
        return True

    def get_tx_power(self) -> int:
        """The transmit power in dBm. Can be set to a value from -2 to 20 for high power devices
        (RFM69HCW, high_power=True) or -18 to 13 for low power devices.
        """
        # Follow table 10 truth table from the datasheet for determining power
        # level from the individual PA level bits and output power register.
        reg_pa_level = self._spi_read_register(_RegPaLevel)

        pa0 = reg_pa_level.pa_0_on
        pa1 = reg_pa_level.pa_1_on
        pa2 = reg_pa_level.pa_2_on
        current_output_power = reg_pa_level.output_power
        if pa0 and not pa1 and not pa2:
            # -18 to 13 dBm range
            return -18 + current_output_power
        if not pa0 and pa1 and not pa2:
            # -2 to 13 dBm range
            return -18 + current_output_power
        if not pa0 and pa1 and pa2 and not self._is_high_power:
            # 2 to 17 dBm range
            return -14 + current_output_power
        if not pa0 and pa1 and pa2 and self._is_high_power:
            # 5 to 20 dBm range
            return -11 + current_output_power
        raise RuntimeError("Power amps state unknown!")

    def set_tx_power(self, val: float):
        """The transmit power in dBm. Can be set to a value from -2 to 20 for high power devices
        (RFM69HCW, high_power=True) or -18 to 13 for low power devices. Only integer power
        levels are actually set (i.e. 12.5 will result in a value of 12 dBm).
        """
        val = int(val)
        # Determine power amplifier and output power values depending on
        # high power state and requested power.
        pa_0_on = pa_1_on = pa_2_on = 0
        output_power = 0
        if self._is_high_power:
            # Handle high power mode.
            assert -2 <= val <= 20
            pa_1_on = 1
            if val <= 13:
                output_power = val + 18
            elif 13 < val <= 17:
                pa_2_on = 1
                output_power = val + 14
            else:  # power >= 18 dBm
                # Note this also needs PA boost enabled separately!
                pa_2_on = 1
                output_power = val + 11
        else:
            # Handle non-high power mode.
            assert -18 <= val <= 13
            # Enable only power amplifier 0 and set output power.
            pa_0_on = 1
            output_power = val + 18
        # Set power amplifiers and output power as computed above.
        reg_pa_level = self._spi_read_register(_RegPaLevel)
        reg_pa_level.pa_0_on = pa_0_on
        reg_pa_level.pa_1_on = pa_1_on
        reg_pa_level.pa_2_on = pa_2_on
        reg_pa_level.output_power = output_power
        self._spi_write_register(reg_pa_level)

        self._tx_power = val

    def set_modem_config(self, bitrate=_RH_DEFAULT_BITRATE):
        """Primary method to update the configured bitrate for this radio
        Updates modulation to be compatible with RadioHead library

        See _RH_BITRATE_TO_CONFIG_MAP
        """
        assert (
            bitrate in _RH_BITRATE_TO_CONFIG_MAP
        ), f"Invalid bitrate, {bitrate} not in {_RH_BITRATE_TO_CONFIG_MAP.keys()}"

        # Configure modulation for RadioHead library, see _RH_BITRATE_TO_CONFIG_MAP
        self._spi_write_u8(_REG_DATA_MOD, _RH_DATAMODUL_GFSK)
        self._spi_write_u8(_REG_PACKET_CONFIG1, _RH_PACKETCONFIG1_WHITE)

        frequency_deviation, rx_bw, afc_bw = _RH_BITRATE_TO_CONFIG_MAP[bitrate]

        self._set_bitrate(bitrate)
        self._set_frequency_deviation(frequency_deviation)
        self._spi_write_u8(_REG_RX_BW, rx_bw)
        self._spi_write_u8(_REG_AFC_BW, afc_bw)

    def get_bitrate(self) -> float:
        """The modulation bitrate in bits/second (or chip rate if Manchester encoding is enabled).
        Can be a value from ~489 to 32mbit/s, but see the datasheet for the exact supported
        values.
        """
        msb = self._spi_read_u8(_REG_BITRATE_MSB)
        lsb = self._spi_read_u8(_REG_BITRATE_LSB)
        return _FXOSC / ((msb << 8) | lsb)

    def _set_bitrate(self, val: float) -> None:
        assert (_FXOSC / 65535) <= val <= 32000000.0
        # Round up to the next closest bit-rate value with addition of 0.5.
        bitrate = int((_FXOSC / val) + 0.5) & 0xFFFF
        self._spi_write_u8(_REG_BITRATE_MSB, bitrate >> 8)
        self._spi_write_u8(_REG_BITRATE_LSB, bitrate & 0xFF)

    def get_frequency_deviation(self) -> float:
        """The frequency deviation in Hertz."""
        msb = self._spi_read_u8(_REG_FDEV_MSB)
        lsb = self._spi_read_u8(_REG_FDEV_LSB)
        return _FSTEP * ((msb << 8) | lsb)

    def _set_frequency_deviation(self, val: float) -> None:
        assert 0 <= val <= (_FSTEP * 16383)  # fdev is a 14-bit unsigned value
        # Round up to the next closest integer value with addition of 0.5.
        fdev = int((val / _FSTEP) + 0.5) & 0x3FFF
        self._spi_write_u8(_REG_FDEV_MSB, fdev >> 8)
        self._spi_write_u8(_REG_FDEV_LSB, fdev & 0xFF)

    def get_preamble_length(self) -> int:
        """The length of the preamble for sent and received packets, an unsigned 16-bit value.
        Received packets must match this length or they are ignored! Set to 4 to match the
        RadioHead RFM69 library.
        """
        msb = self._spi_read_u8(_REG_PREAMBLE_MSB)
        lsb = self._spi_read_u8(_REG_PREAMBLE_LSB)
        return ((msb << 8) | lsb) & 0xFFFF

    def set_preamble_length(self, val: int) -> None:
        """The length of the preamble for sent and received packets, an unsigned 16-bit value.
        Received packets must match this length or they are ignored! Set to 4 to match the
        RadioHead RFM69 library.
        """
        assert 0 <= val <= 65535
        self._spi_write_u8(_REG_PREAMBLE_MSB, (val >> 8) & 0xFF)
        self._spi_write_u8(_REG_PREAMBLE_LSB, val & 0xFF)
        self.preamble_length = val

    def get_sync_word(self) -> bytearray:
        """The synchronization word value.  This is a byte string up to 8 bytes long (64 bits)
        which indicates the synchronization word for transmitted and received packets. Any
        received packet which does not include this sync word will be ignored. The default value
        is 0x2D, 0xD4 which matches the RadioHead RFM69 library. Setting a value of None will
        disable synchronization word matching entirely.
        """
        reg_sync_config = self._spi_read_register(_RegSyncConfig)

        # Handle when sync word is disabled..
        if not reg_sync_config.sync_on:
            return None
        # Sync word is not disabled so read the current value.
        sync_word_length = (
            reg_sync_config.sync_size + 1
        )  # Sync word size is offset by 1
        # according to datasheet.
        sync_word = bytearray(sync_word_length)
        self._spi_read_into(_REG_SYNC_VALUE1, sync_word)
        return sync_word

    def set_sync_word(self, val: Optional[bytearray]) -> None:
        """The synchronization word value.  This is a byte string up to 8 bytes long (64 bits)
        which indicates the synchronization word for transmitted and received packets. Any
        received packet which does not include this sync word will be ignored. The default value
        is 0x2D, 0xD4 which matches the RadioHead RFM69 library. Setting a value of None will
        disable synchronization word matching entirely.
        """
        # Handle disabling sync word when None value is set.
        reg_sync_config = self._spi_read_register(_RegSyncConfig)
        sync_on = 0
        if val is not None:
            # Check sync word is at most 8 bytes.
            assert 1 <= len(val) <= 8
            # Update the value, size and turn on the sync word.
            self._spi_write_from(_REG_SYNC_VALUE1, val)
            reg_sync_config.sync_size = (
                len(val) - 1
            )  # Again sync word size is offset by
            # 1 according to datasheet.
            sync_on = 1

        reg_sync_config.sync_on = sync_on
        self._spi_write_register(reg_sync_config)

    def get_encryption_key(self) -> bytearray:
        """The AES encryption key used to encrypt and decrypt packets by the chip. This can be set
        to None to disable encryption (the default), otherwise it must be a 16 byte long byte
        string which defines the key (both the transmitter and receiver must use the same key
        value).
        """
        # Handle if encryption is disabled.
        reg_packet_config2 = self._spi_read_register(_RegPacketConfig2)
        if not reg_packet_config2.aes_on:
            return None

        # Encryption is enabled so read the key and return it.
        key = bytearray(16)
        self._spi_read_into(_REG_AES_KEY1, key)
        return key

    def set_encryption_key(self, val: bytearray) -> None:
        """The AES encryption key used to encrypt and decrypt packets by the chip. This can be set
        to None to disable encryption (the default), otherwise it must be a 16 byte long byte
        string which defines the key (both the transmitter and receiver must use the same key
        value).
        """
        # Handle if unsetting the encryption key (None value).
        reg_packet_config2 = self._spi_read_register(_RegPacketConfig2)
        aes_on = 0
        if val is not None:
            # Set the encryption key and enable encryption.
            assert len(val) == 16
            self._spi_write_from(_REG_AES_KEY1, val)
            aes_on = 1

        reg_packet_config2.aes_on = aes_on
        self._spi_write_register(reg_packet_config2)

    def _reset(self) -> None:
        """Perform a reset of the chip."""
        # See section 7.2.2 of the datasheet for reset description.
        self._reset_digital_pin.value = True
        time.sleep(0.0001)  # 100 us
        self._reset_digital_pin.value = False
        time.sleep(0.005)  # 5 ms

    def available(self):  # pylint: disable=missing-function-docstring
        if self._mode == TX_MODE:
            return False

        self.set_mode_rx()
        if self.platform_supports_interrupts:
            return bool(self.rx_packet)
        return True

    async def aio_recv(self, *, timeout=1.0) -> RHPacket:
        """Wait to receive a packet from the receiver. If a packet is found the RHPacket returned,
        otherwise None is returned (which indicates timeout elapsed OR interrupted by transmit).
        """
        packet = None

        # Monitor IRQ pin - sets DIO0 pin to PayloadReady
        if not self.available():
            return packet

        if not self.platform_supports_interrupts:
            # Wait until we get PayloadReady IRQ
            is_payload_read = await self.wait_for_payload_ready(timeout=timeout)
            self.handle_interrupt(is_payload_ready=is_payload_read)

        packet = self.rx_packet
        self.rx_packet = None
        return packet

    recv = aio_to_blocking(aio_recv)

    async def aio_send(
        self,
        dest: int,
        data: ReadableBuffer,
        *,
        sequence_number: int = 0,
        flags: int = 0,
    ) -> bool:
        """Send a string of data using the transmitter.
        You can only send 60 bytes at a time
        (limited by chip's FIFO size and appended headers).
        This appends a 4 byte header to be compatible with the RadioHead library.
        The header defaults to using the initialized attributes:
        (dest,src,sequence_number,flags)
        It may be temporarily overidden via the kwargs - dest,sequence_number,flags.
        Values passed via kwargs do not alter the attribute settings.
        """
        assert len(data) <= 60

        # If we're still in TX_MODE, wait until PacketSent
        is_packet_sent = await self.wait_for_packet_sent()
        if not self.platform_supports_interrupts:
            self.handle_interrupt(is_packet_sent=is_packet_sent)

        # Move to Standby
        # NOTE: Clears FIFO if RX or even in TX (which is why we block above)
        self.set_mode_standby()

        # Fill the FIFO with a packet to send.
        # Combine header and data to form payload
        payload = bytearray(5 + len(data))
        payload[0] = 4 + len(data)  # RFM69 FIFO packet length
        payload[1] = dest  # RH destination
        payload[2] = self.address  # RH source
        payload[3] = sequence_number  # RH sequence number
        payload[4] = flags  # RH flags
        payload[5:] = data  # bytes to transmit

        # Write payload to transmit fifo
        self._spi_write_from(_REG_FIFO, payload)

        # Monitor IRQ pin
        self.set_mode_tx()  # Sets DIO0 pin to PacketSent

        if not self.platform_supports_interrupts:
            is_packet_sent = await self.wait_for_packet_sent()
            self.handle_interrupt(is_packet_sent=is_packet_sent)

        return True

    send = aio_to_blocking(aio_send)

    #####
    # Begin RadioHead ported functions from RHReliableDatagram
    #     Public : Python ports of RadioHead functions
    #     Private: Python helper/convenice functions that do NOT exist in RadioHead
    #####
    async def aio_send_with_ack(
        self,
        dest: int,
        data: ReadableBuffer,
        *,
        max_attempts: int = _RH_RELIABLE_DGRAM_RETRIES,
        ack_timeout: float = _RH_RELIABLE_DGRAM_TIMEOUT,
    ):
        """
        Send data and listen for ACK.  If no ACK within (ack_timeout, 2*ack_timeout), retransmit
        and listen up to max_attempts.  Worst case time: (TX time * 2 ack_timeout) * max_attempts.

        Return
            ack_packet: RHPacket if acked, None otherwise
        """
        self._last_sequence_number = (self._last_sequence_number + 1) & 0xFF
        sequence_number = self._last_sequence_number
        attempts = 0  # "retries" in RadioHead library, really this is an attempt
        flags = 0

        while attempts <= max_attempts:
            attempts += 1

            # Mark this as a RETRY
            if attempts > 1:
                flags |= _RH_RELIABLE_DGRAM_PACKET_FLAGS_RETRY

            await self.aio_send(
                dest=dest, data=data, sequence_number=sequence_number, flags=flags
            )

            # Never wait for ACKs to broadcast messages
            if dest == RH_BROADCAST_ADDRESS:
                return None

            if attempts > 1:
                self.send_retransmissions += 1

            # Introduce a random delay so we avoid TX collisions from other nodes
            random_delay = ack_timeout + (ack_timeout * random.random())
            ack_packet = await self.aio_recv(timeout=random_delay)

            # We didn't get a response in time, try re-sending
            if ack_packet is None:
                continue

            ack_packet_src = ack_packet.src
            ack_packet_sequence_number = ack_packet.sequence_number

            is_ack = bool(ack_packet.flags & _RH_RELIABLE_DGRAM_PACKET_FLAGS_ACK)
            is_sequence_number = bool(ack_packet_sequence_number == sequence_number)

            is_seen_before = bool(
                ack_packet_sequence_number == self._seen_ids[ack_packet_src]
            )

            # This is the ACK we're looking for, return True!
            if (
                bool(ack_packet_src == dest)
                and bool(ack_packet.dest == self.address)
                and is_ack
                and is_sequence_number
            ):
                return ack_packet

            # ACK a previously received packet
            if not is_ack and is_seen_before:
                await self.aio_acknowledge_packet(ack_packet)

        return None

    aio_send_to_wait = aio_send_with_ack

    send_with_ack = aio_to_blocking(aio_send_with_ack)
    send_to_wait = send_with_ack

    async def aio_acknowledge_packet(self, packet: RHPacket):
        """
        Instead of (ID, _from), use packet
        """
        return await self.aio_send(
            dest=packet.src,
            data=b"!",
            sequence_number=packet.sequence_number,
            flags=packet.flags | _RH_RELIABLE_DGRAM_PACKET_FLAGS_ACK,
        )

    acknowledge_packet = aio_to_blocking(aio_acknowledge_packet)

    async def aio_recv_with_ack(
        self,
        *,
        timeout=1.0,
    ) -> RHPacket:
        """
        Receive data and send back an ACK on receipt when explicitly address to me.

        Return
            packet: RHPacket received
        """
        packet = await self.aio_recv(timeout=timeout)
        if packet is None:
            return packet

        packet_src = packet.src
        packet_flags = packet.flags
        packet_sequence_number = packet.sequence_number

        # Never ACK an ACK
        if packet_flags & _RH_RELIABLE_DGRAM_PACKET_FLAGS_ACK:
            return None

        # ACK a packet that was destined for us (do not ack Broadcast)
        if packet.dest == self.address:
            await self.aio_acknowledge_packet(packet)

        is_retry = packet_flags & _RH_RELIABLE_DGRAM_PACKET_FLAGS_RETRY
        is_new_sequence_number = bool(
            packet_sequence_number != self._seen_ids[packet_src]
        )
        # RH_ENABLE_EXPLICIT_RETRY_DEDUP
        if (_RH_ENABLE_EXPLICIT_RETRY_DEDUP and not is_retry) or is_new_sequence_number:
            self._seen_ids[packet_src] = packet_sequence_number
            return packet

        return None

    aio_recv_from_ack = aio_recv_with_ack
    recv_with_ack = aio_to_blocking(aio_recv_with_ack)
    recv_from_ack = recv_with_ack
