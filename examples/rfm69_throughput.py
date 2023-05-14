# SPDX-FileCopyrightText: 2023 Matthew Tai
# SPDX-License-Identifier: MIT
"""
    Send, no acks      [% achieved vs theoretical]
    250000 => 73kbps   [29%]
    125000 => 55kbps   [44%]
    57600  => 35kbps   [60%]
    38400  => 26kbps   [67%]
    19200  => 14.8kbps [77%]

    Send, acks         [% achieved vs theoretical]
    (receiver using this library as well)
    =====
    66 byte packets, 500 packets (Most time in TX)
    250000 => 27kbps   [11%]
    125000 => 26kbps   [21%]
    57600  => 19kbps   [33%]
    38400  => 15kbps   [39%]
    19200  => 10kbps   [52%]

    36 byte packets, 500 packets (Mid time in TX)
    250000 => 17kbps   [ 7%]
    125000 => 14kbps   [11%]
    57600  => 12kbps   [21%]
    38400  => 10kbps   [26%]
    19200  => 7.5kbps  [39%]

    7 byte packets, 500 packets  (Least time in TX, library limited)
    250000 => 3.5kbps  [ 1.8%]
    125000 => 3.8kbps  [ 3.0%]
    57600  => 3.0kbps  [ 5.2%]
    38400  => 2.5kbps  [ 6.5%]
    19200  => 2.2kbps  [11.4%]
"""

import asyncio
import board
import keypad
from adafruit_ticks import ticks_ms, ticks_diff, ticks_add, ticks_less
import adafruit_rfm69


SENDER_ADDRESS = 0x10
LISTENER_ADDRESS = 0x01

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
RFM69_RESET = board.D11
RFM69_CS = board.D10
RFM69_IRQ = board.D9

BITRATES_TO_TEST = [
    adafruit_rfm69.RH_BITRATE_250000,
    adafruit_rfm69.RH_BITRATE_125000,
    adafruit_rfm69.RH_BITRATE_57600,
    adafruit_rfm69.RH_BITRATE_38400,
    adafruit_rfm69.RH_BITRATE_19200,
]


def init_client():
    # Initialize SPI device.
    in_spi = board.SPI()
    in_spi_device = adafruit_rfm69.RFM69.spi_device(in_spi, RFM69_CS)

    # Initialze RFM radio
    rfm69_client = adafruit_rfm69.RFM69(
        in_spi_device, RFM69_IRQ, RFM69_RESET, RADIO_FREQ_MHZ
    )
    # Optionally set an encryption key (16 byte AES key). MUST match both
    # on the transmitter and receiver (or be set to None to disable/the default).
    # rfm69_client.set_encryption_key(
    #     b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
    # )

    print("===== Initializing RFM69 client =====")
    print(f"Temperature: {rfm69_client.get_temperature()}C")
    print(f"Frequency: {rfm69_client.get_frequency_mhz()}mhz")
    print(f"Bit rate: {rfm69_client.get_bitrate() / 1000}kbit/s")
    print(f"Frequency deviation: {rfm69_client.get_frequency_deviation()}hz")
    return rfm69_client


async def send_rfm69_packets(rfm69_client, bitrate):
    """Continuous transmission of 66 byte packets kbytes on Adafruit ESP32-S3 TFT Feather
    Testing demonstrates CPU-bound and/or relatively interrupt processing via Python
    """
    rfm69_client.address = SENDER_ADDRESS
    rfm69_client.set_modem_config(bitrate)

    x = 0
    message = "@" * 1
    encoded_message = message.encode("utf-8")
    ticks_start = ticks_ms()
    while x < 500:
        await rfm69_client.aio_send_with_ack(LISTENER_ADDRESS, encoded_message)
        x += 1

    ticks_end = ticks_ms()

    bytes_transmitted = (len(message) + 6) * x
    time_elapsed = ticks_diff(ticks_end, ticks_start) / 1000.0
    bytes_per_sec = bytes_transmitted / time_elapsed
    bits_per_sec = bytes_per_sec * 8
    print(f"SEND {bytes_transmitted} bytes in {time_elapsed} seconds")
    print(f"{bitrate} :: {bytes_per_sec} bytes/sec :: {bits_per_sec} bits/sec")


async def read_rfm69_packets(rfm69_client, bitrate, interval=5.0):
    rfm69_client.address = LISTENER_ADDRESS
    rfm69_client.set_modem_config(bitrate)

    while True:
        await rfm69_client.aio_recv_with_ack(timeout=interval)


def set_bitrate(bitrate_idx):
    bitrate_idx_to_select = bitrate_idx % len(BITRATES_TO_TEST)
    test_bitrate = BITRATES_TO_TEST[bitrate_idx_to_select]
    print(f"Bitrate: {test_bitrate}")
    return test_bitrate


async def main():
    rfm69_client = init_client()

    print("Select bitrate within next 5 seconds...")
    bitrate_idx = 0
    bitrate = set_bitrate(bitrate_idx)
    button_keys = keypad.Keys((board.BUTTON,), value_when_pressed=False, pull=True)

    end_ticks = ticks_add(ticks_ms(), 5 * 1000)
    while ticks_less(ticks_ms(), end_ticks):
        while button_keys.events:
            event = button_keys.events.get()
            if event.pressed:
                bitrate_idx += 1
                bitrate = set_bitrate(bitrate_idx)

        await asyncio.sleep(0.100)

    print(f"Starting test with bitrate {bitrate}")
    # Step 1a - Uncomment for sender
    await send_rfm69_packets(rfm69_client, bitrate)

    # Step 2b - Uncomment for listener
    # await read_rfm69_packets(rfm69_client, bitrate)


asyncio.run(main())
