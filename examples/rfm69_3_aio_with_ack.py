# SPDX-FileCopyrightText: 2023 Matthew Tai
# SPDX-License-Identifier: MIT

# Example to use asyncio and aio_send_with_ack
# This example simultaneously
#    1) Changes the colorwheel on a Neopixel
#    2) TX'ing or RX'ing via the radio

import asyncio
import board
import neopixel
import rainbowio
from adafruit_ticks import ticks_ms

import adafruit_rfm69

# Define pins connected to the chip.
RFM69_IRQ = board.D9
RFM69_CS = board.CE1
RFM69_RESET = board.D25

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define src/dst addresses for Node 1+2 respectively
SRC_ADDRESS = None
DEST_ADDRESS = None

# Step 1a - Uncomment for Node 1
# SRC_ADDRESS  = 100
# DEST_ADDRESS = 200

# Step 1b - Uncomment for Node 2
# SRC_ADDRESS  = 200
# DEST_ADDRESS = 100

assert SRC_ADDRESS is not None and DEST_ADDRESS is not None

# set the time interval (seconds) for sending packets
TX_INTERVAL = 10


def init_client():
    # Initialize SPI device.
    in_spi = board.SPI()
    in_spi_device = adafruit_rfm69.RFM69.spi_device(in_spi, RFM69_CS)

    # Initialze RFM radio
    client = adafruit_rfm69.RFM69(in_spi_device, RFM69_IRQ, RFM69_RESET, RADIO_FREQ_MHZ)
    client.address = SRC_ADDRESS
    # Optionally set an encryption key (16 byte AES key). MUST match both
    # on the transmitter and receiver (or be set to None to disable/the default).
    client.set_encryption_key(
        b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
    )

    print("===== Initializing RFM69 client =====")
    print(f"Temperature: {client.get_temperature()}C")
    print(f"Frequency: {client.get_frequency_mhz()}mhz")
    print(f"Bit rate: {client.get_bitrate() / 1000}kbit/s")
    print(f"Frequency deviation: {client.get_frequency_deviation()}hz")
    return client


def init_neopixel():
    pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
    pixel.brightness = 0.3
    return pixel


async def rainbow(pixel, interval=0.020):
    while True:
        for color_value in range(255):
            pixel[0] = rainbowio.colorwheel(color_value)
            await asyncio.sleep(interval)


async def send_packets(rfm69_client, interval=1.0):
    rfm69_client.address = SRC_ADDRESS

    x = 0
    while True:
        message = f"aio {rfm69_client.address}: {x}"
        ack_packet = await rfm69_client.aio_send_with_ack(
            DEST_ADDRESS, message.encode("utf-8")
        )
        if ack_packet is not None:
            assert DEST_ADDRESS == ack_packet.src
            print(
                "[{}] TX {} ({} db) | {} ".format(
                    int(ticks_ms() / 1000.0), DEST_ADDRESS, ack_packet.rssi, message
                )
            )
        else:
            print(
                "[{}] TX {} (!! db) | {} ".format(
                    int(ticks_ms() / 1000.0), DEST_ADDRESS, message
                )
            )

        x += 1
        await asyncio.sleep(interval)


async def read_packets(rfm69_client, interval=5.0):
    rfm69_client.address = DEST_ADDRESS

    while True:
        current_packet = await rfm69_client.aio_recv_with_ack(timeout=interval)
        if current_packet is None:
            print("[{}] RX !!".format(int(ticks_ms() / 1000.0)))
            continue

        print(
            "[{}] RX {} ({} dB) | {}".format(
                int(current_packet.time),
                current_packet.src,
                current_packet.rssi,
                current_packet.data.decode("utf-8"),
            )
        )


async def main():
    pixel = init_neopixel()
    rfm69_client = init_client()

    coros = [rainbow(pixel)]
    if SRC_ADDRESS == 100:
        coros.append(send_packets(rfm69_client))
    elif SRC_ADDRESS == 200:
        coros.append(read_packets(rfm69_client))

    await asyncio.gather(*[asyncio.create_task(current_coro) for current_coro in coros])


asyncio.run(main())
