# SPDX-FileCopyrightText: 2020 Jerry Needell for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example to send a packet periodically

import time
import board
import adafruit_rfm69

# Define pins connected to the chip.
RFM69_IRQ = board.D9
RFM69_CS = board.CE1
RFM69_RESET = board.D25

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define src/dst addresses for Node 1+2 respectively
SRC_ADDRESS = adafruit_rfm69.RH_BROADCAST_ADDRESS
DEST_ADDRESS = adafruit_rfm69.RH_BROADCAST_ADDRESS

# If you want to test addresses message, folllow Steps 2a+2b
# Step 2a - Addressed message, uncomment for Node 1
# SRC_ADDRESS = 100
# DEST_ADDRESS = 200

# Step 2b - Addressed message, uncomment for Node 2
# SRC_ADDRESS = 200
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


def print_packet(packet):
    if not packet:
        return

    # Received a packet!
    # Print out the raw bytes of the packet:
    print("===== Received packet =====")
    print(f"Dest: {hex(packet.dest)}")
    print(f"Src: {hex(packet.src)}")
    print(f"Seq No: {hex(packet.sequence_number)}")
    print(f"Flags: {hex(packet.flags)}")

    print(f"Payload: {packet.data.decode('utf-8')}")
    print(f"RSSI: {packet.rssi}")
    print(f"Time: {packet.time}")


# initialize counter
counter = 0
rfm69_client = init_client()

# Send a message
rfm69_client.send(
    DEST_ADDRESS, bytes(f"quickstart {rfm69_client.address}: {counter}", "utf-8")
)

print("Waiting for packets...")
time_now = time.monotonic()
while True:
    # Look for a new packet - wait up to 5 seconds:
    rx_packet = rfm69_client.recv(timeout=5.0)
    if rx_packet is not None:
        print_packet(rx_packet)

    # send reading after any packet received
    if time.monotonic() - time_now > TX_INTERVAL:
        # reset timeer
        time_now = time.monotonic()
        # clear flag to send data
        counter = counter + 1
        rfm69_client.send(
            DEST_ADDRESS,
            bytes(f"quickstart {rfm69_client.address}: {counter}", "UTF-8"),
        )
