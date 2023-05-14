# SPDX-FileCopyrightText: 2020 Tony DiCola, Jerry Needell for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example using Interrupts to send a message and then wait indefinitely for messages
# to be received. Interrupts are used only for receive. sending is done with polling.
# This example is for systems that support interrupts like the Raspberry Pi with "blinka"
# CircuitPython does not support interrupts so it will not work on  Circutpython boards
import time
import board
import RPi.GPIO as io
import adafruit_rfm69

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip.
RFM69_IRQ = 22
RFM69_CS = board.CE1
RFM69_RESET = board.D25

SRC_ADDRESS = adafruit_rfm69.RH_BROADCAST_ADDRESS
DEST_ADDRESS = adafruit_rfm69.RH_BROADCAST_ADDRESS


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
    print(f"Received (dest): {hex(packet.dest)}")
    print(f"Received (src): {hex(packet.src)}")
    print(f"Received (sequence_number): {hex(packet.sequence_number)}")
    print(f"Received (flags): {hex(packet.flags)}")

    print(f"Received (raw payload): {packet.data}")
    print(f"RSSI: {packet.rssi}")
    print(f"Time: {packet.time}")


rfm69_client = init_client()

# Magic incantation that tells us we can support interrupts
rfm69_client.platform_supports_interrupts = True


# setup interrupt callback function
def rfm69_callback(rfm69_irq):
    print(f"IRQ detected on pin {rfm69_irq}")
    rfm69_client.handle_interrupt()


# configure the interrupt pin and event handling.
io.setmode(io.BCM)
io.setup(RFM69_IRQ, io.IN, pull_up_down=io.PUD_DOWN)  # activate input
io.add_event_detect(RFM69_IRQ, io.RISING)
io.add_event_callback(RFM69_IRQ, rfm69_callback)

# Send a packet.  Note you can only send a packet up to 60 bytes in length.
# This is a limitation of the radio packet size, so if you need to send larger
# amounts of data you will need to break it into smaller send calls.  Each send
# call will wait for the previous one to finish before continuing.
rfm69_client.send(DEST_ADDRESS, bytes("Hello world!\r\n", "utf-8"))
print("Sent hello world message!")
# Just start listening
rfm69_client.available()

# Wait to receive packets.  Note that this library can't receive data at a fast
# rate, in fact it can only receive and process one 60 byte packet at a time.
# This means you should only use this for low bandwidth scenarios, like sending
# and receiving a single message at a time.
print("Waiting for packets...")

# the loop is where you can do any desire processing
# the global variable packet_received can be used to determine if a packet was received.
while True:
    # the sleep time is arbitrary since any incoming packet will trigger an interrupt
    # and be received.
    time.sleep(0.1)
    if rfm69_client.rx_packet:
        print("received message!")
        print_packet(rfm69_client.rx_packet)
        rfm69_client.rx_packet = None
