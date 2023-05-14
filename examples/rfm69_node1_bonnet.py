# SPDX-FileCopyrightText: 2020 Jerry Needell for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example to send a packet periodically between addressed nodes

import board
import busio
import digitalio

# Import the SSD1306 module.
import adafruit_ssd1306
import adafruit_rfm69

# Button A
btnA = digitalio.DigitalInOut(board.D5)
btnA.direction = digitalio.Direction.INPUT
btnA.pull = digitalio.Pull.UP

# Button B
btnB = digitalio.DigitalInOut(board.D6)
btnB.direction = digitalio.Direction.INPUT
btnB.pull = digitalio.Pull.UP

# Button C
btnC = digitalio.DigitalInOut(board.D12)
btnC.direction = digitalio.Direction.INPUT
btnC.pull = digitalio.Pull.UP

# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# 128x32 OLED Display
reset_pin = digitalio.DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
# Clear the display.
display.fill(0)
display.show()
width = display.width
height = display.height


# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip.
RFM69_IRQ = board.D9
RFM69_CS = board.CE1
RFM69_RESET = board.D25


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


# Initialize SPI device.
in_spi = board.SPI()
in_spi_device = adafruit_rfm69.RFM69.spi_device(in_spi, RFM69_CS)

# Initialze RFM radio


# Attempt to set up the RFM69 Module
try:
    rfm69_client = adafruit_rfm69.RFM69(
        in_spi_device, RFM69_IRQ, RFM69_RESET, RADIO_FREQ_MHZ
    )
    display.text("RFM69: Detected", 0, 0, 1)
except RuntimeError:
    # Thrown on version mismatch
    display.text("RFM69: ERROR", 0, 0, 1)

display.show()


# Optionally set an encryption key (16 byte AES key). MUST match both
# on the transmitter and receiver (or be set to None to disable/the default).
rfm69_client.set_encryption_key(
    b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
)

# set node addresses
rfm69_client.address = 1
destination = 2
# initialize counter
counter = 0
# send a broadcast message from my_node with ID = counter
rfm69_client.send(
    destination,
    bytes(f"Startup message {counter} from node {rfm69_client.address}", "UTF-8"),
)

# Wait to receive packets.
print("Waiting for packets...")
button_pressed = None
while True:
    # Look for a new packet: only accept if addresses to my_node
    rx_packet = rfm69_client.recv()
    # If no packet was received during the timeout then None is returned.
    if rx_packet is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print_packet(rx_packet)
    # Check buttons
    if not btnA.value:
        button_pressed = "A"
        # Button A Pressed
        display.fill(0)
        display.text("AAA", width - 85, height - 7, 1)
        display.show()
    if not btnB.value:
        button_pressed = "B"
        # Button B Pressed
        display.fill(0)
        display.text("BBB", width - 75, height - 7, 1)
        display.show()
    if not btnC.value:
        button_pressed = "C"
        # Button C Pressed
        display.fill(0)
        display.text("CCC", width - 65, height - 7, 1)
        display.show()
        # send reading after any button pressed
    if button_pressed is not None:
        counter = counter + 1
        # send a  mesage to destination_node from my_node
        rfm69_client.send(
            destination,
            bytes(
                f"msg {counter} from node {rfm69_client.address} button {button_pressed}",
                "UTF-8",
            ),
        )
        button_pressed = None
