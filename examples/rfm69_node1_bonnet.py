# SPDX-FileCopyrightText: 2020 Jerry Needell for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example to send a packet periodically between addressed nodes

# Import the SSD1306 module.
import adafruit_ssd1306
import board
import busio
import digitalio

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


# set the time interval (seconds) for sending packets
transmit_interval = 10

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip.
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

# Initialize SPI bus.
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialze RFM radio

# Attempt to set up the RFM69 Module
try:
    rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, RADIO_FREQ_MHZ)
    display.text("RFM69: Detected", 0, 0, 1)
except RuntimeError:
    # Thrown on version mismatch
    display.text("RFM69: ERROR", 0, 0, 1)

display.show()


# Optionally set an encryption key (16 byte AES key). MUST match both
# on the transmitter and receiver (or be set to None to disable/the default).
rfm69.encryption_key = b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"

# set node addresses
rfm69.node = 1
rfm69.destination = 2
# initialize counter
counter = 0
# send a broadcast message from my_node with ID = counter
rfm69.send(bytes(f"Startup message {counter} from node {rfm69.node}", "UTF-8"))

# Wait to receive packets.
print("Waiting for packets...")
button_pressed = None
while True:
    # Look for a new packet: only accept if addresses to my_node
    packet = rfm69.receive(with_header=True)
    # If no packet was received during the timeout then None is returned.
    if packet is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print("Received (raw header):", [hex(x) for x in packet[0:4]])
        print(f"Received (raw payload): {packet[4:]}")
        print(f"Received RSSI: {rfm69.last_rssi}")
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
        rfm69.send(
            bytes(
                f"message number {counter} from node {rfm69.node} button {button_pressed}",
                "UTF-8",
            ),
            keep_listening=True,
        )
        button_pressed = None
