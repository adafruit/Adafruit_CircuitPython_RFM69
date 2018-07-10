
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-rfm69/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/rfm69/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.org/adafruit/Adafruit_CircuitPython_RFM69.svg?branch=master
    :target: https://travis-ci.org/adafruit/Adafruit_CircuitPython_RFM69
    :alt: Build Status

CircuitPython RFM69 packet radio module.  This supports basic RadioHead-compatible sending and
receiving of packets with RFM69 series radios (433/915Mhz).

.. note:: This does NOT support advanced RadioHead features like guaranteed delivery--only 'raw' packets are currently supported.

.. warning:: This is NOT for LoRa radios!

.. note:: This is a 'best effort' at receiving data using pure Python code--there is not interrupt
    support so you might lose packets if they're sent too quickly for the board to process them.
    You will have the most luck using this in simple low bandwidth scenarios like sending and
    receiving a 60 byte packet at a time--don't try to receive many kilobytes of data at a time!

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============
See examples/rfm69_simpletest.py for a simple demo of the usage.
Note: the default baudrate for the SPI is 5000000 (5MHz). 
The maximum setting is 10Mhz but 
transmission errors have been observed expecially when using breakout boards.
For breakout boards or other configurations where the boards are separated,
it may be necessary to reduce the baudrate for reliable data transmission.
The baud rate may be specified as an keyword parameter when initializing the board.
To set it to 1000000 use :

.. code-block:: python

    # Initialze RFM radio
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ,baudrate=1000000)


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_RFM69/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-rfm69 --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
