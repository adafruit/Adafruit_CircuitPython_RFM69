
Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-rfm69/badge/?version=latest

    :target: https://circuitpython.readthedocs.io/projects/rfm69/en/latest/

    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

CircuitPython RFM69 packet radio module.  This supports basic
RadioHead-compatible sending and receiving of packets with RFM69 series radios
(433/915Mhz).  Note this does NOT support advanced RadioHead features like
guaranteed delivery--only 'raw' packets are currently supported.  In addition
this is NOT for LoRa radios! Finally be aware this is a 'best effort' at
receiving data using pure Python code--there is not interrupt support so you
might lose packets if they're sent too quickly for the board to process them.
You will have the most luck using this in simple low bandwidth scenarios like
sending and receiving a 60 byte packet at a time--don't try to receive many
kilobytes of data at a time!

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

See examples/simpletest.py for a simple demo of the usage.

API Reference
=============

.. toctree::
   :maxdepth: 2

   api

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
