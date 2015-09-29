# PyMSP
Python MultiWii Serial Protocol communication library for radio-controlled devices.

## Abstract
This Python library is intended to allow RC pilots to write scripts to control multirotor RC devices. All of the serial communication and MSP commands are handled by the library, allowing the pilot to focus on what the device should do rather than worrying about the details of the communication. All one has to do is tell the library what serial port and baud rate to use, and then a collection of get and set methods will allow one to poll the device for information and send commands.

## Requirements
Requires pyserial. Designed with Python 2.7 32-bit (pyserial does not work correctly with 64-bit Python in Windows, see [this](http://stackoverflow.com/questions/3028786/how-can-i-fix-error-6-the-handle-is-invalid-with-pyserial) post for details).

## More Information
See the [PyMSP Wiki](https://github.com/ke4ukz/PyMSP/wiki) for more information