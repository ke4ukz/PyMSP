# PyMSP
Python MultiWii Serial Protocol communication library for radio-controlled devices.

## Abstract
This Python library is intended to allow RC pilots to write scripts to control multirotor RC devices. All of the serial communication and MSP commands are handled by the library, allowing the pilot to focus on what the device should do rather than worrying about the details of the communication. All one has to do is tell the library what serial port and baud rate to use, and then a collection of get and set methods will allow one to poll the device for information and send commands.


## Flight controller firmware
Commands to get data from and send data to the flight controller have been tested with CleanFlight 1.8.1 and 1.9.0, but in 1.9.0 I couldn't get it to arm (L1 would blink oddly, see https://github.com/cleanflight/cleanflight/issues/1009). Note that if the receiver mode is not set to RX_MSP then sending RC commands will (in my experience) cause weird results as the FC tries to use input both from the handheld transmitter and the computer.

## Useful Links
* MultiWii Serial Protocol command information: http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
* CleanFlight MSP extensions: http://shipow.github.io/cleanflight-web/docs/api/msp_extensions/
* Naze32 manual: http://www.abusemark.com/downloads/naze32_rev3.pdf
* CleanFlight manual: https://github.com/cleanflight/cleanflight/releases/download/v1.10.0/Manual.pdf