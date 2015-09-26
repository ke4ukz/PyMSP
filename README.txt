Commands to get data from and send data to the flight controller work with CleanFlight 1.8.1 and 1.9.0,
but in 1.9.0 I couldn't get it to arm (L1 would blink oddly, see https://github.com/cleanflight/cleanflight/issues/1009).
Note that if the receiver mode is not set to RX_MSP then sending RC commands will (in my experience) cause weird results.

MultiWii Serial Protocol command information: http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
CleanFlight MSP extensions: http://shipow.github.io/cleanflight-web/docs/api/msp_extensions/
Naze32 manual: http://www.abusemark.com/downloads/naze32_rev3.pdf
CleanFlight manual: https://github.com/cleanflight/cleanflight/releases/download/v1.10.0/Manual.pdf