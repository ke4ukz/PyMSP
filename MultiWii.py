#Python MultiWii Serial Protocol communication library for radio-controlled devices
#Copyright (C) 2015 Jonathan Dean
#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

from serial import Serial
from threading import Thread, Event
from time import sleep, time
import struct

class MultiWii(object):
	"""Connect to and communicate with an RC device using the MultiWii Serial Protocol
	
	Attributes:
		responseTimeout (int): Number of seconds to wait for a response to a command
			before giving up. Defaults to 3.
		MULTITYPENAMES (dict): String representation of each device type

	Notes:
		Some values may be needed to tested by observing their values to interpret their meaning.
		Values returned by getIMU depend on the sensor type installed. GPS values currently are
		the raw data returned by the flight controller (if a GPS receiver is installed), but may
		be changed in the future to something more useful (decimal degrees probably).
	"""

	__VERSION__ = "0.0.12"
	__AUTHOR__ = "Jonathan Dean (ke4ukz@gmx.com)"
	#Instance variables:
	#	_port: serial.Serial object
	#	_monitorThread: threading.Thread object that monitors the incoming serial data
	#	_exitNow: threading.Event object that is set when the thread should exit
	#	_responses: dict of {command: _MSPResponse} used to store responses
	#	responseTimeout: number of seconds to wait for a response to a command (defaults to 3)

	MULTITYPENAMES = {0:"Unknown", 1:"TRI", 2:"QUADP", 3:"QUADX", 4:"BI", 5:"GIMBAL", 6:"Y6", 7:"HEX6", 8:"FLYING_WING", 9:"Y4", 10:"HEX6X", 11:"OCTOX8", 12:"OCTOFLATX", 13:"OCTOFLATP", 14:"AIRPLANE", 15:"HELI_120_CCPM", 16:"HELI_90_DEG", 17:"VTAIL4", 18:"HEX6H", 19:"PPM_TO_SERVO", 20:"DUALCOPTER", 21:"SINGLECOPTER"}
	_MODERANGENAMES = {0:"ARM", 1:"ANGLE", 2:"HORIZON", 3:"BARO", 4:"Reserved", 5:"MAG", 6:"HEADFREE", 7:"HEADADJ", 8:"CAMSTAB", 9:"CAMTRIG", 10:"GPSHOME", 11:"GPSHOLD", 12:"PASSTHRU", 13:"BEEPERON", 14:"LEDMAX", 15:"LEDLOW", 16:"LLIGHTS", 17:"CALIB", 18:"GOV", 19:"OSD", 20:"TELEMETRY", 21:"AUTOTUNE", 22:"SONAR"}

	class _MSPCOMMANDS:
		MSP_NULL = 0
		MSP_MODE_RANGES = 34
		MSP_SET_MODE_RANGE = 35
		MSP_ADJUSTMENT_RANGES = 52
		MSP_SET_ADJUSTMENT_RANGE = 53
		MSP_IDENT = 100
		MSP_STATUS = 101
		MSP_RAW_IMU = 102
		MSP_SERVO = 103
		MSP_MOTOR = 104
		MSP_RC = 105
		MSP_RAW_GPS = 106
		MSP_COMP_GPS = 107
		MSP_ATTITUDE = 108
		MSP_ALTITUDE = 109
		MSP_ANALOG = 110
		MSP_BOX = 113
		MSP_MISC = 114
		MSP_BOXNAMES = 116
		MSP_BOXIDS = 119
		MSP_SET_RAW_RC = 200
		MSP_ACC_CALIBRATION = 205
		MSP_MAG_CALIBRATION = 206
		MSP_SET_MISC = 207
		MSP_SET_HEAD = 211
	#end class _MSPCOMMANDS

	class _MSPSTATES:
		"""Enum of MSP States"""
		IDLE = 0
		HEADER_START = 1
		HEADER_M = 2
		HEADER_ARROW = 3
		HEADER_SIZE = 4
		HEADER_CMD = 5
	#end class _MSPSTATES

	class _MSPResponse:
		"""Combine MSP response data and finished communication flag"""
		def __init__(self):
			self.finished = False
			self.data = []
		#end def __init__
	#end class _MSPResponse

# construction/destruction #################################################################################
	def __init__(self):
		self._port = Serial()
		self._monitorThread = Thread(target=self._monitorSerialPort)
		self._exitNow = Event()
		self._responses = {}
		self.responseTimeout = 3
	#end def __init__

	def __del__(self):
		if self._monitorThread.isAlive():
			self._exitNow.set()
		elif self._port.isOpen():
			self._port.close()
	#end def __del__

# connection methods #################################################################################
	def disconnect(self):
		"""Disconnect from a MultiWii RC device"""
		if self._monitorThread.isAlive():
			self._exitNow.set()
			self._monitorThread.join()
	#end def disconnect

	def connect(self, portName, baudRate):
		"""Connect to a MultiWii RC device

		Args:
			portName (str): The serial port name to use for the connection (e.g., "COM3" in Windows or "/dev/ttyUSB0", "/dev/TTYAMA0", etc. in Linux)
			baudRate (int): The communications speed (generally 115200 for CleanFlight)

		Returns:
			bool: True if successful, False otherwise
		"""
		try:
			self._port.setPort(portName)
			self._port.setBaudrate(baudRate)
			self._port.open()
		except Exception as ex:
			print("Error opening serial port: " + str(ex))
			return False
		#end try
		try:
			self._monitorThread.start()
			return True
		except Exception as ex:
			print("Error starting thread: " + str(ex))
			return False
		#end try
	#end def connect

# Byte<->Int functions #################################################################################
#	These methods convert integers to bytearrays and vice versa
#	All integers are assumed to be in the correct range, and and overflowing data will cause an exception
	def _toInt16(self, data):
		if (len(data) == 2):
			return struct.unpack("@h", struct.pack("<BB", data[0], data[1]))[0]
		else:
			return None
	#end def _toInt16

	def _toUInt16(self, data):
		if (len(data) == 2):
			return struct.unpack("@H", struct.pack("<BB", data[0], data[1]))[0]
		else:
			return None
	#end def _toUInt16

	def _toInt32(self, data):
		if (len(data) == 4):
			return struct.unpack("@i", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
		else:
			return None
	#end def _toInt32

	def _toUInt32(self, data):
		if (len(data) == 4):
			return struct.unpack("@I", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
		else:
			return None
	#end def _toUInt32

	def _fromInt16(self, value):
		return struct.unpack("<BB", struct.pack("@h", value))
	#end def _fromInt16

	def _fromUInt16(self, value):
		return struct.unpack("<BB", struct.pack("@H", value))
	#end def _fromUInt16

	def _fromInt32(self, value):
		return struct.unpack("<BBBB", struct.pack("@i", value))
	#end def _fromInt32

	def _fromUInt32(self, value):
		return struct.unpack("<BBBB", struct.pack("@I", value))
	#end def _fromUInt32

# command processing methods #################################################################################
	def _monitorSerialPort(self):
		state = self._MSPSTATES.IDLE
		data = bytearray()
		dataSize = 0
		dataChecksum = 0
		command = self._MSPCOMMANDS.MSP_NULL
		while (not self._exitNow.isSet()):
			if (self._port.inWaiting() > 0):
				inByte = ord(self._port.read())
				if (state == self._MSPSTATES.IDLE):
					state = self._MSPSTATES.HEADER_START if (inByte==36) else self._MSPSTATES.IDLE #chr(36)=='$'
				elif (state == self._MSPSTATES.HEADER_START):
					state = self._MSPSTATES.HEADER_M if (inByte==77) else self._MSPSTATES.IDLE #chr(77)=='M'
				elif (state == self._MSPSTATES.HEADER_M):
					state = self._MSPSTATES.HEADER_ARROW if (inByte==62) else self._MSPSTATES.IDLE #chr(62)=='>'
				elif (state == self._MSPSTATES.HEADER_ARROW):
					dataSize = inByte
					data = bytearray()
					dataChecksum = inByte
					state = self._MSPSTATES.HEADER_SIZE
				elif (state == self._MSPSTATES.HEADER_SIZE):
					command = inByte
					dataChecksum = (dataChecksum ^ inByte)
					state = self._MSPSTATES.HEADER_CMD
				elif (state == self._MSPSTATES.HEADER_CMD) and (len(data) < dataSize):
					data.append(inByte)
					dataChecksum = (dataChecksum ^ inByte)
				elif (state == self._MSPSTATES.HEADER_CMD) and (len(data) >= dataSize):
					if (dataChecksum == inByte):
						#Good command, do something with it
						self._processCommand(command, data)
					else:
						#Bad checksum
						pass
					state = self._MSPSTATES.IDLE
					#end if
				#end if
			else:
				sleep(0)
			#end if
		#end while
		self._port.close()
	#end def _monitorSerialPort

	def _processCommand(self, command, data):
		if (self._responses.has_key(command)):
			self._responses[command].data = data
			self._responses[command].finished = True
			return True
		else:
			return False
		#end if
	#end def _processCommand

	def _sendCommand(self, command, data=None):
		if (data is None):
			dataSize = 0
		else:
			if len(data) < 256:
				dataSize = len(data)
			else:
				return False
		output = bytearray()
		output.append('$')
		output.append('M')
		output.append('<')
		output.append(dataSize)
		checksum = dataSize
		output.append(command)
		checksum = (checksum ^ command)
		if (dataSize > 0):
			for b in data:
				output.append(b)
				checksum = (checksum ^ b)
			#end for
		#end if
		output.append(checksum)
		try:
			self._port.write(output)
			self._responses.update({command: self._MSPResponse()})
		except Exception:
			return False
		return True
	#end def _sendCommand

	def _waitForResponse(self, command):
		if (self._responses.has_key(command)):
			startTime = time()
			while True:
				if self._responses[command].finished:
					return True
				if (time() - startTime > self.responseTimeout):
					return False
				sleep(0)
			#end while
		else:
			return False
	#end def _waitForResponse

	def _sendAndWait(self, command, data=None):
		if (self._sendCommand(command, data)):
			return self._waitForResponse(command)
		else:
			return False
	#end def _sendAndWait

	def _sendAndGet(self, command, expectedSize=None):
		if self._sendAndWait(command):
			rdata = self._responses[command].data
			del self._responses[command]
			if (expectedSize is not None):
				if (len(rdata) == expectedSize):
					return rdata
				else:
					return None
			else:
				return rdata
			#end if
		else:
			return None
		#end if
	#end def _sendAndGet

# get* methods #################################################################################
	def getIdent(self):
		"""Get identifying information from the device

		Returns:
			dict:
				{
					"version": (int)
					"type": (int)
				}
			See MULTITYPENAMES for a string representation of 'type'
		"""
		mspVersion = 0
		quadType = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_IDENT, 7)
		if rdata:
			mspVersion = rdata[0]
			quadType = rdata[1]
		#end if
		return {"version":mspVersion, "type":quadType}
	#end def getIdent
	
	def getAttitude(self):
		"""Get attitude (orientation) data from the device

		Returns:
			dict
			{
				"angx": (int)
				"angy": (int)
				"heading": (int)
			}
		"""
		angx = 0
		angy = 0
		heading = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_ATTITUDE, 6)
		if rdata:
			angx = self._toInt16(rdata[0:2])
			angy = self._toInt16(rdata[2:4])
			heading = self._toInt16(rdata[4:6])
		#end if
		return {"angx":angx, "angy":angy, "heading":heading}
	#end def getAttitude

	def getIMU(self):
		"""Get raw IMU data from the device

		Returns:
			dict
			{
				"accx": (int)
				"accy": (int)
				"accz": (int)
				"gyrx": (int)
				"gyry": (int)
				"gyrz": (int)
				"magx": (int)
				"magy": (int)
				"magz": (int)
			}
			Values depend on the sensor that is installed, and will be zero for any sensor that is not available
		"""
		acc = [0,0,0]
		gyr = [0,0,0]
		mag = [0,0,0]
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_RAW_IMU, 18)
		if rdata:
			acc[0] = self._toInt16(rdata[0:2])
			acc[1] = self._toInt16(rdata[2:4])
			acc[2] = self._toInt16(rdata[4:6])
			gyr[0] = self._toInt16(rdata[6:8])
			gyr[1] = self._toInt16(rdata[8:10])
			gyr[2] = self._toInt16(rdata[10:12])
			mag[0] = self._toInt16(rdata[12:14])
			mag[1] = self._toInt16(rdata[14:16])
			mag[2] = self._toInt16(rdata[16:18])
		#end if
		return {"accx":acc[0], "accy":acc[1], "accz":acc[2],
				"gyrx":gyr[0], "gyry":gyr[1], "gyrz":gyr[2],
				"magx":mag[0], "magy":mag[1], "magz":mag[2]}
	#end def getIMU

	def getRC(self):
		"""Get current RC data from the device

		Returns:
			dict

			{
				"pitch": (int)
				"roll": (int)
				"yaw": (int)
				"throttle": (int)
				"aux1": (int)
				"aux2": (int)
				"aux3": (int)
				"aux4": (int)
			}

		Note: These are the PWM values being used to compute the signal to be sent to the motor. In CleanFlight,
			they should be between 1000 and 2000.
		"""
		pitch = 0
		roll = 0
		yaw = 0
		throttle = 0
		aux = [0,0,0,0]
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_RC)
		if rdata:
			if (len(rdata) >= 8):
				pitch = self._toUInt16(rdata[0:2])
				roll = self._toUInt16(rdata[2:4])
				yaw = self._toUInt16(rdata[4:6])
				throttle = self._toUInt16(rdata[6:8])
				for i in range(0, 3):
					if (len(rdata) >= (10 + 2 * i)):
						aux[i] = self._toUInt16(rdata[8+2*i:10+2*i])
				#end for
			#end if
		#end if
		return {"pitch":pitch, "roll":roll, "yaw":yaw, "throttle":throttle,
				"aux1":aux[0], "aux2":aux[1], "aux3":aux[2], "aux4":aux[3]}
	#end def getRC

	def getAnalog(self):
		"""Get analog sensor data from the device

		Returns:
			dict
			{
				"vbat": (int)
				"powermetersum": (int)
				"rssi": (int)
				"amperage": (int)
			}
		"""
		vbat = 0
		pms = 0
		rssi = 0
		amperage = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_ANALOG, 7)
		if rdata:
			vbat = rdata[0]
			pms = self._toUInt16(rdata[1:3])
			rssp = self._toUInt16(rdata[3:5])
			amperage = self._toUInt16(rdata[5:7])
		#end if
		return {"vbat":vbat, "powermetersum":pms, "rssi":rssi, "amperage":amperage}
	#end def getAnalog

	def getAltitude(self):
		"""Get current altitude of device

		Returns:
			dict
			{
				"altitude": (int)
				"vari": (int)
			}
		"""
		alt = 0
		vari = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_ALTITUDE, 6)
		if rdata:
			alt = self._toInt32(rdata[0:4])
			vari = self._toInt16(rdata[4:6])
		#end if
		return {"altitude":alt, "vari":vari}
	#end def getAltitude

	def getGPS(self):
		"""Get GPS coordinate and fix data from the device

		Returns:
			dict
			{
				"fix": (bool)
				"numsat": (int)
				"latitude": (int)
				"longitude": (int)
				"altitude": (int)
				"speed": (int)
				"course": (int)
			}
		"""
		gpsFix = False
		gpsNumSat = 0
		gpsLat = 0
		gpsLong = 0
		gpsAltitude = 0
		gpsSpeed = 0
		gpsCourse = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_RAW_GPS, 16)
		if rdata:
			gpsFix = True if (rdata[0] == 1) else False
			gpsNumSat = rdata[1]
			gpsLat = self._toUInt32(rdata[2:6])
			gpsLong = self._toUInt32(rdata[6:10])
			gpsAltitude = self._toUInt16(rdata[10:12])
			gpsSpeed = self._toUInt16(rdata[12:14])
			gpsCourse = self._toUInt16(rdata[14:16])
		#end if
		return {"fix":gpsFix, "numsat":gpsNumSat, "latitude":gpsLat, "longitude":gpsLong,
				"altitude":gpsAltitude, "speed":gpsSpeed, "course":gpsCourse}
	#end def getGPS

	def getStatus(self):
		"""Get status of the device

		Returns:
			dict
			{
				"cycletime": (int)
				"i2cerrorcount": (int)
				"sensor": (int)
				"flag": (int)
				"currentset": (int)
			}
		"""
		cycletime=0
		i2cerrorcount=0
		sensor=0
		flag=0
		currentset=0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_STATUS, 11)
		if rdata:
			cycletime = self._toUInt16(rdata[0:2])
			i2cerrorcount = self._toUInt16(rdata[2:4])
			sensor = self._toUInt16(rdata[4:6])
			flag = self._toUInt32(rdata[6:10])
			currentset = rdata[10]
		#end if
		return {"cycletime":cycletime, "i2cerrorcount":i2cerrorcount, "sensor":sensor, "flag":flag, "currentset":currentset}
	#end def getStatus

	def getMotors(self):
		"""Get current motor signal values from the device

		Returns:
			dict
			{
				"motor1": (int)
				"motor2": (int)
				"motor3": (int)
				"motor4": (int)
				"motor5": (int)
				"motor6": (int)
				"motor7": (int)
				"motor8": (int)
			}
		
		Note: These are the PWM values being sent to the ESCs by the flight controller, not the actual
			motor speeds. Motors that are not installed will have a value of zero.
		"""
		motors = [0,0,0,0,0,0,0,0]
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_MOTOR, 16)
		if rdata:
			for i in range(0,7):
				motors[i] = self._toUInt16(rdata[2*i:2*i+2])
			#end for
		#end if
		ret = {}
		for i in range(0,7):
			ret.update({"motor" + str(i+1):motors[i]})
		#end for
		return ret
	#end def getMotors

	def getBoxnames(self):
		"""Get the BOX name strings from the device

		Returns:
			dict
			{
				"boxnames": (str)
			}

		"boxnames" is a string list of the names, separated by ';'.
		"""
		boxNames = ""
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_BOXNAMES)
		if rdata:
			boxNames = "".join(map(chr, rdata))
		#end if
		return {"boxnames": boxNames}
	#end def getBoxnames

	def getModeRanges(self):
		"""Get mode ranges and channels from the device

		Returns:
			dict of dict
			{
				"ARM":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"ANGLE":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"HORIZON":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"BARO":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"Reserved":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"MAG":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"HEADFREE":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"HEADADJ":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"CAMSTAB":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"CAMTRIG":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"GPSHOME":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"GPSHOLD":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"PASSTHRU":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"BEEPERON":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"LEDMAX":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"LEDLOW":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"LLIGHTS":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"CALIB":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"GOV":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"OSD":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"TELEMETRY":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"AUTOTUNE":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
				"SONAR":
					{
						"channel": (int)
						"start": (int)
						"end": (int)
					}
			}

			Note: These ranges are an extension to MSP only used by CleanFlight as a replacement for BOX values.
				See http://shipow.github.io/cleanflight-web/docs/api/msp_extensions/ for more information.
		"""
		curID = 0
		auxChannel = 0
		rStart = 0
		rEnd = 0
		ret = {}
		#Fill in default values in case we don't get a response, or in case the response is incomplete
		for i in range(0, len(self._MODERANGENAMES)):
			ret.update({self._MODERANGENAMES[i]: {"channel":0, "start":0, "end":0}})
		#end for
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_MODE_RANGES)
		if rdata:
			for i in range(0, len(rdata), 4):
				curID = rdata[i]
				auxChannel = rdata[i+1]
				rStart = 900 + 25 * rdata[i+2]
				rEnd = 900 + 25 * rdata[i+3]
				ret.update({self.MSPModeRanges[curID]: {"channel":auxChannel, "start":rStart, "end":rEnd}})
			#end for
		#end if
		return ret
	#end def getModeRanges

	def getMisc(self):
		"""Get miscellaneous data from the device

		Returns:
			dict:
			{
				"powertrigger": (int)
				"minthrottle": (int)
				"maxthrottle": (int)
				"mincommand": (int)
				"failsafethrottle": (int)
				"armedtime": (int)
				"uptime": (int)
				"magdeclination": (int)
				"vbatscale": (int)
				"vbatwarn1": (int)
				"vbatwarn2": (int)
				"vbatcrit": (int)
			}
		"""
		powerTrigger = 0
		minThrottle = 0
		maxThrottle = 0
		minCommand = 0
		failsafeThrottle = 0
		armTime = 0
		lifeTime = 0
		magDeclination = 0
		vBatScale = 0
		vBatWarn1 = 0
		vBatWarn2 = 0
		vBatCrit = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_MISC, 22)
		if rdata:
			powerTrigger = self._toUInt16(rdata[0:2])
			minThrottle = self._toUInt16(rdata[2:4])
			maxThrottle = self._toUInt16(rdata[4:6])
			minCommand = self._toUInt16(rdata[6:8])
			failsafeThrottle = self._toUInt16(rdata[8:10])
			armTime = self._toUInt16(rdata[10:12])
			lifeTime = self._toUInt32(rdata[12:16])
			magDeclination = self._toUInt16(rdata[16:18])
			vBatScale = rdata[18]
			vBatWarn1 = rdata[19]
			vBatWarn2 = rdata[20]
			vBatCrit = rdata[21]
		#end if
		return {"powertrigger":powerTrigger, "minthrottle":minThrottle, "maxthrottle":maxThrottle, "mincommand":minCommand,
				"failsafethrottle":failsafeThrottle, "armedtime":armTime, "uptime":lifeTime, "magdeclination":magDeclination,
				"vbatscale":vBatScale, "vbatwarn1":vBatWarn1, "vbatwarn2":vBatWarn2, "vbatcrit":vBatCrit}
	#end def getMisc
	
	def getDistanceToHome(self):
		"""Get the distance and heading from current position to saved home location.

		Returns:
			dict
			{
				"distance": (int)
				"heading": (int)
			}

		Notes:
			Document says heading is +/- 180 degrees, but data type is UINT16. Need to test this to see what it actually is
		"""
		distance = 0
		heading = 0
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_COMP_GPS, 5)
		if rdata:
			distance = self._toUInt16(rdata[0:2])
			heading = self._toUInt16(rdata[2:4])
		#end if
		return {"distance":distance, "heading":heading}
	#end def getDistanceToHome

	def getServos(self):
		"""Get current servo signal value from the device

		Returns:
			dict
			{
				"servo1": (int)
				"servo2": (int)
				"servo3": (int)
				"servo4": (int)
				"servo5": (int)
				"servo6": (int)
				"servo7": (int)
				"servo8": (int)
			}
		
		Note: These are the PWM values being sent to the servos by the flight controller, not the actual
			servo positions. Servos that are not configured will have a value of zero.

		"""
		servos = [0,0,0,0,0,0,0,0]
		rdata = self._sendAndGet(self._MSPCOMMANDS.MSP_SERVO, 16)
		if rdata:
			for i in range(0, 8):
				servos[i] = self._toUInt16(rdata[2*i, 2*i+2])
			#end for
		#end if
		ret = {}
		for i in range(0, 8):
			ret.update({"servo" + str(i+1):servos[i]})
		#end for
		return ret
	#end def getServos

# set* methods #################################################################################
	def setRC(self, values):
		"""Sends new RC values to the device.

		Args:
			values (dict): New RC values, containing values for one or more of the RC channels.
				Available channels are: "pitch", "yaw", "roll", "throttle", "aux1", "aux2",
				"aux3", and "aux4". Values not specified will be assumed to be zero.

		Returns:
			bool: True if successful, False otherwise

		Notes:
			This method does not get the current values for any channel before setting them.
			If you want to change a couple of values and keep the rest the same, call getRC()
			first, modify the values it returns, and pass that to setRC.
		"""
		data = bytearray()
		throttle = 0
		pitch = 0
		yaw = 0
		roll = 0
		aux = [0,0,0,0]
		if isinstance(values, dict):
			if values.has_key("pitch"):
				pitch = values["pitch"]
			if values.has_key("roll"):
				roll = values["roll"]
			if values.has_key("yaw"):
				yaw = values["yaw"]
			if values.has_key("throttle"):
				throttle = values["throttle"]
			for i in range(1,4):
				if values.has_key("aux" + str(i)):
					aux[i-1] = values["aux" + str(i)]
			#end for
			r = self._fromInt16(roll)
			data.append(r[0])
			data.append(r[1])
			r = self._fromInt16(pitch)
			data.append(r[0])
			data.append(r[1])
			r = self._fromInt16(yaw)
			data.append(r[0])
			data.append(r[1])
			r = self._fromInt16(throttle)
			data.append(r[0])
			data.append(r[1])
			for i in range(0,3):
				r = self._fromInt16(aux[i])
				data.append(r[0])
				data.append(r[1])
			#end for
			return self._sendAndWait(self._MSPCOMMANDS.MSP_SET_RAW_RC, data)
		else:
			return False
	#end def setRC

	def setThrottle(self, value):
		"""Set throttle to a new value.

		Args:
			value (int): The new throttle value

		Returns:
			bool: True if successful, False otherwise
		"""
		rc = self.getRC()
		rc["throttle"] = value
		return self.setRC(rc)
	#end def setThrottle

	def setAux(self, channel, value):
		"""Set specified aux channel to a new value.

		Args:
			channel (int): The channel to set (1 to 4)
			value (int): The new throttle value

		Returns:
			bool: True if successful, False otherwise
		"""
		if channel in range(1,4):
			rc = self.getRC()
			rc["aux" + str(channel)] = value
			return self.setRC(rc)
		else:
			return False
		#end if
	#def setAux

	def setHeading(self, value):
		"""Set a new heading to follow.

		Args:
			value (int): The new heading

		Returns:
			bool: True if successful, False otherwise
		"""
		data = bytearray()
		r = self._fromInt16(value)
		data.append(r[0])
		data.append(r[1])
		return self._sendAndWait(self._MSPCOMMANDS.MSP_SET_HEAD, data)
	#end def setHeading

	def setAccCalibration(self):
		"""Calibrate the device's accelerometer.

		Args:
			None

		Returns:
			True if successful, False otherwise

		Notes:
			Make sure the device is on a flat, level surface when performing calibration
		"""
		return self._sendAndWait(self._MSPCOMMANDS.MSP_ACC_CALIBRATION)
	#end def setAccCalibration

	def setMagCalibration(self):
		"""Calibrate the device's magnetrometer.

		Args:
			None

		Returns:
			True if successful, False otherwise

		Notes:
			Make sure the device is on a flat, level surface when performing calibration
		"""

		return self._sendAndWait(self._MSPCOMMANDS.MSP_MAG_CALIBRATION)
	#end def setMagCalibration

	def setMisc(self, powerTrigger, minThrottle, failsafeThrottle, magDeclination, vBatScale, vBatWarn1, vBatWarn2, vBatCrit):
		"""Set miscellaneous device data.

		Args:
			powerTrigger (int): Don't know what this does
			minthrottle (int): Don't know what this does either
			failsafeThrottle (int): Throttle level when failsafe mode is activated
			magDeclination (int): variation between magnetic north and true north
			vBatScale (int): Scale used for battery voltage monitoring
			vBatWarn1 (int): First battery level warning threshold
			vBatWarn2 (int): Second battery level warning threshold
			vBatCrit (int): Critical battery level threshold

		Returns:
			bool: True on success, False otherwise
		"""
		data = bytearray()
		r = self._fromUInt16(powerTrigger)
		data.append(r[0]);data.append(r[1])
		r = self._fromUInt16(minThrottle)
		data.append(r[0]);data.append(r[1])
		#maxthrottle not used, padding added
		data.append(0);data.append(0)
		#mincommand not used, padding added
		data.append(0);data.append(0)
		r = self._fromUInt16(failsafeThrottle)
		data.append(r[0]);data.append(r[1])
		#armedtime not used, padding added
		data.append(0);data.append(0)
		#uptime not used, padding added
		data.append(0);data.append(0);data.append(0);data.append(0)
		r = self._fromUInt16(magDeclination)
		data.append(r[0]);data.append(r[1])
		data.append(vBatScale and 0xff)
		data.append(vBatWarn1 and 0xff)
		data.append(vBatWarn2 and 0xff)
		data.append(vBatCrit and 0xff)
		return self._sendAndWait(self._MSPCOMMANDS.MSP_SET_MISC, data)
	#end def setMisc