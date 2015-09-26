from serial import Serial
from threading import Thread, Event
from time import sleep, time
import struct

class MultiWii(object):
	__VERSION__ = "0.0.2"
	__AUTHOR__ = "Jonathan Dean (ke4ukz@gmx.com)"
	#Instance variables:
	#	_port: serial.Serial object
	#	_monitorThread: threading.Thread object that monitors the incoming serial data
	#	_exitNow: threading.Event object that is set when the thread should exit
	#	responses: dict of {command: MSPResponse} used to store responses
	#	responseTimeout: number of seconds to wait for a response to a command (defaults to 3)

	MULTITYPENAMES = {0:"Unknown", 1:"TRI", 2:"QUADP", 3:"QUADX", 4:"BI", 5:"GIMBAL", 6:"Y6", 7:"HEX6", 8:"FLYING_WING", 9:"Y4", 10:"HEX6X", 11:"OCTOX8", 12:"OCTOFLATX", 13:"OCTOFLATP", 14:"AIRPLANE", 15:"HELI_120_CCPM", 16:"HELI_90_DEG", 17:"VTAIL4", 18:"HEX6H", 19:"PPM_TO_SERVO", 20:"DUALCOPTER", 21:"SINGLECOPTER"}

	class MSPCOMMMANDS:
		MSP_NULL = 0
		MSP_MODE_RANGES = 34
		MSP_SET_MODE_RANGE = 35
		MSP_ADJUSTMENT_RANGES = 52
		MSP_SET_ADJUSTMENT_RANGE = 53
		MSP_IDENT = 100
		MSP_RAW_IMU = 102
		MSP_MOTOR = 104
		MSP_RC = 105
		MSP_RAW_GPS = 106
		MSP_ATTITUDE = 108
		MSP_ALTITUDE = 109
		MSP_ANALOG = 110
		MSP_BOX = 113
		MSP_BOXNAMES = 116
		MSP_BOXIDS = 119
		MSP_SET_RAW_RC = 200
	#end class MSPCOMMANDS

	class MSPSTATES:
		IDLE = 0
		HEADER_START = 1
		HEADER_M = 2
		HEADER_ARROW = 3
		HEADER_SIZE = 4
		HEADER_CMD = 5
	#end class MSPSTATES

	class MSPResponse:
		def __init__(self):
			self.finished = False
			self.data = []
		#end def __init__
	#end class MSPResponse

	def __init__(self):
		self._port = Serial()
		self._monitorThread = Thread(target=self._monitorSerialPort)
		self._exitNow = Event()
		self.responses = {}
		self.responseTimeout = 3
	#end def __init__

	def _toInt16(self, data):
		if (len(data) == 2):
			return struct.unpack("@h", struct.pack("<BB", data[0], data[1]))
		else:
			return None
	#end def _toInt16

	def _toUInt16(self, data):
		if (len(data) == 2):
			return struct.unpack("@H", struct.pack("<BB", data[0], data[1]))
		else:
			return None
	#end def _toUInt16

	def _toUInt32(self, data):
		if (len(data) == 4):
			return struct.unpack("@I", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))
		else:
			return None
	#end def _toUInt32

	def _monitorSerialPort(self):
		state = self.MSPSTATES.IDLE
		data = bytearray()
		dataSize = 0
		dataChecksum = 0
		command = self.MSPCOMMMANDS.MSP_NULL
		while (not self._exitNow.isSet()):
			if (self._port.inWaiting() > 0):
				inByte = ord(self._port.read())
				if (state == self.MSPSTATES.IDLE):
					state = self.MSPSTATES.HEADER_START if (inByte==36) else self.MSPSTATES.IDLE #chr(36)=='$'
				elif (state == self.MSPSTATES.HEADER_START):
					state = self.MSPSTATES.HEADER_M if (inByte==77) else self.MSPSTATES.IDLE #chr(77)=='M'
				elif (state == self.MSPSTATES.HEADER_M):
					state = self.MSPSTATES.HEADER_ARROW if (inByte==62) else self.MSPSTATES.IDLE #chr(62)=='>'
				elif (state == self.MSPSTATES.HEADER_ARROW):
					dataSize = inByte
					data = bytearray()
					dataChecksum = inByte
					state = self.MSPSTATES.HEADER_SIZE
				elif (state == self.MSPSTATES.HEADER_SIZE):
					command = inByte
					dataChecksum = (dataChecksum ^ inByte)
					state = self.MSPSTATES.HEADER_CMD
				elif (state == self.MSPSTATES.HEADER_CMD) and (len(data) < dataSize):
					data.append(inByte)
					dataChecksum = (dataChecksum ^ inByte)
				elif (state == self.MSPSTATES.HEADER_CMD) and (len(data) >= dataSize):
					if (dataChecksum == inByte):
						#Good command, do something with it
						self._processCommand(command, data)
					else:
						#Bad checksum
						pass
					state = self.MSPSTATES.IDLE
					#end if
				#end if
			else:
				sleep(0)
			#end if
		#end while
		self._port.close()
	#end def _monitorSerialPort

	def _processCommand(self, command, data):
		if (self.responses.has_key(command)):
			self.responses[command].data = data
			self.responses[command].finished = True
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
			self.responses.update({command: self.MSPResponse()})
		except:
			return False
		return True
	#end def _sendCommand

	def _waitForResponse(self, command):
		if (self.responses.has_key(command)):
			startTime = time()
			while True:
				if (self.responses[command].finished == True):
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

	def getIdent(self):
		mspVersion = 0
		quadType = 0
		if (self._sendAndWait(self.MSPCOMMMANDS.MSP_IDENT)):
			rdata = self.responses[self.MSPCOMMMANDS.MSP_IDENT].data
			if (len(rdata) == 7):
				mspVersion = rdata[0]
				quadType = rdata[1]
			del self.responses[self.MSPCOMMMANDS.MSP_IDENT]
		#end if
		return {"version":mspVersion, "type":quadType}
	#end def getIdent
	
	def getAttitude(self):
		angx = 0
		angy = 0
		heading = 0
		if (self._sendAndWait(self.MSPCOMMMANDS.MSP_ATTITUDE)):
			rdata = self.responses[self.MSPCOMMMANDS.MSP_ATTITUDE].data
			if (len(rdata) == 6):
				angx = self._toInt16(rdata[0:2])
				angy = self._toInt16(rdata[2:4])
				heading = self._toInt16(rdata[4:6])
			del self.responses[self.MSPCOMMMANDS.MSP_ATTITUDE]
		#end if
		return {"angx":angx, "angy":angy, "heading":heading}
	#end def getAttitude

	def disconnect(self):
		self._exitNow.set()
		self._monitorThread.join()
	#end def disconnect

	def connect(self, portName, baudRate):
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