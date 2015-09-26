from serial import Serial
from threading import Thread, Event
import time

class MultiWii(object):
	__VERSION__ = "0.0.1"
	__AUTHOR__ = "Jonathan Dean (ke4ukz@gmx.com)"
	#Instance variables:
	#	_port: serial.Serial object
	#	_monitorThread: threading.Thread object that monitors the incoming serial data
	#	_exitNow: threading.Event object that is set when the thread should exit
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
	#end def __init__

	def _monitorSerialPort(self):
		print("Thread running")
		while not self._exitNow.isSet():
			for i in range(1,100):
				pass
			print("running...")
		print("Thread finished")
	#end def _monitorSerialPort

	def disconnect(self):
		print("Trying to stop thread...")
		self._exitNow.set()
		self._monitorThread.join()

	def connect(self, portName, baudRate):
		try:
			#self._port.setPort(portName)
			#self._port.setBaudrate(baudRate)
			#self._port.open()
			print("Starting thread...")
			self._monitorThread.start()
			return True
		except Exception as ex:
			print("Error starting thread: " + str(ex))
			return False

