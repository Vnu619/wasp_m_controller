'''
		OrienBus: A Python library for interfacing oriental motors with modbus using minimalmodbus python driver

		Authors : "Phone Thiha Kyaw, Monisha, Thein Than Tun"
		Version : "1.0.0"
'''

########################################################################################
#
#   NOTES:

#   - Currently library only support profile 2 with default acceleration and deceleartions
#   - Future Versions will have different profiles and error message feedbacks
#
########################################################################################

import minimalmodbus

_WRITE_REGISTER = 125
_FEEDBACK_SPEED_REG_LOWER = 207
_FEEDBACK_SPEED_REG_UPPER = 206

_WRITE_REGISTER_SPEED = 1157
_FWD_DEC = 10
_REV_DEC = 18

_MIN_RPM = 80
_MAX_RPM = 3150

speed = 0

class ModBus(object):

	"""
		ModBus class for talking to instruments (slaves).
		Uses the minimalmodbus python library.
	Args:
		* port (str): The serial port name, for example ``/dev/ttyUSB0`` (Linux),
		  ``/dev/tty.usbserial`` (OS X) or ``COM4`` (Windows).
		* slaveaddress (int): Slave address in the range 1 to 247 (use decimal numbers,
		  not hex). Address 0 is for broadcast, and 248-255 are reserved.
	"""

	def __init__(self, _port, _slave_address):

		self._port = _port
		self._slave_address = _slave_address

		self.instrument = minimalmodbus.Instrument(self._port, self._slave_address)
		self.instrument.serial.baudrate = 115200
		self.instrument.serial.bytesize = 8
		self.instrument.serial.parity  = minimalmodbus.serial.PARITY_EVEN
		self.instrument.serial.stopbits = 1
		self.instrument.serial.timeout  = 0.035
		self.instrument.mode = minimalmodbus.MODE_RTU
		self.instrument.clear_buffers_before_each_transaction = True

		print("Successfully Connected to Slave Address {} ...".format(self._slave_address))

	def writeSpeed(self, speed):

		try:

			if (speed >= _MIN_RPM and speed <= _MAX_RPM):
				self.instrument.write_register(_WRITE_REGISTER_SPEED, speed)
				self.instrument.write_register(_WRITE_REGISTER, _FWD_DEC) # run motor forward with default acceleration

			elif (speed <= -_MIN_RPM and speed >= -_MAX_RPM):
				self.instrument.write_register(_WRITE_REGISTER_SPEED, -speed)
				self.instrument.write_register(_WRITE_REGISTER, _REV_DEC) # run motor backward with default acceleration

			else:
				# self.instrument.write_register(_WRITE_REGISTER, 2) # use this if wants to stop instant (not recommended)
				self.instrument.write_register(_WRITE_REGISTER, 34) # stop motor with default deceleartion
		except:
			pass

	def readSpeed(self):

		global speed
		try:
			speed = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER)-self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)

		except:
			print("Failed to read from instrument")

		return speed

class OrienBus(object):

	def __init__(self, _port):
		self._port = _port

		print("Connecting to port {} ...".format(self._port))

	def initialize(self, _slave_address):
		return ModBus(self._port, _slave_address)
