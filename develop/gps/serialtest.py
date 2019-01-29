import serial

print "Starting program"
serialport=serial.Serial('/dev/ttyAMA0', baudrate=4800,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS
			)

while True:
	if serialport.inWaiting() > 0:
		data=serialport.readline()
		print data
