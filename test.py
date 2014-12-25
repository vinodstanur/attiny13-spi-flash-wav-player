import serial,time, sys

ser=serial.Serial()
ser.timeout=2
ser.port='/dev/ttyUSB0'
ser.baudrate=115200
ser.open()

time.sleep(1)
ser.flushInput()
ser.flushOutput()

dat = 0
count = 0

while 1:
	ser.write(chr(dat))
	if(ord(ser.read()) != dat):
	 	print "error!"
		sys.exit(0)
	dat = dat + 1
	if dat >= 256:
		dat = 0
	
	
	count = count + 1
	if(count % 1000 == 0):
		print str(count)
