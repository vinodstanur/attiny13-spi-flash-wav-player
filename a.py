import serial, sys,time

ser=serial.Serial()
ser.timeout=2
ser.port='/dev/ttyUSB0'
ser.baudrate=115200
ser.open()
time.sleep(0.1)
ser.flushOutput()
ser.flushInput()

count = 0

ser.write('o')
time.sleep(0.01)
ser.write('k')

a = ser.readline()
if a == 'start\n':
	print "got start!\n"

time.sleep(0.01)

while 1:
	for i in range(256):
		time.sleep(0.001)
		ser.write('A')
	time.sleep(0.01)
	d = ser.read()
	if d == 'x':
		print "got x as ack!"
	else:
		print "no ack :("
		sys.exit(0)

	ser.write('z')
