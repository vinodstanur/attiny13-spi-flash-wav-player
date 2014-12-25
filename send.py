import serial, time, sys
import os
#initialize serial port
ser=serial.Serial()
ser.timeout=2
ser.port='/dev/ttyUSB0'
ser.baudrate=115200
ser.open()
ser.flushOutput()
ser.flushInput()

#open and read wave file as raw binary
f=open("s.wav",'rb')
a=f.read()
f.close()

#initiate handshake with attiny13 via bitbanged UART 
ser.write('o')
time.sleep(0.01);
ser.write('k')
time.sleep(0.01);

#read response from attiny13
d = ser.readline()
time.sleep(0.1)
if(d == "start\n"):
	print "Got start!\n"
else:
	print "error: didn't received start!\n"
	sys.exit(0)

#use first 512KB of song (if > 512KB)
if(len(a) > 512*1024):
	a = a[:512*1024]

#zero pad the file to make it 256 bytes aligned to match the page size of spi flash
ln = len(a)
rem = ln % 256
for i in range (256-rem):
	a = a + chr(0)
ln = ln + 256 - rem

#stream 256 bytes packet with proper handshaking 
while ln:
	s = a[:256]
	a = a[256:]
	ln = ln - 256
	print "remaining bytes to write = " + str(ln) 
	
	time.sleep(0.001)
	for i in range(256):
		time.sleep(0.0005)
		ser.write(s[i])

	print "waiting for ack\n"
	d = ser.read()
	if d == "x":
		print "ack received!\n"
	else:
		print "ack error after sending page\n"
		if d == '':
			print "null received!"
		sys.exit(0)
	ser.write("z")

print "Write Successful! Joly :) \n"

