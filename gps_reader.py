import serial
import time
import string
import pynmea2

while True:
	port="/dev/ttyACM0"
	ser=serial.Serial(port, baudrate=9600, timeout=0.5)
	#print("Serial connection successful")
	dataout=pynmea2.NMEAStreamReader()
	newdata=ser.readline().decode().strip()
	if newdata.find('GLL') > 0:
		msg = pynmea2.parse(newdata)
		msg = str(msg).split(",")
		if msg[6] == 'A':
			lat = msg[1]
			long = msg[3]
			print("lat=" + lat)
			print("long=" + long)
			#print(msg)
		else:
			print("GPS is not ready")


	#if newdata[0:6] == "$GPRMC,":
	#	newmsg = pynmea2.parse(newdata)
	#	lat = newmsg.latitude
	#	lng = newmsg.longitude
	#	gps = "Latitude=" + str(lat) +"and Longitude=" + str(lng)
	#	print(newmsg)


