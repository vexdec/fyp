#!/usr/bin/python

import serial
import pynmea2
import os
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class gps_node(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher = self.create_publisher(String, 'gps_data', 10)

        # Initialize serial communication
        self.port = "/dev/ttyACM0"  # Port number
        self.baudrate = 9600
        

        # Define a timer callback to read and publish GPS data
        self.timer = self.create_timer(1, self.read_gps_data)

    def read_gps_data(self):
        #open serial connection
        while True:
            with serial.Serial(self.port, self.baudrate, timeout = 0.5) as ser:
                #ser.flushInput()
                #ser.flushOutput()
                try: 
                    data  = ser.readline().decode('utf-8').strip()
                    # Process the GPS data
                    # (NMEA sentences are comma-separated values)
                    #gps_data = data.split(',')
                    #GPGLL_info = gps_data.find("$GPGLL,")
                    # Extract relevant information (example)
                    #latitude = gps_data[1]
                    #longitude = gps_data[3]
                    #speed = gps_data[7]

                    #print("Latitude:", latitude)
                    #print("Longitude:", longitude)
                    #print("Speed:", speed)
                    if(data.startswith("$GPGLL")):
                        msg = pynmea2.parse(data)
                        #print(pynmea2.parse(data))
                        #print(msg.latitude)
                        print(msg)

                    gps_msg = String()
                    gps_msg.data = data
                    self.publisher.publish(gps_msg)

                except KeyboardInterrupt:
                    # Close the serial port when finished
                    self.ser.close()

#main function
def main(arg=None):
    rclpy.init(args=arg) #Initialize tbhe ROS 2 Python client library
    gps = gps_node() #Create an instance of the SpeedPublisher class
    rclpy.spin(gps) #SPin the node until it is shut down
    gps.destroy_node() # Explicitly destroy the node
    rclpy.shutdown()  #Shutdown the ROS 2 Python client library

if __name__ == "__main__":
    main()  #Call the main function when the scxript is executed directly

