#!/usr/bin/env python
#-----------------------------------------------------------------------
# Ball-tracking data acq. & time sync w/ imaging
# Author: Pavan Ramdya
# Date: 150908
# Modified:
#-----------------------------------------------------------------------

#Basic imports
import serial   
import rospy
import time     
import sys      
import datetime
import shutil
from Phidgets.Devices.Analog import Analog

print('STARTING OPTIC FLOW RECORDING SCRIPT')

# Define time stamp for filename
def timeStamped(fname, fmt='vid_%Y-%m-%d-%H-%M-%S_{fname}'):
    return datetime.datetime.now().strftime(fmt).format(fname=fname)        

# Set filename parameters
filename = timeStamped('opflow.txt')
# TODO make this modifiable as a ros parameter
directory = '/home/flyperson/pavan/ROSvideo/data/'

print('will save to ' + directory + filename)

# Define file moving
def move(src, dest):
    shutil.move(src,dest)

# Define Sensor class
class OpticFlowSensor(serial.Serial):
    def __init__(self, port='/dev/ttyACM0'):
        super(OpticFlowSensor,self).__init__(port=port,baudrate=115200) 
        print('Initializing sensor')
        time.sleep(3)   #required to give acquisition time to catch up
        print('sensor initialized')

    def start(self):
        self.write('r')

    def stop(self):
        self.write('s')

    def readData(self):
        dataList = []
        while self.inWaiting() > 0:
            rawData = self.readline()
            rawData = rawData.strip()
            rawData = rawData.split(',')
            try:
                dataList.append([int(x) for x in rawData])
            except:
                pass
        return dataList

# Set optic flow read-out parameters
port = '/dev/ttyACM0'
sensor = OpticFlowSensor(port)

# Main code
if __name__ == '__main__':
    # open the Phidget for outputing ball yaw as a voltage
    # may generate a Runtime error
    analog = Analog()

    # each of next two steps may generate a Phidget error
    analog.openPhidget()
    # TODO appropriate amount of time? is there lag before closed loop
    # starts?
    analog.waitForAttach(10000)
    analog.setEnabled(0, True)

    # should incur no pattern rotation (middle of 0-5v range)
    # might need a bias in PControl though
    bias = 2.5
    channel = 0 
    analog.setVoltage(channel, bias)

    sensor.start()
    f = open(filename,'w')

    # to correct differences between sensors output, as relates to yaw
    # or otherwise just averages two sensor outputs
    # can also control gain here if necessary before controller
    # RECORD THESE VALUES FOR ANY EXPERIMENTS RUN
    calib1 = 0.5
    calib2 = 0.5

    while not rospy.is_shutdown():
        data = sensor.readData()

        # limit on how fast I can/should update the Phidget?
        yaw1 = data[0]
        yaw2 = data[2]

        # TODO mind direction (+/- out)
        # yaw clockwise should induce counterclockwise panorama motion
        # and vice versa
        out = yaw1 * calib1 + yaw2 * calib2 + bias

        # constraining to [0,5] volts
        analog.setVoltage(channel, max(0.0, min(5.0,out)) )
        
        for vals in data:
            print(vals)
            try:
		# should be dx1, dy1, dx2, dy2, where sensors are {1,2}
                f.write('{0} {1} {2} {3}\t'.format(vals[0], vals[1], vals[2], vals[3]))
                f.write('{0}\n'.format(datetime.datetime.now()))
            except IndexError:
                pass

    # TODO close automatically after set recording time
    f.close()
    # Move time pulse data to data folder
    oldName = '/home/flyperson/.ros/' + filename
    newName = directory + filename
    move(oldName, newName)

print('done')
