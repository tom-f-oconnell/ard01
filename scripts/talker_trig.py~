#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from ctypes import *
import rospy
import time, sys
import datetime
import shutil
import Phidgets.Devices.Analog
from std_msgs.msg import String

# Define time stamp for filename
def timeStamped(fname, fmt='vid_%Y-%m-%d-%H-%M-%S_{fname}'):
    return datetime.datetime.now().strftime(fmt).format(fname=fname)        

# Define file moving
def move(src, dest):
    shutil.move(src,dest)

# Set filename parameters
filename = timeStamped('timeStamps.txt')
directory = '/home/scientist/pavan/ROSvideo/data/'

# prepare analog out
AOCH = 0
A1CH = 1
A2CH = 2
analog = Phidgets.Devices.Analog.Analog()
analog.openPhidget()
analog.waitForAttach(10000)
analog.setEnabled(AOCH, True)
analog.setEnabled(A1CH, True)
analog.setEnabled(A2CH, True)

analog.setVoltage(AOCH, 0.)
analog.setVoltage(A1CH, 0.)
analog.setVoltage(A2CH, 0.)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    f=open(filename,'w')
    analog.setVoltage(A2CH, 5.) # step up
    time.sleep(2.0) # just some time for Scope to start scanning

    analog.setVoltage(A1CH, 10.) # step up
    while not rospy.is_shutdown():
        pulse_time = "pulse %s" % rospy.get_time()
        rospy.loginfo(pulse_time)
        pub.publish(pulse_time)
        analog.setVoltage(AOCH, 2.) # step up
        analog.setVoltage(AOCH, 0.) # step down
        f.write('{0}\n'.format(datetime.datetime.now()))
        rate.sleep()

    analog.setVoltage(A1CH, 0.) # step down
    f.close()
    analog.setVoltage(A2CH, 0.) # step down
    analog.setEnabled(AOCH, False)
    analog.setEnabled(A1CH, False)
    analog.setEnabled(A2CH, False)
    
    time.sleep(.5)
    analog.closePhidget()

    # Move time pulse data to data folder
    oldName = '/home/scientist/.ros/' + filename
    newName = directory + filename
    move(oldName, newName)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
