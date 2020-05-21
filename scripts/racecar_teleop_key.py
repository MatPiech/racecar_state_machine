#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from os import system
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
import time

from autonomous_system import AutonomousSystem

SM = AutonomousSystem()

MAX_LIN_VEL = 1
MAX_ANG_VEL = 1

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

TACTIC = [0]


class Steering:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.is_collision = False

        rospy.init_node('racecar_teleop')
        self.joy_pub = rospy.Publisher('joy', Joy, queue_size=10)
        self.state_pub = rospy.Publisher('state', String, queue_size=10)

        rospy.Subscriber('collision', Bool, self.collision_callback)

    def collision_callback(self, data):
        self.is_collision = data.data

    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        vel = self.constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

        return vel

    def checkAngularLimitVelocity(self, vel):
        vel = self.constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        return vel

    def make_messge(self, control_linear_vel, control_angular_vel):
        joy = Joy()
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        joy.axes = [0.0, control_linear_vel, control_angular_vel, 0.0, 0.0, 0.0]

        self.joy_pub.publish(joy)

    def steerage(self, values, key):
        target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel = values

        if key == 'w':
            target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
        elif key == 's':
            target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
        elif key == 'a':
            target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
        elif key == 'd':
            target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
        elif key == ' ':
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0

        control_angular_vel = round(control_angular_vel, 2)
        target_angular_vel = round(target_angular_vel, 2)

        control_linear_vel = self.makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        control_angular_vel = self.makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))

        self.make_messge(control_linear_vel, control_angular_vel)

        return target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel
    
    def statep(self):
        system('clear')
        print("Racecar simulator console! "+'\033[93m'+"State: "+ str(SM.current_state.name) +'\033[0m')
        SM.get_state_parameters()

        state_msg = String()
        state_msg.data = SM.current_state.name
        self.state_pub.publish(state_msg)

    def manual_mode(self):
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        t = target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel
        
        while(1):
            self.statep()
            print("Sterowanie w,a,s,d; space by zatrzymac; q by wyjsc")
            print(self.vels(t[0], t[1]))

            key = self.getKey()
            
            if key in 'wsad ':
                t = self.steerage(t, key)
            elif key == 'q':
                SM.manual_off()
                break


        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def driving_mode(self):
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        t = target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel

        if TACTIC[0]==1:
            timeout = time.time() + 10
        elif TACTIC[0]==2:
            timeout = time.time() + 5
            timeout2 = time.time() + 15

        while True:
            self.statep()
            print('\033[94m'+'I am doing a mission!'+'\033[0m')
            print("0. Emegrancy mode")

            if self.is_collision == True:
                SM.driving_emergency()
                break

            key = self.getKey()
            if key == "0":
                SM.driving_emergency()
                break
            else:
                if TACTIC[0] == 1:
                    if time.time() > timeout:
                        SM.driving_finished()
                        break
                elif TACTIC[0] == 2:
                    if time.time() < timeout:
                        t = self.steerage(t, 'w')
                    elif time.time() < timeout2:
                        pass
                    else:
                        t = self.steerage(t, ' ')
                        SM.driving_finished()
                        break
                
    def emegrancy_mode(self):
        self.statep()    
        print('\033[91m'+'All systems switched off, vehicle stopped!'+'\033[0m')
        self.steerage((0,0,0,0), ' ')

        time.sleep(5)

        print('\033[91m'+'Confirm emegrancy mode!'+'\033[0m')
        print("0. Confirm")

        while True:
            key = self.getKey()
            if key == "0":
                SM.emergency_off()
                break

    def finished_mode(self):
        timeout = time.time() + 5
        while True:
            self.statep()
            print("0. Emegrancy mode")
            print('\033[94m'+'The mission has been completed!'+'\033[0m')
            print('\033[94m'+'Vehicle stopped!'+'\033[0m')

            if self.is_collision == True:
                SM.finished_emergency()
                break

            key = self.getKey()
            if key == "0":
                SM.finished_emergency()
                break
            else:
                if timeout<time.time():
                    SM.finished_off()
                    break
 
    def ready_mode(self):
        self.statep()    
        print('\033[91m'+'All systems switching on, vehicle starting!'+'\033[0m')
        time.sleep(5)

        while True:
            self.statep()
            print("1. Back")
            print("2. Dummy")
            print("3. Tactic: go straight")        
            print("0. Emegrancy mode")

            if self.is_collision == True:
                SM.ready_emergency()
                break

            key = self.getKey()
            if key == "1":
                SM.ready_off()
                break
            elif key == "2":
                TACTIC[0] = 1
                SM.ready_driving()
                break
            elif key == "3":
                TACTIC[0] = 2
                SM.ready_driving()
                break
            elif key == "0":
                SM.ready_emergency()
                break

    def off_mode(self):
        self.statep()
        print("1. Manual mode")
        print("2. Autonomous mode")
        print("0. Exit")
        TACTIC[0] = 0

        key = self.getKey()
        if key == "1":
            SM.off_manual()
        elif key == "2":
            SM.off_ready()
        elif key == "0":
            exit(0)

    def main(self):
        while True:
            if SM.current_state.value == 'as_off':
                self.off_mode()
            elif SM.current_state.value == 'manual_driving':
                self.manual_mode()
            elif SM.current_state.value == 'as_ready':
                self.ready_mode()
            elif SM.current_state.value == 'as_driving':
                self.driving_mode()
            elif SM.current_state.value == 'as_emergency':
                self.emegrancy_mode()
            elif SM.current_state.value == 'as_finished':
                self.finished_mode()


if __name__=="__main__":
    steer = Steering()
    steer.main()

        
        