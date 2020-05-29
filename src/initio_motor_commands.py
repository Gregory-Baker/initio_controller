#!/usr/bin/python
#
# Python Module to control robot with 4 stepper motors using 4Tronix piStep2 controller
#
# Created by Greg Baker, May 2020
# Copyright Greg Baker, Bristol Robotics Laboratory
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================

# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os, subprocess

class InitioMotorCommands:
  
  # inputs: motor_pins = 4x4 array of motor pins [[RF], [RB], [LF], [LB]]
  def __init__(self, motor_pins):
    self.motors = motor_pins
    self.motors_right = [self.motors[0], self.motors[1]]
    self.motors_left = [self.motors[2], self.motors[3]]

    self.stepCount = 8
    self.seq = [[1,0,0,1],
                [1,0,0,0],
                [1,1,0,0],
                [0,1,0,0],
                [0,1,1,0],
                [0,0,1,0],
                [0,0,1,1],
                [0,0,0,1]]

    self.control = [0,0]

    self.delay = 1.0 		# ms
    self.stop_command = False

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    for motor in self.motors:
      for pin in motor:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)

    f = threading.Thread(target = self.motor_control_thread)
    f.daemon = True
    f.start()


  def cleanup(self):
    self.stop()
    GPIO.cleanup()


  def setStep(self, pins, values):
    for i in range(len(pins)):
      GPIO.output(pins[i], values[i])

  def speedToDelay(self, speed):
    if (speed == 0):
      delay = 0
    elif

    return 

# ------------------------------------------------------------------------

  def forward(self, steps, delay):
    for i in range(steps):
      for j in range(self.stepCount):
        for motor in self.motors:
          self.setStep(motor, self.seq[j])
        time.sleep(float(delay)/1000)

  def forward(self):
    self.control = [1,1]

  def forward(self, speed):
    self.control = [1,1]

# ------------------------------------------------------------------------

  def reverse(self, steps, delay):
    for i in range(steps):
      for j in reversed(range(self.stepCount)):
        for motor in self.motors:
          self.setStep(motor, self.seq[j])
        time.sleep(float(delay)/1000)

  def reverse(self):
    self.control = [-1,-1]

  def reverse(self, speed):
    self.control = [-1,-1]

# ------------------------------------------------------------------------


  def turn_left(self, steps, delay):
    for i in range(steps):
      for j in range(self.stepCount):
        for motor in self.motors_right:
          self.setStep(motor, self.seq[j])
        for motor in self.motors_left:
          self.setStep(motor, self.seq[self.stepCount-1-j])
        time.sleep(float(delay)/1000)

  def turn_left(self):
    self.control = [1,-1]

  def turn_left(self, speed):
    self.control = [1,1]

# ------------------------------------------------------------------------


  def turn_right(self, steps, delay):
    for i in range(steps):
      for j in range(self.stepCount):
        for motor in self.motors_right:
          self.setStep(motor, self.seq[j])
        for motor in self.motors_left:
          self.setStep(motor, self.seq[self.stepCount-1-j])
        time.sleep(float(delay)/1000)

  def turn_right(self, speed):
    self.control = [-1,1]


# ------------------------------------------------------------------------

  #inputs: delay (ms); control = 1X2 array of 1/0/-1, corresponding to 
  def motor_control_thread(self):
    while True:
      for j in range(self.stepCount):
        if (self.control[0] > 0):
          for motor in self.motors_right:
            self.setStep(motor, self.seq[j])
        elif (self.control[0] < 0):
          for motor in self.motors_right:
            self.setStep(motor, self.seq[self.stepCount-1-j])
        else:

          for motor in self.motors_right:
            self.setStep(motor, [0,0,0,0])

        if (self.control[1] > 0):
          for motor in self.motors_left:
            self.setStep(motor, self.seq[j])
        elif (self.control[1] < 0):
          for motor in self.motors_left:
            self.setStep(motor, self.seq[self.stepCount-1-j])
        else:
          for motor in self.motors_left:
            self.setStep(motor, [0,0,0,0])

        time.sleep(float(self.delay)/1000)
        
      if self.stop_command:
        self.stop_command = False
        break

  def stop(self):
    self.control = [0,0]

if __name__ == '__main__':
    
