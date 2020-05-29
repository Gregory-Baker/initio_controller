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
import rospy
# from geometry_msgs.msg import Twist

# Set GPIO Modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

def _clip(value, minimum, maximum):
  if value < minimum:
    return minimum
  elif value > maximum:
    return maximum
  return value


class Motor:
  def __init__(self, motor_pins):
    
    self.motor_pins = motor_pins

    for pin in self.motor_pins:
      GPIO.setup(pin, GPIO.OUT)
      GPIO.output(pin, 0)

    self.seq = [[1,0,0,1],
                [1,0,0,0],
                [1,1,0,0],
                [0,1,0,0],
                [0,1,1,0],
                [0,0,1,0],
                [0,0,1,1],
                [0,0,0,1]]

    self.step_count = 8
    self.cur_step = 0                 
    self.speed_percent = 0  # -100 -> 100
    self.direction = 0
  
    self.delay_min = 1.0    # ms
    self.delay_max = 6.0    # ms

    f = threading.Thread(target = self.move)
    f.daemon = True
    f.start()

  def cleanup(self):
    self.speed_percent = 0
    GPIO.cleanup()

  def cycle(self, index, direction, maximum):
    index += direction
    if (index == maximum and direction == 1):
      return 0
    if (index < 0):
      return (maximum-1) 
    return index

  def speed_to_delay(self, speed_percent):
    speed = _clip(abs(speed_percent), 0, 100)
    delay = self.delay_max - (self.delay_max - self.delay_min)*(speed/100)
    return delay

  def step(self):
    self.cur_step = self.cycle(self.cur_step, self.direction, self.step_count)

    for i in range(4):
      GPIO.output(self.motor_pins[i], self.seq[self.cur_step][i])

  def get_motor_direction(self):
    if (self.speed_percent < 0):
      return -1
    elif (self.speed_percent > 0):
      return 1
    return 0
  
  def set_speed(self, speed):
    self.speed_percent = speed
    
  def move(self):
    while True:
      delay = self.speed_to_delay(self.speed_percent)
      self.direction = self.get_motor_direction()
      self.step()
      time.sleep(float(delay)/1000)

class Driver:
  
  
      
if __name__ == '__main__':
  desbot = Motor([33,32,31,29])
  print "Could I kindly touch you for a speed between -100 and 100"
  speed = int(input())
  desbot.set_speed(speed)
  time.sleep(5)
  desbot.cleanup()

