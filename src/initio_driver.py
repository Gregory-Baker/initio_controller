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
from geometry_msgs.msg import Twist

# Set GPIO Modes


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

  # Motor pins is [[motor_pins_lf], [motor_pins_lf], [motor_pins_lf], [motor_pins_lf]]
  def __init__(self, motor_pins):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    self.motor_lf = Motor(motor_pins[0])
    self.motor_lb = Motor(motor_pins[1])
    self.motor_rf = Motor(motor_pins[2])
    self.motor_rb = Motor(motor_pins[3])

    self.motors = [self.motor_lf, self.motor_lb, self.motor_rf, self.motor_rb]

    self.motors_left = [self.motor_lf, self.motor_lb]
    self.motors_right = [self.motor_rf, self.motor_rb]

    self.speed = 100


  def forward(self):
    for motor in self.motors:
      motor.set_speed(self.speed)

  def reverse(self):
    for motor in self.motors:
      motor.set_speed(self.speed)

  def left_turn(self):
    for motor in self.motors_left:
      motor.set_speed(-self.speed)
    for motor in self.motors_right:
      motor.set_speed(self.speed)

  def right_turn(self):
    for motor in self.motors_left:
      motor.set_speed(self.speed)
    for motor in self.motors_right:
      motor.set_speed(-self.speed)

  def stop(self):
    for motor in self.motors:
      motor.set_speed(0)

  def set_speed(self, speed):
    if (0 < speed < 100):
      self.speed = speed
    else:
      print "Speed value must be between 0 - 100"

  def increase_speed(self):
    if (self.speed >= 90):
      self.speed = 100
    elif (0 <= self.speed < 90):
      self.speed += 10
    else:
      print "Speed outside 0 - 100 range"
    print "Speed: " + self.speed

  def decrease_speed(self):
    if (self.speed <= 10):
      self.speed = 0
    elif (10 < self.speed < 100):
      self.speed -= 10
    else:
      print "Speed outside 0 - 100 range"
    print "Speed: " + self.speed

  def cleanup(self, speed):
    for motor in self.motors:
      self.stop()
      motor.cleanup()
  
      
if __name__ == '__main__':
  lf = [16,18,22,7]
  lb = [15,13,12,11]
  rf = [33,32,31,29]
  rb = [38,37,36,35]
  desbot = Driver([lf, lb, rf, rb])
  print "Motor test"
  duration = 3
  desbot.forward
  time.sleep(duration)
  desbot.reverse
  time.sleep(duration)
  desbot.turn_left
  time.sleep(duration)
  desbot.turn_right
  time.sleep(duration)
  desbot.cleanup()

