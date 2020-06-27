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
import Jetson.GPIO as GPIO, sys, threading, time, os, subprocess, math
import rospy
from geometry_msgs.msg import Twist


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

    # No. halfsteps per rotation for 28BYJ-48 Stepper Motor (32*64*2)
    self.num_steps_per_rot = 4096

    # Velocity and direction variables
    self.ang_speed = 0
    self.direction = 0

    # Flag to kill motor thread
    self.end_flag = False

    # Setup motor thread
    f = threading.Thread(target = self.move)
    f.daemon = True
    f.start()

  # Shutdown motor thread and clean GPIO
  def cleanup(self):
    self.end_flag=True
    time.sleep(0.1)
    GPIO.output(self.motor_pins, GPIO.LOW)
    GPIO.cleanup(self.motor_pins)

  # Get next step index
  def cycle(self, index, direction, maximum):
    index += direction
    if (index == maximum and direction == 1):
      return 0
    if (index < 0):
      return (maximum-1)
    return index

  # Converts angular speed to delay
  def ang_speed_to_delay(self, ang_speed):
    if (ang_speed != 0):
      return abs((2.0*math.pi)/(ang_speed*self.num_steps_per_rot))
    return 0.001

  # Method to step motor
  def step(self):
    self.cur_step = self.cycle(self.cur_step, self.direction, self.step_count)
    for i in range(4):
      GPIO.output(self.motor_pins[i], self.seq[self.cur_step][i])

  # Get motor direction
  def get_motor_direction(self):
    if (self.ang_speed < 0):
      return -1
    elif (self.ang_speed > 0):
      return 1
    return 0

  # Set angular speed
  def set_ang_speed(self, ang_speed):
    self.ang_speed = ang_speed

  # Move motor method (thread) - responds to changes
  def move(self):
    while True:
      delay = self.ang_speed_to_delay(self.ang_speed)
      self.direction = self.get_motor_direction()
      self.step()
      time.sleep(float(delay))

      if self.end_flag:
        self.end_flag = False
        break

class Driver:

  def __init__(self):
    # Set GPIO Mode
    GPIO.setmode(GPIO.BOARD)
    # GPIO.setwarnings(False)

    # Set GPIO pins of stepper motors
    self._motor_lf = Motor([16,18,22,7])
    self._motor_lb = Motor([15,13,12,11])
    self._motor_rf = Motor([33,32,31,29])
    self._motor_rb = Motor([38,37,36,35])

    # Assign motors to lists for iteration later
    self.motors = [self._motor_lf, self._motor_lb, self._motor_rf, self._motor_rb]
    self.motors_left = [self._motor_lf, self._motor_lb]
    self.motors_right = [self._motor_rf, self._motor_rb]

    # Set motor speeds to zero initially
    self._left_ang_speed = 0
    self._right_ang_speed = 0

    # Variables for robot target speed
    self.linear_speed = 0
    self.angular_speed = 0

    rospy.init_node("initio_driver")

    # Get ros parameters
    self._last_received = rospy.get_time()
    self._timeout = rospy.get_param('~timeout', 2)
    self._rate = rospy.get_param('~rate', 10)
    self._max_motor_speed = rospy.get_param('~max_motor_speed', 2.0)
    self._wheel_radius = rospy.get_param('~wheel_radius', 0.0225)
    self._wheel_radius_multiplier = rospy.get_param('~wheel_radius_multiplier', 0.83)
    self._wheel_base = rospy.get_param('~wheel_base', 0.134)
    self._wheel_base_multiplier = rospy.get_param('~wheel_base_multiplier', 1.1)

    rospy.Subscriber('cmd_vel', Twist, self.velocity_received_callback)


  def velocity_received_callback(self, message):
    """Handle new velocity command message"""

    self._last_received = rospy.get_time()

    # extract linear and angular velocities from the message
    linear = message.linear.x
    angular = message.angular.z

    if (linear == self.linear_speed and angular == self.angular_speed):
      return
    else:
      self.linear_speed = linear
      self.angular_speed = angular

    # Calculate the wheel speeds in m/s
    left_speed = linear - angular*self._wheel_base*self._wheel_base_multiplier/2
    right_speed = linear + angular*self._wheel_base*self._wheel_base_multiplier/2

    # Calculate the motor speeds in rad/sec
    left_ang_speed = left_speed/(self._wheel_radius*self._wheel_radius_multiplier)
    right_ang_speed = right_speed/(self._wheel_radius*self._wheel_radius_multiplier)

    # Limit speed to between min/max speed of motor
    self._left_ang_speed = self.clip_speed(left_ang_speed, -self._max_motor_speed, self._max_motor_speed)
    self._right_ang_speed = self.clip_speed(right_ang_speed, -self._max_motor_speed, self._max_motor_speed)

    if (self._left_ang_speed != left_ang_speed):
      print "Left wheel speed limited to {} rad/sec".format(self._left_ang_speed)
    if (self._right_ang_speed != right_ang_speed):
      print "Right wheel speed limited to {} rad/sec".format(self._right_ang_speed)

  def run(self):
    """The control loop of the driver."""

    print("Running initio driver")

    rate = rospy.Rate(self._rate)

    while not rospy.is_shutdown():

      # Stop robot if no commands sent recently
      delay = rospy.get_time() - self._last_received

      if delay < self._timeout:
        for motor in self.motors_left:
	  motor.set_ang_speed(self._left_ang_speed)
        for motor in self.motors_right:
	  motor.set_ang_speed(self._right_ang_speed)
      else:
	for motor in self.motors:
	  motor.set_ang_speed(0)

      rate.sleep()

    print ("Exiting initio driver")


  def clip_speed(self, value, minimum, maximum):
    if value < minimum:
      return minimum
    elif value > maximum:
      return maximum
    return value


  def cleanup(self):
    for motor in self.motors:
     motor.end_flag = True 
     motor.cleanup()
    time.sleep(0.1)
    GPIO.cleanup()
    print "GPIO cleanup"

def main():
  driver = Driver()
  driver.run()
  driver.cleanup()



if __name__ == '__main__':
  main()

