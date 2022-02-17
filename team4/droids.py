#!/usr/bin/python3

import sys
from telnetlib import FORWARD_X
import tty
import termios
from enum import IntEnum, unique
from turtle import forward
import RPi.GPIO as GPIO
from motor import Motor, MotorDirection
from sensors import HCSR04, HCSR04Event
from controller import PIDController, IntersectionType
from pins import RaspiPin, RaspiPinError
import time


class Droid:
    def __init__(self, battery_voltage=9.0, motor_voltage=6.0, warn=True,
                 speed=0.5):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(warn)
        self.speed = speed
        self.motors = Motor(battery_voltage, motor_voltage)

    def __del__(self):
        self._cleanup()

    def _set_led1(self, state):
        GPIO.output(RaspiPin.LED1_PIN, state)

    def _set_led2(self, state):
        GPIO.output(RaspiPin.LED2_PIN, state)

    def _set_oc1(self, state):
        GPIO.output(RaspiPin.OC1_PIN, state)

    def _set_oc2(self, state):
        GPIO.output(RaspiPin.OC2_PIN, state)

    def _cleanup(self):
        GPIO.cleanup()

    def forward(self):
        self.motors.forward(speed=self.speed)

    def backward(self):
        self.motors.backward(speed=self.speed)

    def stop(self):
        self.motors.stop()

    def right(self):
        self.motors.right(speed=self.speed)

    def left(self):
        self.motors.left(speed=self.speed)


class RoverKey(IntEnum):
    UP = 0
    DOWN = 1
    RIGHT = 2
    LEFT = 3
    STOP = 4


class RoverDroid(Droid):
    def __init__(self, battery_voltage=9.0, motor_voltage=6.0, warn=True,
                 speed=0.5):
        Droid.__init__(self, battery_voltage, motor_voltage, warn, speed)

    def _read_char(self):
        """Read chars from keyboard"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == '0x03':
            raise KeyboardInterrupt
        return ch

    def _read_key(self):
        """Converts keys from keyboard to directions to the droid"""
        c1 = self._read_char()
        if ord(c1) != 0x1b:
            # escape char
            return RoverKey.STOP
        c2 = self._read_char()
        if ord(c2) != 0x5b:
            # [ char
            raise KeyboardInterrupt
        c3 = self._read_char()
        return RoverKey(ord(c3) - 65)  # 0=Up, 1=Down, 2=Right, 3=Left arrows

    def run(self):
        """Continuously read keys stroke from the keyboard and applies
        directions to current droid"""
        try:
            print("Use the arrow keys to move the robot")
            print("Press CTRL-c to quit the program")
            while True:
                key = self._read_key()
                if key is RoverKey.UP:
                    print("ROVER", "FORWARD")
                    self.forward()
                elif key is RoverKey.DOWN:
                    print("ROVER", "BACKWARD")
                    self.backward()
                elif key is RoverKey.LEFT:
                    print("ROVER", "TURN LEFT")
                    self.left()
                elif key is RoverKey.RIGHT:
                    print("ROVER", "TURN RIGHT")
                    self.right()
                elif key is RoverKey.STOP:
                    print("ROVER", "STOP")
                    self.stop()
                else:
                    print("ROVER", "UNKNOWN VALUE:", key)
                    raise KeyboardInterrupt
        except KeyboardInterrupt:
            self._cleanup()


class Droid3Sensors(Droid):
    """Class to explore a maze with a controller to drives motors according to
    sensors"""

    def __init__(self, battery_voltage=9.0, motor_voltage=6.0, warn=True,
                 speed=0.5, low=15., high=50., target=20., KP=0.005, KD=0.0002,
                 KI=0.0, sample_time=0.001):
        Droid.__init__(self, battery_voltage, motor_voltage, warn, speed)
        print("Robot 3 Sensors with controller", "speed=%02f" % speed)
        self.left_sensor = HCSR04("LEFT", RaspiPin.ECHO_PIN_3,
                                  RaspiPin.TRIGGER_PIN_3, low=low, high=high)
        self.front_sensor = HCSR04("FRONT", RaspiPin.ECHO_PIN_1,
                                   RaspiPin.TRIGGER_PIN_1, low=2*low, high=high)
        self.right_sensor = HCSR04("RIGHT", RaspiPin.ECHO_PIN_2,
                                   RaspiPin.TRIGGER_PIN_2, low=low, high=high)
        self.controller = PIDController(self.motors, self.left_sensor,
                                        self.front_sensor, self.right_sensor,
                                        speed=speed, target=target, KP=KP,
                                        KD=KD, KI=KI, sample_time=sample_time)

    def explore(self):
        """Explores maze until the exit is found"""
        while True:
            """front_sensor_event, distance = self.controller.get_front_distance()
            right_sensor_event, distance = self.controller.get_right_distance()
            left_sensor_event, distance = self.controller.get_left_distance()

            if front_sensor_event is HCSR04Event.DANGER:
                print("DANGER in front! Turn!")
                self.right()
            else:
                self.forward()            

            if right_sensor_event is HCSR04Event.DANGER:
                print("DANGER at right! STOP!")
                self.stop()
            elif right_sensor_event is HCSR04Event.WALL:
                print("WALL DETECTED !")
            
            if left_sensor_event is HCSR04Event.DANGER:
                print("DANGER at right! STOP!")
                self.stop()"""
            
            front_sensor_event, front_distance = self.controller.get_front_distance()
            right_sensor_event, right_distance = self.controller.get_right_distance()
            left_sensor_event, left_distance = self.controller.get_left_distance()

            print("front sensor event: " + str(front_sensor_event) 
                + " front_distance: " + str(front_distance) 
                + "\nright_sensor_event: " + str(right_sensor_event) 
                + " right_distance: " + str(right_distance) 
                + "\nleft_sensor_event: " + str(left_sensor_event)
                + " left_distance: " + str(left_distance))
            
            if self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathFour:
                print("DEHORS")
                self.motors.stop()
                self.controller.passPathThreeRightFront()
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathThreeLeftFront:
                print("Allez tout droit")
                self.controller.passPathThreeLeftFront()
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathThreeRightFront:
                print("Allez à droite")
                self.controller.passPathThreeRightFront()
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathThreeLeftRight:
                print("Allez à droite")
                self.controller.passPathThreeLeftRight()
                while (self.controller.get_right_distance()[1] is HCSR04Event.NOTHING):
                    self.motors.forward() 
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathTwoLeft:
                print("Allez à gauche")
                self.controller.passPathTwoLeft()
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathTwoRight:
                print("Allez a droite")
                self.controller.passPathTwoRight()
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathTwoFront:
                print("C'est un couloir, allez tout droit")
                print("Going to the next intersection")
                self.controller.go_to_the_next_intersection()
            elif self.controller.get_intersection_type(left_sensor_event, front_sensor_event, right_sensor_event) is IntersectionType.PathOne:
                print("Cul de sac, Demi tour")
                self.controller.passDeadEnd()
            else: 
                print("Erreur, stop")
                


            
            

