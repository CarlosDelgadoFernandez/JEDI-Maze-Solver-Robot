#!/usr/bin/python3

from enum import IntEnum, unique
from turtle import left, right
from motor import Motor, MotorDirection
from sensors import HCSR04, HCSR04Event
import time


@unique
class IntersectionType(IntEnum):
    PathFour = 0
    PathThreeLeftFront = 1
    PathThreeRightFront = 2
    PathThreeLeftRight = 3
    PathTwoLeft = 4
    PathTwoRight = 5
    PathTwoFront = 6
    PathOne = 7
    PathZero = 8


class Controller:
    """Class to control motors according to sensed values.
    Note that sensing is not dependable..."""

    TIME_PASS_INTERSECTION = 1.5

    def __init__(self, motors: Motor, lsensor: HCSR04, fsensor: HCSR04,
                 rsensor: HCSR04, speed=0.5):
        self.motors = motors
        self.speed = speed
        self.left_sensor = lsensor
        self.front_sensor = fsensor
        self.right_sensor = rsensor

    def _get_event(self, sensor, moving=True):
        """returns a HCSR04Event according to the distance measured by the
        sensor"""
        distance = self._get_robust_distance(sensor, moving)
        if distance < sensor.low:
            return (HCSR04Event.DANGER, distance)
        elif distance < sensor.high:
            return (HCSR04Event.WALL, distance)
        else:
            return (HCSR04Event.NOTHING, distance)

    def get_left_distance(self, moving=True):
        """measures distance on the left and returns a HCSR04Event
        and the measured distance"""
        return self._get_event(self.left_sensor, moving)

    def get_front_distance(self, moving=True):
        """measures distance ahead and returns a HCSR04Event
        and the measured distance"""
        return self._get_event(self.front_sensor, moving)

    def get_right_distance(self, moving=True):
        """measures distance on the right and returns a HCSR04Event
        and the measured distance"""
        return self._get_event(self.right_sensor, moving)

    def get_robust_left_distance(self, moving=True):
        """measures distance on the left, without error, stops the droid if
        necessary"""
        return self._get_robust_distance(self.left_sensor, moving)

    def get_robust_front_distance(self, moving=True):
        """measures distance ahead, without error, stops the droid if
        necessary"""
        return self._get_robust_distance(self.front_sensor, moving)

    def get_robust_right_distance(self, moving=True):
        """measures distance on the right, without error, stops the droid if
        necessary"""
        return self._get_robust_distance(self.right_sensor, moving)

    def _get_robust_distance(self, sensor, moving=True):
        """Robust measrurement on sensor, stops motors if necessary and restart
        them according to an acceptable measurement."""
        return sensor.get_distance()

    def danger(self, le, fe, re):
        """returns True if one sensor among the three ones measures a distance
        below its lower threshold, False otherwise"""
        return (le is HCSR04Event.DANGER or fe is HCSR04Event.DANGER
                or re is HCSR04Event.DANGER)

    def is_exit(self, le, fe, re):
        """returns True if the maze exit is reached, False otherwise"""
        return (le is HCSR04Event.NOTHING and fe is HCSR04Event.NOTHING
                and re is HCSR04Event.NOTHING)

    def is_intersection(self, le, fe, re):
        """returns True if the droid is currently in an intersection,
        False otherwise"""
        return (le is HCSR04Event.NOTHING or re is HCSR04Event.NOTHING)

    def is_dead_end(self, le, fe, re):
        """returns True if the droid is in a dead end, False otherwise"""
        return (le is not HCSR04Event.NOTHING and fe is not HCSR04Event.NOTHING
                and re is not HCSR04Event.NOTHING)

    def is_corridor(self, le, fe, re):
        """returns True if the Droid is in a corridor, False otherwise"""
        return ((le is not HCSR04Event.NOTHING and re is not HCSR04Event.NOTHING) and (fe is HCSR04Event.NOTHING))

    def get_intersection_type(self, le, fe, re):
        """returns an IntersectionType according to the measured values on the
        three sensors"""
        if self.is_exit(le, fe, re):
            return IntersectionType.PathFour
        elif self.is_intersection(le, fe, re):
            if (le is HCSR04Event.NOTHING and fe is HCSR04Event.NOTHING and re is not HCSR04Event.NOTHING):
                return IntersectionType.PathThreeLeftFront
            elif (le is not HCSR04Event.NOTHING and fe is HCSR04Event.NOTHING
                  and re is HCSR04Event.NOTHING):
                return IntersectionType.PathThreeRightFront
            elif (le is HCSR04Event.NOTHING and fe is not HCSR04Event.NOTHING
                  and re is HCSR04Event.NOTHING):
                return IntersectionType.PathThreeLeftRight
            elif (le is HCSR04Event.NOTHING and fe is not HCSR04Event.NOTHING
                  and re is not HCSR04Event.NOTHING):
                return IntersectionType.PathTwoLeft
            elif (le is not HCSR04Event.NOTHING and
                  fe is not HCSR04Event.NOTHING and re is HCSR04Event.NOTHING):
                return IntersectionType.PathTwoRight
            else:
                raise ValueError("Unknown situation")
        elif self.is_dead_end(le, fe, re):
            return IntersectionType.PathOne
        elif self.is_corridor(le, fe, re):
            return IntersectionType.PathTwoFront
        else:
            return IntersectionType.PathZero


class PIDController(Controller):
    def __init__(self, motors: Motor,
                 lsensor: HCSR04, fsensor: HCSR04, rsensor: HCSR04,
                 speed=0.7, target=20., KP=0.05, KD=0.005, KI=0.01,
                 sample_time=0.01):
        """Read https://en.wikipedia.org/wiki/PID_controller
        and https://projects.raspberrypi.org/en/projects/robotPID/5"""
        Controller.__init__(self, motors, lsensor, fsensor, rsensor, speed=speed)
        self.md_speed = self.speed
        self.mg_speed = self.speed
        self.target = target
        # https://en.wikipedia.org/wiki/PID_controller
        self.sample_time = sample_time
        self.last_sample = 0.0
        self.KP = KP
        self.KD = KD
        self.KI = KI
        self.prev_error = 0.0
        self.curr_err = 0.0
        self.sum_error = 0.0

    def adjust(self):
        """adjusts motors speeds to compensate error on objective function
        (namely a distance on left or right wall)"""
        

        print("still in corridor")
        left_event, left_distance = self.get_left_distance()
        right_event, right_distance = self.get_right_distance()
        front_event, front_distance = self.get_front_distance()
        
        wide_distance= left_distance + right_distance
        objectif = 20

        while (self.is_corridor(left_event, front_event, right_event)):
            
            self.prev_err = self.curr_err 
            self.curr_err = (objectif - right_distance)
            self.sum_err = self.sample_time*(self.prev_err + self.curr_err)/2
            deriv = (self.curr_err-self.prev_err)/self.sample_time

            
            if (right_event is not HCSR04Event.NOTHING and left_event is not HCSR04Event.NOTHING):
                self.mg_speed =  self.speed - self.KP*self.curr_err  - self.KI*self.sum_err# - self.KD*deriv
                self.md_speed =  self.speed + self.KP*self.curr_err  + self.KI*self.sum_err# + self.KD*deriv
                self.mg_speed = max(min(0.8, self.mg_speed), 0.2)
                self.md_speed = max(min(0.8, self.md_speed), 0.2)
                #self.mg_speed = max(min((self.speed+0.3), self.mg_speed), (self.speed-0.3))
                #self.md_speed = max(min((self.speed+0.3), self.mg_speed), (self.speed-0.3))
                                            
                    
                print("")
                print("")
                print("Center is at: " + str(objectif))
                print("Distance to right: " + str(right_distance))
                print("Distance to left: " + str(left_distance))
                print("Distance to TARGET: " + str(self.curr_err))
                print("left pwm",  self.mg_speed)
                print("right pwm",  self.md_speed)
                self.motors._set_motors(self.mg_speed, MotorDirection.FORWARD, self.md_speed, MotorDirection.FORWARD)
                    

            """if (right_event is HCSR04Event.DANGER):    
                print("Right Limit Reached")
                self.motors.stop()"""

            left_event, left_distance = self.get_left_distance()
            right_event, right_distance = self.get_right_distance()
            front_event, front_distance = self.get_front_distance()

    def adjustAtIntersection(self):
        print("still in intersection")
        right_event, right_distance = self.get_right_distance()
        
        #wide_distance= left_distance + right_distance
        objectif = 25

        while (self.get_left_distance()[0] is HCSR04Event.NOTHING):
            
            self.prev_err = self.curr_err 
            self.curr_err = (objectif - right_distance)
            self.sum_err = self.sample_time*(self.prev_err + self.curr_err)/2
            deriv = (self.curr_err-self.prev_err)/self.sample_time

            
            if (right_event is HCSR04Event.WALL):
                self.mg_speed =  self.speed - self.KP*self.curr_err  - self.KI*self.sum_err# - self.KD*deriv
                self.md_speed =  self.speed + self.KP*self.curr_err  + self.KI*self.sum_err# + self.KD*deriv
                self.mg_speed = max(min(0.8, self.mg_speed), 0.2)
                self.md_speed = max(min(0.8, self.md_speed), 0.2)
                #self.mg_speed = max(min((self.speed+0.3), self.mg_speed), (self.speed-0.3))
                #self.md_speed = max(min((self.speed+0.3), self.mg_speed), (self.speed-0.3))
                                            
                print("Center is at: " + str(objectif))
                print("Distance to right: " + str(right_distance))
                print("Distance to TARGET: " + str(self.curr_err))
                print("left pwm",  self.mg_speed)
                print("right pwm",  self.md_speed)
                self.motors._set_motors(self.mg_speed, MotorDirection.FORWARD, self.md_speed, MotorDirection.FORWARD)
                    

            """if (right_event is HCSR04Event.DANGER):    
                print("Right Limit Reached")
                self.motors.stop()"""

            right_event, right_distance = self.get_right_distance()


    def go_to_the_next_intersection(self):
        """drives the droid 'safely' to the next intersection, safely meaning
        that the droids to not meet a wall and stops when it detects an
        IntersectionType that is not PathTwoFront, i.e. the droid is not in a
        corridor anymore"""

        self.adjust()

    def passPathThreeLeftFront(self):
        self.motors.forward()
        print("Left event: " + (str(self.get_left_distance()[0])))
        #self.adjustAtIntersection()

    def passPathThreeRightFront(self):
        self.motors.forward()
        time.sleep(self.TIME_PASS_INTERSECTION)
        self.motors.right()
        self.motors.forward()
        print("Right event :" + str(self.get_right_distance()[0]))
        while (self.get_right_distance()[0] is HCSR04Event.NOTHING):
            self.motors.forward()

    def passPathThreeLeftRight(self):
        while self.get_front_distance()[1] > 25:
            self.motors.forward()
        self.motors.right()
        while (self.get_right_distance()[0] is HCSR04Event.NOTHING):
            self.motors.forward()

    def passPathTwoLeft(self):
        while self.get_front_distance()[1] > 25:
            self.motors.forward()
        self.motors.left()
        self.motors.forward()
    
    def passPathTwoRight(self):
        while self.get_front_distance()[1] > 25:
            self.motors.forward()
        self.motors.right()
        self.motors.forward()

    def passDeadEnd(self):
        while self.get_front_distance()[1] > 25:
            self.motors.forward()
            self.adjust()
        self.motors.u_turn()

    

    
        
