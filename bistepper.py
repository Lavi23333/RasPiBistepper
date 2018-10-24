from constant import *
import RPi.GPIO as GPIO
import math
import threading
import time
import wiringpi

GPIO.setwarnings(False)


class BiStepper:

    def __init__(self, flag=RESET_DIR, motor=""):
        """Init the bi-stepper system.

        :param flag: Reset_DIR means reset the directions;KEEP_DIR means keep
        the directions.
        :param motor: Reset the motor's parameters. Default is both.
        """

        self.debug = False
        if flag:
            self.init_gpio()
        if motor != "valve":
            if flag:
                self.head_dir = OPEN_DIR
            self.head_pos = 0
            self.head_target = 0
            self.head_step = 0
            self.head_v = 0
            self.head_degree = 0
            self.head_state = IDLE
        if motor != "head":
            if flag:
                self.valve_dir = OPEN_DIR
            self.valve_pos = 0
            self.valve_target = 0
            self.valve_step = 0
            self.valve_v = 0
            self.valve_v_target = 0
            self.valve_state = IDLE

    def irrigation(self, head, head_speed=HEAD_V, valve_speed=VALVE_V):
        """Start irrigation process. Valve motor will following the head motor
        in two threads.

        :param head: The target position of head motor.
        :param head_speed: The target speed of the head motor.
        :param valve_speed: The target speed of the valve motor.
        :return: None
        """
        self.headprepare(head, head_speed)
        self.valve_v_target = valve_speed
        h_thread = threading.Thread(target=self.headmove)
        v_thread = threading.Thread(target=self.valvefollow)
        h_thread.join()
        v_thread.join()

    def bimove(self, head, valve, head_speed=HEAD_V,
               valve_speed=VALVE_V):
        """Start bistepper process. Valve motor will moves together with head motor.

        :param head: The target position of head motor.
        :param valve: The target speed of the valve motor.
        :param head_speed: The target speed of the head motor.
        :param valve_speed: The target speed of the valve motor.
        :return: None
        """
        h_thread = threading.Thread(target=self.rt,
                                    args=(
                                        head, head_speed,))
        v_thread = threading.Thread(target=self.of,
                                    args=(valve, valve_speed,))
        h_thread.start()
        v_thread.start()
        h_thread.join()
        v_thread.join()

    def bistop(self, head_offset=0, valve_offset=0):
        """Return both motor to the init position together. Speed is maximum and
        direction is clockwise.

        :param head_offset: The offset value of head motor.
        :param valve_offset: The offset value of valve motor.
        :return: None
        """
        h_thread = threading.Thread(target=self.headhome, args=(head_offset,),
                                    name='headmove')
        v_thread = threading.Thread(target=self.valvehome, args=(valve_offset,),
                                    name='valvefollow')
        h_thread.start()
        v_thread.start()
        h_thread.join()
        v_thread.join()

    def headhome(self, head_offset=0):
        """Move head motor to the init position.

        :param head_offset: The offset value of the head motor.
        :return: None
        """
        self.headstop()
        self.__init__(KEEP_DIR, "head")
        self.rt(head_offset)
        self.__init__(KEEP_DIR, "head")

    def valvehome(self, valve_offset=0):
        """Move valve motor to the init position.

        :param valve_offset: The offset value of the valve motor.
        :return: None
        """
        self.valvestop()
        self.__init__(KEEP_DIR, "valve")
        self.of(valve_offset)
        self.__init__(KEEP_DIR, "valve")

    def rt(self, head, h_speed=HEAD_V):
        """Spin the head motor to the target position

        :param head: The target position of the head motor.
        :param h_speed: The target speed of the head motor.
        :return: None
        """
        self.headprepare(head, h_speed, CHECK_DIR)
        self.headmove()

    def of(self, valve, v_speed=VALVE_V):
        """Spin the valve motor to the target position.

        :param valve: The target position of the valve motor.
        :param v_speed: The target speed of the valve motor.
        :return: None
        """
        self.valveprepare(valve, v_speed)
        self.valvemove()

    def headstop(self):
        """Return head motor home.

        :return: None
        """

        while True:
            pre_dir = self.head_dir
            if GPIO.input(HEAD_HOME) == MATCH:
                print 'match'
                self.head_dir = OPEN_DIR
                self.head_step = self.angle2step(HEAD_POS_COUNT * 2, HEAD_SLACK,
                                                 pre_dir == self.head_dir)
                self.update_dir("head")
                self.awake("head")
                for i in range(self.head_step):
                    self.step("head", HEAD_V)
                self.sleep("head")
            else:
                print 'not match'
                self.head_dir = CLOSE_DIR
                self.update_dir("head")
                self.awake("head")
                while GPIO.input(HEAD_HOME) == NOT_MATCH:
                    self.step("head", HEAD_V)
                print 'last steps'
                self.head_step = self.angle2step(HEAD_POS_COUNT, HEAD_SLACK)
                for i in range(self.head_step):
                    self.step("head", HEAD_V)
                self.sleep("head")
                break

    def valvestop(self):
        """Return valve motor home.

        :return: None
        """

        while True:
            pre_dir = self.valve_dir
            if GPIO.input(VALVE_HOME) == MATCH:
                self.valve_dir = OPEN_DIR
                self.valve_step = self.angle2step(VALVE_POS_COUNT * 2,
                                                  VALVE_SLACK,
                                                  pre_dir == self.valve_dir)
                self.update_dir("valve")
                self.awake("valve")
                for i in range(self.valve_step):
                    self.step("valve", VALVE_V)
                self.sleep("valve")
            else:
                self.valve_dir = CLOSE_DIR
                self.update_dir("valve")
                self.awake("valve")
                while GPIO.input(VALVE_HOME) == NOT_MATCH:
                    self.step("valve", VALVE_V)
                self.valve_step = self.angle2step(VALVE_POS_COUNT, VALVE_SLACK)
                for i in range(self.valve_step):
                    self.step("valve", VALVE_V)
                self.sleep("valve")
                break

    def headprepare(self, head, h_speed, dir_check=CHECK_DIR):
        """Calculate the direction, step, speed for the head motor.

        :param head: Target position of the head motor.
        :param h_speed: Target speed of the head motor.
        :param dir_check: Whether to select the shortest path.
        :return: None
        """

        self.head_state = RUNNING
        self.head_target = head
        self.head_step = head - self.head_pos
        pre_dir = self.head_dir
        if dir_check:
            if self.head_step < -180:
                self.head_dir = OPEN_DIR
                self.head_step = 360 + self.head_step
            elif self.head_step < 0:
                self.head_dir = CLOSE_DIR
                self.head_step = -self.head_step
            elif self.head_step < 180:
                self.head_dir = OPEN_DIR
            else:
                self.head_dir = CLOSE_DIR
                self.head_step = 360 - self.head_step
        else:
            self.head_dir = OPEN_DIR
            if self.head_step < 0:
                self.head_step += 360
        self.update_dir("head")
        self.head_step = self.angle2step(self.head_step,
                                         HEAD_SLACK,
                                         pre_dir == self.head_dir)
        self.head_v = max(HEAD_sV - math.floor(
            self.head_step / 2) * HEAD_Ramp, h_speed)

    def valveprepare(self, valve, v_speed):
        """Calculate the direction, step, speed for hte valve motor.

        :param valve: The target position of the valve motor.
        :param v_speed: The target speed of the valve motor.
        :return: None
        """
        self.valve_state = RUNNING
        self.valve_target = valve
        self.valve_step = abs(valve - self.valve_pos)
        pre_dir = self.valve_dir
        if valve >= self.valve_pos:
            self.valve_dir = OPEN_DIR
        else:
            self.valve_dir = CLOSE_DIR
        self.update_dir("valve")
        self.valve_step = self.angle2step(self.valve_step,
                                          VALVE_SLACK,
                                          pre_dir == self.valve_dir)
        self.valve_v = max(VALVE_sV - math.floor(
            self.valve_step / 2) * VALVE_Ramp, v_speed)

    def headmove(self):
        """Move the head motor to the target position.

        :return: None
        """
        ramp_start = (HEAD_sV - self.head_v) / HEAD_Ramp
        ramp_stop = self.head_step - ramp_start
        self.awake("head")
        v = HEAD_sV
        for step in range(self.head_step):
            self.step("head", v)
            if step < ramp_start:
                v = v - HEAD_Ramp
            elif step >= ramp_stop:
                v = v + 1
            self.head_degree = int(math.floor(step / STEP_PER_ANGLE))
        self.sleep("head")
        self.head_pos = self.head_target
        self.head_degree = 0
        self.head_state = IDLE

    def valvemove(self):
        """Move the valve motor to the target position.

        :return: None
        """
        ramp_start = (VALVE_sV - self.valve_v) / VALVE_Ramp
        ramp_stop = self.valve_step - ramp_start
        self.awake("valve")
        v = VALVE_sV
        for step in range(self.valve_step):
            self.step("valve", v)
            if step < ramp_start:
                v = v - HEAD_Ramp
            elif step >= ramp_stop:
                v = v + 1
        self.sleep("valve")
        self.valve_pos = self.valve_target
        self.valve_state = IDLE

    def valvefollow(self):
        """During the irrigation process, the valve motor will keep listening the
        position of the head motor and move accordingly.

        :return: None
        """
        while self.head_state == RUNNING:
            time.sleep(VALVE_CHECK_FREQ)
            if self.valve_pos != self.valve_target:
                self.valveprepare(self.valve_target, self.valve_v_target)
                v_thread = threading.Thread(target=self.valvemove())
                v_thread.start()
                v_thread.join()

    def update_dir(self, motor):
        """Update the direction of the target motor.

        :param motor: Target motor name.
        :return: None.
        """
        if motor == "head":
            GPIO.output(HEAD_DIR, self.head_dir)
        else:
            GPIO.output(VALVE_DIR, self.valve_dir)

    @staticmethod
    def init_gpio():
        """Init all the gpio ports for direction, step, home sensor.

        :return: None
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(HEAD_DIR, GPIO.OUT)
        GPIO.setup(HEAD_STEP, GPIO.OUT)
        GPIO.setup(HEAD_HOME, GPIO.IN, GPIO.PUD_DOWN)
        GPIO.setup(HEAD_SLEEP, GPIO.OUT)
        GPIO.setup(VALVE_DIR, GPIO.OUT)
        GPIO.setup(VALVE_STEP, GPIO.OUT)
        GPIO.setup(VALVE_HOME, GPIO.IN, GPIO.PUD_DOWN)
        GPIO.setup(VALVE_SLEEP, GPIO.OUT)

        GPIO.output(HEAD_DIR, COUNTERCLOCKWISE)
        GPIO.output(HEAD_STEP, GPIO.LOW)
        GPIO.output(HEAD_SLEEP, GPIO.HIGH)
        GPIO.output(VALVE_DIR, COUNTERCLOCKWISE)
        GPIO.output(VALVE_STEP, GPIO.LOW)
        GPIO.output(VALVE_SLEEP, GPIO.HIGH)

    @staticmethod
    def angle2step(angle, slack, dir_change=True):
        """Transfer angle value to step value.

        :param angle: Target angle value
        :param slack: Slack value
        :param dir_change: Whether the direction is changed.
        :return: Target step value.
        """
        return int(
            abs((angle + (not dir_change) * slack) * GEARRATIO * ADJUSTMENT))

    @staticmethod
    def awake(motor):
        """Wake up the motor.

        :param motor: Target motor name.
        :return: None
        """
        if motor == "head":
            GPIO.output(HEAD_SLEEP, GPIO.LOW)
        else:
            GPIO.output(VALVE_SLEEP, GPIO.LOW)

    @staticmethod
    def sleep(motor):
        """Sleep the motor.

        :param motor: Target motor name.
        :return: None
        """
        if motor == "head":
            GPIO.output(HEAD_SLEEP, GPIO.HIGH)
        else:
            GPIO.output(VALVE_SLEEP, GPIO.HIGH)

    @staticmethod
    def step(motor, delay):
        """Get the motor to move 1 step.

        :param motor: Target motor name.
        :param delay: None.
        :return:
        """
        if motor == "head":
            GPIO.output(HEAD_STEP, GPIO.HIGH)
            wiringpi.delayMicroseconds(delay)
            GPIO.output(HEAD_STEP, GPIO.LOW)
            wiringpi.delayMicroseconds(delay)
        else:
            GPIO.output(VALVE_STEP, GPIO.HIGH)
            wiringpi.delayMicroseconds(delay)
            GPIO.output(VALVE_STEP, GPIO.LOW)
            wiringpi.delayMicroseconds(delay)
