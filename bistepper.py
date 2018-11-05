from constant import *
import RPi.GPIO as GPIO
import math
import threading
import time
import wiringpi
import water

GPIO.setwarnings(False)


class BiStepper:

    def __init__(self, flag=RESET_DIR, motor=""):
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
            self.valve_follow_pos = 0
        self.water = None

    def irrigation(self, head, water, head_v=HEAD_V, valve_v=VALVE_V):
        self.water = water
        self.headprepare(head, head_v, NOT_CHECK_DIR)
        h_thread = threading.Thread(target=self.headmove)
        v_thread = threading.Thread(target=self.valvefollow, args=(valve_v,))
        h_thread.start()
        v_thread.start()
        h_thread.join()
        v_thread.join()

    def bimove(self, head, valve, head_speed=HEAD_V,
               valve_speed=VALVE_V):
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
        h_thread = threading.Thread(target=self.headhome, args=(head_offset,),
                                    name='headmove')
        v_thread = threading.Thread(target=self.valvehome, args=(valve_offset,),
                                    name='valvehome')
        h_thread.start()
        v_thread.start()
        h_thread.join()
        v_thread.join()

    def headhome(self, head_offset=0):
        self.headstop()
        self.__init__(KEEP_DIR, "head")
        self.rt(head_offset)
        self.__init__(KEEP_DIR, "head")

    def valvehome(self, valve_offset=0):
        self.valvestop()
        self.__init__(KEEP_DIR, "valve")
        self.of(valve_offset)
        self.__init__(KEEP_DIR, "valve")

    def rt(self, head, h_speed=HEAD_V):
        self.headprepare(head, h_speed, CHECK_DIR)
        self.headmove()

    def of(self, valve, v_speed=VALVE_V):
        self.valveprepare(valve, v_speed)
        self.valvemove()

    def headstop(self):
        while True:
            pre_dir = self.head_dir
            if GPIO.input(HEAD_HOME) == MATCH:
                self.head_dir = OPEN_DIR
                self.head_step = self.angle2step(HEAD_POS_COUNT * 2, HEAD_SLACK,
                                                 pre_dir == self.head_dir)
                self.update_dir("head")
                v = HEAD_sV
                ramp_down_step = self.head_step - int(
                    (HEAD_sV - HEAD_V) / HEAD_RAMP)
                self.awake("head")
                for i in range(self.head_step):
                    self.step("head", v)
                    if v > HEAD_V and i < ramp_down_step:
                        v = v - 1
                    elif i >= ramp_down_step:
                        v = v + 1
                self.sleep("head")
            else:
                self.head_dir = CLOSE_DIR
                self.update_dir("head")
                v = HEAD_sV
                self.head_step = self.angle2step(HEAD_POS_COUNT, HEAD_SLACK)
                ramp_down_step = self.head_step - int(
                    (HEAD_sV - HEAD_V) / HEAD_RAMP)
                self.awake("head")
                while GPIO.input(HEAD_HOME) == NOT_MATCH:
                    self.step("head", v)
                    if v > HEAD_V:
                        v = v - 1
                for i in range(self.head_step):
                    self.step("head", v)
                    if i >= ramp_down_step:
                        v = v + 1
                self.sleep("head")
                break

    def valvestop(self):
        while True:
            pre_dir = self.valve_dir
            if GPIO.input(VALVE_HOME) == MATCH:
                self.valve_dir = OPEN_DIR
                self.valve_step = self.angle2step(VALVE_POS_COUNT * 2,
                                                  VALVE_SLACK,
                                                  pre_dir == self.valve_dir)
                self.update_dir("valve")
                v = VALVE_sV
                ramp_down_step = self.valve_step - int(
                    (VALVE_sV - VALVE_V) / VALVE_RAMP)
                self.awake("valve")
                for i in range(self.valve_step):
                    self.step("valve", v)
                    if v > VALVE_V and i < ramp_down_step:
                        v = v - 1
                    elif i >= ramp_down_step:
                        v = v + 1
                self.sleep("valve")
            else:
                self.valve_dir = CLOSE_DIR
                self.update_dir("valve")
                v = VALVE_sV
                self.valve_step = self.angle2step(VALVE_POS_COUNT, VALVE_SLACK)
                ramp_down_step = self.valve_step - int(
                    (VALVE_sV - VALVE_V) / VALVE_RAMP)
                self.awake("valve")
                while GPIO.input(VALVE_HOME) == NOT_MATCH:
                    self.step("valve", v)
                    if v > VALVE_V:
                        v = v - 1
                for i in range(self.valve_step):
                    self.step("valve", v)
                    if i >= ramp_down_step:
                        v = v + 1
                self.sleep("valve")
                break

    def headprepare(self, head, h_speed, dir_check=CHECK_DIR):
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
            self.head_step / 2) * HEAD_RAMP, h_speed)

    def valveprepare(self, valve, v_speed):
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
            self.valve_step / 2) * VALVE_RAMP, v_speed)

    def headmove(self):
        ramp_start = (HEAD_sV - self.head_v) / HEAD_RAMP
        ramp_stop = self.head_step - ramp_start
        self.awake("head")
        v = HEAD_sV
        t = wiringpi.micros()
        for step in range(self.head_step):
            makeup = wiringpi.micros() - t
            if v < makeup:
                makeup = v
            self.step("head", v, makeup)
            t = wiringpi.micros()
            if step < ramp_start:
                v = v - HEAD_RAMP
            elif step >= ramp_stop:
                v = v + 1
            self.head_degree = int(step / STEP_PER_ANGLE)

        self.sleep("head")
        self.head_pos = self.head_target
        self.head_degree = 0
        self.head_state = IDLE

    def valvemove(self):
        ramp_start = (VALVE_sV - self.valve_v) / VALVE_RAMP
        ramp_stop = self.valve_step - ramp_start
        self.awake("valve")
        v = VALVE_sV
        t = wiringpi.micros()
        for step in range(self.valve_step):
            makeup = wiringpi.micros() - t
            if v < makeup:
                makeup = v
            self.step("valve", v, makeup)
            t = wiringpi.micros()
            if step < ramp_start:
                v = v - HEAD_RAMP
            elif step >= ramp_stop:
                v = v + 1
        self.sleep("valve")
        self.valve_pos = self.valve_target
        self.valve_state = IDLE

    def valvefollow(self, valve_speed):
        degree = self.head_degree
        while self.head_pos != self.head_target:
            time.sleep(0.01)
            if self.head_degree != degree:
                degree = self.head_degree
                self.valve_follow_pos = self.water.update()
                if self.valve_follow_pos != self.valve_pos and self.valve_state == IDLE:
                    self.of(self.valve_follow_pos, valve_speed)
        water.flag = False

    def update_dir(self, motor):
        if motor == "head":
            GPIO.output(HEAD_DIR, self.head_dir)
        else:
            GPIO.output(VALVE_DIR, self.valve_dir)

    @staticmethod
    def init_gpio():
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
        GPIO.output(HEAD_SLEEP, GPIO.LOW)
        GPIO.output(VALVE_DIR, COUNTERCLOCKWISE)
        GPIO.output(VALVE_STEP, GPIO.LOW)
        GPIO.output(VALVE_SLEEP, GPIO.LOW)

    @staticmethod
    def angle2step(angle, slack, dir_change=True):
        return int(
            abs((angle + (not dir_change) * slack) * GEARRATIO * ADJUSTMENT))

    @staticmethod
    def awake(motor):
        if motor == "head":
            GPIO.output(HEAD_SLEEP, GPIO.HIGH)
        else:
            GPIO.output(VALVE_SLEEP, GPIO.HIGH)

    @staticmethod
    def sleep(motor):
        if motor == "head":
            GPIO.output(HEAD_SLEEP, GPIO.LOW)
        else:
            GPIO.output(VALVE_SLEEP, GPIO.LOW)

    @staticmethod
    def step(motor, delay, makeup=0):
        if motor == "head":
            wiringpi.delayMicroseconds(delay - makeup)
            GPIO.output(HEAD_STEP, GPIO.HIGH)
            wiringpi.delayMicroseconds(delay)
            GPIO.output(HEAD_STEP, GPIO.LOW)

        else:
            wiringpi.delayMicroseconds(delay - makeup)
            GPIO.output(VALVE_STEP, GPIO.HIGH)
            wiringpi.delayMicroseconds(delay)
            GPIO.output(VALVE_STEP, GPIO.LOW)
