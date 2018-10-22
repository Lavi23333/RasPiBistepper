def setup(pin, pin_type, add=0):
    if pin_type == 1:
        print("Set ", pin, " pin as OUTPUT.")
    else:
        print ("Set ", pin, " pin as ", add, " INPUT.")


def output(pin, level):
    if level == 1:
        print("Set ", pin, "high.")
    else:
        print("Set ", pin, "low.")


def setmode(mode):
    print ("Init mode as ", mode)


def input(pin):
    print ("Read from pin ", pin)
    pass


HIGH = 1
LOW = 0

OUT = 1
IN = 0
PUD_DOWN = 1
BOARD = ""
