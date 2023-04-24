import math
import time
import constants
import kinematics
from constants import *
import pypot.dynamixel
ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0],baudrate=1000000)
dxl_io.disable_torque([61, 62, 63])

while True:

    angle = dxl_io.get_present_position([61, 62, 63])
    print(angle)

    #kinematics.computeDK()



    


