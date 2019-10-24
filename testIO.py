import RobotRaconteur as RR
from RobotRaconteur.Client import *
import numpy as np
import time

UR=RRN.ConnectService('tcp://localhost:2355/URConnection/Universal_Robot')

UR.setf_signal("DO0",0)

