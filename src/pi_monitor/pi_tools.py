#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# ----- suppress the fallback warning of gpiozero
import warnings
warnings.simplefilter('ignore')
try:
    from gpiozero import CPUTemperature
except ImportError:
    print(f'Import Error: gpiozero not intalled')    

class PIUtil():
    def __init__(self):
        self.cpu = None
        try:
            self.cpu = CPUTemperature()
        except:
            print(f'PI Utility Error: not running on a PI or pigpio not installed')

    def is_alive(self):
        return self.cpu is not None

    def get_cpu_temperature(self):
        if self.cpu is None:
            return None
        try:
            return self.cpu.temperature
        except:
            return None
        
PIUTIL = PIUtil()