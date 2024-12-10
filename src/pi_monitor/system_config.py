#!/usr/bin/env python3
# 
# # Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import yaml, os
from enum import Enum
from collections.abc import MutableMapping
# ros params
import rospy
# project specifics

# This enum class models the config names used by the CGRAS coordinator
class SystemConfigNames(Enum):
    """ Maps the config names as a sting to a constant
    """
    # timer for refresh the system and GUI
    SYSTEM_TIMER = 'system_timer'
    # ros topic
    ROS_PI_TEMP_TOPIC = 'ros_pi_temp_topic'

# This class models the configuration values specified in a yaml file and present as a python dictionary
class SystemConfig(MutableMapping):
    """ The class providing easy query of the hierarcy of configurations in yaml
    """
    def __init__(self, config_file:str, *args, **kwargs):
        """the constructor
        :param scene_config_file: the path to the yaml configuration file
        :type scene_config_file: str, optional
        """
        # load data from the config yaml file
        if config_file is None:
            raise AssertionError(f'{__class__.__name__} parameter (config_file) is None')
        # YAML path
        YAML_PATH = 'cgras_pi_monitor'
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
            if YAML_PATH not in self.config:
                raise AssertionError(f'{__class__.__name__} the config yaml file does not contain a branch named cgras_coordinator')
            self.config = self.config[YAML_PATH] 
        self.update(dict(*args, **kwargs))
    
    # functions implemented for a subclass of python dictionary MutableMapping
    def __getitem__(self, key):
        name = f'~{self._keytransform(key)}'
        value = rospy.get_param(name, self.config[self._keytransform(key)])
        return value

    def __setitem__(self, key, value):
        self.config[self._keytransform(key)] = value

    def __delitem__(self, key):
        del self.config[self._keytransform(key)]

    def __iter__(self):
        return iter(self.config)
    
    def __len__(self):
        return len(self.config)

    def _keytransform(self, key):
        if isinstance(key, SystemConfigNames):
            key = key.value
        return key

    # define model functions
    # return a dict object for the system configs
    def to_params(self, param_enum_cls:list=None):
        if param_enum_cls is None:
            param_enum_cls = [SystemConfigNames]
        output = {}
        for param_enum in param_enum_cls:
            for param in param_enum:
                if param.value in self.config:
                    output[param.value] = self.get(param.value)
        return output

# ------------------------
# test program
if __name__ == '__main__':
    # the rospy param will override the config file if a ros node is running
    rospy.init_node('config_manager')
    print(f'ros_param: {rospy.get_param("~data_folder")}')
    
    CONFIG:SystemConfig = SystemConfig(os.path.join(os.path.dirname(__file__), '../../config/system_config.yaml'))
    print(f'{type(SystemConfigNames.CGRAS_DATA_FOLDER)}')
    print(f'{isinstance(SystemConfigNames.CGRAS_DATA_FOLDER, Enum)}')
    print(f'{CONFIG[SystemConfigNames.CGRAS_DATA_FOLDER]}')
    print(f'{CONFIG.get(SystemConfigNames.CGRAS_DATA_FOLDER)}')
    print(f'{CONFIG.get("no_definition", "default")}')
    
    print(f'### The config dictionary')
    config_dict = CONFIG.to_params()
    print(f'{config_dict}')