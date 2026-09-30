#!/usr/bin/python3

import os, platform
if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"
    
from pysimbotlib.core import PySimbotApp, Robot
from kivy.logger import Logger

from kivy.config import Config
# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

import random

# update robot every 0.5 seconds (2 frames per sec)
REFRESH_INTERVAL = 1/2

class MyRobot(Robot):
    
    def update(self):
        Logger.info("Smell Angle: {0}".format(self.smell_nearest()))
        Logger.info("Distance: {0}".format(self.distance()))

if __name__ == '__main__':
    app = PySimbotApp(robot_cls=MyRobot, num_objectives=4, robot_see_each_other=True, interval=REFRESH_INTERVAL, map="no_wall", enable_wasd_control=True, simulation_forever=True)
    app.run()