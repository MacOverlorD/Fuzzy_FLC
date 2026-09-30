#!/usr/bin/python3

import os, platform
if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"
    
from pysimbotlib.core import PySimbotApp, Robot, Simbot
from kivy.logger import Logger

from kivy.config import Config
# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')
Config.set('graphics', 'maxfps', 10)

import time
import random

class RandomWalkRobot(Robot):

    def update(self):
        self.distance()
        r = random.randint(0, 3)
        self.move(5)
        if r == 1:
            self.turn(15)
        elif r == 2:
            self.turn(-15)

start_time = time.time()
def before_sim(simbot_map: Simbot):
    global start_time
    start_time = time.time()
    Logger.info("Simulation: Before simulation.")
    Logger.info("Simulation: You can now do something with map objects or robots")
    for r in simbot_map.robots:
        r.pos = (400, 30)
        r.set_color(random.random(), random.random(), random.random())

def after_sim(simbot_map: Simbot):

    # There are some simbot and robot calcalated statistics and property during simulation
    # - simbot.score
    # - simbot.simulation_count
    # - simbot.eat_count
    # - simbot.food_move_count
    # - simbot.score
    # - simbot.scoreStr

    # - simbot.robot[i].eat_count
    # - simbot.robot[i].collision_count
    # - simbot.robot[i].color

    for r in simbot_map.robots:
        Logger.info("Simulation: robot pos = {0}".format(r.pos))
        Logger.info("Simulation: robot eat_count = {0}".format(r.eat_count))
        Logger.info("Simulation: robot collision_count = {0}".format(r.collision_count))

    Logger.info("Simulation: End simulation. Robot[0] is at {0}".format(simbot_map.robots[0].pos))
    Logger.info("Simulation: Score = {0}".format(simbot_map.score))
    Logger.info("Time: {0}".format(time.time() - start_time))

if __name__ == '__main__':
    app = PySimbotApp(robot_cls=RandomWalkRobot,
                        num_robots=30,
                        max_tick=500,
                        interval=1/1000.0,
                        simulation_forever=True,
                        customfn_before_simulation=before_sim,
                        customfn_after_simulation=after_sim,
                        food_move_after_eat=False)
    app.run()