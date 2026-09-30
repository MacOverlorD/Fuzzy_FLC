#!/usr/bin/env python3
"""Mission 3: collect food until timeout using three front IR sensors."""

import os
import platform

if platform.system() in ("Linux", "Darwin"):
    os.environ["KIVY_VIDEO"] = "ffpyplayer"

from kivy.config import Config
from kivy.logger import Logger
from pysimbotlib.core import PySimbotApp, Robot


Config.set("kivy", "log_level", "info")
Config.set("graphics", "maxfps", "60")


class Mission3SimbotRobot(Robot):
    SAFETY_DISTANCE = 30
    CLOSED_DISTANCE = 5
    HIT_DISTANCE = 0  # Contact with an object; also handled by rule 4.

    MOVE_STEP = 10
    MAX_MOVE_STEP = 10  # Never request more than 10 pixels per move.
    SLOW_MOVE_STEP = 2
    FOOD_TURN_STEP = 5
    TURN_STEP = 45
    ESCAPE_TURNS = 4  # Four 45-degree turns face away from the collision.
    ESCAPE_MOVES = 4
    ESCAPE_MOVE_STEP = 5
    ESCAPE_SCAN_TURN = 15  # Small turns help find exits in narrow passages.
    NO_PROGRESS_TICKS = 20
    PROGRESS_DISTANCE = 3  # Moving this far counts as leaving the current spot.

    progress_position = None
    no_progress_ticks = 0
    escape_steps = 0
    last_collision_count = 0
    avoid_direction = 0  # Remember one turn direction while the front is blocked.

    # Assignment names mapped to the PySimbot methods.
    def getIRValues(self):
        return self.distance()

    def smellAllFoods(self):
        return self.smell()

    def moveForward(self, pixels=MOVE_STEP):
        self.move(max(0, min(int(pixels), self.MAX_MOVE_STEP)))

    def spinLeft(self, degrees=TURN_STEP):
        self.turn(-abs(degrees))

    def spinRight(self, degrees=TURN_STEP):
        self.turn(abs(degrees))

    def update(self):
        self.execute()

    def execute(self):
        # Keep seeking the relocated food until the simulator reaches max_tick.
        ir = self.getIRValues()
        front = ir[0]
        right = ir[1]
        left = ir[7]
        # A collision can happen between the three sensor rays.
        hit = self.stuck or self.collision_count > self.last_collision_count
        self.last_collision_count = self.collision_count
        no_progress = self.has_no_progress()
        if hit or no_progress:
            self.escape_steps = self.ESCAPE_TURNS + self.ESCAPE_MOVES
            if self.avoid_direction == 0:
                self.avoid_direction = -1 if right < left else 1

        if self.escape_steps > 0:
            self.escape_wall(front, left, right)
            return

        if front > self.SAFETY_DISTANCE:
            self.avoid_direction = 0

        # Rule 1: all three sensors are safe -> move and turn toward food.
        if (front > self.SAFETY_DISTANCE
                and right > self.SAFETY_DISTANCE
                and left > self.SAFETY_DISTANCE):
            food_angle = self.smellAllFoods()
            # Negative smell means left; positive smell means right.
            turn_angle = max(-self.FOOD_TURN_STEP, min(food_angle, self.FOOD_TURN_STEP))
            self.turn(turn_angle)
            # Move slowly when food is far to the side or behind us.
            if abs(food_angle) <= 30:
                self.moveForward()
            else:
                self.moveForward(self.SLOW_MOVE_STEP)

        # Rule 2: front is safe and both sides have at least 5 pixels -> move only.
        elif (front > self.SAFETY_DISTANCE
                and right >= self.CLOSED_DISTANCE
                and left >= self.CLOSED_DISTANCE):
            self.moveForward()

        # Rule 3: front is safe, but at least one side is below 5 -> turn only.
        elif front > self.SAFETY_DISTANCE:
            self.turn_away_from_obstacle(left, right)

        # Rule 4: front is at or below 30 (including HIT_DISTANCE) -> turn only.
        else:
            # Choose once, then keep turning until the front is safe.
            if self.avoid_direction == 0:
                self.avoid_direction = -1 if right < left else 1
            self.turn(self.avoid_direction * self.TURN_STEP)

    def has_no_progress(self):
        """Detect turning in place, even when the simulator reports no hit."""
        # Let recovery finish; do not restart it just because it is turning.
        if self.progress_position is None or self.escape_steps > 0:
            self.progress_position = tuple(self.pos)
            self.no_progress_ticks = 0
            return False

        dx = self.pos[0] - self.progress_position[0]
        dy = self.pos[1] - self.progress_position[1]
        if dx * dx + dy * dy >= self.PROGRESS_DISTANCE ** 2:
            self.progress_position = tuple(self.pos)
            self.no_progress_ticks = 0
        else:
            self.no_progress_ticks += 1

        if self.no_progress_ticks >= self.NO_PROGRESS_TICKS:
            self.no_progress_ticks = 0
            return True
        return False

    def escape_wall(self, front, left, right):
        """After a hit or no progress: turn, find an exit, and move clear."""
        if self.escape_steps > self.ESCAPE_MOVES:
            self.turn(self.avoid_direction * self.TURN_STEP)
            self.escape_steps -= 1
        elif (front > self.SAFETY_DISTANCE
                and min(left, right) >= self.CLOSED_DISTANCE):
            self.moveForward(self.ESCAPE_MOVE_STEP)
            self.escape_steps -= 1
        else:
            self.turn(self.avoid_direction * self.ESCAPE_SCAN_TURN)

    def turn_away_from_obstacle(self, left, right):
        """Turn toward the side with more space. Turn right if tied."""
        # Use a smaller turn when both sides are confined.
        turn_size = self.ESCAPE_SCAN_TURN if max(left, right) <= self.SAFETY_DISTANCE else self.TURN_STEP
        if right < left:
            self.spinLeft(turn_size)
        else:
            self.spinRight(turn_size)


def after_simulation(simbot):
    robot = simbot.robots[0]
    Logger.info("Mission3: simulation finished")
    Logger.info("Mission3: food eaten=%d" % robot.eat_count)
    Logger.info("Mission3: collisions=%d" % robot.collision_count)
    Logger.info("Mission3: score=%s" % simbot.scoreStr)


if __name__ == "__main__":
    app = PySimbotApp(
        robot_cls=Mission3SimbotRobot,
        num_robots=1,
        num_objectives=1,
        map="default",
        interval=1.0 / 60.0,
        max_tick=5000,
        food_move_after_eat=True,
        simulation_forever=False,
        enable_wasd_control=False,
        customfn_after_simulation=after_simulation,
    )
    app.run()
