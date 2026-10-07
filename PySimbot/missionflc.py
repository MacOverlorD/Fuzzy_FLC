#!/usr/bin/env python3
"""Mission 4: collect food with a fuzzy logic controller.

The controller follows the Mission 4 assignment and the course slides:

* inputs: five useful sectors from the eight IR sensors and the food bearing;
* fuzzy sets: Near/Far for distance and Left/Center/Right for bearing;
* inference: product T-norm over the eight-rule safety/goal rule base; and
* defuzzification: zero-order Takagi-Sugeno weighted averages.

The simulator configuration deliberately keeps the same ``default`` map and
5000-tick mission settings used by ``missionssw.py``.
"""

import os
import platform
from collections import deque

if platform.system() in ("Linux", "Darwin"):
    os.environ["KIVY_VIDEO"] = "ffpyplayer"

from kivy.config import Config
from kivy.logger import Logger
from pysimbotlib.core import PySimbotApp, Robot


Config.set("kivy", "log_level", "info")
Config.set("graphics", "maxfps", "60")


class Mission4FuzzyRobot(Robot):
    """Navigate with a zero-order Takagi-Sugeno fuzzy controller."""

    # Distance membership reaches fully Far at 60 px, as defined in the slides.
    DISTANCE_FAR = 60.0
    SMELL_CENTER_WIDTH = 45.0
    EPSILON = 1.0e-5

    MAX_TURN = 60.0
    MAX_MOVE = 10.0

    # Stateful safety reflex from the slides.  It is only used after a physical
    # collision/stall or a detected local loop; normal navigation stays fuzzy.
    RECOVERY_TURN = 75.0
    RECOVERY_MOVE = -2
    NO_PROGRESS_TICKS = 20
    PROGRESS_DISTANCE = 3.0
    LOOP_WINDOW = 120
    LOOP_RADIUS = 20.0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.last_collision_count = 0
        self.consecutive_stuck = 0
        self.progress_position = None
        self.no_progress_ticks = 0
        self.position_history = deque(maxlen=self.LOOP_WINDOW)
        self.history_eat_count = 0
        self.loop_escape_direction = 1

    # Assignment-style names mapped to the PySimbot API.
    def getIRValues(self):
        return self.distance()

    def smellAllFoods(self):
        return self.smell()

    @staticmethod
    def _clip(value, low=0.0, high=1.0):
        return max(low, min(float(value), high))

    @classmethod
    def distance_memberships(cls, distance):
        """Return ``(near, far)`` degrees for an IR distance in pixels."""
        far = cls._clip(distance / cls.DISTANCE_FAR)
        return 1.0 - far, far

    @classmethod
    def smell_memberships(cls, angle):
        """Return ``(left, center, right)`` for a signed food bearing."""
        # PySimbot already reports the bearing in [-180, 180].  Normalizing here
        # also makes this method safe to call with an equivalent wrapped angle.
        angle = ((float(angle) + 180.0) % 360.0) - 180.0
        left = cls._clip(-angle / cls.SMELL_CENTER_WIDTH)
        center = cls._clip(
            (cls.SMELL_CENTER_WIDTH - abs(angle)) / cls.SMELL_CENTER_WIDTH
        )
        right = cls._clip(angle / cls.SMELL_CENTER_WIDTH)
        return left, center, right

    @classmethod
    def fuzzy_decision(cls, ir_values, smell_angle):
        """Fuzzify inputs, evaluate the rule base, and return crisp commands.

        The return value is ``(turn_degrees, move_pixels, fired_rules)``.  The
        third item is useful for explaining or testing the controller and does
        not affect the simulator.
        """
        if len(ir_values) < 8:
            raise ValueError("PySimbot must provide all eight IR sensor values")

        # PySimbot sensor layout: 0 front, 1 front-right, 2 right,
        # 6 left, and 7 front-left.  Rear sensors are available but are not
        # needed by the course rule base.
        front_near, front_far = cls.distance_memberships(ir_values[0])
        front_right_near, _ = cls.distance_memberships(ir_values[1])
        right_near, right_far = cls.distance_memberships(ir_values[2])
        left_near, left_far = cls.distance_memberships(ir_values[6])
        front_left_near, _ = cls.distance_memberships(ir_values[7])
        smell_left, smell_center, smell_right = cls.smell_memberships(smell_angle)

        # R1 dynamically chooses one escape direction so equal left/right
        # evidence cannot cancel into a zero-turn deadlock.
        if left_far >= right_far:
            emergency_turn = -60.0
            emergency_opening = left_far
        else:
            emergency_turn = 60.0
            emergency_opening = right_far

        # Goal-seeking rules are gated by front/corner clearance.  Therefore a
        # food bearing cannot override an imminent frontal collision reflex.
        clear_path = front_far * (
            1.0 - max(front_right_near, front_left_near)
        )

        # (name, firing strength, singleton turn, singleton move)
        rules = [
            ("R1 emergency front", front_near * emergency_opening,
             emergency_turn, 1.0),
            ("R2 front-right clearance", front_right_near, -45.0, 2.0),
            ("R3 front-left clearance", front_left_near, 45.0, 2.0),
            ("R4 right-wall nudge", right_near, -20.0, 5.0),
            ("R5 left-wall nudge", left_near, 20.0, 5.0),
            ("R6 target ahead", clear_path * smell_center, 0.0, 10.0),
            ("R7 target right", clear_path * smell_right, 30.0, 7.0),
            ("R8 target left", clear_path * smell_left, -30.0, 7.0),
        ]

        total_weight = sum(weight for _, weight, _, _ in rules)
        turn = sum(weight * output for _, weight, output, _ in rules)
        move = sum(weight * output for _, weight, _, output in rules)
        turn /= total_weight + cls.EPSILON
        move /= total_weight + cls.EPSILON

        # These clamps are redundant for a weighted average of the listed
        # singletons, but they make the actuator contract explicit and safe.
        turn = cls._clip(turn, -cls.MAX_TURN, cls.MAX_TURN)
        move = cls._clip(move, 0.0, cls.MAX_MOVE)
        fired_rules = tuple(
            (name, weight) for name, weight, _, _ in rules if weight > cls.EPSILON
        )
        return turn, move, fired_rules

    def _has_no_progress(self):
        """Detect prolonged rotation/stall around one small position."""
        current = tuple(self.pos)
        if self.progress_position is None:
            self.progress_position = current
            return False

        dx = current[0] - self.progress_position[0]
        dy = current[1] - self.progress_position[1]
        if dx * dx + dy * dy >= self.PROGRESS_DISTANCE ** 2:
            self.progress_position = current
            self.no_progress_ticks = 0
            return False

        self.no_progress_ticks += 1
        if self.no_progress_ticks < self.NO_PROGRESS_TICKS:
            return False

        self.progress_position = current
        self.no_progress_ticks = 0
        return True

    def _has_local_loop(self):
        """Notice a closed local orbit that has not collected new food."""
        current = tuple(self.pos)
        if self.eat_count != self.history_eat_count:
            self.history_eat_count = self.eat_count
            self.position_history.clear()

        looped = False
        if len(self.position_history) == self.position_history.maxlen:
            old_x, old_y = self.position_history[0]
            dx = current[0] - old_x
            dy = current[1] - old_y
            looped = dx * dx + dy * dy <= self.LOOP_RADIUS ** 2

        self.position_history.append(current)
        if looped:
            self.position_history.clear()
        return looped

    def _recovery_command(self, collision, stalled, looped):
        """Return a stateful symmetry-breaking command, or ``None``."""
        if not (collision or stalled or looped):
            self.consecutive_stuck = 0
            return None

        if collision:
            self.consecutive_stuck += 1
            direction = 1 if self.consecutive_stuck % 2 == 1 else -1
        else:
            # Alternate the tangential bias each time a local minimum recurs.
            direction = self.loop_escape_direction
            self.loop_escape_direction *= -1

        return direction * self.RECOVERY_TURN, self.RECOVERY_MOVE

    def update(self):
        ir_values = self.getIRValues()
        smell_angle = self.smellAllFoods()

        collision = (
            self.stuck
            or self.collision_count > self.last_collision_count
        )
        self.last_collision_count = self.collision_count
        stalled = self._has_no_progress()
        looped = self._has_local_loop()
        recovery = self._recovery_command(collision, stalled, looped)

        if recovery is None:
            turn, move, _ = self.fuzzy_decision(ir_values, smell_angle)
        else:
            turn, move = recovery

        # PySimbot uses negative turn for left and positive turn for right.
        self.turn(turn)
        # Robot.move() accepts a float but internally truncates it.  Rounding
        # avoids turning a fuzzy singleton such as 10 px into 9 px only because
        # the defuzzification denominator contains the small safety epsilon.
        self.move(int(round(move)))


def after_simulation(simbot):
    robot = simbot.robots[0]
    Logger.info("Mission4FLC: simulation finished")
    Logger.info("Mission4FLC: food eaten=%d" % robot.eat_count)
    Logger.info("Mission4FLC: collisions=%d" % robot.collision_count)
    Logger.info("Mission4FLC: score=%s" % simbot.scoreStr)


if __name__ == "__main__":
    app = PySimbotApp(
        robot_cls=Mission4FuzzyRobot,
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
