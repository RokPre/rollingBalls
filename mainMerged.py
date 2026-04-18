"""Standalone merged foosball agent.

This file is intended to be the editable starting point for future work.
It combines:
- Avis defensive logic for rods 1 and 2
- Rok midfield and attack logic for rods 4 and 6

The public API matches the other agents in this repository:
- ``init()``
- ``main(sim_state)``
"""
from __future__ import annotations

import json
from math import atan
from math import pi
from math import tan
from typing import cast
from typing import Literal
from typing import TypedDict


URL = "http://localhost:8080"
FPS = 50
ROD_HEIGHT = 121.5
PERIOD = 1 / FPS
PLAYER_HEIGHT = 74.48
PLAYER_WIDTH = 20
BALL_PRED_COUNT = 500
REACH = 50


class RodGeom(TypedDict):
    id: int
    team: Literal["red", "blue"]
    position: int
    travel: int
    players: int
    first_offset: int
    spacing: int


class FieldGeom(TypedDict):
    dimension_x: int
    dimension_y: int


class Geometry(TypedDict):
    field: FieldGeom
    goal_width: int
    ball_size: int
    rods: list[RodGeom]


SimState = tuple[float, float, float, float, list[float], list[float]]
MotorCommand = tuple[int, float, float, float, float]


with open("geometry.json", "r", encoding="utf-8") as geometry_file:
    GEOM = cast(Geometry, json.load(geometry_file))


BALL_SIZE = int(GEOM["ball_size"])
FIELD_X = float(GEOM["field"]["dimension_x"])
FIELD_Y = float(GEOM["field"]["dimension_y"])
GOAL_W = float(GEOM["goal_width"])
GOAL_TOP_Y = (FIELD_Y - GOAL_W) / 2.0
GOAL_BOT_Y = (FIELD_Y + GOAL_W) / 2.0
GOAL_MID_Y = FIELD_Y / 2.0
CORNER_PAD = 0.3 * BALL_SIZE

ROT_BLOCK = 0.0
ROT_LIFT = 0.25
ROT_STRIKE = -0.25
ROT_VEL = 1.0
TRANS_VEL = 1.0


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class Rod:
    def __init__(self, rod_id: int):
        self.rod_id = rod_id
        if rod_id == 1:
            self.drive_id = 1
        elif rod_id == 2:
            self.drive_id = 2
        elif rod_id == 4:
            self.drive_id = 3
        elif rod_id == 6:
            self.drive_id = 4
        else:
            raise ValueError(f"Unsupported controllable rod_id: {rod_id}")

        self.state = "idle"

        rod_geom = GEOM["rods"][rod_id - 1]
        self.team = str(rod_geom["team"])
        self.travel = int(rod_geom["travel"])
        self.players = int(rod_geom["players"])
        self.spacing = int(rod_geom["spacing"])
        self.position = int(rod_geom["position"])
        self.first_offset = int(rod_geom["first_offset"])

        self.player_min_max_y: list[tuple[int, int]] = self.players * [(0, 0)]
        for i in range(self.players):
            player_y_min = self.first_offset + self.spacing * i
            player_y_max = player_y_min + self.travel
            self.player_min_max_y[i] = (player_y_min, player_y_max)

        self.translation_r = 0.0
        self.translation_y: list[float] = self.players * [0.0]
        self.rotation = 0.0
        self.player_x = 0.0
        self.player_vx = 0.0
        self.ball_pred_pos: list[tuple[float, float]] = BALL_PRED_COUNT * [(0.0, 0.0)]

    def determine_state(self, ball_x: float, ball_y: float) -> None:
        if not (0 <= ball_x <= FIELD_X and 0 <= ball_y <= FIELD_Y):
            self.state = "idle"
            return

        if 0 <= ball_x < self.position - REACH:
            self.state = "avoid"
        elif self.position - REACH <= ball_x < self.position + REACH:
            self.state = "possession"
        else:
            self.state = "defend"

    def update_variables(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float, rod_pos: list[float], rod_rot: list[float]) -> None:
        self.translation_r = rod_pos[self.rod_id - 1]
        self.rotation = rod_rot[self.rod_id - 1]

        for i in range(self.players):
            self.translation_y[i] = self.player_min_max_y[i][0] + self.travel * self.translation_r

        self.ball_pred_pos = BALL_PRED_COUNT * [(0.0, 0.0)]
        for i in range(BALL_PRED_COUNT):
            ball_pred_x = ball_x + PERIOD * ball_vx * i
            ball_pred_y = ball_y + PERIOD * ball_vy * i
            self.ball_pred_pos[i] = (ball_pred_x, ball_pred_y)

        prev_player_x = self.player_x
        self.player_x = self.position + tan(self.rotation * 2 * pi) * ROD_HEIGHT
        self.player_vx = (self.player_x - prev_player_x) / PERIOD

    def can_player_reach_y(self, player_id: int, y: float) -> bool:
        if not (1 <= player_id <= self.players):
            raise ValueError(f"player_id should be between 1 and {self.players}")
        lo, hi = self.player_min_max_y[player_id - 1]
        return lo <= y <= hi

    def players_by_distance_from_y(self, y: float) -> list[int]:
        sorted_list = sorted(
            range(len(self.translation_y)),
            key=lambda i: abs(y - self.translation_y[i]),
        )
        return [i + 1 for i in sorted_list]

    def player_pos_y_to_relative(self, player_id: int, y: float) -> float:
        if not (1 <= player_id <= self.players):
            raise ValueError(f"player_id should be between 1 and {self.players}")

        player_min_y = self.first_offset + self.spacing * (player_id - 1)
        player_max_y = player_min_y + self.travel
        return clamp((y - player_min_y) / (player_max_y - player_min_y), 0.0, 1.0)

    def move_any_player_to_y(self, y: float) -> float:
        players_by_distance = self.players_by_distance_from_y(y)
        for player in players_by_distance:
            if self.can_player_reach_y(player, y):
                return self.player_pos_y_to_relative(player, y)
        return self.player_pos_y_to_relative(players_by_distance[0], y)

    def move_any_player_to_x(self, target_x: float) -> float:
        dx = self.position - target_x
        dz = PLAYER_HEIGHT - BALL_SIZE / 2
        phi = atan(dx / dz)
        return phi / (2 * pi)

    def ball_pred_pos_from_x(self, x: float) -> list[tuple[float, float]]:
        sorted_list = sorted(
            range(len(self.ball_pred_pos)),
            key=lambda i: abs(x - self.ball_pred_pos[i][0]),
        )
        return [self.ball_pred_pos[i] for i in sorted_list]

    def can_shoot(self, ball_x: float, ball_y: float) -> bool:
        ball_in_front_of_player = any(
            abs(py - ball_y) < (BALL_SIZE / 2 + PLAYER_WIDTH / 2)
            for py in self.translation_y
        )
        return (self.player_x < ball_x) and ball_in_front_of_player

    def attack(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float) -> tuple[float, float, float, float]:
        translation_r = self.move_any_player_to_y(ball_y)

        # Ball is slow, hit it so that it does not get stuck.
        if abs(ball_vx) < 0.05 and abs(ball_vy) < 0.05 and self.can_shoot(ball_x, ball_y):
            return translation_r, 1.0, ROT_STRIKE, 1.0

        if abs(ball_vy) < 0.3:
            if ball_y < FIELD_Y / 2:
                lower_bound = PLAYER_WIDTH / 2
                upper_bound = lower_bound + BALL_SIZE / 2
                middle_bound = (lower_bound + upper_bound) / 2
            else:
                lower_bound = -PLAYER_WIDTH / 2 - BALL_SIZE / 2
                upper_bound = lower_bound + BALL_SIZE / 2
                middle_bound = (lower_bound + upper_bound) / 2

            translation_r = self.move_any_player_to_y(ball_y - middle_bound)
            if any(player_y + lower_bound < ball_y < player_y + upper_bound for player_y in self.translation_y):
                return translation_r, 1.0, ROT_STRIKE, 1.0

        if self.can_shoot(ball_x, ball_y):
            rotation = -0.2
        else:
            rotation = self.move_any_player_to_x(ball_x - BALL_SIZE)  # Wind up before shoot.
            rotation = max(0.0, rotation)

        return translation_r, 1.0, rotation, 1.0

    def pass_forward(self, ball_x: float, ball_y: float) -> tuple[float, float, float, float]:
        translation_r = self.move_any_player_to_y(ball_y)
        if self.can_shoot(ball_x, ball_y):
            return translation_r, 1.0, -0.2, 1.0

        if self.can_shoot(ball_x, ball_y):
            rotation = ROT_STRIKE
        else:
            rotation = self.move_any_player_to_x(ball_x - BALL_SIZE)  # Wind up before shoot.

        rotation_v = abs(self.player_x - ball_x) / (REACH / 2)
        return translation_r, 1.0, rotation, rotation_v

    def offense_defend(self, ball_x: float, ball_vx: float) -> tuple[float, float, float, float]:
        sorted_ball_pos = self.ball_pred_pos_from_x(self.position)
        _, predicted_y = sorted_ball_pos[0]
        translation_r = self.move_any_player_to_y(predicted_y)

        distance_to_ball = abs(ball_x - self.position) / FIELD_X
        limit = (1 - distance_to_ball) / 2
        translation_r = min(max(translation_r, 0.5 - limit), 0.5 + limit)

        return translation_r, 1.0, ROT_BLOCK, 1.0

    def offense_avoid(self, ball_x: float) -> tuple[float, float, float, float]:
        sorted_ball_pos = self.ball_pred_pos_from_x(self.position)
        _, predicted_y = sorted_ball_pos[0]
        translation_r = self.move_any_player_to_y(predicted_y)

        distance_to_ball = abs(ball_x - self.position) / FIELD_X
        limit = (1 - distance_to_ball) / 2
        translation_r = min(max(translation_r, 0.5 - limit), 0.5 + limit)

        return translation_r, 1.0, ROT_LIFT, 1.0

    def defense_targets(self, ball_y: float) -> tuple[float, float]:
        upper_corner = GOAL_TOP_Y + CORNER_PAD
        lower_corner = GOAL_BOT_Y - CORNER_PAD

        if ball_y < GOAL_MID_Y:
            keeper_y = upper_corner
            defender_y = keeper_y + BALL_SIZE
        else:
            keeper_y = lower_corner
            defender_y = keeper_y - BALL_SIZE

        keeper_y = clamp(keeper_y, 0.0, FIELD_Y)
        defender_y = clamp(defender_y, 0.0, FIELD_Y)
        return keeper_y, defender_y

    def rod_translation_for_target_y(self, target_y: float) -> float:
        best_cal = 0.5
        best_err = float("inf")
        for ip in range(self.players):
            pos_needed = target_y - self.first_offset - ip * self.spacing
            cal = pos_needed / self.travel if self.travel > 1e-9 else 0.5
            cal = clamp(cal, 0.0, 1.0)
            player_y = self.first_offset + cal * self.travel + ip * self.spacing
            err = abs(player_y - target_y)
            if err < best_err:
                best_err = err
                best_cal = cal
        return best_cal

    def slow_ball_is_dangerous(self, ball_x: float, ball_vx: float) -> bool:
        return ball_x < 320.0 and ball_vx < 120.0

    def goalie_defend(self, ball_x: float, ball_y: float, ball_vx: float) -> tuple[float, float, float, float]:
        keeper_y, _ = self.defense_targets(ball_y)
        target_y = ball_y if self.slow_ball_is_dangerous(ball_x, ball_vx) else keeper_y
        translation_r = self.rod_translation_for_target_y(target_y)

        aligned = abs(ball_y - self.translation_y[0]) < (PLAYER_WIDTH + BALL_SIZE) / 2
        if self.player_x < ball_x and aligned and abs(ball_x - self.position) < 65:
            rotation = ROT_STRIKE
        else:
            rotation = self.move_any_player_to_x(ball_x)
            rotation = max(0.0, rotation)

        return translation_r, TRANS_VEL, rotation, ROT_VEL

    def defender_cover(self, ball_x: float, ball_y: float, ball_vx: float) -> tuple[float, float, float, float]:
        _, defender_y = self.defense_targets(ball_y)
        target_y = ball_y if self.slow_ball_is_dangerous(ball_x, ball_vx) else defender_y
        translation_r = self.rod_translation_for_target_y(target_y)

        aligned = any(abs(ball_y - py) < (PLAYER_WIDTH + BALL_SIZE) / 2 for py in self.translation_y)
        if aligned and abs(ball_x - self.position) < 70:
            rotation = ROT_STRIKE
        else:
            rotation = ROT_BLOCK

        return translation_r, TRANS_VEL, rotation, ROT_VEL

    def step(self, sim_state: SimState) -> MotorCommand:
        ball_x, ball_y, ball_vx, ball_vy, rod_pos, rod_rot = sim_state
        self.update_variables(ball_x, ball_y, ball_vx, ball_vy, rod_pos, rod_rot)
        self.determine_state(ball_x, ball_y)

        if self.rod_id == 1:
            if self.state in {"avoid", "possession", "defend"}:
                tran, tran_vel, rot, rot_vel = self.goalie_defend(ball_x, ball_y, ball_vx)
            else:
                tran, tran_vel, rot, rot_vel = 0.5, 0.5, ROT_BLOCK, 0.5
        elif self.rod_id == 2:
            if self.state in {"possession", "defend"}:
                tran, tran_vel, rot, rot_vel = self.defender_cover(ball_x, ball_y, ball_vx)
            elif self.state == "avoid":
                tran, tran_vel, rot, rot_vel = self.offense_avoid(ball_x)
            else:
                tran, tran_vel, rot, rot_vel = 0.5, 0.5, ROT_BLOCK, 0.5
        else:
            match self.state:
                case "avoid":
                    tran, tran_vel, rot, rot_vel = self.offense_avoid(ball_x)
                case "possession":
                    if self.rod_id == 4:
                        tran, tran_vel, rot, rot_vel = self.pass_forward(ball_x, ball_y)
                    elif self.rod_id == 6:
                        tran, tran_vel, rot, rot_vel = self.attack(ball_x, ball_y, ball_vx, ball_vy)
                case "defend":
                    tran, tran_vel, rot, rot_vel = self.offense_defend(ball_x, ball_vx)
                case _:
                    tran, tran_vel, rot, rot_vel = 0.0, 0.0, 0.0, 0.0

        return (self.drive_id, tran, rot, tran_vel, rot_vel)


def init() -> None:
    global rods
    rods = [Rod(rod_id) for rod_id in [1, 2, 4, 6]]


def _get_rods() -> list[Rod]:
    if "rods" not in globals():
        init()
    return cast(list[Rod], rods)


def main(sim_state: SimState) -> list[MotorCommand]:
    motor_commands: list[MotorCommand] = []
    for rod in _get_rods():
        motor_commands.append(rod.step(sim_state))
    return motor_commands


init()
