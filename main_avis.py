"""Avis foosball agent module.

The public entrypoint is `main(sim_state)`, which makes this file importable by
`compare.py` and `run.py`. Running the module directly still supports the
external localhost HTTP loop for manual use outside the simulator process.
"""

from __future__ import annotations

import json
from math import atan
from math import pi
from math import tan
from typing import cast
from typing import Literal
from typing import TypedDict

import requests

# Constants
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


with open("geometry.json", "r", encoding="utf-8") as g:
    GEOM = cast(Geometry, json.load(g))

BALL_SIZE: int = int(GEOM["ball_size"])
SimState = tuple[float, float, float, float, list[float], list[float]]
MotorCommand = tuple[int, float, float, float, float]


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


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


class rod:
    def __init__(self, rod_id: int):
        self.rod_id: int = rod_id
        self.driveID: int
        if rod_id == 1:
            self.driveID = 1
        elif rod_id == 2:
            self.driveID = 2
        elif rod_id == 4:
            self.driveID = 3
        elif rod_id == 6:
            self.driveID = 4
        else:
            raise ValueError(f"Unsupported controllable rod_id: {rod_id}")

        self.state: str = "idle"

        rod_geom: RodGeom = GEOM["rods"][rod_id - 1]
        self.team: str = str(rod_geom["team"])
        self.travel: int = int(rod_geom["travel"])
        self.players: int = int(rod_geom["players"])
        self.spacing: int = int(rod_geom["spacing"])
        self.position: int = int(rod_geom["position"])
        self.first_offset: int = int(rod_geom["first_offset"])

        self.player_min_max_y: list[tuple[int, int]] = self.players * [(0, 0)]
        for i in range(self.players):
            player_y_min = self.first_offset + self.spacing * i
            player_y_max = player_y_min + self.travel
            self.player_min_max_y[i] = (player_y_min, player_y_max)

        self.translation_r: float = 0.0
        self.translation_y: list[float] = self.players * [0.0]
        self.rotation: float = 0.0
        self.player_x: float = 0.0
        self.player_vx: float = 0.0
        self.ball_pred_pos: list[tuple[float, float]] = BALL_PRED_COUNT * [(0.0, 0.0)]

    def determine_state(self, ball_x: float, ball_y: float):
        if not (0 <= ball_x <= FIELD_X and 0 <= ball_y <= FIELD_Y):
            self.state = "idle"
            return

        if 0 <= ball_x < self.position - REACH:
            self.state = "avoid"
        elif self.position - REACH <= ball_x < self.position + REACH:
            self.state = "possession"
        else:
            self.state = "defend"

    def update_variables(
        self,
        ball_x: float,
        ball_y: float,
        ball_vx: float,
        ball_vy: float,
        rod_pos: list[float],
        rod_rot: list[float],
    ):
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
        sorted_list: list[int] = sorted(
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
        sorted_list: list[int] = sorted(
            range(len(self.ball_pred_pos)),
            key=lambda i: abs(x - self.ball_pred_pos[i][0]),
        )
        return [self.ball_pred_pos[i] for i in sorted_list]

    def can_shoot(self, ball_x: float, ball_y: float) -> bool:
        ball_in_front_of_player = any(
            abs(py - ball_y) < (BALL_SIZE / 2 + PLAYER_WIDTH / 2)
            for py in self.translation_y
        )
        return self.player_x < ball_x and ball_in_front_of_player

    # ---------- YOUR ATTACK / FORWARD PLAY ----------
    def attack(
        self,
        ball_x: float,
        ball_y: float,
        ball_vx: float,
        ball_vy: float,
        rod_pos: list[float],
        rod_rot: list[float],
    ) -> tuple[float, float, float, float]:
        translation_r = self.move_any_player_to_y(ball_y)

        # enemy rod in front of attacker: rod 7 when we control rod 6
        enemy_rod_translation_r: float = rod_pos[self.rod_id]

        enemy_geom = GEOM["rods"][self.rod_id]
        enemy_first_offset = int(enemy_geom["first_offset"])
        enemy_players = int(enemy_geom["players"])
        enemy_spacing = int(enemy_geom["spacing"])
        enemy_travel = int(enemy_geom["travel"])

        enemy_translation_y = [
            enemy_first_offset + enemy_spacing * i + enemy_travel * enemy_rod_translation_r
            for i in range(enemy_players)
        ]

        # keep this variable for possible future tuning, but do not force logic with it now
        _enemy_player_in_line_with_ball = any(
            abs(ball_y - player_y) < (BALL_SIZE / 2 + PLAYER_WIDTH)
            for player_y in enemy_translation_y
        )

        # very slow / stuck ball -> just hit it
        if abs(ball_vx) < 0.05 and abs(ball_vy) < 0.05:
            return translation_r, 1.0, ROT_STRIKE, 1.0

        # side-hit for slightly diagonal shot
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
            return translation_r, 1.0, ROT_BLOCK, 1.0

        player_behind_ball = (self.player_x < ball_x) and (abs(ball_x - self.player_x) < REACH)
        player_in_line_with_ball = any(
            abs(ball_y - player_y) < (BALL_SIZE / 2 + PLAYER_WIDTH)
            for player_y in self.translation_y
        )

        rotation = ROT_BLOCK
        if player_behind_ball and player_in_line_with_ball:
            rotation = -0.2
        elif not player_in_line_with_ball:
            rotation = self.move_any_player_to_x(ball_x - BALL_SIZE / 2)
            rotation = max(0.0, rotation)

        return translation_r, 1.0, rotation, 1.0

    def pass_forward(self, ball_x: float, ball_y: float) -> tuple[float, float, float, float]:
        translation_r = self.move_any_player_to_y(ball_y)
        if self.can_shoot(ball_x, ball_y):
            return translation_r, 1.0, -0.2, 1.0

        rotation = self.move_any_player_to_x(ball_x - BALL_SIZE)
        rotation_v = abs(self.player_x - ball_x) / (REACH / 2)
        return translation_r, 1.0, rotation, rotation_v

    # ---------- OUR DEFENSE ----------
    def rod_translation_for_target_y(self, target_y: float) -> float:
        best_cal = 0.5
        best_err = float("inf")
        for ip in range(self.players):
            pos_needed = target_y - self.first_offset - ip * self.spacing
            cal = pos_needed / self.travel if self.travel > 1e-9 else 0.5
            cal = clamp(cal, 0.0, 1.0)
            py = self.first_offset + cal * self.travel + ip * self.spacing
            err = abs(py - target_y)
            if err < best_err:
                best_err = err
                best_cal = cal
        return best_cal

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

    def slow_ball_is_dangerous(self, ball_x: float, ball_vx: float) -> bool:
        return ball_x < 320.0 and ball_vx < 120.0

    def goalie_defend(self, ball_x: float, ball_y: float, ball_vx: float) -> tuple[float, float, float, float]:
        keeper_y, _ = self.defense_targets(ball_y)

        # For slow balls near our goal, track the real ball more directly
        if self.slow_ball_is_dangerous(ball_x, ball_vx):
            target_y = ball_y
        else:
            target_y = keeper_y

        translation_r = self.rod_translation_for_target_y(target_y)

        # strike if ball is really in front of goalie
        aligned = abs(ball_y - self.translation_y[0]) < (PLAYER_WIDTH + BALL_SIZE) / 2
        if self.player_x < ball_x and aligned and abs(ball_x - self.position) < 65:
            rotation = ROT_STRIKE
        else:
            rotation = ROT_BLOCK

        return translation_r, TRANS_VEL, rotation, ROT_VEL

    def defender_cover(self, ball_x: float, ball_y: float, ball_vx: float) -> tuple[float, float, float, float]:
        _, defender_y = self.defense_targets(ball_y)

        if self.slow_ball_is_dangerous(ball_x, ball_vx):
            target_y = ball_y
        else:
            target_y = defender_y

        translation_r = self.rod_translation_for_target_y(target_y)

        # hit slow close balls instead of just watching them pass
        aligned = any(abs(ball_y - py) < (PLAYER_WIDTH + BALL_SIZE) / 2 for py in self.translation_y)
        if aligned and abs(ball_x - self.position) < 70:
            rotation = ROT_STRIKE
        else:
            rotation = ROT_BLOCK

        return translation_r, TRANS_VEL, rotation, ROT_VEL

    def avoid(self, ball_x: float, ball_y: float, ball_vx: float) -> tuple[float, float, float, float]:
        if self.rod_id == 1:
            return self.goalie_defend(ball_x, ball_y, ball_vx)
        if self.rod_id == 2:
            return self.defender_cover(ball_x, ball_y, ball_vx)

        sorted_ball_pos = self.ball_pred_pos_from_x(self.position)
        _, pred_y = sorted_ball_pos[0]
        translation_r = self.move_any_player_to_y(pred_y)

        distance_to_ball = abs(ball_x - self.position) / FIELD_X
        limit = (1 - distance_to_ball) / 2
        translation_r = min(max(translation_r, 0.5 - limit), 0.5 + limit)

        return translation_r, 1.0, ROT_LIFT, 1.0

    def defend(self, ball_x: float, ball_y: float, ball_vx: float) -> tuple[float, float, float, float]:
        if self.rod_id == 1:
            return self.goalie_defend(ball_x, ball_y, ball_vx)
        if self.rod_id == 2:
            return self.defender_cover(ball_x, ball_y, ball_vx)

        sorted_ball_pos = self.ball_pred_pos_from_x(self.position)
        _, pred_y = sorted_ball_pos[0]
        translation_r = self.move_any_player_to_y(pred_y)

        distance_to_ball = abs(ball_x - self.position) / FIELD_X
        limit = (1 - distance_to_ball) / 2
        translation_r = min(max(translation_r, 0.5 - limit), 0.5 + limit)

        return translation_r, 1.0, ROT_BLOCK, 1.0

    def step(self, sim_state: tuple[float, float, float, float, list[float], list[float]]) -> tuple[int, float, float, float, float]:
        ball_x, ball_y, ball_vx, ball_vy, rod_pos, rod_rot = sim_state
        self.update_variables(ball_x, ball_y, ball_vx, ball_vy, rod_pos, rod_rot)
        self.determine_state(ball_x, ball_y)

        match self.state:
            case "avoid":
                if self.rod_id in (1, 2):
                    tran, tran_vel, rot, rot_vel = self.avoid(ball_x, ball_y, ball_vx)
                else:
                    tran, tran_vel, rot, rot_vel = self.avoid(ball_x, ball_y, ball_vx)

            case "possession":
                if self.rod_id == 1:
                    tran, tran_vel, rot, rot_vel = self.goalie_defend(ball_x, ball_y, ball_vx)
                elif self.rod_id == 2:
                    # keep defense rod defensive even in possession zone
                    tran, tran_vel, rot, rot_vel = self.defender_cover(ball_x, ball_y, ball_vx)
                elif self.rod_id == 6:
                    tran, tran_vel, rot, rot_vel = self.attack(ball_x, ball_y, ball_vx, ball_vy, rod_pos, rod_rot)
                else:
                    tran, tran_vel, rot, rot_vel = self.pass_forward(ball_x, ball_y)

            case "defend":
                tran, tran_vel, rot, rot_vel = self.defend(ball_x, ball_y, ball_vx)

            case _:
                tran, tran_vel, rot, rot_vel = 0.5, 0.5, ROT_BLOCK, 0.5

        return (self.driveID, tran, rot, tran_vel, rot_vel)


def init() -> None:
    """Reset the module-local rod state used by the public entrypoint."""
    global rods
    rods = [rod(i) for i in [1, 2, 4, 6]]


def _get_rods() -> list[rod]:
    """Return initialized rods, creating them on first use."""
    if "rods" not in globals():
        init()
    return cast(list[rod], rods)


def _validate_sim_state(sim_state: SimState) -> None:
    """Validate the state shape expected by the module entrypoint."""
    if len(sim_state) != 6:
        raise ValueError(
            "sim_state must contain 6 items: ball_x, ball_y, ball_vx, ball_vy, rod_positions, rod_rotations."
        )

    _, _, _, _, rod_positions, rod_rotations = sim_state
    if len(rod_positions) < 8:
        raise ValueError("rod_positions must contain at least 8 values.")
    if len(rod_rotations) < 8:
        raise ValueError("rod_rotations must contain at least 8 values.")


def _build_sim_state_from_http_state(field_state: dict) -> SimState:
    """Convert localhost `/State` data into the tuple expected by `main`."""
    cam0 = field_state["camData"][0]
    cam1 = field_state["camData"][1]

    rod_positions = [
        (cam0["rods"][index]["Item1"] + cam1["rods"][index]["Item1"]) / 2.0
        for index in range(8)
    ]
    rod_rotations = [
        (cam0["rods"][index]["Item2"] + cam1["rods"][index]["Item2"]) / 2.0
        for index in range(8)
    ]

    return (
        (cam0["ball_x"] + cam1["ball_x"]) / 2.0,
        (cam0["ball_y"] + cam1["ball_y"]) / 2.0,
        (cam0["ball_vx"] + cam1["ball_vx"]) / 2.0,
        (cam0["ball_vy"] + cam1["ball_vy"]) / 2.0,
        rod_positions,
        rod_rotations,
    )


def _build_motor_payload(commands: list[MotorCommand]) -> dict[str, list[dict[str, float | int]]]:
    """Convert internal tuple commands into the HTTP payload format."""
    return {
        "commands": [
            {
                "driveID": drive_id,
                "translationTargetPosition": translation_target,
                "rotationTargetPosition": rotation_target,
                "translationVelocity": translation_velocity,
                "rotationVelocity": rotation_velocity,
            }
            for drive_id, translation_target, rotation_target, translation_velocity, rotation_velocity in commands
        ]
    }


def main(sim_state: SimState) -> list[MotorCommand]:
    """Return commands for rods 1, 2, 4, and 6 from the simulator state."""
    _validate_sim_state(sim_state)

    motor_commands: list[MotorCommand] = []
    for r in _get_rods():
        motor_command = r.step(sim_state)
        motor_commands.append(motor_command)
    return motor_commands


def run_external() -> None:
    """Run the agent against the external localhost HTTP endpoint."""
    session = requests.Session()

    while True:
        response = session.get(f"{URL}/State", timeout=1)
        response.raise_for_status()
        field_state = response.json()

        sim_state = _build_sim_state_from_http_state(field_state)
        commands = main(sim_state)
        payload = _build_motor_payload(commands)

        session.post(f"{URL}/Motors/SendCommand", json=payload, timeout=1).raise_for_status()


init()

if __name__ == "__main__":
    run_external()
