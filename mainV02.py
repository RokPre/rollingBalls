import json
import multiprocessing as mp
import time
from math import atan
from math import pi
from math import tan
from multiprocessing import Queue
from typing import cast
from typing import Literal
from typing import TypedDict

import requests

# Constants
URL = "http://localhost:8080"
# URL = "http://192.168.222.15:8080"
FPS = 50
ROD_HEIGHT = 121.5
PERIOD = 1 / FPS
PLAYER_HEIGHT = 74.48
PLAYER_WIDTH = 20
BALL_PRED_COUNT = 200
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


class RodCam(TypedDict):
    Item1: float
    Item2: float


class CamFrame(TypedDict):
    rods: list[RodCam]
    ball_x: float
    ball_y: float
    ball_vx: float
    ball_vy: float
    ball_size: float


class AxisState(TypedDict):
    axisEncoderPosition: float


class DriveState(TypedDict):
    translation: AxisState
    rotation: AxisState


class DrivesStates(TypedDict):
    drives: list[DriveState]


class FieldState(TypedDict):
    motorsReady: bool
    camData: list[CamFrame]
    drivesStates: DrivesStates


class FieldStateMerged(TypedDict):
    motorsReady: bool
    camData: CamFrame
    drivesStates: DrivesStates


class MotorCommand(TypedDict):
    driveID: int
    translationTargetPosition: float
    translationVelocity: float
    rotationTargetPosition: float
    rotationVelocity: float


with open("geometry.json", "r", encoding="utf-8") as g:
    GEOM = cast(Geometry, json.load(g))

BALL_SIZE: int = int(GEOM["ball_size"])


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

        # State of the rod
        self.state: str = "idle"

        # Geometry attributes of rod
        rod: RodGeom = GEOM["rods"][rod_id - 1]
        self.team: str = str(rod["team"])
        self.travel: int = int(rod["travel"])
        self.players: int = int(rod["players"])
        self.spacing: int = int(rod["spacing"])
        self.position: int = int(rod["position"])
        self.first_offset: int = int(rod["first_offset"])

        # Set the min and max offset for each player
        self.player_min_max_y: list[tuple[int, int]] = self.players * [(0, 0)]
        for i in range(0, self.players):
            player_y_min = self.first_offset + self.spacing * i
            player_y_max = player_y_min + self.travel
            self.player_min_max_y[i] = (player_y_min, player_y_max)

        self.translation_r: float = 0.0
        self.translation_y: list[float] = self.players * [0.0]
        self.rotation: float = 0.0
        self.player_x: float = 0.0
        self.ball_pred_pos: list[tuple[float, float]] = BALL_PRED_COUNT * [(0.0, 0.0)]

    def determine_state(self, cam_data: CamFrame):
        """
        Set the rod state from the current ball position.

        State 0 means the ball is outside the playable field. Otherwise the
        state is based on the ball's x position relative to this rod:
            - 1 when the ball is left of the rod's reach,
            - 2 when it is within reach, and
            - 3 when it is right of the rod's reach.
        """
        # Ball not in play:
        ball_x = cam_data["ball_x"]
        ball_y = cam_data["ball_y"]
        ball_vx = cam_data["ball_vx"]
        ball_vy = cam_data["ball_vy"]

        # Ball is not in play
        if not (0 <= ball_x <= 1210 and 0 <= ball_y <= 700):
            self.state = 0
            return

        # Ball position
        if 0 <= ball_x < self.position - REACH:
            self.state = "avoid"
        elif self.position - REACH <= ball_x < self.position + REACH:
            self.state = "possession"
        elif self.position + REACH < ball_x:
            self.state = "defend"

    def update_variables(self, cam_data: CamFrame):
        """
        Update the internal state of the rod calss.
        # Input:
            - cam_data: Data from the cameras
        # Updates:
            - self.translation_r: Rod translation in relative coordinates directly from the camera.
            - self.rotation: Rod rotation in relative coordinates from 0 to 1.
            - self.translation_y: List of player y coordinates.
            - self.ball_pred_pos: A list of predicted ball positions.
            - self.player_x: Estimated x position of the players tip (legs).
        """

        # Update the rod relative pos from cam data
        self.translation_r = cam_data["rods"][self.rod_id - 1]["Item1"]
        self.rotation = cam_data["rods"][self.rod_id - 1]["Item2"]

        # Update the rod absolute pos from cam data
        for i in range(0, self.players):
            self.translation_y[i] = self.player_min_max_y[i][0] + self.travel * self.translation_r

        # Update the ball position prediction array
        self.ball_pred_pos = BALL_PRED_COUNT * [(0.0, 0.0)]
        ball_x = cam_data["ball_x"]
        ball_y = cam_data["ball_y"]
        ball_vx = cam_data["ball_vx"]
        ball_vy = cam_data["ball_vy"]
        for i in range(0, BALL_PRED_COUNT):
            ball_pred_x: float = ball_x + PERIOD * ball_vx * i
            ball_pred_y: float = ball_y + PERIOD * ball_vy * i
            self.ball_pred_pos[i] = (ball_pred_x, ball_pred_y)

        # tan(phi) = dx / dz => dx = tan(phi) * dz
        self.player_x = self.position + tan(self.rotation * 2 * pi) * ROD_HEIGHT

    def can_player_reach_y(self, player_id: int, y: float) -> bool:
        """
        Check if a given player can reach the y position.

        # Args:
            - rod_id - This is the id specified in the geometry.json file.
            - player_id: the id of the player. Top player has id 1, bottom player can have at most 5.
            - y: the value that we want the player to be at

        # Returns:
            - relative_rod_position: The position that the rod has to be in , so that that specific player has the right y value. If player can not reach that y value, raises ValueError.

        # Raises:
            ValueError if rod id is not between 1 and 8, if player id is too high or low and if y is unreachable.

        """
        if not (1 <= player_id <= self.players):
            raise ValueError(f"player_id should be between 1 and {self.players}")

        return (self.player_min_max_y[player_id - 1][0] <= y <= self.player_min_max_y[player_id - 1][0])

    def players_by_distance_from_y(self, y: float) -> list[int]:
        """
        Lists all players on rod, from distance to a given y coordinate.
        # Args:
            - y: the value where from which we will base the sorting.
        # Returns:
            - sorted_list: sorted list of player ids, based from their distance to the point y.
        """
        sorted_list: list[int] = sorted(
            range(len(self.translation_y)),
            key=lambda i: abs(y - self.translation_y[i])
        )
        return [i + 1 for i in sorted_list]

    def player_pos_y_to_relative(self, player_id: int, y: float) -> float:
        """
        Convers the euclidian y to the relavie position that can be sent to the server

        # Args:
            - rod_id - This is the id specified in the geometry.json file.
            - player_id: the id of the player. Top player has id 1, bottom player can have at most 5.
            - y: the value that we want the player to be at

        # Returns:
            - relative_rod_position: The position that the rod has to be in , so that that specific player has the right y value. If player can not reach that y value, raises ValueError.

        # Raises:
            ValueError if rod id is not between 1 and 8, if player id is too high or low.
        """

        if not (1 <= self.rod_id <= 8):
            raise ValueError("rod_id should be between 1 and 8")

        if not (1 <= player_id <= self.players):
            raise ValueError(f"player_id should be between 1 and {self.players}")

        player_min_y: int = self.first_offset + self.spacing * (player_id - 1)
        player_max_y: int = player_min_y + self.travel

        relative_pos: float = max(min((y - player_min_y) / (player_max_y - player_min_y), 1), 0)

        return relative_pos

    def move_any_player_to_y(self, y: float) -> float:
        """
        Finds the best player to move to move to y, based on the distance and if the player can reach it.
        # Input:
            y: the position to move any player to.
        # Output:
            translation_r: the translation that can be sent to the motor.
        """
        players_by_distance = self.players_by_distance_from_y(y)
        for player in players_by_distance:
            if self.can_player_reach_y(player, y):
                return self.player_pos_y_to_relative(player, y)
        return self.player_pos_y_to_relative(players_by_distance[0], y)

    def move_any_player_to_x(self, target_x: float) -> float:
        """
        Computes the angle that the player has to be at to have the tip at a given x coordinate.
        # Args:
            - target_x: the x at which we want the player tip to be at.
        # Returns:
            - angle: the angle of the player, ready to be sent as a command.
        """

        dx = self.position - target_x
        dz = PLAYER_HEIGHT - BALL_SIZE / 2

        phi = atan(dx / dz)

        return phi / (2 * pi)

    def ball_pred_pos_from_x(self, x: float) -> list[tuple[float, float]]:
        """
        List all ball poss, based on distance to a given point x.
        # Args:
            - x: the value where from which we will base the sorting.
        # Returns:
            - sorted_list: sorted list of ball predicted position, based from their distance to the point x.
        """
        sorted_list: list[int] = sorted(
            range(len(self.ball_pred_pos)),
            key=lambda i: abs(x - self.ball_pred_pos[i][0])
        )
        return [self.ball_pred_pos[i] for i in sorted_list]

    def attack(self, field_state: FieldStateMerged) -> tuple[float, float, float, float]:
        cam_data = field_state["camData"]
        ball_x = cam_data["ball_x"]
        ball_y = cam_data["ball_y"]
        ball_vy = cam_data["ball_vy"]
        ball_vx = cam_data["ball_vx"]
        translation_r = self.move_any_player_to_y(ball_y)

        enemy_rod: RodCam = cam_data["rods"][self.rod_id]
        enemy_translation_r: float = enemy_rod["Item1"]

        enemy_geom = GEOM["rods"][self.rod_id - 1]
        enemy_first_offset: int = int(enemy_geom["first_offset"])
        enemy_players: int = int(enemy_geom["players"])
        enemy_spacing: int = int(enemy_geom["spacing"])
        enemy_travel: int = int(enemy_geom["travel"])

        enemy_translation_y = [enemy_first_offset + enemy_spacing * i + enemy_travel * enemy_translation_r for i in range(0, enemy_players)]

        enemy_player_in_line_with_ball = any([(ball_y - BALL_SIZE / 2 - PLAYER_WIDTH < abs(player_y) < ball_y + BALL_SIZE / 2 + PLAYER_WIDTH) for player_y in enemy_translation_y])
        print("ball_vx", ball_vx, "ball_vy", ball_vy)

        # Shoot the ball of its verry slow, when it gets stuck.
        if abs(ball_vx) < 0.05 and abs(ball_vy) < 0.05:
            return translation_r, 1, -0.25, 1
        if abs(ball_vy) < 0.3:
            if ball_y < GEOM["field"]["dimension_y"] / 2:
                lower_bound = PLAYER_WIDTH / 2
                upper_bound = lower_bound + BALL_SIZE / 2
                middle_bound = (lower_bound + upper_bound) / 2
            else:
                lower_bound = - PLAYER_WIDTH / 2 - BALL_SIZE / 2
                upper_bound = lower_bound + BALL_SIZE / 2
                middle_bound = (lower_bound + upper_bound) / 2

            translation_r = self.move_any_player_to_y(ball_y - middle_bound)
            if any([(player_y + lower_bound < abs(ball_y) < player_y + upper_bound) for player_y in self.translation_y]):
                print("player in line")
                return translation_r, 1, -0.25, 1
            else:
                print("player not in line")
                return translation_r, 1, 0, 1

        player_behind_ball = (self.player_x < ball_x) and (abs(ball_x - self.player_x) < (REACH))
        player_in_line_with_ball = any([(ball_y - BALL_SIZE / 2 - PLAYER_WIDTH < abs(player_y) < ball_y + BALL_SIZE / 2 + PLAYER_WIDTH) for player_y in self.translation_y])

        # Player can shoot the ball.
        rotation = 0
        if player_behind_ball and player_in_line_with_ball:
            rotation = -0.2

        return translation_r, 1, rotation, 1

    def defend(self, field_state: FieldStateMerged) -> tuple[float, float, float, float]:
        sorted_ball_pos = self.ball_pred_pos_from_x(self.position)
        _, ball_y = sorted_ball_pos[0]
        translation_r = self.move_any_player_to_y(ball_y)
        return translation_r, 1, 0, 1

    def goalie_defend(self, field_state: FieldStateMerged) -> tuple[float, float, float, float]:
        cam_data = field_state["camData"]
        ball_x = cam_data["ball_x"]
        ball_y = cam_data["ball_y"]

        # Move goalie to the best defense position.
        sorted_ball_pos = self.ball_pred_pos_from_x(self.position)
        _, ball_y = sorted_ball_pos[0]
        translation_r = self.move_any_player_to_y(ball_y)

        # If ball is in front of the goalie, shoot it.
        if self.player_x < ball_x and abs(ball_y - self.translation_y[0]) < (PLAYER_WIDTH + BALL_SIZE) / 2:
            rotation = -0.2
        else:
            rotation = self.move_any_player_to_x(ball_x)
            rotation = max(0, rotation)
        return translation_r, 1, rotation, 1

    def avoid(self, field_state: FieldStateMerged) -> tuple[float, float, float, float]:
        cam_data = field_state["camData"]
        ball_y = cam_data["ball_y"]
        translation_r = self.move_any_player_to_y(ball_y)
        return translation_r, 1, 0.2, 1

    def step(self, field_state: FieldStateMerged) -> MotorCommand:
        cam_data: CamFrame = field_state["camData"]
        self.update_variables(cam_data)
        self.determine_state(cam_data)

        match self.state:
            case "avoid":
                if self.rod_id == 1:
                    tran, tran_vel, rot, rot_vel = self.goalie_defend(field_state)
                else:
                    tran, tran_vel, rot, rot_vel = self.avoid(field_state)

            case "possession":
                if self.rod_id == 1:
                    tran, tran_vel, rot, rot_vel = self.goalie_defend(field_state)
                else:
                    tran, tran_vel, rot, rot_vel = self.attack(field_state)

            case "defend":
                tran, tran_vel, rot, rot_vel = self.defend(field_state)
            case _:
                tran, tran_vel, rot, rot_vel = 0, 0, 0, 0

        return {"driveID": int(self.driveID), "translationTargetPosition": tran, "translationVelocity": tran_vel, "rotationTargetPosition": rot, "rotationVelocity": rot_vel}


class enemyRod:
    def __init__(self, rod_id: int):
        self.rod_id: int = rod_id

        # Geometry attributes of rod
        rod: RodGeom = GEOM["rods"][rod_id - 1]
        self.team: str = str(rod["team"])
        self.travel: int = int(rod["travel"])
        self.players: int = int(rod["players"])
        self.spacing: int = int(rod["spacing"])
        self.position: int = int(rod["position"])
        self.first_offset: int = int(rod["first_offset"])

        # Set the min and max offset for each player
        self.player_min_max_y: list[tuple[int, int]] = self.players * [(0, 0)]
        for i in range(0, self.players):
            player_y_min = self.first_offset + self.spacing * i
            player_y_max = player_y_min + self.travel
            self.player_min_max_y[i] = (player_y_min, player_y_max)

        self.translation_r: float = 0.0
        self.translation_y: list[float] = self.players * [0.0]
        self.rotation: float = 0.0
        self.player_x: float = 0.0
        self.ball_pred_pos: list[tuple[float, float]] = BALL_PRED_COUNT * [(0.0, 0.0)]

    def update_variables(self, cam_data: CamFrame):
        """
        Update the internal state of the rod calss.
        # Input:
            - cam_data: Data from the cameras
        # Updates:
            - self.translation_r: Rod translation in relative coordinates directly from the camera.
            - self.rotation: Rod rotation in relative coordinates from 0 to 1.
            - self.translation_y: List of player y coordinates.
            - self.player_x: Estimated x position of the players tip (legs).
        """

        # Update the rod relative pos from cam data
        self.translation_r = cam_data["rods"][self.rod_id - 1]["Item1"]
        self.rotation = cam_data["rods"][self.rod_id - 1]["Item2"]

        # Update the rod absolute pos from cam data
        for i in range(0, self.players):
            self.translation_y[i] = self.player_min_max_y[i][0] + self.travel * self.translation_r

        # Update the ball position prediction array
        self.ball_pred_pos = BALL_PRED_COUNT * [(0.0, 0.0)]

        # tan(phi) = dx / dz => dx = tan(phi) * dz
        self.player_x = self.position + tan(self.rotation * 2 * pi) * ROD_HEIGHT


def merge_cam_data(field_state: FieldState) -> FieldStateMerged:
    cam0 = field_state["camData"][0]
    cam1 = field_state["camData"][1]

    rods_merged: list[RodCam] = [
        {
            "Item1": (r0["Item1"] + r1["Item1"]) / 2,
            "Item2": (r0["Item2"] + r1["Item2"]) / 2,
        }
        for r0, r1 in zip(cam0["rods"], cam1["rods"])
    ]

    cam_merged: CamFrame = {
        "rods": rods_merged,
        "ball_x": (cam0["ball_x"] + cam1["ball_x"]) / 2,
        "ball_y": (cam0["ball_y"] + cam1["ball_y"]) / 2,
        "ball_vx": (cam0["ball_vx"] + cam1["ball_vx"]) / 2,
        "ball_vy": (cam0["ball_vy"] + cam1["ball_vy"]) / 2,
        "ball_size": cam0["ball_size"],
    }

    return {
        "motorsReady": field_state["motorsReady"],
        "camData": cam_merged,
        "drivesStates": field_state["drivesStates"],
    }


next_time = None


def my_sleep():
    global next_time

    now = time.perf_counter()

    if next_time is None:
        next_time = now + PERIOD
        return

    sleep_time = next_time - now

    if sleep_time > 0:
        time.sleep(sleep_time)

    next_time += PERIOD


def rod_worker(rod_id: int, in_q: Queue[FieldStateMerged | None], out_q: Queue[MotorCommand]):
    r = rod(rod_id)
    while True:
        obs: FieldStateMerged | None = in_q.get()
        if obs is None:
            break
        cmd = r.step(obs)
        out_q.put(cmd)


def main():
    ctx = mp.get_context("spawn")
    rod_ids = [1, 2, 4, 6]

    in_queues = {rid: ctx.Queue(maxsize=1) for rid in rod_ids}
    out_queue = ctx.Queue()

    procs = [
        ctx.Process(target=rod_worker, args=(rid, in_queues[rid], out_queue), daemon=True)
        for rid in rod_ids
    ]

    for p in procs:
        p.start()

    session = requests.Session()

    try:
        while True:

            r = session.get(f"{URL}/State", timeout=1)
            r.raise_for_status()
            field_state: FieldState = cast(FieldState, r.json())

            merged_cam_data = merge_cam_data(field_state)

            for rid in rod_ids:
                q = in_queues[rid]
                while not q.empty():
                    try:
                        q.get_nowait()
                    except Exception:
                        break
                q.put_nowait(merged_cam_data)

            commands = [out_queue.get() for _ in rod_ids]
            payload = {"commands": commands}

            resp = session.post(f"{URL}/Motors/SendCommand", json=payload, timeout=1)
            resp.raise_for_status()

            my_sleep()

    except KeyboardInterrupt:
        pass

    finally:
        for rid in rod_ids:
            in_queues[rid].put(None)
        for p in procs:
            p.join(timeout=1.0)


if __name__ == "__main__":
    main()
