import json
import multiprocessing as mp
import time
from math import pi
from math import tan
from multiprocessing import Queue
from typing import cast
from typing import Literal
from typing import TypedDict

import requests

# Constants
URL = "http://localhost:8080"
# URL = "http://192.168.222.15:8081"
FPS = 50
ROD_HEIGHT = 121.5
PERIOD = 1 / FPS
BALL_PRED_COUNT = 100


# Type setting
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
        self.state: int = 0

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
        return

    def update_variables(self, cam_data: CamFrame):
        """
        Update the internal state of the rod calss.
        # Input:
            - cam_data: Data from the cameras
        # Updates:
            - self.translation_r: Rod translation in relative coordinates directly from the camera.
            - self.translation_y: Rod translation in absolute coordinates.
            - self.ball_pred_pos: A list of predicted ball positions.
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

    def step(self, field_state: FieldStateMerged) -> MotorCommand:
        cam_data: CamFrame = field_state["camData"]
        self.update_variables(cam_data)
        self.determine_state(cam_data)

        match self.state:
            case _:
                tran, tran_vel, rot, rot_vel = 0, 0, 0, 0

        return {"driveID": int(self.driveID), "translationTargetPosition": tran, "translationVelocity": tran_vel, "rotationTargetPosition": rot, "rotationVelocity": rot_vel}


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
