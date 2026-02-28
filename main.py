import json
import time

import requests

URL = "http://localhost:8080"
FPS = 50
PERIOD = 1 / FPS
BALL_PRED_COUNT = 25

with open("geometry.json", "r") as g:
    GEOM = json.load(g)

BALL_SIZE: int = int(GEOM["ball_size"])


class rod:
    def __init__(self, rod_id: int):
        self.rod_id: int = rod_id
        self.rod_id_command: int = rod_id
        if rod_id == 4:
            self.rod_id_command = 3
        elif rod_id == 6:
            self.rod_id_command = 4

        rod: dict[str, int | str] = dict(GEOM["rods"][rod_id - 1])

        self.state: int
        self.id: int = int(rod["id"])
        self.team: str = str(rod["team"])
        self.position: int = int(rod["position"])
        self.travel: int = int(rod["travel"])
        self.players: int = int(rod["players"])
        self.first_offset: int = int(rod["first_offset"])
        self.spacing: int = int(rod["spacing"])
        self.rotatin: float = float(0)
        self.translation_r: float = float(0)  # Relative
        self.translation_y: list[float] = self.players * [float(0)]
        self.ball_pred_pos: list[tuple[float, float]]

    def determine_state(self, cam_data: dict[str, float]):
        # Ball not in play:
        ball_x = cam_data["ball_x"]
        ball_y = cam_data["ball_y"]
        # Ball is not in play
        if not (0 <= ball_x <= 1210):
            self.state = 0
            return

        # Ball is not in play
        if not (0 <= ball_y <= 700):
            self.state = 0
            return

        # Rod has controll of the ball
        if abs(self.position - ball_x) < 50:
            # Ball is behind the rod
            if ball_x <= self.position:
                self.state = 1
                return
            # Ball is in fron of the rod
            if self.position < ball_x:
                self.state = 2
                return

        # Ball is behind the rod
        if ball_x <= self.position:
            self.state = 3
            return

        # Ball is in fron of the rod
        if self.position < ball_x:
            self.state = 4
            return

    def prepare_attack(self):
        return 0, 0, 0, 0

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

        rods_geom: dict[str, int | str] = GEOM["rods"][self.rod_id - 1]
        number_of_players: int = int(rods_geom["players"])

        if not (1 <= player_id <= number_of_players):
            raise ValueError(f"player_id should be between 1 and {number_of_players}")

        first_offset: int = int(rods_geom["first_offset"])
        spacing: int = int(rods_geom["spacing"])
        travel: int = int(rods_geom["travel"])

        player_min_y: int = first_offset + spacing * (player_id - 1)
        player_max_y: int = player_min_y + travel

        return (player_min_y <= y <= player_max_y)

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
            ValueError if rod id is not between 1 and 8, if player id is too high or low and if y is unreachable.
        """

        if not (1 <= self.rod_id <= 8):
            raise ValueError("rod_id should be between 1 and 8")

        rods_geom: dict[str, int | str] = GEOM["rods"][self.rod_id - 1]
        number_of_players: int = int(rods_geom["players"])

        if not (1 <= player_id <= number_of_players):
            raise ValueError(f"player_id should be between 1 and {number_of_players}")

        first_offset: int = int(rods_geom["first_offset"])
        spacing: int = int(rods_geom["spacing"])
        travel: int = int(rods_geom["travel"])

        player_min_y: int = first_offset + spacing * (player_id - 1)
        player_max_y: int = player_min_y + travel

        if not (player_min_y <= y <= player_max_y):
            raise ValueError(f"y should be between {player_min_y} and {player_max_y} for this specific player. y: {y}, rod_id: {rod_id}, player_id: {player_id}")

        relative_pos: float = (y - player_min_y) / (player_max_y - player_min_y)

        return relative_pos

    def update_variables(self, cam_data: dict[str, float]):
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
        self.translation_r = cam_data[f"rod{self.rod_id}_item1"]

        # Update the rod absolute pos from cam data
        for i in range(0, self.players):
            player_min_y: int = self.first_offset + self.spacing * i
            player_max_y: int = player_min_y + self.travel
            player_y: float = player_min_y + (player_max_y - player_min_y) * self.translation_r
            self.translation_y[i] = player_y

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

    def shoot(self, cam_data: dict[str, float]) -> tuple[float, float, float, float]:
        shoot_x: float = float(self.position + BALL_SIZE / 2)

        # Find best shoot position
        min_dist = float("inf")
        best_shoot_ball_pos: tuple[float, float] = (-1, -1)
        for i in range(0, len(self.ball_pred_pos)):
            dist = self.ball_pred_pos[i][0] - shoot_x
            if dist < min_dist:
                min_dist: float = dist
                best_shoot_ball_pos = self.ball_pred_pos[i]

        # Sort players by distance to best_shoot_ball_pos.
        sorted_players_ids = self.players_by_distance_from_y(best_shoot_ball_pos[1])

        for player_id in sorted_players_ids:
            if self.can_player_reach_y(player_id, best_shoot_ball_pos[1]):
                player_id_to_move = player_id
                break
        else:
            return 0.0, 0.0, 0.0, 0.0

        translation_r: float = self.player_pos_y_to_relative(player_id_to_move, best_shoot_ball_pos[1])

        if abs(best_shoot_ball_pos[1] - self.translation_y[player_id_to_move - 1]) < BALL_SIZE / 2:
            return translation_r, 1, -0.25, 1

        return translation_r, 1, 0.0, 1

    def defend(self, cam_data: dict[str, float]) -> tuple[float, float, float, float]:
        # Find last value thats in front of the rod
        move_player_y = None
        for i, pos in enumerate(self.ball_pred_pos[::-1]):
            if self.position <= pos[0]:
                move_player_y = pos[1]
                break

        if move_player_y == None:
            move_player_y = cam_data["ball_y"]

        # Sort players by distnace to point
        sorted_players_ids = self.players_by_distance_from_y(move_player_y)

        # Check if player can reach
        for player_id in sorted_players_ids:
            if self.can_player_reach_y(player_id, move_player_y):
                translation_r = self.player_pos_y_to_relative(player_id, move_player_y)
                return translation_r, 1, 0, 1

        return 0.5, 1, 0, 1

    def avoid(self):
        return 0, 0, 0.25, 1

    def step(self, cam_data: dict[str, float]) -> dict[str, int | float]:
        self.update_variables(cam_data)
        self.determine_state(cam_data)

        match self.state:
            case 1:
                tran, tran_vel, rot, rot_vel = self.prepare_attack()
            case 2:
                tran, tran_vel, rot, rot_vel = self.shoot(cam_data)
            case 3:
                tran, tran_vel, rot, rot_vel = self.avoid()
            case 4:
                tran, tran_vel, rot, rot_vel = self.defend(cam_data)
            case _:
                tran, tran_vel, rot, rot_vel = 0.5, 0.5, 0, 0.5

        return {"driveID": int(self.rod_id_command), "translationTargetPosition": tran, "translationVelocity": tran_vel, "rotationTargetPosition": rot, "rotationVelocity": rot_vel}


def merge_cam_data(cam_data0: dict[str, float], cam_data1: dict[str, float]) -> dict[str, float]:
    merged: dict[str, float] = {}

    for key in cam_data0:
        v0 = cam_data0[key]
        v1 = cam_data1[key]
        merged[key] = (v0 + v1) / 2.0

    return merged


def flatten_state(field_state) -> tuple[dict[str, float], dict[str, float]]:
    cam_data0 = field_state["camData"][0]
    cam_data1 = field_state["camData"][1]

    cam0: dict[str, float] = {"rod1_item1": cam_data0["rods"][0]["Item1"], "rod1_item2": cam_data0["rods"][0]["Item2"], "rod2_item1": cam_data0["rods"][1]["Item1"], "rod2_item2": cam_data0["rods"][1]["Item2"], "rod3_item1": cam_data0["rods"][2]["Item1"], "rod3_item2": cam_data0["rods"][2]["Item2"], "rod4_item1": cam_data0["rods"][3]["Item1"], "rod4_item2": cam_data0["rods"][3]["Item2"], "rod5_item1": cam_data0["rods"][4]["Item1"], "rod5_item2": cam_data0["rods"][4]["Item2"], "rod6_item1": cam_data0["rods"][5]["Item1"], "rod6_item2": cam_data0["rods"][5]["Item2"], "rod7_item1": cam_data0["rods"][6]["Item1"], "rod7_item2": cam_data0["rods"][6]["Item2"], "rod8_item1": cam_data0["rods"][7]["Item1"], "rod8_item2": cam_data0["rods"][7]["Item2"], "ball_x": cam_data0["ball_x"], "ball_y": cam_data0["ball_y"], "ball_vx": cam_data0["ball_vx"], "ball_vy": cam_data0["ball_vy"], "ball_size": cam_data0["ball_size"]}
    cam1: dict[str, float] = {"rod1_item1": cam_data1["rods"][0]["Item1"], "rod1_item2": cam_data1["rods"][0]["Item2"], "rod2_item1": cam_data1["rods"][1]["Item1"], "rod2_item2": cam_data1["rods"][1]["Item2"], "rod3_item1": cam_data1["rods"][2]["Item1"], "rod3_item2": cam_data1["rods"][2]["Item2"], "rod4_item1": cam_data1["rods"][3]["Item1"], "rod4_item2": cam_data1["rods"][3]["Item2"], "rod5_item1": cam_data1["rods"][4]["Item1"], "rod5_item2": cam_data1["rods"][4]["Item2"], "rod6_item1": cam_data1["rods"][5]["Item1"], "rod6_item2": cam_data1["rods"][5]["Item2"], "rod8_item1": cam_data1["rods"][7]["Item1"], "rod8_item2": cam_data1["rods"][7]["Item2"], "rod7_item1": cam_data1["rods"][6]["Item1"], "rod7_item2": cam_data1["rods"][6]["Item2"], "ball_x": cam_data1["ball_x"], "ball_y": cam_data1["ball_y"], "ball_vx": cam_data1["ball_vx"], "ball_vy": cam_data1["ball_vy"], "ball_size": cam_data1["ball_size"]}
    return cam0, cam1


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


def main():
    count = 0
    start_time = time.perf_counter()

    # Initialise the rods
    rod_1 = rod(1)  # Goalie
    rod_2 = rod(2)  # Defenders
    rod_4 = rod(4)  # Middle
    rod_6 = rod(6)  # Attack

    # Create the session
    session: requests.Session = requests.Session()

    # Main loop
    while True:
        # Get the state
        r = session.get(f"{URL}/State")
        field_state: dict[str, dict[str, float | list[dict[str, float]]]] = dict(r.json())

        # Reformat the cam data
        cam0: dict[str, float]
        cam1: dict[str, float]
        cam0, cam1 = flatten_state(field_state)
        merged_cam_data = merge_cam_data(cam0, cam1)

        # Calculate command to be sent
        gcommand: dict[str, int | float] = rod_1.step(merged_cam_data)
        dcommand: dict[str, int | float] = rod_2.step(merged_cam_data)
        mcommand: dict[str, int | float] = rod_4.step(merged_cam_data)
        acommand: dict[str, int | float] = rod_6.step(merged_cam_data)

        # Create payload
        payload = {"commands": [gcommand, dcommand, mcommand, acommand]}

        # Send playload
        resp = session.post(f"{URL}/Motors/SendCommand", json=payload)
        resp.raise_for_status()

        # print("rod_1", rod_1.state)
        # print("rod_2", rod_2.state)
        # print("rod_4", rod_4.state)
        # print("rod_6", rod_6.state)

        # HZ
        my_sleep()
        count += 1

        curr_time = time.perf_counter()
        if count % 50 == 0:
            print("FPS:", 1 / ((curr_time - start_time) / count))


if __name__ == "__main__":
    main()
