# 4 states
# 0. No play
# 1. Blue possesion
# 2. Red possesion
# 3. Moving fast
# 4. Moving slow
import json
import math

import httpx
import requests

from defense_modul import Defense


URL = "http://localhost:8080"
OPTIMAL_SHOOT_DISTANCE = 5  # in milimeters
FRAME_RATE = 1 / 50
NUM_OF_PREDICTS = 100


with open("geometry.json", "r") as g:
    GEOM = json.load(g)
    R_BALL = GEOM["ball_size"] / 2

DEFENSE = Defense(
    geometry_path="geometry.json",
    rod_keeper=0, drive_keeper=1,
    rod_def1=1, drive_def1=2,
    rod_def2=3, drive_def2=3,
)


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def rod_players_from_state(cam_data):
    rods_state = cam_data["camData"][0]["rods"]
    field_y = GEOM["field"]["dimension_y"]
    rods_geom = GEOM["rods"]

    out = {}

    for i, (rs, rg) in enumerate(zip(rods_state, rods_geom)):
        u = float(rs["Item1"])
        u = _clamp(u, 0.0, 1.0)

        theta = float(rs["Item2"]) % (2 * math.pi)

        x = float(rg["position"])
        base = float(rg["first_offset"]) + u * float(rg["travel"])
        players = int(rg["players"])
        spacing = float(rg["spacing"])

        pts = []
        for k in range(players):
            y = base + k * spacing

            y = _clamp(y, 0.0, float(field_y))

            pts.append({"x": x, "y": y, "theta": theta})

        out[rg["id"]] = pts

    return out


def move_player_to_y(rod_id: int, player_number: int, y_coordinate: float):
    """
    rod_id: geometry rod id (1..8)
    player_number: 0-based index along the rod (0..players-1)
    y_coordinate: y in mm target point on the field.

    Assumptions:
      Item1 / translationTargetPosition is normalized to [0, 1]
      driveID equals rod_id for motorized rods
    """
    rods = GEOM["rods"]
    field_y = float(GEOM["field"]["dimension_y"])

    rod = next((r for r in rods if int(r["id"]) == int(rod_id)), None)
    if rod is None:
        raise ValueError(f"Unknown rod_id={rod_id}")

    players = int(rod["players"])
    if not (0 <= player_number < players):
        raise ValueError(f"player_number out of range: {player_number} for rod players={players}")

    y = _clamp(y_coordinate, 0.0, field_y)

    first_offset = float(rod["first_offset"])
    spacing = float(rod["spacing"])
    travel = float(rod["travel"])
    if travel <= 0.0:
        raise ValueError(f"Invalid travel={travel} for rod_id={rod_id}")

    base_without_u = first_offset + player_number * spacing

    u = (y - base_without_u) / travel
    u = _clamp(u, 0.0, 1.0)

    theta = 0.0

    return u


def determine_state(cam_data) -> int:
    """
    This function return the state that our logic is in.
    Input: cam_data
    Output: State (0 - ball out of play,
                   1 - ball is moving fast towards blue goal
                   2 - ball is moving fast towards red goal
                   3 - ball is moving slow and not in possesion by any team
                   4 - blue team has ball possesion
                   5 - red team has ball possesion)
    """
    cam0 = cam_data["camData"][0]
    ball_x = cam0["ball_x"]
    ball_y = cam0["ball_y"]
    ball_vx = cam0["ball_vx"]
    ball_vy = cam0["ball_vy"]
    ball_velocity = math.sqrt(float(ball_vx)**2 + float(ball_vy)**2)

    # Ball is outside of play
    if 0 > ball_x or ball_x > 1210 or 0 > ball_y > 700:
        return 0

    # Ball is moving fast towards the blue goal
    if 3 < ball_vx:
        return 1

    # Ball is moving fast towards the red goal
    if ball_vx < -3:
        return 2

    found_rod = None
    rods = GEOM["rods"]
    # Check all the rods positions
    for i, rod in enumerate(rods):
        position = int(rod["position"])

        # Goalies have a different margin than the other players.
        if i == 0:  # Red goalie
            left_margin = 40
        else:  # Other players
            left_margin = 50

        if i == len(rods):  # Blue goalie
            right_margin = 40
        else:  # Other players
            right_margin = 50

        # Check if ball is within field
        if position - left_margin < ball_x < position + right_margin:
            found_rod = rod
            break
    # Not in possesion
    else:
        return 3

    # Blue team has possesion of th ball
    if found_rod is not None and found_rod["team"] == "blue":
        return 4

    # Red team has possesion of the ball
    if found_rod is not None and found_rod["team"] == "red":
        return 5

    return 0


def shoot():
    # TODO: Only move players in front of the ball up.
    payload = {"commands": [
        {"driveID": 1, "rotationTargetPosition": 1, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 2, "rotationTargetPosition": 1, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 4, "rotationTargetPosition": 1, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 6, "rotationTargetPosition": 1, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
    ]}
    return payload


def block():
    payload = {"commands": [
        {"driveID": 1, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 2, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 4, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 6, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
    ]}
    return payload


def reposition():
    # TODO: Prepare for the ball
    payload = {"commands": [
        {"driveID": 1, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 2, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 4, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
        {"driveID": 6, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
    ]}
    return payload


# def defend():
#     # TODO: Alberim
#     payload = {"commands": [
#         {"driveID": 1, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
#         {"driveID": 2, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
#         {"driveID": 4, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
#         {"driveID": 6, "rotationTargetPosition": 0, "rotationVelocity": 1, "translationTargetPosition": 0, "translationVelocity": 0},
#     ]}
#     return payload
def defend(cam_data):
    cam0 = cam_data["camData"][0]

    ball_y = cam0["ball_y"]
    ball_visible = ball_y >= 0

    commands = DEFENSE.step(
        ball_y=float(ball_y),
        ball_visible=ball_visible
    )

    return {"commands": commands}


def attack(cam_data):
    # Predict the ball position
    cam0 = cam_data["camData"][0]
    ball_x = cam0["ball_x"]  # x coordinate of the ball in milimeters.
    ball_y = cam0["ball_y"]  # x coordinate of the ball in milimeters.

    ball_vx = cam0["ball_vx"]  # x velocity of the ball
    ball_vy = cam0["ball_vy"]  # y velocity of the ball

    predicted_ball_pos = []

    pred_ball_x: float = ball_x
    pred_ball_y: float = ball_y

    for i in range(1, NUM_OF_PREDICTS + 1):

        old_pred_ball_x = pred_ball_x
        old_pred_ball_y = pred_ball_y

        pred_ball_x = float(pred_ball_x + ball_vx * FRAME_RATE)
        pred_ball_y = float(pred_ball_y + ball_vy * FRAME_RATE)

        # print(pred_ball_x)
        # print(pred_ball_y)

        if 0 + R_BALL > pred_ball_x:
            ball_vx = -ball_vx
            pred_ball_x = float(old_pred_ball_x + ball_vx * FRAME_RATE)
            print("Wall bounce x")

        if pred_ball_x > 1200 - R_BALL:
            ball_vx = -ball_vx
            pred_ball_x = float(old_pred_ball_x + ball_vx * FRAME_RATE)
            print("Wall bounce x")

        if 0 + R_BALL > pred_ball_y:
            ball_vy = -ball_vy
            pred_ball_y = float(old_pred_ball_y + ball_vy * FRAME_RATE)
            print("Wall bounce y")

        if pred_ball_y > 700 - R_BALL:
            ball_vy = -ball_vy
            pred_ball_y = float(old_pred_ball_y + ball_vy * FRAME_RATE)
            print("Wall bounce y")

        ball_pos: tuple[float, float] = (pred_ball_x, pred_ball_y)
        predicted_ball_pos.append(ball_pos)

    # Check if the ball will be in the optimal position for a shoot
    # Check which rod has the ball
    found_rod: dict[str, str | int] = {}
    rods = GEOM["rods"]
    # Check all the rods positions
    for i, rod in enumerate(rods):
        position = int(rod["position"])

        # Goalies have a different margin than the other players.
        if i == 0:  # Red goalie
            left_margin = 40
        else:  # Other players
            left_margin = 50

        if i == len(rods):  # Blue goalie
            right_margin = 40
        else:  # Other players
            right_margin = 50

        # Check if ball is within field
        if position - left_margin < ball_x < position + right_margin:
            found_rod = rod
            break

    # Find cloases player to the shooting_point
    rod_id = found_rod["id"]

    players_by_rod_id = rod_players_from_state(cam_data)
    players_rod = players_by_rod_id[rod_id]

    min_distance = float("inf")
    best_player_id = None
    best_ball_pos = None

    for ball_pos in predicted_ball_pos:
        for idx, player in enumerate(players_rod):

            distance = abs(player["y"] - ball_pos[1])

            if distance < min_distance:
                min_distance = distance
                best_player_id = idx
                best_ball_pos = ball_pos

    if best_player_id is not None and best_ball_pos is not None:
        y_payload = move_player_to_y(rod_id, best_player_id, best_ball_pos[1])

    # Check if shooting_point is reachabel before ball get there
    # Move the player to the shooting point
    # Move all players in front upwards

    # TODO: Check if player is clsoe enough to the ball so that
    if best_player_id is not None:
        player_y = players_rod[best_player_id]["y"]
        distance = abs(player_y - best_ball_pos[1])

        # print(type(best_ball_pos))
        # print(best_ball_pos)

        if distance <= OPTIMAL_SHOOT_DISTANCE:
            # Shoot the ball
            payload = {
                "commands": [
                    {
                        "driveID": rod_id,
                        "translationTargetPosition": 0,
                        "translationVelocity": 0,
                        "rotationTargetPosition": -0.2,
                        "rotationVelocity": 1
                    }
                ]
            }

        elif y_payload is not None:
            # Track the ball
            u = move_player_to_y(rod_id, best_player_id, best_ball_pos[1])
            payload = {
                "commands": [
                    {
                        "driveID": rod_id,
                        "translationTargetPosition": u,
                        "translationVelocity": 1,
                        "rotationTargetPosition": 0,
                        "rotationVelocity": 1
                    }
                ]
            }
        else:
            payload = {
                "commands": [
                    {
                        "driveID": 0,
                        "translationTargetPosition": 0,
                        "translationVelocity": 0,
                        "rotationTargetPosition": 0,
                        "rotationVelocity": 0
                    }
                ]
            }
        return payload


def main(cam_data):
    state = determine_state(cam_data)
    print(state)
    match state:
        # 0 - ball out of play,
        case 0:
            pass
        # 1 - ball is moving fast towards blue goal
        case 1:
            # Raise all red players
            return shoot()
        # 2 - ball is moving fast towards red goal
        case 2:
            return block()
        # 3 - ball is moving slow and not in possesion by any team
        case 3:
            return reposition()
        # 4 - blue team has ball possesion
        case 4:
            return defend(cam_data)
        # 5 - red team has ball possesion
        case 5:
            return attack(cam_data)

    return None


if __name__ == "__main__":
    session = requests.Session()
    while True:
        try:
            r = session.get(f"{URL}/State")
            field_state = r.json()

            payload = main(field_state)

            if payload is not None:
                resp = session.post(f"{URL}/Motors/SendCommand", json=payload)
                print(resp.status_code)

        except KeyboardInterrupt:
            break
