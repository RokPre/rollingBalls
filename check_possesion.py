import json
import time

import requests

with open("geometry.json", "r") as geom:
    geom = json.load(geom)


base = "http://localhost:8080"
s = requests.Session()

while True:
    try:
        # Get the state of the simulation
        state = s.get(f"{base}/State", headers={"accept": "application/json"}).json()
        cam0: dict[str, float | list[dict[str, float]]] = state["camData"][0]
        cam1: dict[str, float | list[dict[str, float]]] = state["camData"][1]
        ball_x = cam0["ball_x"]  # x coordinate of the ball in milimeters.
        if isinstance(ball_x, float):
            ball_x = float(ball_x)
        else:
            print("Ball pos is not float")
            continue

        if geom["field"]["dimension_x"] < ball_x < 0:
            print("ball is outside the field")
            continue

        rods: list[dict[str, int | str]] = geom["rods"]

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
                print(rod)
                break
        else:
            print("Ball is outside any player possession")

        time.sleep(1 / 50)

    except KeyboardInterrupt:
        break
