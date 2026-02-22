import json
import time

import requests

with open("geometry.json", "r") as geom:
    geom = json.load(geom)

r_ball = geom["ball_size"] / 2  # Radius of ball


base = "http://localhost:8080"
s = requests.Session()

frame_rate = 1 / 100
NUM_OF_PREDICTS = 500

while True:
    try:
        # Get the state of the simulation
        state = s.get(f"{base}/State", headers={"accept": "application/json"}).json()
        cam0: dict[str, float | list[dict[str, float]]] = state["camData"][0]
        cam1: dict[str, float | list[dict[str, float]]] = state["camData"][1]

        ball_x = cam0["ball_x"]  # x coordinate of the ball in milimeters.
        ball_y = cam0["ball_y"]  # x coordinate of the ball in milimeters.

        # Most likely mm/s
        ball_vx = cam0["ball_vx"]  # x velocity of the ball
        ball_vy = cam0["ball_vy"]  # y velocity of the ball

        predicted_ball_pos = []

        pred_ball_x = ball_x
        pred_ball_y = ball_y

        for i in range(1, NUM_OF_PREDICTS + 1):

            old_pred_ball_x = pred_ball_x
            old_pred_ball_y = pred_ball_y

            pred_ball_x = pred_ball_x + ball_vx * frame_rate
            pred_ball_y = pred_ball_y + ball_vy * frame_rate

            # print(pred_ball_x)
            # print(pred_ball_y)

            if 0 + r_ball > pred_ball_x:
                ball_vx = -ball_vx
                pred_ball_x = old_pred_ball_x + ball_vx * frame_rate
                print("Wall bounce x")

            if pred_ball_x > 1200 - r_ball:
                ball_vx = -ball_vx
                pred_ball_x = old_pred_ball_x + ball_vx * frame_rate
                print("Wall bounce x")

            if 0 + r_ball > pred_ball_y:
                ball_vy = -ball_vy
                pred_ball_y = old_pred_ball_y + ball_vy * frame_rate
                print("Wall bounce y")

            if pred_ball_y > 700 - r_ball:
                ball_vy = -ball_vy
                pred_ball_y = old_pred_ball_y + ball_vy * frame_rate
                print("Wall bounce y")

            ball_pos = (pred_ball_x, pred_ball_y)
            predicted_ball_pos.append(ball_pos)

        time.sleep(frame_rate)

    except KeyboardInterrupt:
        break
