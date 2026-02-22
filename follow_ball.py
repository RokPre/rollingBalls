import time

import requests

s = requests.Session()
base = "http://localhost:8080"

while True:
    try:
        # Get the state of the simulation
        state = s.get(f"{base}/State", headers={"accept": "application/json"}).json()
        cam0 = state["camData"][0]
        ball_y = float(cam0["ball_y"])  # y coordinate of the ball in milimeters.

        # Ball is outside the field
        if ball_y < 0:
            print("ball not detected", ball_y)
            time.sleep(1 / 50)
            continue

        # Calcualte to goalie_pos: 0 => 245 mm, 1=> 455 mm.
        goalie_pos = min(max(0.0, (ball_y - 245.0) / 210.0), 1.0)

        # Payload to move the goalie
        payload = {"commands": [{
            "driveID": 1,  # Goali id
            "rotationTargetPosition": 0.0,
            "rotationVelocity": 0.0,
            "translationTargetPosition": goalie_pos,
            "translationVelocity": 1.0,  # Max speed
        }]}

        # Send payload to url.
        resp = s.post(f"{base}/Motors/SendCommand", json=payload)

        time.sleep(1 / 50)

    except KeyboardInterrupt:
        break
