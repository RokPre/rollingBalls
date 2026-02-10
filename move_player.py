import json
import random

import requests

url = "http://localhost:8080/Motors/SendCommand"

for i in range(1, 5):
    payload = {
        "commands": [
            {
                "driveID": i,
                "rotationTargetPosition": 0.1,
                "rotationVelocity": 1,
                "translationTargetPosition": random.random(),
                "translationVelocity": 1,
            }
        ]
    }

    r = requests.post(
        url,
        json=payload,
        headers={"accept": "*/*"},
        timeout=5,
    )


base = "http://localhost:8080"

r = requests.get(f"{base}/State", headers={"accept": "application/json"}, timeout=5)
print("status:", r.status_code)
print("content-type:", r.headers.get("content-type"))

state = r.json()
print(json.dumps(state, indent=2)[:4000])


r = requests.get(f"{base}/Camera/State", headers={"accept": "application/json"}, timeout=5)
print("status:", r.status_code)
cam_state = r.json()
print(json.dumps(cam_state, indent=2)[:4000])
