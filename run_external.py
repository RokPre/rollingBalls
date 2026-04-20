import importlib
import sys
import time
from typing import Any

import requests

# URL = "http://192.168.90.149:8000"
URL = "http://192.168.90.132:24003"
# URL = "http://localhost:8080"
FPS = 50
PERIOD = 1 / FPS
REQUEST_TIMEOUT_S = 1.0


def normalize_motor_commands(commands: list[Any]) -> list[dict[str, float | int]]:
    """
    Convert module output into the command dictionary format expected by the
    external HTTP endpoint. Supports both tuple and dict command styles.
    """
    normalized_commands = []
    for command in commands:
        if isinstance(command, dict):
            normalized_commands.append(
                {
                    "driveID": int(command["driveID"]),
                    "translationTargetPosition": float(command["translationTargetPosition"]),
                    "translationVelocity": float(command["translationVelocity"]),
                    "rotationTargetPosition": float(command["rotationTargetPosition"]),
                    "rotationVelocity": float(command["rotationVelocity"]),
                }
            )
            continue

        drive_id, translation_target, rotation_target, translation_velocity, rotation_velocity = command
        normalized_commands.append(
            {
                "driveID": int(drive_id),
                "translationTargetPosition": float(translation_target),
                "translationVelocity": float(translation_velocity),
                "rotationTargetPosition": float(rotation_target),
                "rotationVelocity": float(rotation_velocity),
            }
        )

    return normalized_commands


def sleep_until_next_tick(next_tick_s: float | None) -> float:
    now = time.perf_counter()
    if next_tick_s is None:
        return now + PERIOD

    remaining_sleep_s = next_tick_s - now
    if remaining_sleep_s > 0:
        time.sleep(remaining_sleep_s)

    return next_tick_s + PERIOD


prev_cam_data: tuple[float, float, float, float, list[float], list[float]] = (0.0, 0.0, 0.0, 0.0, [], [],)


def merge_cam_data(field_state: dict[str, Any]) -> tuple[float, float, float, float, list[float], list[float]]:
    global prev_cam_data

    cam_data = field_state.get("camData", [])

    if len(cam_data) < 2:
        return prev_cam_data

    cam_0 = cam_data[0]
    cam_1 = cam_data[1]

    cam_0_valid = cam_0.get("dataValid", True)
    cam_1_valid = cam_1.get("dataValid", True)

    if cam_0_valid and cam_1_valid:
        pass
    elif cam_0_valid:
        cam_1 = cam_0
    elif cam_1_valid:
        cam_0 = cam_1
    else:
        return prev_cam_data

    cam_0_ball_size = cam_0.get("ball_size")
    cam_1_ball_size = cam_1.get("ball_size")

    if cam_0_ball_size < 0:
        cam_0_ball_size = 0.0
    if cam_1_ball_size < 0:
        cam_1_ball_size = 0.0

    if cam_0_ball_size is None or cam_1_ball_size is None:
        return prev_cam_data
    if cam_0_ball_size == 0 and cam_1_ball_size == 0:
        return prev_cam_data

    total_ball_size = cam_0_ball_size + cam_1_ball_size

    cam_0_weight = cam_0_ball_size / total_ball_size
    cam_1_weight = cam_1_ball_size / total_ball_size

    cam_0_ball_x = cam_0.get("ball_x")
    cam_0_ball_y = cam_0.get("ball_y")
    cam_0_ball_vx = cam_0.get("ball_vx")
    cam_0_ball_vy = cam_0.get("ball_vy")
    cam_1_ball_x = cam_1.get("ball_x")
    cam_1_ball_y = cam_1.get("ball_y")
    cam_1_ball_vx = cam_1.get("ball_vx")
    cam_1_ball_vy = cam_1.get("ball_vy")

    if not isinstance(cam_0_ball_x, (int, float)) and not isinstance(cam_1_ball_x, (int, float)):
        cam_0_ball_x = prev_cam_data[0]
        cam_1_ball_x = prev_cam_data[0]
    elif not isinstance(cam_0_ball_x, (int, float)):
        cam_0_ball_x = cam_1_ball_x
    elif not isinstance(cam_1_ball_x, (int, float)):
        cam_1_ball_x = cam_0_ball_x

    if not isinstance(cam_0_ball_y, (int, float)) and not isinstance(cam_1_ball_y, (int, float)):
        cam_0_ball_y = prev_cam_data[1]
        cam_1_ball_y = prev_cam_data[1]
    elif not isinstance(cam_0_ball_y, (int, float)):
        cam_0_ball_y = cam_1_ball_y
    elif not isinstance(cam_1_ball_y, (int, float)):
        cam_1_ball_y = cam_0_ball_y

    if not isinstance(cam_0_ball_vx, (int, float)) and not isinstance(cam_1_ball_vx, (int, float)):
        cam_0_ball_vx = prev_cam_data[2]
        cam_1_ball_vx = prev_cam_data[2]
    elif not isinstance(cam_0_ball_vx, (int, float)):
        cam_0_ball_vx = cam_1_ball_vx
    elif not isinstance(cam_1_ball_vx, (int, float)):
        cam_1_ball_vx = cam_0_ball_vx

    if not isinstance(cam_0_ball_vy, (int, float)) and not isinstance(cam_1_ball_vy, (int, float)):
        cam_0_ball_vy = prev_cam_data[3]
        cam_1_ball_vy = prev_cam_data[3]
    elif not isinstance(cam_0_ball_vy, (int, float)):
        cam_0_ball_vy = cam_1_ball_vy
    elif not isinstance(cam_1_ball_vy, (int, float)):
        cam_1_ball_vy = cam_0_ball_vy

    output_ball_x = cam_0_ball_x * cam_0_weight + cam_1_ball_x * cam_1_weight
    output_ball_y = cam_0_ball_y * cam_0_weight + cam_1_ball_y * cam_1_weight
    output_ball_vx = cam_0_ball_vx * cam_0_weight + cam_1_ball_vx * cam_1_weight
    output_ball_vy = cam_0_ball_vy * cam_0_weight + cam_1_ball_vy * cam_1_weight

    cam_0_rods = list(cam_0.get("rods", []))
    cam_1_rods = list(cam_1.get("rods", []))

    output_rods_pos: list[float] = []
    output_rods_rot: list[float] = []

    for rod_0, rod_1 in zip(cam_0_rods, cam_1_rods):
        rod_0_pos = rod_0.get("Item1")
        rod_0_rot = rod_0.get("Item2")
        rod_1_pos = rod_1.get("Item1")
        rod_1_rot = rod_1.get("Item2")

        if not isinstance(rod_0_pos, (int, float)) and not isinstance(rod_1_pos, (int, float)):
            rod_0_pos = 0.0
            rod_1_pos = 0.0
        elif not isinstance(rod_0_pos, (int, float)):
            rod_0_pos = rod_1_pos
        elif not isinstance(rod_1_pos, (int, float)):
            rod_1_pos = rod_0_pos

        if not isinstance(rod_0_rot, (int, float)) and not isinstance(rod_1_rot, (int, float)):
            rod_0_rot = 0.0
            rod_1_rot = 0.0
        elif not isinstance(rod_0_rot, (int, float)):
            rod_0_rot = rod_1_rot
        elif not isinstance(rod_1_rot, (int, float)):
            rod_1_rot = rod_0_rot

        output_rods_pos.append((rod_0_pos + rod_1_pos) / 2)
        output_rods_rot.append((rod_0_rot + rod_1_rot) / 2)

    output = output_ball_x, output_ball_y, output_ball_vx, output_ball_vy, output_rods_pos, output_rods_rot
    prev_cam_data = output

    return output


def run_external(module_name: str) -> None:
    module = importlib.import_module(module_name)
    if hasattr(module, "init"):
        module.init()

    session = requests.Session()
    next_tick_s: float | None = None

    while True:
        try:
            state_response = session.get(f"{URL}/State", timeout=REQUEST_TIMEOUT_S)
            state_response.raise_for_status()
            field_state = state_response.json()

            if not field_state.get("motorsReady", True):
                next_tick_s = sleep_until_next_tick(next_tick_s)
                continue

            merged_cam_data = merge_cam_data(field_state)
            # print(merged_cam_data)

            module_commands = module.main(merged_cam_data)
            payload = {"commands": normalize_motor_commands(module_commands)}
            # print("------")

            command_response = session.post(
                f"{URL}/Motors/SendCommand",
                json=payload,
                timeout=REQUEST_TIMEOUT_S,
            )
            command_response.raise_for_status()

            next_tick_s = sleep_until_next_tick(next_tick_s)

        except KeyboardInterrupt:
            return
        except requests.RequestException as error:
            print(f"HTTP error: {error}")
            next_tick_s = sleep_until_next_tick(next_tick_s)
        except (KeyError, TypeError, ValueError, IndexError) as error:
            print(f"State parsing error: {error}")
            next_tick_s = sleep_until_next_tick(next_tick_s)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 run_external.py <module>")
        sys.exit(1)

    run_external(sys.argv[1])
