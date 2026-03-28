import importlib
import math
import sys
import time
from typing import Any

import requests

URL = "http://localhost:8080"
FPS = 50
PERIOD = 1 / FPS
REQUEST_TIMEOUT_S = 1.0


def parse_numeric_value(value: Any) -> float | None:
    """
    Convert a camera value into a finite float.
    Returns None for invalid entries such as missing values or "NaN" strings.
    """
    try:
        parsed_value = float(value)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(parsed_value):
        return None

    return parsed_value


def average_valid_values(values: list[float | None], default: float = 0.0) -> float:
    valid_values = [value for value in values if value is not None]
    if not valid_values:
        return default
    return sum(valid_values) / len(valid_values)


def frame_value(frame: dict[str, Any], key: str, default: float = 0.0) -> float | None:
    """
    Read a numeric value from a camera frame.
    Invalid camera frames contribute no value to the average.
    """
    if not frame.get("dataValid", True):
        return None
    return parse_numeric_value(frame.get(key, default))


def merge_cam_data(field_state: dict[str, Any]) -> dict[str, Any]:
    """
    Merge the external API camera state into a single observation.
    This mirrors the merge approach used by the older localhost-based runners.
    """
    cam_frames = list(field_state.get("camData", []))
    if not cam_frames:
        raise ValueError("State payload does not contain any camera data.")
    if len(cam_frames) == 1:
        cam_frames = [cam_frames[0], cam_frames[0]]

    cam0, cam1 = cam_frames[0], cam_frames[1]
    rods0 = list(cam0.get("rods", []))
    rods1 = list(cam1.get("rods", []))
    if len(rods0) != len(rods1):
        raise ValueError("Camera rod lists do not have the same length.")

    merged_rods = []
    for rod0, rod1 in zip(rods0, rods1):
        merged_rods.append(
            {
                "Item1": average_valid_values(
                    [
                        parse_numeric_value(rod0.get("Item1")),
                        parse_numeric_value(rod1.get("Item1")),
                    ]
                ),
                "Item2": average_valid_values(
                    [
                        parse_numeric_value(rod0.get("Item2")),
                        parse_numeric_value(rod1.get("Item2")),
                    ]
                ),
            }
        )

    return {
        "rods": merged_rods,
        "ball_x": average_valid_values([frame_value(cam0, "ball_x"), frame_value(cam1, "ball_x")], default=-1.0),
        "ball_y": average_valid_values([frame_value(cam0, "ball_y"), frame_value(cam1, "ball_y")], default=-1.0),
        "ball_vx": average_valid_values([frame_value(cam0, "ball_vx"), frame_value(cam1, "ball_vx")]),
        "ball_vy": average_valid_values([frame_value(cam0, "ball_vy"), frame_value(cam1, "ball_vy")]),
        "ball_size": average_valid_values([frame_value(cam0, "ball_size"), frame_value(cam1, "ball_size")]),
    }


def field_state_to_sim_state(field_state: dict[str, Any]) -> tuple[float, float, float, float, list[float], list[float]]:
    merged_cam_data = merge_cam_data(field_state)
    rod_positions = [float(rod["Item1"]) for rod in merged_cam_data["rods"]]
    rod_rotations = [float(rod["Item2"]) for rod in merged_cam_data["rods"]]
    return (
        float(merged_cam_data["ball_x"]),
        float(merged_cam_data["ball_y"]),
        float(merged_cam_data["ball_vx"]),
        float(merged_cam_data["ball_vy"]),
        rod_positions,
        rod_rotations,
    )


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

            sim_state = field_state_to_sim_state(field_state)
            module_commands = module.main(sim_state)
            payload = {"commands": normalize_motor_commands(module_commands)}

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
        except (KeyError, TypeError, ValueError) as error:
            print(f"State parsing error: {error}")
            next_tick_s = sleep_until_next_tick(next_tick_s)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 run_external.py <module>")
        sys.exit(1)

    run_external(sys.argv[1])
