"""Run compare simulations in parallel and merge the final data."""
import importlib
import json
import multiprocessing as mp
import os
import sys
from math import hypot
from statistics import fmean

with open("geometry.json", "r", encoding="utf-8") as geometry_file:
    GEOM = json.load(geometry_file)

FIELD_X = float(GEOM["field"]["dimension_x"])
MID_FIELD_X = FIELD_X / 2.0
RED_ROD_X = [float(GEOM["rods"][rod_index]["position"]) for rod_index in (0, 1, 3, 5)]
BLUE_ROD_X = [float(GEOM["rods"][rod_index]["position"]) for rod_index in (7, 6, 4, 2)]
ROD_NAMES = ["goalie", "defense", "midfield", "attack"]
OBSERVATION_PERIOD_S = 0.002 * 10 * 5
SHOT_X_WINDOW_MM = 50.0
SHOT_MIN_SPEED_MPS = 0.7
SHOT_MIN_SPEED_GAIN_MPS = 0.35
SHOT_COOLDOWN_STEPS = 3
GOAL_TARGET = 100
PROGRESS = None
PROGRESS_LOCK = None


def _worker_count() -> int:
    cpu_count = os.cpu_count() or 1
    return max(1, int(cpu_count * 0.8))


def _load_team(simulator, player_team, module_name: str, fs):
    if module_name in {"internal", "", "builtin"}:
        simulator.set_external_mode(player_team, False)
        return None

    simulator.set_external_mode(player_team, True)
    return importlib.import_module(module_name).main


def _init_worker(progress, progress_lock) -> None:
    global PROGRESS
    global PROGRESS_LOCK
    PROGRESS = progress
    PROGRESS_LOCK = progress_lock


def _print_progress() -> None:
    with PROGRESS_LOCK:
        print("Progress", list(PROGRESS), flush=True)


def _run_single_match(worker_id: int, file1: str, file2: str) -> dict[str, object]:
    import fuzbai_simulator as fs

    simulator = fs.FuzbAISimulator(
        internal_step_factor=10,
        sample_steps=5,
        realtime=False,
        simulated_delay_s=0.055,
        model_path=None,
        visual_config=fs.VisualConfig(
            trace_length=0,
            trace_ball=False,
            trace_rod_mask=0,
            enable_viewer=False,
        ),
    )

    red_team = _load_team(simulator, fs.PlayerTeam.Red, file1, fs)
    blue_team = _load_team(simulator, fs.PlayerTeam.Blue, file2, fs)

    ball_pos: list[tuple[float, float]] = []
    ball_vel: list[tuple[float, float]] = []
    last_score = (0, 0)

    while True:
        if simulator.terminated() or simulator.truncated():
            simulator.reset_simulation()

        red_obs = simulator.delayed_observation(fs.PlayerTeam.Red, None)
        if red_team is not None:
            simulator.set_motor_command(red_team(red_obs), fs.PlayerTeam.Red)

        if blue_team is not None:
            blue_obs = simulator.delayed_observation(fs.PlayerTeam.Blue, None)
            simulator.set_motor_command(blue_team(blue_obs), fs.PlayerTeam.Blue)

        ball_x, ball_y, ball_vx, ball_vy, _, _ = red_obs
        ball_pos.append((ball_x, ball_y))
        ball_vel.append((ball_vx, ball_vy))

        simulator.step_simulation()
        score = tuple(simulator.score())
        if score != last_score:
            progress_index = 2 * (worker_id - 1)
            PROGRESS[progress_index] = score[0]
            PROGRESS[progress_index + 1] = score[1]
            _print_progress()
            last_score = score

        if score[0] >= GOAL_TARGET or score[1] >= GOAL_TARGET:
            return {
                "worker_id": worker_id,
                "score": score,
                "ball_pos": ball_pos,
                "ball_vel": ball_vel,
            }


def _merge_results(results: list[dict[str, object]]) -> tuple[tuple[int, int], list[tuple[float, float]], list[tuple[float, float]]]:
    total_score = [0, 0]
    ball_pos: list[tuple[float, float]] = []
    ball_vel: list[tuple[float, float]] = []

    for result in results:
        red_score, blue_score = result["score"]
        total_score[0] += red_score
        total_score[1] += blue_score
        ball_pos.extend(result["ball_pos"])
        ball_vel.extend(result["ball_vel"])

    return (total_score[0], total_score[1]), ball_pos, ball_vel


def _mean(values: list[float]) -> float:
    return float(fmean(values)) if values else 0.0


def analysis(score: tuple[int, int], ball_pos: list[tuple[float, float]], ball_vel: list[tuple[float, float]]) -> None:
    if not ball_pos or not ball_vel:
        print("No match data collected.")
        return

    total_play_time_s = len(ball_pos) * OBSERVATION_PERIOD_S
    ball_pos_x = _mean([pos[0] for pos in ball_pos])
    ball_vel_x = _mean([vel[0] for vel in ball_vel])
    ball_vel_y = _mean([vel[1] for vel in ball_vel])
    ball_speeds = [hypot(vel_x, vel_y) for vel_x, vel_y in ball_vel]
    average_ball_speed = _mean(ball_speeds)
    max_ball_speed = max(ball_speeds)

    red_possession_time = [0, 0, 0, 0]
    blue_possession_time = [0, 0, 0, 0]
    possession_owner_by_sample = []
    for ball_x, _ in ball_pos:
        if 40 <= ball_x < 130:
            red_possession_time[0] += 1
            possession_owner_by_sample.append("red")
        elif 180 <= ball_x < 280:
            red_possession_time[1] += 1
            possession_owner_by_sample.append("red")
        elif 330 <= ball_x < 430:
            blue_possession_time[3] += 1
            possession_owner_by_sample.append("blue")
        elif 480 <= ball_x < 580:
            red_possession_time[2] += 1
            possession_owner_by_sample.append("red")
        elif 630 <= ball_x < 730:
            blue_possession_time[2] += 1
            possession_owner_by_sample.append("blue")
        elif 780 <= ball_x < 880:
            red_possession_time[3] += 1
            possession_owner_by_sample.append("red")
        elif 930 <= ball_x < 1030:
            blue_possession_time[1] += 1
            possession_owner_by_sample.append("blue")
        elif 1080 <= ball_x < 1170:
            blue_possession_time[0] += 1
            possession_owner_by_sample.append("blue")
        else:
            possession_owner_by_sample.append("none")

    total_red_possession_time = sum(red_possession_time)
    total_blue_possession_time = sum(blue_possession_time)
    total_possession_time = total_red_possession_time + total_blue_possession_time

    if total_possession_time == 0:
        red_possession_percentage = 0.0
        blue_possession_percentage = 0.0
    else:
        red_possession_percentage = 100.0 * total_red_possession_time / total_possession_time
        blue_possession_percentage = 100.0 * total_blue_possession_time / total_possession_time

    best_rod_red = red_possession_time.index(max(red_possession_time))
    best_rod_blue = blue_possession_time.index(max(blue_possession_time))

    red_attacking_half_samples = sum(1 for ball_x, _ in ball_pos if ball_x >= MID_FIELD_X)
    blue_attacking_half_samples = len(ball_pos) - red_attacking_half_samples
    red_attacking_half_percentage = 100.0 * red_attacking_half_samples / len(ball_pos)
    blue_attacking_half_percentage = 100.0 * blue_attacking_half_samples / len(ball_pos)

    red_shot_speeds = []
    blue_shot_speeds = []
    red_shots_by_rod = [0, 0, 0, 0]
    blue_shots_by_rod = [0, 0, 0, 0]
    shot_cooldown = 0
    for index in range(1, len(ball_pos)):
        if shot_cooldown > 0:
            shot_cooldown -= 1
            continue

        previous_ball_x, _ = ball_pos[index - 1]
        previous_ball_vx, previous_ball_vy = ball_vel[index - 1]
        ball_vx, ball_vy = ball_vel[index]

        previous_ball_speed = hypot(previous_ball_vx, previous_ball_vy)
        current_ball_speed = hypot(ball_vx, ball_vy)
        speed_gain = current_ball_speed - previous_ball_speed

        if current_ball_speed < SHOT_MIN_SPEED_MPS or speed_gain < SHOT_MIN_SPEED_GAIN_MPS:
            continue

        closest_red_rod = min(range(len(RED_ROD_X)), key=lambda i: abs(previous_ball_x - RED_ROD_X[i]))
        closest_blue_rod = min(range(len(BLUE_ROD_X)), key=lambda i: abs(previous_ball_x - BLUE_ROD_X[i]))
        red_rod_is_close = abs(previous_ball_x - RED_ROD_X[closest_red_rod]) <= SHOT_X_WINDOW_MM
        blue_rod_is_close = abs(previous_ball_x - BLUE_ROD_X[closest_blue_rod]) <= SHOT_X_WINDOW_MM

        if ball_vx > 0 and red_rod_is_close:
            red_shot_speeds.append(current_ball_speed)
            red_shots_by_rod[closest_red_rod] += 1
            shot_cooldown = SHOT_COOLDOWN_STEPS
        elif ball_vx < 0 and blue_rod_is_close:
            blue_shot_speeds.append(current_ball_speed)
            blue_shots_by_rod[closest_blue_rod] += 1
            shot_cooldown = SHOT_COOLDOWN_STEPS

    red_average_shot_speed = _mean(red_shot_speeds)
    blue_average_shot_speed = _mean(blue_shot_speeds)
    red_fastest_shot = max(red_shot_speeds) if red_shot_speeds else 0.0
    blue_fastest_shot = max(blue_shot_speeds) if blue_shot_speeds else 0.0
    red_shot_conversion = 100.0 * score[0] / len(red_shot_speeds) if red_shot_speeds else 0.0
    blue_shot_conversion = 100.0 * score[1] / len(blue_shot_speeds) if blue_shot_speeds else 0.0
    red_average_time_per_goal_s = total_play_time_s / score[0] if score[0] > 0 else 0.0
    blue_average_time_per_goal_s = total_play_time_s / score[1] if score[1] > 0 else 0.0

    longest_red_possession_streak = 0
    longest_blue_possession_streak = 0
    current_red_possession_streak = 0
    current_blue_possession_streak = 0
    for possession_owner in possession_owner_by_sample:
        if possession_owner == "red":
            current_red_possession_streak += 1
            current_blue_possession_streak = 0
        elif possession_owner == "blue":
            current_blue_possession_streak += 1
            current_red_possession_streak = 0
        else:
            current_red_possession_streak = 0
            current_blue_possession_streak = 0

        longest_red_possession_streak = max(longest_red_possession_streak, current_red_possession_streak)
        longest_blue_possession_streak = max(longest_blue_possession_streak, current_blue_possession_streak)

    red_shots_by_rod_named = dict(zip(ROD_NAMES, red_shots_by_rod))
    blue_shots_by_rod_named = dict(zip(ROD_NAMES, blue_shots_by_rod))

    print("Combined score", score)
    print("Total play time [s]", total_play_time_s)
    print("Ball position x", ball_pos_x)
    print("Ball velocity x", ball_vel_x)
    print("Ball velocity y", ball_vel_y)
    print("Average ball speed", average_ball_speed)
    print("Max ball speed", max_ball_speed)
    print("Red possession time [s]", total_red_possession_time * OBSERVATION_PERIOD_S)
    print("Blue possession time [s]", total_blue_possession_time * OBSERVATION_PERIOD_S)
    print("Red possession percentage", red_possession_percentage)
    print("Blue possession percentage", blue_possession_percentage)
    print("Red rod with most possession", best_rod_red)
    print("Blue rod with most possession", best_rod_blue)
    print("Red attacking-half percentage", red_attacking_half_percentage)
    print("Blue attacking-half percentage", blue_attacking_half_percentage)
    print("Red shots", len(red_shot_speeds))
    print("Blue shots", len(blue_shot_speeds))
    print("Red shot conversion [%]", red_shot_conversion)
    print("Blue shot conversion [%]", blue_shot_conversion)
    print("Red average shot speed", red_average_shot_speed)
    print("Blue average shot speed", blue_average_shot_speed)
    print("Red fastest shot", red_fastest_shot)
    print("Blue fastest shot", blue_fastest_shot)
    print("Red shots by rod", red_shots_by_rod_named)
    print("Blue shots by rod", blue_shots_by_rod_named)
    print("Red longest possession streak [s]", longest_red_possession_streak * OBSERVATION_PERIOD_S)
    print("Blue longest possession streak [s]", longest_blue_possession_streak * OBSERVATION_PERIOD_S)
    print("Red average time per goal [s]", red_average_time_per_goal_s)
    print("Blue average time per goal [s]", blue_average_time_per_goal_s)


def main(file1: str, file2: str) -> None:
    worker_count = _worker_count()
    print("Worker processes", worker_count)

    ctx = mp.get_context("spawn")
    progress = ctx.Array("i", worker_count * 2)
    progress_lock = ctx.Lock()
    print("Progress", list(progress), flush=True)

    with ctx.Pool(
        processes=worker_count,
        initializer=_init_worker,
        initargs=(progress, progress_lock),
    ) as pool:
        worker_args = [(worker_id, file1, file2) for worker_id in range(1, worker_count + 1)]
        results = []
        for result in pool.starmap(_run_single_match, worker_args):
            print(f"Simulator {result['worker_id']} score {result['score']}")
            results.append(result)

    score, ball_pos, ball_vel = _merge_results(results)
    analysis(score, ball_pos, ball_vel)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_mp.py <path to file1> <path to file2>")
        raise SystemExit(1)

    main(sys.argv[1], sys.argv[2])
