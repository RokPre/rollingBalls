"""Run scripted blue-half shots to test red defensive algorithms."""
from __future__ import annotations

import argparse
import importlib
import json
from math import hypot
from statistics import fmean
from threading import Thread
from typing import Callable

import fuzbai_simulator as fs


INTERNAL_STEP_FACTOR = 1
SAMPLE_STEPS = 5
SIMULATED_DELAY_S = 0.055
STEP_SECONDS = 0.002 * INTERNAL_STEP_FACTOR
PRE_SHOT_SETTLE_STEPS = round(0.500 / STEP_SECONDS)
PASS_FAKE_DURATIONS_S = [0.100, 0.250, 0.500, 0.750, 1.000]
FAKE_NUDGE_SPEED_MPS = 1
SHOT_SPEED_MPS = 3.0
MAX_STEPS_PER_SHOT = 500
BALL_Z_MM = 2.5
LIFTED_ROTATION = 0.25

Command = tuple[int, float, float, float, float]
SimState = tuple[float, float, float, float, list[float], list[float]]
TeamMain = Callable[[SimState], list[Command]]
ShotCase = tuple[str, tuple[float, float], tuple[float, float], float]


with open("geometry.json", "r", encoding="utf-8") as geometry_file:
    GEOM = json.load(geometry_file)

FIELD_X = float(GEOM["field"]["dimension_x"])
FIELD_Y = float(GEOM["field"]["dimension_y"])
BALL_SIZE = float(GEOM["ball_size"])
GOAL_WIDTH = float(GEOM["goal_width"])
MID_FIELD_X = FIELD_X / 2.0
GOAL_LOW_Y = (FIELD_Y - GOAL_WIDTH) / 2.0 + BALL_SIZE / 2
GOAL_HIGH_Y = (FIELD_Y + GOAL_WIDTH) / 2.0 - BALL_SIZE / 2
BLUE_ATTACKER_X = float(
    next(rod["position"] for rod in GEOM["rods"] if rod["team"] == "blue" and rod["players"] == 3)
)

BLUE_PARK_COMMANDS: list[Command] = [
    # Blue motor commands are mirrored by the simulator, so -0.25 keeps the
    # physical blue rods at the same lifted +0.25 rotation used by set_rod_states.
    (drive_id, 0.5, -LIFTED_ROTATION, 1.0, 1.0) for drive_id in range(1, 5)
]
RED_DISABLED_COMMANDS: list[Command] = [
    (3, 0.5, LIFTED_ROTATION, 1.0, 1.0),
    (4, 0.5, LIFTED_ROTATION, 1.0, 1.0),
]


def shot_y_values() -> list[float]:
    """Return 10 y lanes for scripted shots."""
    y_min = FIELD_Y * 0.1
    y_max = FIELD_Y * 0.9
    return [y_min + index * (y_max - y_min) / 9.0 for index in range(10)]


def shot_points() -> list[tuple[float, float]]:
    """Return 100 geometry-based direct-shot start points in the blue half."""
    blue_width = FIELD_X - MID_FIELD_X
    x_values = [MID_FIELD_X + index * blue_width / 9.0 for index in range(10)]
    return [(x, y) for x in x_values for y in shot_y_values()]


def goal_targets() -> list[tuple[float, float]]:
    """Return 10 target points across the red goal."""
    y_values = [GOAL_LOW_Y + index * (GOAL_HIGH_Y - GOAL_LOW_Y) / 9.0 for index in range(10)]
    return [(0.0, y) for y in y_values]


def pass_points() -> list[tuple[float, float]]:
    """Return top and bottom pass starts from the blue attacker x-position."""
    y_values = shot_y_values()
    top_and_bottom_y = [*y_values[:5], *y_values[-5:]]
    return [(BLUE_ATTACKER_X, y) for y in top_and_bottom_y]


def shot_cases() -> list[ShotCase]:
    """Return direct shots plus extra pass/fake shots."""
    direct_cases = [
        ("direct", point, target, 0.0) for point in shot_points() for target in goal_targets()
    ]
    pass_cases = [
        ("pass", point, target, fake_duration_s)
        for point in pass_points()
        for target in goal_targets()
        for fake_duration_s in PASS_FAKE_DURATIONS_S
    ]
    return [*direct_cases, *pass_cases]


def shot_velocity(
    point: tuple[float, float],
    target: tuple[float, float],
) -> tuple[float, float, float]:
    """Return velocity aimed from the start point into the goal target."""
    start_x, start_y = point
    target_x, target_y = target
    dx = target_x - start_x
    dy = target_y - start_y
    distance = hypot(dx, dy)
    return (
        SHOT_SPEED_MPS * dx / distance,
        SHOT_SPEED_MPS * dy / distance,
        0.0,
    )


def fake_velocity(
    point: tuple[float, float],
    target: tuple[float, float],
) -> tuple[float, float, float]:
    """Return a y-only fake nudge toward the target lane."""
    _, start_y = point
    _, target_y = target
    direction = 1.0 if target_y >= start_y else -1.0
    return (0.0, direction * FAKE_NUDGE_SPEED_MPS, 0.0)


def make_simulator(view: bool) -> fs.FuzbAISimulator:
    """Create a simulator for either fast headless mode or realtime viewer mode."""
    return fs.FuzbAISimulator(
        internal_step_factor=INTERNAL_STEP_FACTOR,
        sample_steps=SAMPLE_STEPS,
        realtime=view,
        simulated_delay_s=SIMULATED_DELAY_S,
        model_path=None,
        visual_config=fs.VisualConfig(
            trace_length=300 if view else 0,
            trace_ball=view,
            trace_rod_mask=0,
            enable_viewer=view,
        ),
    )


def initial_rod_states() -> tuple[list[float], list[float]]:
    """Park non-tested rods lifted while leaving red goalie and defense neutral."""
    positions = [0.5] * 8
    rotations = [0.0] * 8

    # Physical rod ids: blue = 3, 5, 7, 8; red midfield/attack = 4, 6.
    for rod_index in (2, 4, 6, 7, 3, 5):
        rotations[rod_index] = LIFTED_ROTATION

    return positions, rotations


def reset_shot(simulator: fs.FuzbAISimulator, point: tuple[float, float]) -> None:
    """Reset table, park rods, and place a stationary ball."""
    start_x, start_y = point
    positions, rotations = initial_rod_states()
    simulator.reset_simulation()
    simulator.set_rod_states(positions, rotations)
    simulator.serve_ball((start_x, start_y, BALL_Z_MM))
    simulator.nudge_ball((0.0, 0.0, 0.0))


def red_defense_commands(red_team: TeamMain, observation: SimState) -> list[Command]:
    """Use only goalie and defense commands from the tested red module."""
    commands = red_team(observation)
    active_defense = [command for command in commands if command[0] in {1, 2}]
    return [*active_defense, *RED_DISABLED_COMMANDS]


def settle_defense(simulator: fs.FuzbAISimulator, red_team: TeamMain) -> None:
    """Let the defense react to the placed ball before the shot starts."""
    for _ in range(PRE_SHOT_SETTLE_STEPS):
        red_obs = simulator.delayed_observation(fs.PlayerTeam.Red, None)
        simulator.set_motor_command(red_defense_commands(red_team, red_obs), fs.PlayerTeam.Red)
        simulator.set_motor_command(BLUE_PARK_COMMANDS, fs.PlayerTeam.Blue)
        simulator.step_simulation()


def fake_shot(
    simulator: fs.FuzbAISimulator,
    red_team: TeamMain,
    point: tuple[float, float],
    target: tuple[float, float],
    fake_duration_s: float,
) -> None:
    """Move the ball only in y direction before the real shot."""
    simulator.nudge_ball(fake_velocity(point, target))
    for _ in range(round(fake_duration_s / STEP_SECONDS)):
        red_obs = simulator.delayed_observation(fs.PlayerTeam.Red, None)
        simulator.set_motor_command(red_defense_commands(red_team, red_obs), fs.PlayerTeam.Red)
        simulator.set_motor_command(BLUE_PARK_COMMANDS, fs.PlayerTeam.Blue)
        simulator.step_simulation()


def run_shot(
    simulator: fs.FuzbAISimulator,
    red_team: TeamMain,
    shot_kind: str,
    point: tuple[float, float],
    target: tuple[float, float],
    fake_duration_s: float,
) -> dict[str, object]:
    """Run one shot and return simple result data."""
    reset_shot(simulator, point)
    settle_defense(simulator, red_team)
    if shot_kind == "pass":
        fake_shot(simulator, red_team, point, target, fake_duration_s)

    red_obs = simulator.delayed_observation(fs.PlayerTeam.Red, None)
    shot_start = (red_obs[0], red_obs[1])
    velocity = shot_velocity(shot_start, target)
    simulator.nudge_ball(velocity)
    initial_score = tuple(simulator.score())
    ball_speeds = []

    result = "timeout"
    steps = 0
    for steps in range(1, MAX_STEPS_PER_SHOT + 1):
        red_obs = simulator.delayed_observation(fs.PlayerTeam.Red, None)
        simulator.set_motor_command(red_defense_commands(red_team, red_obs), fs.PlayerTeam.Red)
        simulator.set_motor_command(BLUE_PARK_COMMANDS, fs.PlayerTeam.Blue)

        _, _, ball_vx, ball_vy, _, _ = red_obs
        ball_speeds.append(hypot(ball_vx, ball_vy))

        simulator.step_simulation()
        score = tuple(simulator.score())

        if score[1] > initial_score[1]:
            result = "goal_conceded"
            break
        if score[0] > initial_score[0]:
            result = "red_goal"
            break
        if simulator.terminated():
            result = "terminated"
            break
        if simulator.truncated():
            result = "truncated"
            break

    return {
        "kind": shot_kind,
        "point": point,
        "shot_start": shot_start,
        "target": target,
        "fake_duration_s": fake_duration_s,
        "fake_velocity": (
            fake_velocity(point, target) if shot_kind == "pass" else (0.0, 0.0, 0.0)
        ),
        "velocity": velocity,
        "result": result,
        "steps": steps,
        "time_s": steps * STEP_SECONDS,
        "score": tuple(simulator.score()),
        "average_ball_speed": fmean(ball_speeds) if ball_speeds else 0.0,
        "max_ball_speed": max(ball_speeds) if ball_speeds else 0.0,
    }


def print_summary(results: list[dict[str, object]]) -> None:
    """Print per-shot results and aggregate defense statistics."""
    for index, result in enumerate(results, start=1):
        shot_kind = result["kind"]
        start_x, start_y = result["point"]
        shot_start_x, shot_start_y = result["shot_start"]
        target_x, target_y = result["target"]
        velocity_x, velocity_y, _ = result["velocity"]
        fake_duration_s = float(result["fake_duration_s"])
        print(
            f"Shot {index:02d}: "
            f"type={shot_kind} "
            f"start=({start_x:.1f}, {start_y:.1f}) "
            f"shot_start=({shot_start_x:.1f}, {shot_start_y:.1f}) "
            f"target=({target_x:.1f}, {target_y:.1f}) "
            f"fake_s={fake_duration_s:.2f} "
            f"velocity=({velocity_x:.2f}, {velocity_y:.2f}) "
            f"result={result['result']} "
            f"time_s={result['time_s']:.2f} "
            f"score={result['score']}"
        )

    print_group_summary("Overall summary", results)
    print_group_summary(
        "Direct shots summary",
        [result for result in results if result["kind"] == "direct"],
    )
    print_group_summary(
        "Pass shots summary",
        [result for result in results if result["kind"] == "pass"],
    )


def print_group_summary(title: str, results: list[dict[str, object]]) -> None:
    """Print aggregate statistics for one shot group."""
    goals_conceded = sum(1 for result in results if result["result"] == "goal_conceded")
    red_goals = sum(1 for result in results if result["result"] == "red_goal")
    direct_shots = sum(1 for result in results if result["kind"] == "direct")
    pass_shots = sum(1 for result in results if result["kind"] == "pass")
    successful_defenses = len(results) - goals_conceded
    average_time = fmean([float(result["time_s"]) for result in results]) if results else 0.0
    average_ball_speed = (
        fmean([float(result["average_ball_speed"]) for result in results]) if results else 0.0
    )
    max_ball_speed = max([float(result["max_ball_speed"]) for result in results], default=0.0)

    print(title)
    print("Shots", len(results))
    print("Direct shots", direct_shots)
    print("Pass shots", pass_shots)
    print("Goals conceded", goals_conceded)
    print("Successful defenses", successful_defenses)
    print("Red goals", red_goals)
    print("Save percentage", 100.0 * successful_defenses / len(results) if results else 0.0)
    print("Average shot duration [s]", average_time)
    print("Average ball speed", average_ball_speed)
    print("Max ball speed", max_ball_speed)


def run_tests(simulator: fs.FuzbAISimulator, red_team: TeamMain, view: bool) -> None:
    """Run all scripted shots."""
    simulator.set_external_mode(fs.PlayerTeam.Red, True)
    simulator.set_external_mode(fs.PlayerTeam.Blue, True)

    results = []
    for shot_kind, point, target, fake_duration_s in shot_cases():
        if not view or simulator.viewer_running():
            results.append(
                run_shot(
                    simulator,
                    red_team,
                    shot_kind,
                    point,
                    target,
                    fake_duration_s,
                )
            )
        else:
            break

    print_summary(results)


def main() -> None:
    parser = argparse.ArgumentParser(description="Test red defense against scripted blue shots.")
    parser.add_argument("module", help="Python module with a main(sim_state) function, e.g. mainRok")
    parser.add_argument("--view", action="store_true", help="Show realtime simulator viewer")
    args = parser.parse_args()

    red_team = importlib.import_module(args.module).main
    simulator = make_simulator(args.view)

    if not args.view:
        run_tests(simulator, red_team, args.view)
        return

    physics_thread = Thread(target=run_tests, args=(simulator, red_team, args.view))
    physics_thread.start()

    viewer = fs.ViewerProxy()
    try:
        while viewer.running() and physics_thread.is_alive():
            viewer.render()
    finally:
        physics_thread.join()


if __name__ == "__main__":
    main()
