import importlib
import sys
from threading import Thread

import fuzbai_simulator as fs


def physics(simulator: fs.FuzbAISimulator, red_team, blue_team):
    while simulator.viewer_running():
        # Terminated: goal scored or ball outside the field.
        # Truncated: ball stopped moving.
        if simulator.terminated() or simulator.truncated():
            simulator.reset_simulation()

        # Query delayed observation (actual measurement).
        # (ball_x [mm], ball_y [mm], ball_vx [m/s], ball_vy [m/s], rod_positions [0, 1], rod_rotations [-64, 64])
        if red_team != None:
            red_obs = simulator.delayed_observation(fs.PlayerTeam.Red, None)
            # Set motor commands
            red_motor_commands = red_team(red_obs)
            simulator.set_motor_command(red_motor_commands, fs.PlayerTeam.Red)

        if blue_team != None:
            blue_obs = simulator.delayed_observation(fs.PlayerTeam.Blue, None)
            blue_motor_commnads = blue_team(blue_obs)
            simulator.set_motor_command(blue_motor_commnads, fs.PlayerTeam.Blue)

        # Move time forward in simulation.
        simulator.step_simulation()


def main(file1: str, file2: str):
    # Create a simulation instance.
    # Multiple of these can exist, just not with the viewer enabled.
    sim = fs.FuzbAISimulator(
        # internal_step_factor: .step_simulation() = N * (2 ms)
        # sample_steps: save state to delay buffer every N * (2 ms). .delayed_observation() returns discrete samples every N * 2ms.
        internal_step_factor=10,
        sample_steps=5,
        realtime=True,
        simulated_delay_s=0.055,
        model_path=None,
        visual_config=fs.VisualConfig(
            trace_length=0, trace_ball=False,
            trace_rod_mask=0, enable_viewer=True
        )
    )

    # Disable the built-in agent on the red team.
    if file1 == "internal" or file1 == "" or file1 == "builtin":
        sim.set_external_mode(fs.PlayerTeam.Red, False)
        red_team = None
    else:
        sim.set_external_mode(fs.PlayerTeam.Red, True)
        module = importlib.import_module(file1)
        red_team = module.main
    if file2 == "internal" or file2 == "" or file2 == "builtin":
        sim.set_external_mode(fs.PlayerTeam.Blue, False)
        blue_team = None
    else:
        sim.set_external_mode(fs.PlayerTeam.Blue, True)
        module = importlib.import_module(file2)
        blue_team = module.main

    # Start the physics simulation.
    physics_thread = Thread(target=physics, args=(sim, red_team, blue_team))
    physics_thread.start()

    # Control the viewer in the main thread.
    viewer = fs.ViewerProxy()
    viewer.render_loop()
    physics_thread.join()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python compare.py <path to file1> <path to file2>")
        exit(1)

    main(sys.argv[1], sys.argv[2])
