import json
import time
from dataclasses import dataclass

import requests

URL = "http://localhost:8080"
FPS = 50
PERIOD = 1.0 / FPS
MAX_PRED_TIME = 1.2

with open("geometry.json", "r", encoding="utf-8") as g:
    GEOM = json.load(g)

FIELD_X = float(GEOM["field"]["dimension_x"])
FIELD_Y = float(GEOM["field"]["dimension_y"])
BALL_SIZE = float(GEOM["ball_size"])
BALL_R = BALL_SIZE / 2.0
GOAL_W = float(GEOM["goal_width"])
GOAL_MID_Y = FIELD_Y / 2.0
GOAL_TOP_Y = (FIELD_Y - GOAL_W) / 2.0
GOAL_BOT_Y = (FIELD_Y + GOAL_W) / 2.0

MY_RODS = [1, 2, 4, 6]
ROD_TO_DRIVE = {1: 1, 2: 2, 4: 3, 6: 4}


@dataclass
class MotorCommand:
    driveID: int
    translationTargetPosition: float
    translationVelocity: float
    rotationTargetPosition: float
    rotationVelocity: float

    def as_dict(self) -> dict[str, float | int]:
        return {
            "driveID": self.driveID,
            "translationTargetPosition": float(self.translationTargetPosition),
            "translationVelocity": float(self.translationVelocity),
            "rotationTargetPosition": float(self.rotationTargetPosition),
            "rotationVelocity": float(self.rotationVelocity),
        }


class Rod:
    def __init__(self, rod_id: int):
        geom = GEOM["rods"][rod_id - 1]
        self.rod_id = rod_id
        self.drive_id = ROD_TO_DRIVE[rod_id]
        self.position = float(geom["position"])
        self.travel = float(geom["travel"])
        self.players = int(geom["players"])
        self.first_offset = float(geom["first_offset"])
        self.spacing = float(geom["spacing"])
        self.translation_r = 0.5
        self.rotation_r = 0.0
        self.player_y = [0.0] * self.players
        self.update_from_cam({f"rod{rod_id}_item1": 0.5, f"rod{rod_id}_item2": 0.0})

    def update_from_cam(self, cam: dict[str, float]) -> None:
        self.translation_r = float(cam.get(f"rod{self.rod_id}_item1", self.translation_r))
        self.rotation_r = float(cam.get(f"rod{self.rod_id}_item2", self.rotation_r))
        for i in range(self.players):
            self.player_y[i] = self.first_offset + self.spacing * i + self.travel * self.translation_r

    def best_translation_for_y(self, target_y: float) -> tuple[float, float, float, int]:
        best_cal = 0.5
        best_player_y = self.first_offset + 0.5 * self.travel
        best_err = float("inf")
        best_idx = 0

        for i in range(self.players):
            pos_needed = target_y - self.first_offset - i * self.spacing
            cal = pos_needed / self.travel if self.travel > 1e-9 else 0.5
            cal = clamp(cal, 0.0, 1.0)
            player_y = self.first_offset + i * self.spacing + cal * self.travel
            err = abs(player_y - target_y)
            if err < best_err:
                best_err = err
                best_cal = cal
                best_player_y = player_y
                best_idx = i

        return best_cal, best_player_y, best_err, best_idx


class FoosballController:
    def __init__(self):
        self.rods = {rid: Rod(rid) for rid in MY_RODS}
        self.next_time = None
        self.swing_frames = {rid: 0 for rid in MY_RODS}
        self.cooldown_frames = {rid: 0 for rid in MY_RODS}

    def step(self, cam: dict[str, float]) -> list[dict[str, float | int]]:
        for rid in MY_RODS:
            self.rods[rid].update_from_cam(cam)

        for rid in MY_RODS:
            if self.cooldown_frames[rid] > 0:
                self.cooldown_frames[rid] -= 1

        ball_visible = self.ball_visible(cam)
        if not ball_visible:
            return [self.home_command(rid).as_dict() for rid in MY_RODS]

        ball_x = float(cam["ball_x"])
        ball_y = float(cam["ball_y"])
        vx = float(cam["ball_vx"])

        owner = self.find_ball_owner(ball_x, ball_y)
        attack_owner = owner if owner in (4, 6) else None
        clear_owner = owner if owner in (1, 2) else None

        commands: dict[int, MotorCommand] = {}

        # Defense first: keeper + two defenders.
        commands[1] = self.keeper_command(cam, clear_owner == 1)
        commands[2] = self.defender_command(cam, clear_owner == 2)
        commands[4] = self.midfield_command(cam, attack_owner == 4, attack_owner)
        commands[6] = self.attacker_command(cam, attack_owner == 6, attack_owner)

        # During an active attack, lift friendly rods that are further forward so the
        # shot lane stays open toward the opponent goal.
        if attack_owner in (4, 6) and self.swing_frames[attack_owner] > 0:
            self.apply_attack_lane_clear(commands, attack_owner, ball_y)

        # When ball is deep on opponent side, defenders can relax to compact shape.
        if ball_x > 0.75 * FIELD_X and vx > 0:
            commands[1] = self.command_for_target(1, GOAL_MID_Y, rot=0.0)
            commands[2] = self.command_for_target(2, GOAL_MID_Y, rot=0.0)

        return [commands[rid].as_dict() for rid in MY_RODS]

    def apply_attack_lane_clear(self, commands: dict[int, MotorCommand], attack_rod_id: int, ball_y: float) -> None:
        attack_x = self.rods[attack_rod_id].position
        for rid in MY_RODS:
            if rid == attack_rod_id:
                continue
            # Only move friendly rods that are further toward the opponent goal.
            if self.rods[rid].position <= attack_x:
                continue
            commands[rid] = self.lane_clear_command(rid, ball_y)

    def lane_clear_command(self, rod_id: int, ball_y: float) -> MotorCommand:
        rod = self.rods[rod_id]
        # Move the rod away from the current shot lane and rotate it upward so it
        # interferes as little as possible with the ball trajectory.
        offset = 0.30 * FIELD_Y
        if ball_y <= GOAL_MID_Y:
            clear_y = clamp(ball_y + offset, BALL_R, FIELD_Y - BALL_R)
        else:
            clear_y = clamp(ball_y - offset, BALL_R, FIELD_Y - BALL_R)
        cal, _, _, _ = rod.best_translation_for_y(clear_y)
        return MotorCommand(rod.drive_id, cal, 1.0, 0.65, 1.0)

    def ball_visible(self, cam: dict[str, float]) -> bool:
        ball_x = float(cam.get("ball_x", -1.0))
        ball_y = float(cam.get("ball_y", -1.0))
        return 0.0 <= ball_x <= FIELD_X and 0.0 <= ball_y <= FIELD_Y

    def find_ball_owner(self, ball_x: float, ball_y: float) -> int | None:
        control_margin_x = BALL_R + 22.0
        control_margin_y = BALL_R + 12.0
        best_rod = None
        best_score = float("inf")

        for rid, rod in self.rods.items():
            dx = abs(ball_x - rod.position)
            if dx > control_margin_x:
                continue
            for py in rod.player_y:
                dy = abs(ball_y - py)
                if dy <= control_margin_y:
                    score = dx + 0.5 * dy
                    if score < best_score:
                        best_score = score
                        best_rod = rid
        return best_rod

    def keeper_command(self, cam: dict[str, float], should_clear: bool) -> MotorCommand:
        rod_x = self.rods[1].position
        target_y = self.intercept_y(cam, rod_x)

        # Keep compact inside the goal mouth.
        target_y = clamp(target_y, GOAL_TOP_Y + 0.25 * BALL_SIZE, GOAL_BOT_Y - 0.25 * BALL_SIZE)

        if should_clear:
            return self.attack_or_prepare(1, cam, preferred_y=target_y)
        return self.command_for_target(1, target_y, rot=0.0)

    def defender_command(self, cam: dict[str, float], should_clear: bool) -> MotorCommand:
        ball_x = float(cam["ball_x"])
        ball_y = float(cam["ball_y"])
        vx = float(cam["ball_vx"])
        rod_x = self.rods[2].position

        if ball_x < 0.45 * FIELD_X or (vx < 0.0 and ball_x < 0.60 * FIELD_X):
            target_y = self.intercept_y(cam, rod_x)
        else:
            # Compact defensive shape similar to your dedicated defense module.
            if ball_y < GOAL_MID_Y:
                target_y = GOAL_MID_Y - BALL_SIZE
            else:
                target_y = GOAL_MID_Y + BALL_SIZE

        if should_clear:
            return self.attack_or_prepare(2, cam, preferred_y=target_y)
        return self.command_for_target(2, target_y, rot=0.0)

    def midfield_command(self, cam: dict[str, float], has_ball: bool, attack_owner: int | None) -> MotorCommand:
        ball_x = float(cam["ball_x"])
        ball_y = float(cam["ball_y"])
        vx = float(cam["ball_vx"])
        rod_x = self.rods[4].position

        if has_ball:
            return self.attack_or_prepare(4, cam, preferred_y=ball_y)

        if ball_x < rod_x + 180.0 and (vx <= 120.0 or ball_x < rod_x):
            target_y = self.intercept_y(cam, rod_x)
        elif attack_owner == 6:
            target_y = GOAL_MID_Y
        else:
            target_y = 0.65 * ball_y + 0.35 * GOAL_MID_Y

        return self.command_for_target(4, target_y, rot=0.08)

    def attacker_command(self, cam: dict[str, float], has_ball: bool, attack_owner: int | None) -> MotorCommand:
        ball_x = float(cam["ball_x"])
        vx = float(cam["ball_vx"])
        rod_x = self.rods[6].position

        if has_ball:
            return self.attack_or_prepare(6, cam, preferred_y=float(cam["ball_y"]))

        if ball_x < rod_x + 140.0 and (vx <= 180.0 or ball_x < rod_x):
            target_y = self.intercept_y(cam, rod_x)
        elif attack_owner == 4:
            target_y = GOAL_MID_Y
        else:
            target_y = GOAL_MID_Y

        return self.command_for_target(6, target_y, rot=0.12)

    def attack_or_prepare(self, rod_id: int, cam: dict[str, float], preferred_y: float) -> MotorCommand:
        rod = self.rods[rod_id]
        ball_y = float(cam["ball_y"])
        ball_x = float(cam["ball_x"])
        ball_vx = float(cam.get("ball_vx", 0.0))

        if self.swing_frames[rod_id] > 0:
            self.swing_frames[rod_id] -= 1
            if self.swing_frames[rod_id] == 0:
                self.cooldown_frames[rod_id] = 6
            cal, _, _, _ = rod.best_translation_for_y(preferred_y)
            return MotorCommand(rod.drive_id, cal, 1.0, -0.35, 1.0)

        target_y = self.choose_attack_y(rod_id, cam)
        cal, player_y, err, _ = rod.best_translation_for_y(target_y)

        # Our team plays from left to right, so a safe strike should happen only when
        # the ball is on or slightly in front of the figure. This reduces own goals
        # caused by hitting a ball that is still behind the rod toward our goal.
        front_margin = BALL_R + 26.0
        back_margin = 6.0
        ball_in_front = (rod.position - back_margin) <= ball_x <= (rod.position + front_margin)
        ball_clearly_behind = ball_x < (rod.position - back_margin)
        moving_forward_enough = ball_vx >= -35.0
        y_ready = abs(player_y - ball_y) <= max(14.0, BALL_R * 0.9)

        if ball_in_front and moving_forward_enough and y_ready and self.cooldown_frames[rod_id] == 0:
            self.swing_frames[rod_id] = 3
            return MotorCommand(rod.drive_id, cal, 1.0, -0.35, 1.0)

        prep_rot = 0.10 if rod_id in (4, 6) else 0.05
        if err > BALL_SIZE or ball_clearly_behind:
            prep_rot = 0.0
        return MotorCommand(rod.drive_id, cal, 1.0, prep_rot, 1.0)

    def choose_attack_y(self, rod_id: int, cam: dict[str, float]) -> float:
        # Base attack point is current ball y. For the striker, bias the shot away from enemy goalie.
        ball_y = float(cam["ball_y"])
        if rod_id != 6:
            return ball_y

        enemy_goalie_y = self.enemy_goalie_y(cam)
        if enemy_goalie_y < GOAL_MID_Y:
            biased_y = GOAL_BOT_Y - 0.35 * GOAL_W
        else:
            biased_y = GOAL_TOP_Y + 0.35 * GOAL_W

        # Do not over-bias; keep the shot reachable around the actual ball position.
        return clamp(0.65 * ball_y + 0.35 * biased_y, BALL_R, FIELD_Y - BALL_R)

    def enemy_goalie_y(self, cam: dict[str, float]) -> float:
        rel = float(cam.get("rod8_item1", 0.5))
        geom = GEOM["rods"][7]
        return float(geom["first_offset"] + geom["travel"] * rel)

    def command_for_target(self, rod_id: int, target_y: float, rot: float = 0.0) -> MotorCommand:
        rod = self.rods[rod_id]
        cal, _, _, _ = rod.best_translation_for_y(target_y)
        return MotorCommand(rod.drive_id, cal, 1.0, rot, 1.0)

    def home_command(self, rod_id: int) -> MotorCommand:
        return MotorCommand(ROD_TO_DRIVE[rod_id], 0.5, 1.0, 0.0, 0.8)

    def intercept_y(self, cam: dict[str, float], rod_x: float) -> float:
        path = predict_ball_path(
            ball_x=float(cam["ball_x"]),
            ball_y=float(cam["ball_y"]),
            ball_vx=float(cam["ball_vx"]),
            ball_vy=float(cam["ball_vy"]),
            dt=PERIOD,
            max_time=MAX_PRED_TIME,
        )

        # Prefer first future crossing of the rod x plane.
        for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
            if (x0 - rod_x) == 0:
                return clamp(y0, BALL_R, FIELD_Y - BALL_R)
            crossed = (x0 - rod_x) * (x1 - rod_x) <= 0.0
            if crossed and abs(x1 - x0) > 1e-9:
                alpha = (rod_x - x0) / (x1 - x0)
                y = y0 + alpha * (y1 - y0)
                return clamp(y, BALL_R, FIELD_Y - BALL_R)

        # Fallback to nearest predicted point in x.
        best_x, best_y = min(path, key=lambda p: abs(p[0] - rod_x))
        _ = best_x
        return clamp(best_y, BALL_R, FIELD_Y - BALL_R)

    def sleep(self) -> None:
        now = time.perf_counter()
        if self.next_time is None:
            self.next_time = now + PERIOD
            return
        sleep_time = self.next_time - now
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.next_time += PERIOD


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def predict_ball_path(
    ball_x: float,
    ball_y: float,
    ball_vx: float,
    ball_vy: float,
    dt: float,
    max_time: float,
) -> list[tuple[float, float]]:
    steps = max(2, int(max_time / dt))
    x = ball_x
    y = ball_y
    vx = ball_vx
    vy = ball_vy
    out: list[tuple[float, float]] = [(x, y)]

    for _ in range(steps):
        nx = x + vx * dt
        ny = y + vy * dt

        if nx < BALL_R:
            nx = 2.0 * BALL_R - nx
            vx = -vx
        elif nx > FIELD_X - BALL_R:
            nx = 2.0 * (FIELD_X - BALL_R) - nx
            vx = -vx

        if ny < BALL_R:
            ny = 2.0 * BALL_R - ny
            vy = -vy
        elif ny > FIELD_Y - BALL_R:
            ny = 2.0 * (FIELD_Y - BALL_R) - ny
            vy = -vy

        x, y = nx, ny
        out.append((x, y))

    return out


def flatten_state(field_state: dict) -> tuple[dict[str, float], dict[str, float]]:
    cam_data0 = field_state["camData"][0]
    cam_data1 = field_state["camData"][1]

    def flatten(cam_data: dict) -> dict[str, float]:
        flat: dict[str, float] = {}
        for i, rod in enumerate(cam_data["rods"], start=1):
            flat[f"rod{i}_item1"] = float(rod["Item1"])
            flat[f"rod{i}_item2"] = float(rod["Item2"])
        flat["ball_x"] = float(cam_data["ball_x"])
        flat["ball_y"] = float(cam_data["ball_y"])
        flat["ball_vx"] = float(cam_data["ball_vx"])
        flat["ball_vy"] = float(cam_data["ball_vy"])
        flat["ball_size"] = float(cam_data["ball_size"])
        return flat

    return flatten(cam_data0), flatten(cam_data1)


def merge_cam_data(cam_data0: dict[str, float], cam_data1: dict[str, float]) -> dict[str, float]:
    merged: dict[str, float] = {}
    for key in cam_data0:
        merged[key] = (float(cam_data0[key]) + float(cam_data1[key])) / 2.0
    return merged


def main() -> None:
    controller = FoosballController()
    session = requests.Session()

    count = 0
    start_time = time.perf_counter()

    try:
        while True:
            response = session.get(f"{URL}/State", timeout=1.0)
            response.raise_for_status()
            field_state = response.json()

            cam0, cam1 = flatten_state(field_state)
            merged_cam = merge_cam_data(cam0, cam1)
            commands = controller.step(merged_cam)

            payload = {"commands": commands}
            resp = session.post(f"{URL}/Motors/SendCommand", json=payload, timeout=1.0)
            resp.raise_for_status()

            controller.sleep()
            count += 1
            if count % 50 == 0:
                now = time.perf_counter()
                print("FPS:", 1.0 / ((now - start_time) / count))

    except KeyboardInterrupt:
        print("Stopped by user.")


if __name__ == "__main__":
    main()
