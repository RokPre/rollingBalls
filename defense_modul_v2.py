import json
from typing import Optional


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class Defense:
    """
    Defensive controller for the red team rods:
      - keeper  : rod 1 -> drive 1
      - defense : rod 2 -> drive 2
      - midfield screen: rod 4 -> drive 3

    Main ideas:
      1. Predict the ball lane with wall bounces.
      2. Put each defensive rod on the predicted interception line.
      3. Switch between three modes:
         - home      : ball not visible / out of field
         - screen    : ball moving away or no direct threat yet
         - intercept : ball moving toward our goal
         - panic     : ball already deep in our half
      4. Keep motion stable by preferring commands close to the previous one.
    """

    def __init__(
        self,
        geometry_path: str = "geometry.json",
        rod_keeper: int = 0,
        drive_keeper: int = 1,
        rod_def1: int = 1,
        drive_def1: int = 2,
        rod_def2: int = 3,
        drive_def2: int = 3,
    ):
        with open(geometry_path, "r", encoding="utf-8") as f:
            self.geometry = json.load(f)

        self.rod_keeper = rod_keeper
        self.rod_def1 = rod_def1
        self.rod_def2 = rod_def2

        self.drive_keeper = drive_keeper
        self.drive_def1 = drive_def1
        self.drive_def2 = drive_def2

        self.field_x = float(self.geometry["field"]["dimension_x"])
        self.field_y = float(self.geometry["field"]["dimension_y"])
        self.goal_w = float(self.geometry["goal_width"])
        self.ball_size = float(self.geometry.get("ball_size", 34.0))

        self.goal_top_y = 0.5 * (self.field_y - self.goal_w)
        self.goal_bot_y = 0.5 * (self.field_y + self.goal_w)
        self.goal_mid_y = 0.5 * self.field_y

        self.keeper_x = float(self.geometry["rods"][self.rod_keeper]["position"])
        self.def1_x = float(self.geometry["rods"][self.rod_def1]["position"])
        self.def2_x = float(self.geometry["rods"][self.rod_def2]["position"])

        # Where we want the keeper to live: inside the goal mouth, not on hard extremes.
        self.goal_guard_margin = 0.55 * self.ball_size
        self.goal_guard_top = self.goal_top_y + self.goal_guard_margin
        self.goal_guard_bot = self.goal_bot_y - self.goal_guard_margin

        # Small offsets so rods do not perfectly stack on the exact same line.
        self.layer_sep = 0.90 * self.ball_size
        self.deep_layer_sep = 1.20 * self.ball_size

        # Motion / selection tuning.
        self.TRANS_VEL = 1.0
        self.ROT_VEL = 0.8
        self.ROT_NORMAL = 0.0
        self.selection_switch_penalty = 0.20 * self.ball_size

        # Heuristic thresholds in mm/s and mm.
        self.toward_goal_vx = -25.0
        self.fast_shot_vx = -140.0
        self.slow_ball_v = 25.0
        self.panic_x = self.def1_x + 2.0 * self.ball_size
        self.close_keeper_x = self.keeper_x + 2.5 * self.ball_size

        # Stateful memory for stability and debugging.
        self.last_trans = {
            self.rod_keeper: 0.5,
            self.rod_def1: 0.5,
            self.rod_def2: 0.5,
        }
        self.last_mode = "home"

    # ---------- geometry helpers ----------

    def rod_geom(self, rod_index: int) -> dict:
        return self.geometry["rods"][rod_index]

    def rod_home_y(self, rod_index: int) -> float:
        rg = self.rod_geom(rod_index)
        travel = float(rg["travel"])
        first = float(rg["first_offset"])
        spacing = float(rg["spacing"])
        npl = int(rg["players"])
        centers = [first + 0.5 * travel + i * spacing for i in range(npl)]
        best = min(centers, key=lambda y: abs(y - self.goal_mid_y))
        return float(best)

    def clamp_field_y(self, y: float) -> float:
        return clamp(y, 0.0, self.field_y)

    def clamp_goal_y(self, y: float) -> float:
        return clamp(y, self.goal_guard_top, self.goal_guard_bot)

    def best_translation_for_target_y(self, rod_index: int, target_y: float):
        """
        Pick the best player on a rod for the target y.
        We minimize:
            |player_y - target_y| + penalty_for_large_jump_from_previous_command
        """
        rg = self.rod_geom(rod_index)
        travel = float(rg["travel"])
        first = float(rg["first_offset"])
        spacing = float(rg["spacing"])
        npl = int(rg["players"])

        prev = self.last_trans[rod_index]
        best_cost = float("inf")
        best_cal = prev
        best_py = first + travel * prev
        best_err = abs(best_py - target_y)
        best_player = 0

        for ip in range(npl):
            if travel <= 1e-9:
                cal = 0.5
            else:
                cal = (target_y - first - ip * spacing) / travel
            cal = clamp(cal, 0.0, 1.0)

            py = first + cal * travel + ip * spacing
            err = abs(py - target_y)
            switch_cost = self.selection_switch_penalty * abs(cal - prev)
            cost = err + switch_cost

            if cost < best_cost:
                best_cost = cost
                best_cal = cal
                best_py = py
                best_err = err
                best_player = ip

        return best_cal, best_py, best_err, best_player

    # ---------- ball prediction ----------

    def ball_in_field(self, ball_x: float, ball_y: float) -> bool:
        return 0.0 <= ball_x <= self.field_x and 0.0 <= ball_y <= self.field_y

    def reflect_y(self, y_unfolded: float) -> float:
        """Mirror a y coordinate into [0, field_y] with perfect wall bounces."""
        period = 2.0 * self.field_y
        y_mod = y_unfolded % period
        if y_mod <= self.field_y:
            return y_mod
        return period - y_mod

    def predict_y_at_x(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float, target_x: float) -> Optional[float]:
        if abs(ball_vx) < 1e-9:
            return None
        dt = (target_x - ball_x) / ball_vx
        if dt < 0.0:
            return None
        y_unfolded = ball_y + ball_vy * dt
        return self.reflect_y(y_unfolded)

    def speed(self, vx: float, vy: float) -> float:
        return (vx * vx + vy * vy) ** 0.5

    # ---------- tactical modes ----------

    def home_targets(self):
        return (
            self.clamp_goal_y(self.goal_mid_y),
            self.rod_home_y(self.rod_def1),
            self.rod_home_y(self.rod_def2),
        )

    def screen_targets(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float):
        """
        Soft zonal defense.
        Used when there is no immediate shot line toward our goal.
        """
        lane_y = self.clamp_field_y(ball_y + 0.10 * ball_vy)
        lane_mix = 0.72 * lane_y + 0.28 * self.goal_mid_y

        # Keeper stays inside goal mouth and does not overreact to far-away ball.
        keeper_y = self.clamp_goal_y(lane_mix)

        # Two-man defense rod shadows the main lane.
        def1_y = self.clamp_field_y(0.85 * lane_y + 0.15 * self.goal_mid_y)

        # Midfield screen follows the current ball lane more aggressively.
        # It is the earliest blocker, so keep it active.
        if ball_x > self.def2_x:
            def2_y = lane_y
        else:
            # Once the ball is already behind this rod, keep it closer to center as a fallback lane blocker.
            def2_y = 0.55 * lane_y + 0.45 * self.goal_mid_y
        def2_y = self.clamp_field_y(def2_y)

        return keeper_y, def1_y, def2_y

    def intercept_targets(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float):
        """
        Put each rod on the predicted shot line.
        Keeper stays slightly center-biased when the shot is still far away,
        but becomes exact when the threat is close.
        """
        yk = self.predict_y_at_x(ball_x, ball_y, ball_vx, ball_vy, self.keeper_x)
        yd1 = self.predict_y_at_x(ball_x, ball_y, ball_vx, ball_vy, self.def1_x)
        yd2 = self.predict_y_at_x(ball_x, ball_y, ball_vx, ball_vy, self.def2_x)

        if yd1 is None:
            yd1 = ball_y
        if yd2 is None:
            yd2 = ball_y
        if yk is None:
            yk = ball_y

        # Ball still far away -> keeper stays a bit more central.
        dist_to_keeper = max(ball_x - self.keeper_x, 0.0)
        center_bias = clamp((dist_to_keeper - 120.0) / 650.0, 0.0, 0.35)
        keeper_raw = (1.0 - center_bias) * yk + center_bias * self.goal_mid_y
        keeper_y = self.clamp_goal_y(keeper_raw)

        # Rod 2 blocks exact line in our half.
        def1_y = self.clamp_field_y(yd1)

        # Rod 4 blocks earlier. If the ball is already behind it, keep a backup lane instead.
        if ball_x >= self.def2_x - 0.5 * self.ball_size:
            def2_y = self.clamp_field_y(yd2)
        else:
            def2_y = self.clamp_field_y(0.65 * yd1 + 0.35 * self.goal_mid_y)

        return keeper_y, def1_y, def2_y

    def panic_targets(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float):
        """
        Ball is already deep in our half.
        Collapse the defense around the live ball / immediate lane.
        """
        short_lead_y = self.clamp_field_y(ball_y + 0.04 * ball_vy)
        keeper_y = self.clamp_goal_y(short_lead_y)

        # Rod 2 tracks almost the same line, but shifted slightly toward the open side.
        sign = -1.0 if short_lead_y < self.goal_mid_y else 1.0
        def1_y = self.clamp_field_y(short_lead_y + sign * self.layer_sep)

        # Rod 4 stays as secondary screen near the same lane, but less aggressively.
        def2_y = self.clamp_field_y(short_lead_y + sign * self.deep_layer_sep)

        return keeper_y, def1_y, def2_y

    def choose_mode(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy: float, ball_visible: bool) -> str:
        if (not ball_visible) or (not self.ball_in_field(ball_x, ball_y)):
            return "home"

        speed = self.speed(ball_vx, ball_vy)

        # Deep danger or slow scramble in our half.
        if ball_x <= self.panic_x and (ball_vx < 60.0 or speed <= self.slow_ball_v):
            return "panic"

        # Ball really close to keeper area.
        if ball_x <= self.close_keeper_x:
            return "panic"

        # Clear incoming shot toward our goal.
        if ball_vx <= self.toward_goal_vx:
            return "intercept"

        return "screen"

    # ---------- public API ----------

    def step(
        self,
        ball_x: float,
        ball_y: float,
        ball_vx: float = 0.0,
        ball_vy: float = 0.0,
        ball_visible: bool = True,
    ):
        mode = self.choose_mode(ball_x, ball_y, ball_vx, ball_vy, ball_visible)
        self.last_mode = mode

        if mode == "home":
            ky, d1y, d2y = self.home_targets()
        elif mode == "panic":
            ky, d1y, d2y = self.panic_targets(ball_x, ball_y, ball_vx, ball_vy)
        elif mode == "intercept":
            ky, d1y, d2y = self.intercept_targets(ball_x, ball_y, ball_vx, ball_vy)
        else:
            ky, d1y, d2y = self.screen_targets(ball_x, ball_y, ball_vx, ball_vy)

        k_cal, k_py, k_err, _ = self.best_translation_for_target_y(self.rod_keeper, ky)
        d1_cal, d1_py, d1_err, _ = self.best_translation_for_target_y(self.rod_def1, d1y)
        d2_cal, d2_py, d2_err, _ = self.best_translation_for_target_y(self.rod_def2, d2y)

        self.last_trans[self.rod_keeper] = float(k_cal)
        self.last_trans[self.rod_def1] = float(d1_cal)
        self.last_trans[self.rod_def2] = float(d2_cal)

        # Defensive rotation stays neutral. This is the safest default.
        rot_target = self.ROT_NORMAL
        rot_vel = self.ROT_VEL

        return [
            {
                "driveID": self.drive_keeper,
                "rotationTargetPosition": rot_target,
                "rotationVelocity": rot_vel,
                "translationTargetPosition": float(k_cal),
                "translationVelocity": self.TRANS_VEL,
            },
            {
                "driveID": self.drive_def1,
                "rotationTargetPosition": rot_target,
                "rotationVelocity": rot_vel,
                "translationTargetPosition": float(d1_cal),
                "translationVelocity": self.TRANS_VEL,
            },
            {
                "driveID": self.drive_def2,
                "rotationTargetPosition": rot_target,
                "rotationVelocity": rot_vel,
                "translationTargetPosition": float(d2_cal),
                "translationVelocity": self.TRANS_VEL,
            },
        ]

    def debug_state(self, ball_x: float, ball_y: float, ball_vx: float = 0.0, ball_vy: float = 0.0, ball_visible: bool = True) -> dict:
        """Useful while tuning."""
        mode = self.choose_mode(ball_x, ball_y, ball_vx, ball_vy, ball_visible)
        return {
            "mode": mode,
            "pred_keeper_y": self.predict_y_at_x(ball_x, ball_y, ball_vx, ball_vy, self.keeper_x),
            "pred_def1_y": self.predict_y_at_x(ball_x, ball_y, ball_vx, ball_vy, self.def1_x),
            "pred_def2_y": self.predict_y_at_x(ball_x, ball_y, ball_vx, ball_vy, self.def2_x),
            "last_trans": dict(self.last_trans),
        }
