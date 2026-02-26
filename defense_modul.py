# defense_modul.py
import json

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class Defense:
    def __init__(
        self,
        geometry_path: str = "geometry.json",
        rod_keeper: int = 0, drive_keeper: int = 1,
        rod_def1: int = 1, drive_def1: int = 2,
        rod_def2: int = 3, drive_def2: int = 3,
    ):
        with open(geometry_path, "r", encoding="utf-8") as f:
            self.geometry = json.load(f)

        self.rod_keeper = rod_keeper
        self.rod_def1 = rod_def1
        self.rod_def2 = rod_def2

        self.drive_keeper = drive_keeper
        self.drive_def1 = drive_def1
        self.drive_def2 = drive_def2

        # from geometry.json (mm)
        self.field_y = float(self.geometry["field"]["dimension_y"])
        self.goal_w = float(self.geometry["goal_width"])
        self.ball_len = float(self.geometry.get("ball_size", 34.0))

        self.goal_top_y = (self.field_y - self.goal_w) / 2.0
        self.goal_bot_y = (self.field_y + self.goal_w) / 2.0
        self.goal_mid_y = self.field_y / 2.0

        # keep a tiny pad so we don't slam into extremes
        self.corner_pad = 0.3 * self.ball_len

        # output command constants (you can override in main)
        self.ROT_NORMAL = 0.0
        self.ROT_VEL = 0.6
        self.TRANS_VEL = 1.0

    def rod_translation_for_target_y(self, rod_index: int, target_y: float):
        """
        Find translation cal (0..1) that puts one of the players on that rod closest to target_y.
        Returns (cal, player_y, error)
        """
        rg = self.geometry["rods"][rod_index]
        travel = float(rg["travel"])
        first = float(rg["first_offset"])
        spacing = float(rg["spacing"])
        npl = int(rg["players"])

        best_cal = 0.5
        best_py = first + 0.5 * travel
        best_err = float("inf")

        for ip in range(npl):
            pos_needed = target_y - first - ip * spacing
            cal = pos_needed / travel if travel > 1e-9 else 0.5
            cal = clamp(cal, 0.0, 1.0)

            py = first + cal * travel + ip * spacing
            err = abs(py - target_y)
            if err < best_err:
                best_err = err
                best_cal = cal
                best_py = py

        return best_cal, best_py, best_err

    def targets_y(self, ball_y: float):
        """
        Pure logic targets in mm:
          - upper: keeper -> upper corner, def1 -> +ball_len toward middle
          - lower: keeper -> lower corner, def1 -> -ball_len toward middle
          - def2 -> goal mid (super simple cover)
        """
        upper_corner = self.goal_top_y + self.corner_pad
        lower_corner = self.goal_bot_y - self.corner_pad

        if ball_y < self.goal_mid_y:  # upper half (smaller y)
            keeper_y = upper_corner
            def1_y = keeper_y + self.ball_len
        else:                         # lower half
            keeper_y = lower_corner
            def1_y = keeper_y - self.ball_len

        # option A: keep 2nd defender in the middle of goal
        def2_y = self.goal_mid_y

        #  option B (if you prefer): uncomment to make def2 follow ball directly
        # def2_y = ball_y

        # keep inside field
        keeper_y = clamp(keeper_y, 0.0, self.field_y)
        def1_y = clamp(def1_y, 0.0, self.field_y)
        def2_y = clamp(def2_y, 0.0, self.field_y)

        return keeper_y, def1_y, def2_y

    def step(self, ball_y: float, ball_visible: bool = True):
        """
        Returns motor commands list for drive 1/2/3.
        """
        if not ball_visible:
            return [
                {"driveID": self.drive_keeper, "rotationTargetPosition": self.ROT_NORMAL, "rotationVelocity": self.ROT_VEL,
                 "translationTargetPosition": 0.5, "translationVelocity": self.TRANS_VEL},
                {"driveID": self.drive_def1, "rotationTargetPosition": self.ROT_NORMAL, "rotationVelocity": self.ROT_VEL,
                 "translationTargetPosition": 0.5, "translationVelocity": self.TRANS_VEL},
                {"driveID": self.drive_def2, "rotationTargetPosition": self.ROT_NORMAL, "rotationVelocity": self.ROT_VEL,
                 "translationTargetPosition": 0.5, "translationVelocity": self.TRANS_VEL},
            ]

        ky, d1y, d2y = self.targets_y(ball_y)

        k_cal, _, _ = self.rod_translation_for_target_y(self.rod_keeper, ky)
        d1_cal, _, _ = self.rod_translation_for_target_y(self.rod_def1, d1y)
        d2_cal, _, _ = self.rod_translation_for_target_y(self.rod_def2, d2y)

        return [
            {"driveID": self.drive_keeper, "rotationTargetPosition": self.ROT_NORMAL, "rotationVelocity": self.ROT_VEL,
             "translationTargetPosition": float(k_cal), "translationVelocity": self.TRANS_VEL},
            {"driveID": self.drive_def1, "rotationTargetPosition": self.ROT_NORMAL, "rotationVelocity": self.ROT_VEL,
             "translationTargetPosition": float(d1_cal), "translationVelocity": self.TRANS_VEL},
            {"driveID": self.drive_def2, "rotationTargetPosition": self.ROT_NORMAL, "rotationVelocity": self.ROT_VEL,
             "translationTargetPosition": float(d2_cal), "translationVelocity": self.TRANS_VEL},
        ]