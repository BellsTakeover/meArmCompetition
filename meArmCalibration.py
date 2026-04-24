"""
meArmCalibration.py

Purpose:
- Each person chooses arm1, arm2, or arm3
- Each person calibrates 3 bucket-related Cartesian coordinates:
    rest -> intake -> outtake -> rest
- Then the same file can run the arm automatically and log motion timing

Important note:
- This script calibrates in x/y/z bucket positions, not raw base/elbow/wrist angles --> let's see if it works
- That matches the earlier meArm code style that uses (Zero.py): --> let's see if it was best choice
    arm.move_to(x, y, z)
    arm.get_position()

ARM roles:
- arm1 : bucket 1 -> bucket 2
- arm2 : bucket 2 -> bucket 3
- arm3 : bucket 3 -> bucket 4

Calibration controls:
- Arrow keys: move in X/Y
- W / S: move in Z
- [: decrease step size
- ]: increase step size
- 1: edit rest point
- 2: edit intake point
- 3: edit outtake point
- ENTER: save current point
- SPACE: move to the currently selected saved point
- H: move to rest point
- Q or ESC: save and quit

Files created:
- mearm_transfer_points.json : saved coordinates for all 3 arms
- mearm_move_log.csv : timing logs from calibration travel and run mode

User guide (Megan + Wayne):
1. In Thonny, set ARM_ID near the top of this file to arm1, arm2, or arm3 (what we decided on)
2. Leave MODE = "calibrate" and press Run to save the 3 positions
3. Change MODE = "run" and change CYCLES = 5, then press Run again.
4. Send to Isabella (copy and paste into an email works just make sure to ):
   - mearm_transfer_points.json
   - mearm_move_log.csv
"""

import argparse
import csv
import json
import logging
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple

import pygame
import meArm

# Basic settings used throughout the program
DEFAULT_ADDRESS = 0x6F
POINTS_FILE = "mearm_transfer_points.json"
LOG_FILE = "mearm_move_log.csv"
POSITION_TOL_MM = 4.0
MOVE_TIMEOUT_S = 4.0
MOVE_SETTLE_S = 0.25
KEY_INTERVAL_S = 0.03
WINDOW_SIZE = (780, 520)
DEFAULT_STEP_MM = 5.0
MIN_STEP_MM = 1.0
MAX_STEP_MM = 20.0

# Settings to make the script easier to run from Thonny
# These defaults are used when you press Run in Thonny without command-line arguments.
# Each teammate should usually only need to change ARM_ID.
ARM_ID = "arm1"          # change to "arm1", "arm2", or "arm3"
MODE = "calibrate"       # use "calibrate" to save points or "run" to start the automatic cycle
CYCLES = 3                # used only in run mode
ADDRESS = DEFAULT_ADDRESS
POINTS_FILE = POINTS_FILE
LOG_FILE = LOG_FILE

ARM_TASKS = {
    "arm1": "bucket 1 -> bucket 2",
    "arm2": "bucket 2 -> bucket 3",
    "arm3": "bucket 3 -> bucket 4",
}

# Starter-safe movement limits for each arm. Adjust these after testing the real setup.
ARM_WORKSPACES: Dict[str, Dict[str, float]] = {
    "arm1": {"x_min": -130.0, "x_max": -10.0, "y_min": 85.0, "y_max": 210.0, "z_min": 35.0, "z_max": 125.0},
    "arm2": {"x_min":  -55.0, "x_max":  55.0, "y_min": 85.0, "y_max": 210.0, "z_min": 35.0, "z_max": 125.0},
    "arm3": {"x_min":   10.0, "x_max": 130.0, "y_min": 85.0, "y_max": 210.0, "z_min": 35.0, "z_max": 125.0},
}

POSE_NAMES = ["rest", "intake", "outtake"]


# Small helpers for storing and checking arm positions
@dataclass
class Pose:
    x: float
    y: float
    z: float

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)



def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class TimedMeArm:
    def __init__(self, arm_id: str, address: int = DEFAULT_ADDRESS):
        if arm_id not in ARM_WORKSPACES:
            raise ValueError(f"Unknown arm_id {arm_id}. Choose from {list(ARM_WORKSPACES)}")

        self.arm_id = arm_id
        self.workspace = ARM_WORKSPACES[arm_id]
        self.logger = logging.getLogger(arm_id)
        self.logger.setLevel(logging.INFO)
        self.arm = meArm.meArm(logger=self.logger)

    def current_pose(self) -> Pose:
        x, y, z = self.arm.get_position()
        return Pose(float(x), float(y), float(z))

    def clip_pose(self, pose: Pose) -> Pose:
        return Pose(
            x=clamp(pose.x, self.workspace["x_min"], self.workspace["x_max"]),
            y=clamp(pose.y, self.workspace["y_min"], self.workspace["y_max"]),
            z=clamp(pose.z, self.workspace["z_min"], self.workspace["z_max"]),
        )

    def radial_ok(self, pose: Pose, r_min: float = 75.0, r_max: float = 225.0) -> bool:
        r = math.sqrt(pose.x**2 + pose.y**2)
        return r_min <= r <= r_max

    def validate_pose(self, pose: Pose) -> Pose:
        pose = self.clip_pose(pose)
        if not self.radial_ok(pose):
            raise ValueError(
                f"Pose {pose} failed coarse reach check. "
                f"Adjust the bucket placement or workspace for {self.arm_id}."
            )
        return pose

    def at_pose(self, target: Pose, tol_mm: float = POSITION_TOL_MM) -> bool:
        now = self.current_pose()
        return (
            abs(now.x - target.x) <= tol_mm and
            abs(now.y - target.y) <= tol_mm and
            abs(now.z - target.z) <= tol_mm
        )

    def move_to(self, target: Pose, settle_s: float = MOVE_SETTLE_S) -> Dict[str, float]:
        safe_target = self.validate_pose(target)
        start_pose = self.current_pose()
        t0 = time.time()

        self.arm.move_to(safe_target.x, safe_target.y, safe_target.z)

        reached = False
        while time.time() - t0 < MOVE_TIMEOUT_S:
            if self.at_pose(safe_target):
                reached = True
                break
            time.sleep(0.02)

        time.sleep(settle_s)
        t1 = time.time()
        end_pose = self.current_pose()

        return {
            "elapsed_s": round(t1 - t0, 4),
            "reached": int(reached),
            "start_x": round(start_pose.x, 2),
            "start_y": round(start_pose.y, 2),
            "start_z": round(start_pose.z, 2),
            "target_x": round(safe_target.x, 2),
            "target_y": round(safe_target.y, 2),
            "target_z": round(safe_target.z, 2),
            "end_x": round(end_pose.x, 2),
            "end_y": round(end_pose.y, 2),
            "end_z": round(end_pose.z, 2),
        }


# Save and load the rest, intake, and outtake positions
def default_points() -> Dict[str, Dict[str, Dict[str, float]]]:
    return {
        "arm1": {
            "rest": {"x": -55.0, "y": 120.0, "z": 85.0},
            "intake": {"x": -85.0, "y": 165.0, "z": 55.0},
            "outtake": {"x": -30.0, "y": 185.0, "z": 55.0},
        },
        "arm2": {
            "rest": {"x": 0.0, "y": 120.0, "z": 85.0},
            "intake": {"x": -10.0, "y": 165.0, "z": 55.0},
            "outtake": {"x": 10.0, "y": 185.0, "z": 55.0},
        },
        "arm3": {
            "rest": {"x": 55.0, "y": 120.0, "z": 85.0},
            "intake": {"x": 30.0, "y": 165.0, "z": 55.0},
            "outtake": {"x": 85.0, "y": 185.0, "z": 55.0},
        },
    }



def load_points(path: str = POINTS_FILE) -> Dict[str, Dict[str, Dict[str, float]]]:
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    data = default_points()
    save_points(data, path)
    return data



def save_points(data: Dict[str, Dict[str, Dict[str, float]]], path: str = POINTS_FILE) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)


# Timing log helpers
def append_move_log(row: Dict[str, object], path: str = LOG_FILE) -> None:
    file_exists = os.path.exists(path)
    fieldnames = [
        "timestamp", "arm_id", "task", "cycle", "segment",
        "elapsed_s", "reached",
        "start_x", "start_y", "start_z",
        "target_x", "target_y", "target_z",
        "end_x", "end_y", "end_z",
    ]

    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)



def log_move_event(controller: TimedMeArm, info: Dict[str, float], cycle: int, segment: str, path: str = LOG_FILE) -> None:
    row = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "arm_id": controller.arm_id,
        "task": ARM_TASKS[controller.arm_id],
        "cycle": cycle,
        "segment": segment,
        **info,
    }
    append_move_log(row, path)


# Pygame screen used for keyboard calibration --> chatgpt code
def draw_text(
    screen,
    font,
    small_font,
    arm_id: str,
    task: str,
    pose_name: str,
    edit_pose: Pose,
    current_pose: Pose,
    saved_points: Dict[str, Dict[str, float]],
    step_mm: float,
    status: str,
) -> None:
    screen.fill((245, 245, 245))

    lines = [
        f"ARM: {arm_id}   TASK: {task}",
        f"Currently editing: {pose_name.upper()}",
        f"Live commanded pose   X={edit_pose.x:6.1f}  Y={edit_pose.y:6.1f}  Z={edit_pose.z:6.1f}",
        f"Arm reported pose     X={current_pose.x:6.1f}  Y={current_pose.y:6.1f}  Z={current_pose.z:6.1f}",
        f"Step size: {step_mm:.1f} mm",
        "",
        "Controls:",
        "Arrow keys = X/Y like your earlier controller",
        "W / S = Z up / down",
        "1 = edit REST, 2 = edit INTAKE, 3 = edit OUTTAKE",
        "ENTER = save the current pose for this point",
        "SPACE = move arm to the saved pose currently selected",
        "[ = decrease step size, ] = increase step size",
        "H = move to REST pose, Q or ESC = save and quit",
        "",
        f"REST    : X={saved_points['rest']['x']:6.1f}  Y={saved_points['rest']['y']:6.1f}  Z={saved_points['rest']['z']:6.1f}",
        f"INTAKE  : X={saved_points['intake']['x']:6.1f}  Y={saved_points['intake']['y']:6.1f}  Z={saved_points['intake']['z']:6.1f}",
        f"OUTTAKE : X={saved_points['outtake']['x']:6.1f}  Y={saved_points['outtake']['y']:6.1f}  Z={saved_points['outtake']['z']:6.1f}",
        "",
        f"Status: {status}",
    ]

    y = 15
    for i, line in enumerate(lines):
        text = font.render(line, True, (0, 0, 0)) if i < 5 else small_font.render(line, True, (20, 20, 20))
        screen.blit(text, (15, y))
        y += 28 if i < 5 else 23

    pygame.display.flip()



def calibration_mode(controller: TimedMeArm, points_path: str, log_path: str) -> None:
    points = load_points(points_path)
    saved = points[controller.arm_id]

    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption(f"meArm Calibration - {controller.arm_id}")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 30)
    small_font = pygame.font.SysFont(None, 24)

    current_edit_name = "rest"
    current_pose = controller.current_pose()
    edit_pose = controller.clip_pose(current_pose)
    step_mm = DEFAULT_STEP_MM
    status = "Use the keys to move the arm, then ENTER to save each point."
    last_key_time = 0.0
    running = True

    while running:
        now = time.time()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_1:
                    current_edit_name = "rest"
                    edit_pose = Pose(**saved["rest"])
                    status = "Editing REST"
                elif event.key == pygame.K_2:
                    current_edit_name = "intake"
                    edit_pose = Pose(**saved["intake"])
                    status = "Editing INTAKE"
                elif event.key == pygame.K_3:
                    current_edit_name = "outtake"
                    edit_pose = Pose(**saved["outtake"])
                    status = "Editing OUTTAKE"
                elif event.key == pygame.K_RETURN:
                    safe_pose = controller.validate_pose(edit_pose)
                    saved[current_edit_name] = asdict(safe_pose)
                    save_points(points, points_path)
                    status = f"Saved {current_edit_name.upper()}"
                elif event.key == pygame.K_SPACE:
                    safe_pose = controller.validate_pose(Pose(**saved[current_edit_name]))
                    info = controller.move_to(safe_pose)
                    log_move_event(controller, info, cycle=0, segment=f"calibration_to_{current_edit_name}", path=log_path)
                    status = (
                        f"Moved to saved {current_edit_name.upper()} | "
                        f"time={info['elapsed_s']:.3f}s reached={int(info['reached'])}"
                    )
                    edit_pose = safe_pose
                elif event.key == pygame.K_h:
                    safe_pose = controller.validate_pose(Pose(**saved["rest"]))
                    info = controller.move_to(safe_pose)
                    log_move_event(controller, info, cycle=0, segment="calibration_home_rest", path=log_path)
                    status = f"Moved HOME/REST | time={info['elapsed_s']:.3f}s reached={int(info['reached'])}"
                    edit_pose = safe_pose
                elif event.key == pygame.K_LEFTBRACKET:
                    step_mm = max(MIN_STEP_MM, step_mm - 1.0)
                    status = f"Step size = {step_mm:.1f} mm"
                elif event.key == pygame.K_RIGHTBRACKET:
                    step_mm = min(MAX_STEP_MM, step_mm + 1.0)
                    status = f"Step size = {step_mm:.1f} mm"

        if now - last_key_time >= KEY_INTERVAL_S:
            keys = pygame.key.get_pressed()
            moved = False
            new_pose = Pose(edit_pose.x, edit_pose.y, edit_pose.z)

            # Keep the same control style as the earlier version.
            # Arrow keys move X/Y, and W/S moves Z.
            if keys[pygame.K_UP]:
                new_pose.y += step_mm
                moved = True
            elif keys[pygame.K_DOWN]:
                new_pose.y -= step_mm
                moved = True
            elif keys[pygame.K_RIGHT]:
                new_pose.x += step_mm
                moved = True
            elif keys[pygame.K_LEFT]:
                new_pose.x -= step_mm
                moved = True
            elif keys[pygame.K_w]:
                new_pose.z += step_mm
                moved = True
            elif keys[pygame.K_s]:
                new_pose.z -= step_mm
                moved = True

            if moved:
                try:
                    new_pose = controller.validate_pose(new_pose)
                    controller.move_to(new_pose, settle_s=0.02)
                    edit_pose = new_pose
                    status = f"Moved {current_edit_name.upper()} preview"
                except Exception as exc:
                    status = f"Blocked move: {exc}"
                last_key_time = now

        current_pose = controller.current_pose()
        draw_text(
            screen,
            font,
            small_font,
            controller.arm_id,
            ARM_TASKS[controller.arm_id],
            current_edit_name,
            edit_pose,
            current_pose,
            saved,
            step_mm,
            status,
        )
        clock.tick(60)

    save_points(points, points_path)
    pygame.quit()


# Automatic movement loop after the points have been calibrated
def run_mode(controller: TimedMeArm, points_path: str, cycles: int, log_path: str) -> None:
    points = load_points(points_path)
    arm_points = points[controller.arm_id]

    rest = Pose(**arm_points["rest"])
    intake = Pose(**arm_points["intake"])
    outtake = Pose(**arm_points["outtake"])

    sequence: List[Tuple[str, Pose]] = [
        ("rest_to_intake", intake),
        ("intake_to_outtake", outtake),
        ("outtake_to_rest", rest),
    ]

    print(f"\nRunning {controller.arm_id} | task: {ARM_TASKS[controller.arm_id]}")
    print(f"Using points file: {points_path}")
    print(f"Cycles: {cycles}\n")

    home_info = controller.move_to(rest)
    log_move_event(controller, home_info, cycle=0, segment="startup_to_rest", path=log_path)
    print(f"Moved to REST first: {home_info['elapsed_s']:.3f}s")

    summary: Dict[str, List[float]] = {name: [] for name, _ in sequence}

    for cycle in range(1, cycles + 1):
        print(f"Cycle {cycle}/{cycles}")
        for segment_name, target_pose in sequence:
            info = controller.move_to(target_pose)
            summary[segment_name].append(float(info["elapsed_s"]))
            log_move_event(controller, info, cycle=cycle, segment=segment_name, path=log_path)

            print(
                f"  {segment_name:18s}  "
                f"time={info['elapsed_s']:.3f}s  reached={int(info['reached'])}  "
                f"end=({info['end_x']:.1f}, {info['end_y']:.1f}, {info['end_z']:.1f})"
            )

    print("\nTiming summary")
    for segment_name, values in summary.items():
        avg = sum(values) / len(values) if values else 0.0
        fastest = min(values) if values else 0.0
        slowest = max(values) if values else 0.0
        print(f"  {segment_name:18s} avg={avg:.3f}s  min={fastest:.3f}s  max={slowest:.3f}s")

    print(f"\nSaved timing log to {log_path}\n")


# Command-line setup
def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%H:%M:%S",
    )



def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Guided 3-arm meArm calibration + automation")
    parser.add_argument(
        "--arm-id",
        default=ARM_ID,
        choices=["arm1", "arm2", "arm3"],
        help="Which arm this Pi is controlling",
    )
    parser.add_argument(
        "--mode",
        default="calibrate",
        choices=["calibrate", "run"],
        help="calibrate = keyboard setup, run = automatic timing run",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=3,
        help="How many repeat cycles to run in automatic mode",
    )
    parser.add_argument(
        "--address",
        type=lambda x: int(x, 0),
        default=DEFAULT_ADDRESS,
        help="I2C address of the meArm controller, e.g. 0x6F",
    )
    parser.add_argument(
        "--points-file",
        default=POINTS_FILE,
        help="JSON file for saved rest/intake/outtake points",
    )
    parser.add_argument(
        "--log-file",
        default=LOG_FILE,
        help="CSV file for timing logs",
    )
    return parser.parse_args()



def main() -> int:
    setup_logging()
    args = parse_cli_args()

    controller = TimedMeArm(arm_id=args.arm_id, address=args.address)
    stop_requested = False

    def _handle_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True
        try:
            points = load_points(args.points_file)
            rest = Pose(**points[args.arm_id]["rest"])
            controller.move_to(rest)
        except Exception:
            pass
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    if args.mode == "calibrate":
        calibration_mode(controller, args.points_file, args.log_file)
    elif args.mode == "run":
        run_mode(controller, args.points_file, args.cycles, args.log_file)

    if not stop_requested:
        print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
