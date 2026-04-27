"""
meArmCalibration.py

Purpose:
- Each person chooses arm1, arm2, or arm3
- Each person calibrates 3 bucket-related Cartesian coordinates:
    rest -> intake -> outtake -> rest
- Then the same file can run the arm automatically and log motion timing

Important note:
- This script calibrates in x/y/z bucket positions, not raw base/elbow/wrist angles --> let's see if it 
    -- Didn;t work so moved to raw base/elbow/wrist angles
- That matches the earlier meArm code style that uses (Zero.py): --> let's see if it was best choice
    arm.move_to(x, y, z)
    arm.get_position()

ARM roles:
- arm1 : bucket 1 -> bucket 2
- arm2 : bucket 2 -> bucket 3
- arm3 : bucket 3 -> bucket 4

Calibration controls:
- Up / Down: base + / -
- Right / Left : shoulder + / -
- A / S: elbow + / -
- [ / ]: decrease / increase step size
- 1 / 2 / 3: select REST / INTAKE / OUTTAKE
- Enter save current angles to selected point
- Space: move to selected saved point and log timing
- H : move to REST point and log timing
- T : run timing test REST -> INTAKE -> OUTTAKE -> REST
- Q or ESC : save and quit

Files created:
- mearm_transfer_points.json : saved coordinates for all 3 arms
- mearm_move_log.csv : timing logs from calibration travel and run mode

User guide (Megan + Wayne):
1. In Thonny, set ARM_ID near the top of this file to arm1, arm2, or arm3 (what we decided on)
2. Push RUN and the GUI that chatgpt created should run you through the processes
3. After you finish calibrating press 'T' to run the time check
4. Send to Isabella (copy and paste into an email works just make sure to ):
   - mearm_joint_points.json
   - mearm_joint_move_log.csv
"""

import csv
import json
import os
import time
from dataclasses import dataclass, asdict
from typing import Dict, List

import board
import pygame
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# -----------------------------
# Thonny-friendly settings
# -----------------------------
ARM_ID = "arm1"   # change to: arm1, arm2, arm3
PWM_ADDRESS = 0x60
POINTS_FILE = "mearm_joint_points.json"
LOG_FILE = "mearm_joint_move_log.csv"
WINDOW_SIZE = (780, 540)
DEFAULT_STEP_DEG = 2.0
MIN_STEP_DEG = 0.5
MAX_STEP_DEG = 10.0
KEY_INTERVAL_S = 0.06
MOVE_SETTLE_S = 0.18
PREVIEW_SETTLE_S = 0.04
TIMING_TEST_CYCLES = 3 # change if I want more timing cycle

# Servo channels used by the meArm files shared in this conversation.
CHANNELS = {
    "base": 0,
    "shoulder": 1,
    "elbow": 14,
}

# These limits are intentionally conservative and can be edited later.
JOINT_LIMITS = {
    "base": {"min": 10.0, "max": 170.0},
    "shoulder": {"min": 20.0, "max": 160.0},
    "elbow": {"min": 10.0, "max": 170.0},
}

POSE_NAMES = ["rest", "intake", "outtake"]


@dataclass
class JointPose:
    base: float
    shoulder: float
    elbow: float


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class JointCalibrator:
    def __init__(self, pwm_address: int = PWM_ADDRESS):
        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c, address=pwm_address, reference_clock_speed=25_000_000)
        self.pca.frequency = 50
        self.servos = {
            name: servo.Servo(self.pca.channels[ch], min_pulse=500, max_pulse=2500)
            for name, ch in CHANNELS.items()
        }
        self.last_pose: JointPose | None = None

    def release(self) -> None:
        try:
            self.pca.deinit()
        except Exception:
            pass

    def apply_pose(self, pose: JointPose, settle_s: float = MOVE_SETTLE_S) -> float:
        clipped = clip_pose(pose)
        t0 = time.perf_counter()
        self.servos["base"].angle = clipped.base
        self.servos["shoulder"].angle = clipped.shoulder
        self.servos["elbow"].angle = clipped.elbow
        time.sleep(settle_s)
        self.last_pose = clipped
        return time.perf_counter() - t0



def default_points() -> Dict[str, Dict[str, Dict[str, float]]]:
    return {
        "arm1": {
            "rest": {"base": 90.0, "shoulder": 95.0, "elbow": 90.0},
            "intake": {"base": 75.0, "shoulder": 110.0, "elbow": 95.0},
            "outtake": {"base": 105.0, "shoulder": 110.0, "elbow": 95.0},
        },
        "arm2": {
            "rest": {"base": 90.0, "shoulder": 95.0, "elbow": 90.0},
            "intake": {"base": 82.0, "shoulder": 110.0, "elbow": 95.0},
            "outtake": {"base": 98.0, "shoulder": 110.0, "elbow": 95.0},
        },
        "arm3": {
            "rest": {"base": 90.0, "shoulder": 95.0, "elbow": 90.0},
            "intake": {"base": 105.0, "shoulder": 110.0, "elbow": 95.0},
            "outtake": {"base": 75.0, "shoulder": 110.0, "elbow": 95.0},
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



def as_pose(d: Dict[str, float]) -> JointPose:
    return JointPose(base=float(d["base"]), shoulder=float(d["shoulder"]), elbow=float(d["elbow"]))



def clip_pose(pose: JointPose) -> JointPose:
    return JointPose(
        base=clamp(pose.base, JOINT_LIMITS["base"]["min"], JOINT_LIMITS["base"]["max"]),
        shoulder=clamp(pose.shoulder, JOINT_LIMITS["shoulder"]["min"], JOINT_LIMITS["shoulder"]["max"]),
        elbow=clamp(pose.elbow, JOINT_LIMITS["elbow"]["min"], JOINT_LIMITS["elbow"]["max"]),
    )



def append_move_log(row: Dict[str, object], path: str = LOG_FILE) -> None:
    file_exists = os.path.exists(path)
    fieldnames = [
        "timestamp",
        "arm_id",
        "event",
        "cycle",
        "segment",
        "elapsed_s",
        "start_base",
        "start_shoulder",
        "start_elbow",
        "end_base",
        "end_shoulder",
        "end_elbow",
    ]

    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)



def log_joint_move(event: str, cycle: int, segment: str, start_pose: JointPose, end_pose: JointPose, elapsed_s: float,
                   path: str = LOG_FILE) -> None:
    append_move_log(
        {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "arm_id": ARM_ID,
            "event": event,
            "cycle": cycle,
            "segment": segment,
            "elapsed_s": round(elapsed_s, 4),
            "start_base": round(start_pose.base, 2),
            "start_shoulder": round(start_pose.shoulder, 2),
            "start_elbow": round(start_pose.elbow, 2),
            "end_base": round(end_pose.base, 2),
            "end_shoulder": round(end_pose.shoulder, 2),
            "end_elbow": round(end_pose.elbow, 2),
        },
        path,
    )



def draw_text(screen, font, small_font, selected_name, edit_pose, saved, step_deg, status, timing_lines):
    screen.fill((245, 245, 245))
    lines = [
        f"ARM: {ARM_ID}",
        f"Currently editing: {selected_name.upper()}",
        f"Live pose        base={edit_pose.base:6.1f}  shoulder={edit_pose.shoulder:6.1f}  elbow={edit_pose.elbow:6.1f}",
        f"Step size: {step_deg:.1f} degrees",
        "",
        "Controls:",
        "Up / Down = base + / -",
        "Right / Left = shoulder + / -",
        "A / S = elbow + / -",
        "1 = REST, 2 = INTAKE, 3 = OUTTAKE",
        "Enter = save current angles",
        "Space = move to selected saved point",
        "H = move to REST, T = timing test, [ / ] = step size, Q or ESC = save and quit",
        "",
        f"REST    : base={saved['rest']['base']:6.1f}  shoulder={saved['rest']['shoulder']:6.1f}  elbow={saved['rest']['elbow']:6.1f}",
        f"INTAKE  : base={saved['intake']['base']:6.1f}  shoulder={saved['intake']['shoulder']:6.1f}  elbow={saved['intake']['elbow']:6.1f}",
        f"OUTTAKE : base={saved['outtake']['base']:6.1f}  shoulder={saved['outtake']['shoulder']:6.1f}  elbow={saved['outtake']['elbow']:6.1f}",
        "",
        f"Status: {status}",
        "",
        "Recent timing:",
    ]
    lines.extend(timing_lines)

    y = 15
    for i, line in enumerate(lines):
        text = font.render(line, True, (0, 0, 0)) if i < 4 else small_font.render(line, True, (20, 20, 20))
        screen.blit(text, (15, y))
        y += 30 if i < 4 else 24
    pygame.display.flip()



def format_timing_lines(summary: Dict[str, List[float]]) -> List[str]:
    if not summary:
        return ["No timing runs yet. Press SPACE, H, or T to log motion times."]

    lines: List[str] = []
    for segment, values in summary.items():
        if not values:
            continue
        avg = sum(values) / len(values)
        lines.append(
            f"{segment:<18} avg={avg:.3f}s  min={min(values):.3f}s  max={max(values):.3f}s  n={len(values)}"
        )
    return lines or ["No timing runs yet. Press SPACE, H, or T to log motion times."]



def run_timing_sequence(driver: JointCalibrator, saved: Dict[str, Dict[str, float]], summary: Dict[str, List[float]],
                        cycles: int = TIMING_TEST_CYCLES) -> str:
    rest = clip_pose(as_pose(saved["rest"]))
    intake = clip_pose(as_pose(saved["intake"]))
    outtake = clip_pose(as_pose(saved["outtake"]))
    sequence = [
        ("rest_to_intake", intake),
        ("intake_to_outtake", outtake),
        ("outtake_to_rest", rest),
    ]

    # Start from rest once.
    start_pose = driver.last_pose or rest
    elapsed = driver.apply_pose(rest)
    log_joint_move("timing_start", 0, "startup_to_rest", start_pose, rest, elapsed)

    for cycle in range(1, cycles + 1):
        current_start = rest
        for segment_name, target in sequence:
            elapsed = driver.apply_pose(target)
            summary.setdefault(segment_name, []).append(elapsed)
            log_joint_move("timing_test", cycle, segment_name, current_start, target, elapsed)
            current_start = target

    last_avg_parts = []
    for segment_name, values in summary.items():
        if values:
            last_avg_parts.append(f"{segment_name} avg={sum(values)/len(values):.3f}s")
    return "Timing test done | " + " | ".join(last_avg_parts[:3])



def main():
    if ARM_ID not in {"arm1", "arm2", "arm3"}:
        raise ValueError("ARM_ID must be arm1, arm2, or arm3")

    points = load_points(POINTS_FILE)
    saved = points[ARM_ID]
    selected_name = "rest"
    edit_pose = clip_pose(as_pose(saved[selected_name]))
    step_deg = DEFAULT_STEP_DEG
    status = "Use direct joint control to set REST / INTAKE / OUTTAKE."
    last_key_time = 0.0
    timing_summary: Dict[str, List[float]] = {
        "move_to_selected": [],
        "move_to_rest": [],
        "rest_to_intake": [],
        "intake_to_outtake": [],
        "outtake_to_rest": [],
    }

    driver = JointCalibrator(pwm_address=PWM_ADDRESS)
    initial_elapsed = driver.apply_pose(edit_pose)
    log_joint_move("startup", 0, "startup_pose", edit_pose, edit_pose, initial_elapsed)

    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption(f"meArm Joint Calibration - {ARM_ID}")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 30)
    small_font = pygame.font.SysFont(None, 24)

    running = True
    try:
        while running:
            now = time.time()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_1:
                        selected_name = "rest"
                        edit_pose = clip_pose(as_pose(saved[selected_name]))
                        status = "Editing REST"
                    elif event.key == pygame.K_2:
                        selected_name = "intake"
                        edit_pose = clip_pose(as_pose(saved[selected_name]))
                        status = "Editing INTAKE"
                    elif event.key == pygame.K_3:
                        selected_name = "outtake"
                        edit_pose = clip_pose(as_pose(saved[selected_name]))
                        status = "Editing OUTTAKE"
                    elif event.key == pygame.K_RETURN:
                        saved[selected_name] = asdict(edit_pose)
                        save_points(points, POINTS_FILE)
                        status = f"Saved {selected_name.upper()}"
                    elif event.key == pygame.K_SPACE:
                        start_pose = driver.last_pose or edit_pose
                        edit_pose = clip_pose(as_pose(saved[selected_name]))
                        elapsed = driver.apply_pose(edit_pose)
                        timing_summary["move_to_selected"].append(elapsed)
                        log_joint_move("saved_move", 0, f"to_{selected_name}", start_pose, edit_pose, elapsed)
                        status = f"Moved to saved {selected_name.upper()} | time={elapsed:.3f}s"
                    elif event.key == pygame.K_h:
                        start_pose = driver.last_pose or edit_pose
                        edit_pose = clip_pose(as_pose(saved["rest"]))
                        elapsed = driver.apply_pose(edit_pose)
                        timing_summary["move_to_rest"].append(elapsed)
                        log_joint_move("saved_move", 0, "to_rest", start_pose, edit_pose, elapsed)
                        status = f"Moved to REST | time={elapsed:.3f}s"
                    elif event.key == pygame.K_t:
                        status = run_timing_sequence(driver, saved, timing_summary, TIMING_TEST_CYCLES)
                        edit_pose = clip_pose(as_pose(saved["rest"]))
                    elif event.key == pygame.K_LEFTBRACKET:
                        step_deg = max(MIN_STEP_DEG, step_deg - 0.5)
                        status = f"Step size = {step_deg:.1f} degrees"
                    elif event.key == pygame.K_RIGHTBRACKET:
                        step_deg = min(MAX_STEP_DEG, step_deg + 0.5)
                        status = f"Step size = {step_deg:.1f} degrees"

            if now - last_key_time >= KEY_INTERVAL_S:
                keys = pygame.key.get_pressed()
                moved = False
                new_pose = JointPose(edit_pose.base, edit_pose.shoulder, edit_pose.elbow)

                if keys[pygame.K_UP]:
                    new_pose.base += step_deg
                    moved = True
                elif keys[pygame.K_DOWN]:
                    new_pose.base -= step_deg
                    moved = True
                elif keys[pygame.K_RIGHT]:
                    new_pose.shoulder += step_deg
                    moved = True
                elif keys[pygame.K_LEFT]:
                    new_pose.shoulder -= step_deg
                    moved = True
                elif keys[pygame.K_a]:
                    new_pose.elbow += step_deg
                    moved = True
                elif keys[pygame.K_s]:
                    new_pose.elbow -= step_deg
                    moved = True

                if moved:
                    edit_pose = clip_pose(new_pose)
                    start_pose = driver.last_pose or edit_pose
                    elapsed = driver.apply_pose(edit_pose, settle_s=PREVIEW_SETTLE_S)
                    log_joint_move("preview", 0, selected_name, start_pose, edit_pose, elapsed)
                    status = f"Previewing {selected_name.upper()} | time={elapsed:.3f}s"
                    last_key_time = now

            timing_lines = format_timing_lines(timing_summary)
            draw_text(screen, font, small_font, selected_name, edit_pose, saved, step_deg, status, timing_lines)
            clock.tick(60)
    finally:
        save_points(points, POINTS_FILE)
        pygame.quit()
        driver.release()


if __name__ == "__main__":
    main()
