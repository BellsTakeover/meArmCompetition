import time
import threading
from Adafruit_MotorHAT import Adafruit_MotorHAT

# =============================================================================
#TOP-LEVEL SETTINGS
# =============================================================================
ARM_ID = "arm2"                   # change to arm1, arm2, or arm3
RUN_MODE = "full"                 # "arm_test", "full", "pump_test", "pump_raw_test"

ARM_TEST_CYCLES = 3
FULL_CYCLES = 8                    # 9th cycle uses FINAL_VOLUME_ML
FULL_VOLUME_ML = 10.0
FINAL_VOLUME_ML = 8.0
PUSH_MODE = "fast"                # "fast", "normal", or "ramp" --> used from previous testing

PUMP_TEST_CYCLES = 3
PUMP_TEST_VOLUME_ML = 1.0
PUMP_TEST_PUSH_MODE = "normal"    # "normal", "fast", or "ramp"

RAW_TEST_STEPS = 100
RAW_TEST_DIRECTION = "PULL"       # "PULL" or "PUSH"
RAW_TEST_STYLE = 2                 # 1=SINGLE, 2=DOUBLE, 3=INTERLEAVE, 4=MICROSTEP
RAW_TEST_STEP_DELAY_S = 0.0015

# =============================================================================
#HARDWARE SETTINGS
# =============================================================================
HAT_ADDRESS = 0x6F
I2C_BUS = 1
PWM_FREQUENCY_HZ = 50
PUMP_STEPPER_PORT = 1              # 1 uses M1+M2, 2 uses M3+M4

CHANNELS = {
    "base": 0,
    "shoulder": 1,
    "elbow": 14,
}

# =============================================================================
#DEFAULT ARM TIMING
# =============================================================================
#INTAKE / OUTTAKE:
#-base moves first while shoulder stays at previous position
#-then shoulder lowers slowly in small steps
# -then elbow follow
#REST:
#-shoulder lifts first
#-then base
#-then elbow
DEFAULT_BASE_LEAD_DELAY_S = 0.35
DEFAULT_SHOULDER_TO_ELBOW_DELAY_S = 0.20
DEFAULT_REST_SHOULDER_LEAD_DELAY_S = 0.20
DEFAULT_REST_ELBOW_DELAY_S = 0.40
DEFAULT_MOVE_SETTLE_S = 1.50
DEFAULT_SHOULDER_RAMP_STEP_US = 20
DEFAULT_SHOULDER_RAMP_STEP_DELAY_S = 0.04
DEFAULT_INTAKE_CUP_SETTLE_S = 0.40
DEFAULT_OUTTAKE_CUP_SETTLE_S = 0.25

# =============================================================================
#DEFAULT STEPPER TUNING
# =============================================================================
#These make the syringe motion gentler and more tolerant without needing a
#separate mode. They are applied in the normal/full run too.
DEFAULT_PULL_STYLE = 2             # 1=SINGLE, 2=DOUBLE, 3=INTERLEAVE, 4=MICROSTEP
DEFAULT_PUSH_STYLE = 2

DEFAULT_PULL_START_DELAY_S = 0.0035
DEFAULT_PULL_STEP_DELAY_S = 0.0025
DEFAULT_PULL_END_DELAY_S = 0.0028

DEFAULT_PUSH_START_DELAY_S = 0.0025
DEFAULT_PUSH_STEP_DELAY_S = 0.0018
DEFAULT_PUSH_END_DELAY_S = 0.0020

DEFAULT_RAMP_STEPS = 50
DEFAULT_CHUNK_SIZE_STEPS = 60
DEFAULT_CHUNK_PAUSE_S = 0.01
DEFAULT_DIRECTION_SETTLE_S = 0.10

DEFAULT_BURST_PRELOAD_ML = 0.02
DEFAULT_BURST_SETTLE_S = 0.10
DEFAULT_BURST_ANTI_DRIP_RETRACT_ML = 0.005
DEFAULT_BURST_ANTI_DRIP_DELAY_S = 0.0018

# =============================================================================
#MEASURED SYRINGE TIMING DATA FROM PRIOR TESTS
# =============================================================================
ARM_TIMING_SAMPLES_MS = {
    "arm1": {
        "pull_full_ms": [6513.39, 6502.60, 6531.43, 6263.82, 6520.37, 6259.35, 6222.22, 6221.22],
        "push_full_ms": [6225.54, 6212.70, 6573.11, 6218.69, 6524.37, 6224.90, 6548.23, 6524.66],
        "pull_final_ms": 5801.51,
        "push_final_ms": 5802.93,
    },
    "arm2": {
        "pull_full_ms": [24251.06, 24329.45, 24306.80, 24040.32, 24240.05, 24335.05, 24061.49, 24336.45],
        "push_full_ms": [24301.85, 24289.16, 24309.08, 24336.35, 24326.18, 24335.22, 24336.28, 24033.54],
        "pull_final_ms": 21635.78,
        "push_final_ms": 21322.95,
    },
    "arm3": {
        "pull_full_ms": [19120.50, 19471.57, 19463.18, 19448.54, 19437.75, 19509.77, 19510.63, 19529.83],
        "push_full_ms": [19127.15, 19473.42, 19466.54, 19453.69, 19524.35, 19233.18, 19550.53, 19518.75],
        "pull_final_ms": 17090.32,
        "push_final_ms": 17068.71,
    },
}


#this just turns a list of timing samples into one average so the code can reuse old measured syringe timing
def avg_ms(values):
    return sum(values) / len(values) if values else 0.0


# =============================================================================
#PER-ARM SETTINGS
# =============================================================================
ARM_CONFIGS = {
    "arm1": {
        "rest": (1400, 1000, 2500),
        "intake": (900, 1050, 1650),
        "outtake": (1850, 1000, 1650),
        "startup_delay_s": 0.0,
        "base_lead_delay_s": DEFAULT_BASE_LEAD_DELAY_S,
        "shoulder_to_elbow_delay_s": DEFAULT_SHOULDER_TO_ELBOW_DELAY_S,
        "rest_shoulder_lead_delay_s": DEFAULT_REST_SHOULDER_LEAD_DELAY_S,
        "rest_elbow_delay_s": DEFAULT_REST_ELBOW_DELAY_S,
        "move_settle_s": DEFAULT_MOVE_SETTLE_S,
        "shoulder_ramp_step_us": DEFAULT_SHOULDER_RAMP_STEP_US,
        "shoulder_ramp_step_delay_s": DEFAULT_SHOULDER_RAMP_STEP_DELAY_S,
        "intake_cup_settle_s": DEFAULT_INTAKE_CUP_SETTLE_S,
        "outtake_cup_settle_s": DEFAULT_OUTTAKE_CUP_SETTLE_S,
        "usteps_per_ml": 166.666,
        "pull_style": DEFAULT_PULL_STYLE,
        "push_style": DEFAULT_PUSH_STYLE,
        "pull_start_delay_s": DEFAULT_PULL_START_DELAY_S,
        "pull_step_delay_s": DEFAULT_PULL_STEP_DELAY_S,
        "pull_end_delay_s": DEFAULT_PULL_END_DELAY_S,
        "push_start_delay_s": DEFAULT_PUSH_START_DELAY_S,
        "push_step_delay_s": DEFAULT_PUSH_STEP_DELAY_S,
        "push_end_delay_s": DEFAULT_PUSH_END_DELAY_S,
        "ramp_steps": DEFAULT_RAMP_STEPS,
        "chunk_size_steps": DEFAULT_CHUNK_SIZE_STEPS,
        "chunk_pause_s": DEFAULT_CHUNK_PAUSE_S,
        "direction_settle_s": DEFAULT_DIRECTION_SETTLE_S,
        "burst_preload_ml": DEFAULT_BURST_PRELOAD_ML,
        "burst_settle_s": DEFAULT_BURST_SETTLE_S,
        "burst_anti_drip_retract_ml": DEFAULT_BURST_ANTI_DRIP_RETRACT_ML,
        "burst_anti_drip_delay_s": DEFAULT_BURST_ANTI_DRIP_DELAY_S,
        "sim_pull_dwell_s": avg_ms(ARM_TIMING_SAMPLES_MS["arm1"]["pull_full_ms"]) / 1000.0,
        "sim_push_dwell_s": avg_ms(ARM_TIMING_SAMPLES_MS["arm1"]["push_full_ms"]) / 1000.0,
        "sim_pull_dwell_final_s": ARM_TIMING_SAMPLES_MS["arm1"]["pull_final_ms"] / 1000.0,
        "sim_push_dwell_final_s": ARM_TIMING_SAMPLES_MS["arm1"]["push_final_ms"] / 1000.0,
    },
    "arm2": {
        "rest": (1400, 1800, 2100),
        "intake": (1000, 1100, 2100),
        "outtake": (2050, 1100, 2100),
        "startup_delay_s": 21.0,
        "base_lead_delay_s": DEFAULT_BASE_LEAD_DELAY_S,
        "shoulder_to_elbow_delay_s": DEFAULT_SHOULDER_TO_ELBOW_DELAY_S,
        "rest_shoulder_lead_delay_s": DEFAULT_REST_SHOULDER_LEAD_DELAY_S,
        "rest_elbow_delay_s": DEFAULT_REST_ELBOW_DELAY_S,
        "move_settle_s": DEFAULT_MOVE_SETTLE_S,
        "shoulder_ramp_step_us": DEFAULT_SHOULDER_RAMP_STEP_US,
        "shoulder_ramp_step_delay_s": DEFAULT_SHOULDER_RAMP_STEP_DELAY_S,
        "intake_cup_settle_s": DEFAULT_INTAKE_CUP_SETTLE_S,
        "outtake_cup_settle_s": DEFAULT_OUTTAKE_CUP_SETTLE_S,
        "usteps_per_ml": 625.0,
        "pull_style": DEFAULT_PULL_STYLE,
        "push_style": DEFAULT_PUSH_STYLE,
        "pull_start_delay_s": DEFAULT_PULL_START_DELAY_S,
        "pull_step_delay_s": DEFAULT_PULL_STEP_DELAY_S,
        "pull_end_delay_s": DEFAULT_PULL_END_DELAY_S,
        "push_start_delay_s": DEFAULT_PUSH_START_DELAY_S,
        "push_step_delay_s": DEFAULT_PUSH_STEP_DELAY_S,
        "push_end_delay_s": DEFAULT_PUSH_END_DELAY_S,
        "ramp_steps": DEFAULT_RAMP_STEPS,
        "chunk_size_steps": DEFAULT_CHUNK_SIZE_STEPS,
        "chunk_pause_s": DEFAULT_CHUNK_PAUSE_S,
        "direction_settle_s": DEFAULT_DIRECTION_SETTLE_S,
        "burst_preload_ml": DEFAULT_BURST_PRELOAD_ML,
        "burst_settle_s": DEFAULT_BURST_SETTLE_S,
        "burst_anti_drip_retract_ml": DEFAULT_BURST_ANTI_DRIP_RETRACT_ML,
        "burst_anti_drip_delay_s": DEFAULT_BURST_ANTI_DRIP_DELAY_S,
        "sim_pull_dwell_s": avg_ms(ARM_TIMING_SAMPLES_MS["arm2"]["pull_full_ms"]) / 1000.0,
        "sim_push_dwell_s": avg_ms(ARM_TIMING_SAMPLES_MS["arm2"]["push_full_ms"]) / 1000.0,
        "sim_pull_dwell_final_s": ARM_TIMING_SAMPLES_MS["arm2"]["pull_final_ms"] / 1000.0,
        "sim_push_dwell_final_s": ARM_TIMING_SAMPLES_MS["arm2"]["push_final_ms"] / 1000.0,
    },
    "arm3": {
        "rest": (1400, 1000, 2500),
        "intake": (900, 1050, 1650),
        "outtake": (1850, 1000, 1650),
        "startup_delay_s": 78.0,
        "base_lead_delay_s": DEFAULT_BASE_LEAD_DELAY_S,
        "shoulder_to_elbow_delay_s": DEFAULT_SHOULDER_TO_ELBOW_DELAY_S,
        "rest_shoulder_lead_delay_s": DEFAULT_REST_SHOULDER_LEAD_DELAY_S,
        "rest_elbow_delay_s": DEFAULT_REST_ELBOW_DELAY_S,
        "move_settle_s": DEFAULT_MOVE_SETTLE_S,
        "shoulder_ramp_step_us": DEFAULT_SHOULDER_RAMP_STEP_US,
        "shoulder_ramp_step_delay_s": DEFAULT_SHOULDER_RAMP_STEP_DELAY_S,
        "intake_cup_settle_s": DEFAULT_INTAKE_CUP_SETTLE_S,
        "outtake_cup_settle_s": DEFAULT_OUTTAKE_CUP_SETTLE_S,
        "usteps_per_ml": 125.0,
        "pull_style": DEFAULT_PULL_STYLE,
        "push_style": DEFAULT_PUSH_STYLE,
        "pull_start_delay_s": DEFAULT_PULL_START_DELAY_S,
        "pull_step_delay_s": DEFAULT_PULL_STEP_DELAY_S,
        "pull_end_delay_s": DEFAULT_PULL_END_DELAY_S,
        "push_start_delay_s": DEFAULT_PUSH_START_DELAY_S,
        "push_step_delay_s": DEFAULT_PUSH_STEP_DELAY_S,
        "push_end_delay_s": DEFAULT_PUSH_END_DELAY_S,
        "ramp_steps": DEFAULT_RAMP_STEPS,
        "chunk_size_steps": DEFAULT_CHUNK_SIZE_STEPS,
        "chunk_pause_s": DEFAULT_CHUNK_PAUSE_S,
        "direction_settle_s": DEFAULT_DIRECTION_SETTLE_S,
        "burst_preload_ml": DEFAULT_BURST_PRELOAD_ML,
        "burst_settle_s": DEFAULT_BURST_SETTLE_S,
        "burst_anti_drip_retract_ml": DEFAULT_BURST_ANTI_DRIP_RETRACT_ML,
        "burst_anti_drip_delay_s": DEFAULT_BURST_ANTI_DRIP_DELAY_S,
        "sim_pull_dwell_s": avg_ms(ARM_TIMING_SAMPLES_MS["arm3"]["pull_full_ms"]) / 1000.0,
        "sim_push_dwell_s": avg_ms(ARM_TIMING_SAMPLES_MS["arm3"]["push_full_ms"]) / 1000.0,
        "sim_pull_dwell_final_s": ARM_TIMING_SAMPLES_MS["arm3"]["pull_final_ms"] / 1000.0,
        "sim_push_dwell_final_s": ARM_TIMING_SAMPLES_MS["arm3"]["push_final_ms"] / 1000.0,
    },
}


#this controller = main supporter of the file
#it owns the motor hat, the meArm joint positions, and the syringe state all in one place
class SingleHatController:
    def __init__(self, addr=HAT_ADDRESS, i2c_bus=I2C_BUS):
        self.mh = Adafruit_MotorHAT(addr=addr, i2c_bus=i2c_bus)
        self._set_pwm_freq_50hz()
        self.current_pulses = {"base": None, "shoulder": None, "elbow": None}

        if PUMP_STEPPER_PORT == 1:
            self.stepper = self.mh.getStepper(200, 1)
        elif PUMP_STEPPER_PORT == 2:
            self.stepper = self.mh.getStepper(200, 2)
        else:
            raise ValueError("PUMP_STEPPER_PORT must be 1 or 2")

        self.current_volume_ml = 0.0
        self.ml_used = 0.0
        self.stepper_pos = 0
        self.stop_requested = False
        self.sequence_started = False
        self._lock = threading.Lock()

#50 Hz is the servo-friendly PWM frequency the arm side expects
    def _set_pwm_freq_50hz(self):
        self.mh._pwm.setPWMFreq(PWM_FREQUENCY_HZ)

    def release(self):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

    # -------------------------------------------------------------------------
    # meArm helpers
    # -------------------------------------------------------------------------
#the hat wants a 12-bit PWM value, not raw microseconds
#so this converts a familiar servo pulse like 1500 us into the actual number the board needs
    def set_servo(self, channel: int, pulse_us: int) -> None:
        pulse_length = (1_000_000.0 / PWM_FREQUENCY_HZ) / 4096.0
        pulse_width = int(float(pulse_us) / pulse_length)
        self.mh._pwm.setPWM(channel, 0, pulse_width)

#this is the simplest way to move one joint
#it sends the pulse and also remembers where that joint was last told to go
    def _set_joint(self, joint_name: str, pulse_us: int) -> None:
        self.set_servo(CHANNELS[joint_name], pulse_us)
        self.current_pulses[joint_name] = pulse_us

#instead of jumping the shoulder straight to a new target, this walks it there in small steps
#that helps the tubing lower more smoothly into the cup
    def _ramp_joint(self, joint_name: str, end_us: int, step_us: int, step_delay_s: float) -> None:
        start_us = self.current_pulses[joint_name]
        if start_us is None:
            self._set_joint(joint_name, end_us)
            return
        if start_us == end_us:
            self._set_joint(joint_name, end_us)
            return
        if step_us <= 0:
            step_us = 1

        direction = 1 if end_us > start_us else -1
        current = start_us
        while (direction == 1 and current < end_us) or (direction == -1 and current > end_us):
            current += direction * step_us
            if direction == 1 and current > end_us:
                current = end_us
            elif direction == -1 and current < end_us:
                current = end_us
            self._set_joint(joint_name, current)
            time.sleep(step_delay_s)

#this controls the overall meArm motion style for each named pose
#intake/outtake and rest are intentionally different because entering a cup is not the same as pulling back out of it
    def move_to_pose(self, pose_name: str, pose: tuple, cfg: dict) -> float:
        base_us, shoulder_us, elbow_us = pose
        t0 = time.perf_counter()

        print(f"  -> {pose_name:8s}  base={base_us}  shoulder={shoulder_us}  elbow={elbow_us}")

        if pose_name.lower() in ("intake", "outtake"):
            # base moves first while shoulder stays at the previous pose
            self._set_joint("base", base_us)
            time.sleep(cfg["base_lead_delay_s"])

            # shoulder lowers slowly
            self._ramp_joint(
                "shoulder",
                shoulder_us,
                int(cfg["shoulder_ramp_step_us"]),
                cfg["shoulder_ramp_step_delay_s"],
            )
            time.sleep(cfg["shoulder_to_elbow_delay_s"])

            # elbow finishes the move
            self._set_joint("elbow", elbow_us)
        else:
            # going back to rest: shoulder lifts first, then base, then elbow
            self._set_joint("shoulder", shoulder_us)
            time.sleep(cfg["rest_shoulder_lead_delay_s"])

            self._set_joint("base", base_us)
            time.sleep(cfg["rest_elbow_delay_s"])

            self._set_joint("elbow", elbow_us)

        time.sleep(cfg["move_settle_s"])
        return (time.perf_counter() - t0) * 1000.0

    # -------------------------------------------------------------------------
    # Syringe helpers
    # -------------------------------------------------------------------------
#these small helpers keep track of whether the user requested an emergency stop
    def clear_stop(self):
        self.stop_requested = False

    def request_stop(self):
        self.stop_requested = True
        print("\n[STOP] Stop requested.")

    def remaining_capacity_ml(self):
        return max(0.0, 10.0 - self.current_volume_ml)

    def status(self):
        return (
            f"Current={self.current_volume_ml:.3f} mL | "
            f"Remaining={self.remaining_capacity_ml():.3f} mL | "
            f"Dispensed={self.ml_used:.3f} mL | "
            f"Pos={self.stepper_pos} steps"
        )

#this is the key syringe calibration line
#if the syringe is moving too little or too much liquid, this conversion is one of the first places to look
#was made for easier calibration amongst different syringe/stepper motors
    def ml_to_steps(self, ml, usteps_per_ml):
        return max(0, int(round(ml * float(usteps_per_ml))))

#the library wants FORWARD/BACKWARD, but the rest of the code thinks in PUSH/PULL
#this helper translates between those two languages
    def _step_direction(self, direction):
        if direction == "PUSH":
            return Adafruit_MotorHAT.FORWARD
        elif direction == "PULL":
            return Adafruit_MotorHAT.BACKWARD
        raise ValueError("direction must be 'PUSH' or 'PULL'")

#this is the smoother stepper move
#it starts gently, cruises in the middle, then eases back out near the end so the motor is less likely to struggle
    def _move_steps_ramped(self, steps, direction, start_delay_s, cruise_delay_s, end_delay_s, style, ramp_steps):
        move_dir = self._step_direction(direction)
        if steps <= 0:
            return

        ramp_steps = min(ramp_steps, max(1, steps // 2))
        for i in range(steps):
            if self.stop_requested:
                break

            if i < ramp_steps:
                frac = i / max(1, ramp_steps)
                step_delay_s = start_delay_s + (cruise_delay_s - start_delay_s) * frac
            elif i >= steps - ramp_steps:
                frac = (i - (steps - ramp_steps)) / max(1, ramp_steps)
                step_delay_s = cruise_delay_s + (end_delay_s - cruise_delay_s) * frac
            else:
                step_delay_s = cruise_delay_s

            self.stepper.onestep(direction=move_dir, style=style)
            time.sleep(step_delay_s)

            if direction == "PUSH":
                self.stepper_pos += 1
            else:
                self.stepper_pos -= 1

#big syringe moves are broken into chunks here
#that gives the motor tiny breathers instead of forcing one huge uninterrupted move
    def _move_steps_chunked_ramped(
        self,
        steps,
        direction,
        start_delay_s,
        cruise_delay_s,
        end_delay_s,
        style,
        ramp_steps,
        chunk_size_steps,
        chunk_pause_s,
    ):
        remaining = steps
        first_chunk = True

        while remaining > 0 and not self.stop_requested:
            this_chunk = min(chunk_size_steps, remaining)
            chunk_start = start_delay_s if first_chunk else cruise_delay_s
            chunk_end = end_delay_s if remaining <= chunk_size_steps else cruise_delay_s

            self._move_steps_ramped(
                this_chunk,
                direction,
                chunk_start,
                cruise_delay_s,
                chunk_end,
                style,
                ramp_steps,
            )

            remaining -= this_chunk
            first_chunk = False
            if remaining > 0 and not self.stop_requested:
                time.sleep(chunk_pause_s)

#this is the standard measured syringe move
#it figures out how much it can safely move, converts mL to steps, runs the stepper, then updates the tracked fluid amount
    def bolus(self, direction, ml, cfg):
        with self._lock:
            if direction == "PULL":
                actual_ml = min(ml, self.remaining_capacity_ml())
            elif direction == "PUSH":
                actual_ml = min(ml, self.current_volume_ml)
            else:
                raise ValueError("direction must be 'PULL' or 'PUSH'")

            if actual_ml <= 0:
                print(f"[WARN] {direction} skipped. {self.status()}")
                return 0.0

            requested_steps = self.ml_to_steps(actual_ml, cfg["usteps_per_ml"])
            start_pos = self.stepper_pos
            t0 = time.perf_counter()

            print(f"[ACTION] {direction} {actual_ml:.3f} mL ({requested_steps} steps)")
            time.sleep(cfg["direction_settle_s"])

            if direction == "PULL":
                self._move_steps_chunked_ramped(
                    requested_steps,
                    "PULL",
                    cfg["pull_start_delay_s"],
                    cfg["pull_step_delay_s"],
                    cfg["pull_end_delay_s"],
                    cfg["pull_style"],
                    int(cfg["ramp_steps"]),
                    int(cfg["chunk_size_steps"]),
                    cfg["chunk_pause_s"],
                )
            else:
                self._move_steps_chunked_ramped(
                    requested_steps,
                    "PUSH",
                    cfg["push_start_delay_s"],
                    cfg["push_step_delay_s"],
                    cfg["push_end_delay_s"],
                    cfg["push_style"],
                    int(cfg["ramp_steps"]),
                    int(cfg["chunk_size_steps"]),
                    cfg["chunk_pause_s"],
                )

            moved_steps = abs(self.stepper_pos - start_pos)
            moved_ml = moved_steps / float(cfg["usteps_per_ml"])

            if direction == "PULL":
                self.current_volume_ml += moved_ml
            else:
                self.current_volume_ml -= moved_ml
                self.ml_used += moved_ml

            self.current_volume_ml = max(0.0, min(self.current_volume_ml, 10.0))
            print(f"[DONE] {direction} complete | moved={moved_ml:.3f} mL | {self.status()}")
            return (time.perf_counter() - t0) * 1000.0

#this is the faster push option
#it can preload a tiny amount first, do the main push, then do a tiny anti-drip retract at the end
    def fast_push_burst(self, ml, cfg):
        with self._lock:
            if ml <= 0:
                return 0.0

            preload_ml = cfg["burst_preload_ml"]
            total_push_needed = preload_ml + ml
            if total_push_needed > self.current_volume_ml:
                ml = max(0.0, self.current_volume_ml - preload_ml)

            if ml <= 0:
                print(f"[WARN] FAST PUSH BURST skipped. {self.status()}")
                return 0.0

            t0 = time.perf_counter()
            print(f"[ACTION] FAST PUSH BURST {ml:.3f} mL")
            time.sleep(cfg["burst_settle_s"])

            if self.stop_requested:
                return 0.0

            if preload_ml > 0:
                preload_steps = self.ml_to_steps(preload_ml, cfg["usteps_per_ml"])
                start_pos = self.stepper_pos
                self._move_steps_chunked_ramped(
                    preload_steps,
                    "PUSH",
                    cfg["push_start_delay_s"],
                    cfg["push_step_delay_s"],
                    cfg["push_end_delay_s"],
                    cfg["push_style"],
                    int(cfg["ramp_steps"]),
                    int(cfg["chunk_size_steps"]),
                    cfg["chunk_pause_s"],
                )
                moved_steps = abs(self.stepper_pos - start_pos)
                moved_ml = moved_steps / float(cfg["usteps_per_ml"])
                self.current_volume_ml -= moved_ml
                self.ml_used += moved_ml

            if self.stop_requested:
                self.current_volume_ml = max(0.0, min(self.current_volume_ml, 10.0))
                print(f"[DONE] FAST PUSH BURST stopped early | {self.status()}")
                return 0.0

            burst_steps = self.ml_to_steps(ml, cfg["usteps_per_ml"])
            start_pos = self.stepper_pos
            # faster cruise during burst, but still ramped
            burst_cruise = min(cfg["push_step_delay_s"], cfg["push_step_delay_s"])
            self._move_steps_chunked_ramped(
                burst_steps,
                "PUSH",
                cfg["push_start_delay_s"],
                burst_cruise,
                cfg["push_end_delay_s"],
                cfg["push_style"],
                int(cfg["ramp_steps"]),
                int(cfg["chunk_size_steps"]),
                cfg["chunk_pause_s"],
            )
            moved_steps = abs(self.stepper_pos - start_pos)
            moved_ml = moved_steps / float(cfg["usteps_per_ml"])
            self.current_volume_ml -= moved_ml
            self.ml_used += moved_ml

            retract_ml = cfg["burst_anti_drip_retract_ml"]
            if (not self.stop_requested) and retract_ml > 0:
                retract_ml = min(retract_ml, self.remaining_capacity_ml())
                if retract_ml > 0:
                    retract_steps = self.ml_to_steps(retract_ml, cfg["usteps_per_ml"])
                    start_pos = self.stepper_pos
                    self._move_steps_chunked_ramped(
                        retract_steps,
                        "PULL",
                        cfg["pull_start_delay_s"],
                        cfg["burst_anti_drip_delay_s"],
                        cfg["pull_end_delay_s"],
                        cfg["pull_style"],
                        int(cfg["ramp_steps"]),
                        int(cfg["chunk_size_steps"]),
                        cfg["chunk_pause_s"],
                    )
                    moved_steps = abs(self.stepper_pos - start_pos)
                    moved_ml = moved_steps / float(cfg["usteps_per_ml"])
                    self.current_volume_ml += moved_ml

            self.current_volume_ml = max(0.0, min(self.current_volume_ml, 10.0))
            print(f"[DONE] FAST PUSH BURST complete | {self.status()}")
            return (time.perf_counter() - t0) * 1000.0

    def ramp_push_test(self, ml, cfg):
        # keep a simple alias so the rest of the code can still choose "ramp"
        return self.fast_push_burst(ml, cfg)

#this is the bare-bones raw stepper test used when you just want to see if the motor is stepping correctly
    def move_raw_steps(self, steps, direction, step_delay_s, style):
        t0 = time.perf_counter()
        start_pos = self.stepper_pos
        move_dir = self._step_direction(direction)
        print(f"[RAW TEST] {direction} {steps} steps | style={style} | step_delay={step_delay_s}")
        for _ in range(steps):
            if self.stop_requested:
                break
            self.stepper.onestep(direction=move_dir, style=style)
            time.sleep(step_delay_s)
            if direction == "PUSH":
                self.stepper_pos += 1
            else:
                self.stepper_pos -= 1
        moved_steps = abs(self.stepper_pos - start_pos)
        print(f"[RAW TEST DONE] moved={moved_steps} steps | {self.status()}")
        return (time.perf_counter() - t0) * 1000.0


#this waits for a certain amount of time, but it keeps checking for a stop request while waiting
def wait_or_stop(duration_s: float, controller=None) -> bool:
    t0 = time.perf_counter()
    while (time.perf_counter() - t0) < duration_s:
        if controller is not None and controller.stop_requested:
            return False
        time.sleep(0.02)
    return True


#this runs in a background thread so pressing Enter during a run can stop the sequence
def wait_for_stop_enter(controller: SingleHatController):
    input()
    controller.request_stop()


#this picks which push style to use so the main sequence does not have to care about the low-level details
def do_push(controller: SingleHatController, push_mode: str, volume_ml: float, cfg: dict) -> float:
    if push_mode == "fast":
        return controller.fast_push_burst(volume_ml, cfg)
    if push_mode == "ramp":
        return controller.ramp_push_test(volume_ml, cfg)
    return controller.bolus("PUSH", volume_ml, cfg)


#arm_test is for timing and debugging the meArm path without actually moving liquid
#that makes it much easier to see whether the arm choreography is right before the syringe is brought back in
def run_arm_test(controller: SingleHatController, arm_id: str, cfg: dict, cycles: int):
    rest = cfg["rest"]
    intake = cfg["intake"]
    outtake = cfg["outtake"]

    print(f"\n[ARM TEST] {arm_id}")
    print("Sequence per cycle:")
    print("  REST -> INTAKE -> [simulated PULL dwell] -> REST -> OUTTAKE -> [simulated PUSH dwell] -> REST")
    print(f"Startup delay: {cfg['startup_delay_s']:.2f} s")
    print(f"Base lead delay: {cfg['base_lead_delay_s']:.2f} s")
    print(f"Shoulder ramp step: {cfg['shoulder_ramp_step_us']} us")
    print(f"Shoulder ramp delay: {cfg['shoulder_ramp_step_delay_s']:.2f} s")
    print(f"Shoulder->elbow delay: {cfg['shoulder_to_elbow_delay_s']:.2f} s")
    print(f"Rest shoulder lead delay: {cfg['rest_shoulder_lead_delay_s']:.2f} s")
    print(f"Rest elbow delay: {cfg['rest_elbow_delay_s']:.2f} s")
    print(f"Settle delay: {cfg['move_settle_s']:.2f} s")
    print(f"Simulated pull dwell: {cfg['sim_pull_dwell_s']:.2f} s")
    print(f"Simulated push dwell: {cfg['sim_push_dwell_s']:.2f} s\n")

    if cfg["startup_delay_s"] > 0:
        print(f"[WAIT] startup delay {cfg['startup_delay_s']:.2f} s")
        time.sleep(cfg["startup_delay_s"])

    rest_to_intake_times = []
    intake_dwell_times = []
    intake_to_rest_times = []
    rest_to_outtake_times = []
    outtake_dwell_times = []
    outtake_to_rest_times = []
    cycle_total_times = []
    """
    print("Press ENTER to start arm test.")
    input()
    """
    print("[START] Arm test starting automatically...")
    controller.move_to_pose("REST", rest, cfg)

    for cycle in range(1, cycles + 1):
        print(f"\n--- ARM TEST Cycle {cycle}/{cycles} ---")
        cycle_start = time.perf_counter()

        t_ms = controller.move_to_pose("INTAKE", intake, cfg)
        rest_to_intake_times.append(t_ms)
        print(f"[TIMING] REST -> INTAKE: {t_ms:.2f} ms")

        dwell_s = cfg["sim_pull_dwell_s"] if cycle < cycles or FINAL_VOLUME_ML == FULL_VOLUME_ML else cfg["sim_pull_dwell_final_s"]
        t0 = time.perf_counter()
        time.sleep(dwell_s)
        t_ms = (time.perf_counter() - t0) * 1000.0
        intake_dwell_times.append(t_ms)
        print(f"[TIMING] simulated PULL dwell: {t_ms:.2f} ms")

        t_ms = controller.move_to_pose("REST", rest, cfg)
        intake_to_rest_times.append(t_ms)
        print(f"[TIMING] INTAKE -> REST: {t_ms:.2f} ms")

        t_ms = controller.move_to_pose("OUTTAKE", outtake, cfg)
        rest_to_outtake_times.append(t_ms)
        print(f"[TIMING] REST -> OUTTAKE: {t_ms:.2f} ms")

        dwell_s = cfg["sim_push_dwell_s"] if cycle < cycles or FINAL_VOLUME_ML == FULL_VOLUME_ML else cfg["sim_push_dwell_final_s"]
        t0 = time.perf_counter()
        time.sleep(dwell_s)
        t_ms = (time.perf_counter() - t0) * 1000.0
        outtake_dwell_times.append(t_ms)
        print(f"[TIMING] simulated PUSH dwell: {t_ms:.2f} ms")

        t_ms = controller.move_to_pose("REST", rest, cfg)
        outtake_to_rest_times.append(t_ms)
        print(f"[TIMING] OUTTAKE -> REST: {t_ms:.2f} ms")

        cycle_total_ms = (time.perf_counter() - cycle_start) * 1000.0
        cycle_total_times.append(cycle_total_ms)
        print(f"[TIMING] Cycle {cycle}: total={cycle_total_ms:.2f} ms")

    def avg(vals):
        return sum(vals) / len(vals) if vals else 0.0

    print("\n[ARM TEST SUMMARY]")
    print(f"REST -> INTAKE avg: {avg(rest_to_intake_times):.2f} ms")
    print(f"simulated PULL dwell avg: {avg(intake_dwell_times):.2f} ms")
    print(f"INTAKE -> REST avg: {avg(intake_to_rest_times):.2f} ms")
    print(f"REST -> OUTTAKE avg: {avg(rest_to_outtake_times):.2f} ms")
    print(f"simulated PUSH dwell avg: {avg(outtake_dwell_times):.2f} ms")
    print(f"OUTTAKE -> REST avg: {avg(outtake_to_rest_times):.2f} ms")
    print(f"Total cycle avg: {avg(cycle_total_times):.2f} ms")


#pump_test is the quickest way to re-check syringe timing and recalibrate usteps_per_ml
def run_pump_test(controller: SingleHatController, arm_id: str, cfg: dict, cycles: int, volume_ml: float, push_mode: str):
    print(f"\n[PUMP TEST] {arm_id}")
    print(f"usteps_per_ml = {cfg['usteps_per_ml']}")
    print(f"pull_style = {cfg['pull_style']} | push_style = {cfg['push_style']}")
    print(f"pull delays: start={cfg['pull_start_delay_s']}, cruise={cfg['pull_step_delay_s']}, end={cfg['pull_end_delay_s']}")
    print(f"push delays: start={cfg['push_start_delay_s']}, cruise={cfg['push_step_delay_s']}, end={cfg['push_end_delay_s']}")
    print(f"ramp_steps = {cfg['ramp_steps']} | chunk_size_steps = {cfg['chunk_size_steps']} | chunk_pause_s = {cfg['chunk_pause_s']}")
    print(f"direction_settle_s = {cfg['direction_settle_s']}")
    print(f"cycles = {cycles} | test volume = {volume_ml} mL | push_mode = {push_mode}")

    controller.clear_stop()
    controller.current_volume_ml = 0.0
    controller.ml_used = 0.0
    controller.stepper_pos = 0

    print("Press ENTER to start pump test.")
    input()

    timing_data = []
    run_start = time.perf_counter()
    for cycle in range(1, cycles + 1):
        print(f"\n--- PUMP TEST Cycle {cycle}/{cycles} ---")
        cycle_start = time.perf_counter()
        pull_ms = controller.bolus("PULL", volume_ml, cfg)
        if controller.stop_requested:
            break
        push_ms = do_push(controller, push_mode, volume_ml, cfg)
        if controller.stop_requested:
            break
        cycle_total_ms = (time.perf_counter() - cycle_start) * 1000.0
        timing_data.append((cycle, pull_ms, push_ms, cycle_total_ms))
        print(f"[TIMING] Cycle {cycle}: pull={pull_ms:.2f} ms | push={push_ms:.2f} ms | total={cycle_total_ms:.2f} ms")

    run_total_ms = (time.perf_counter() - run_start) * 1000.0
    print("\n[PUMP TEST SUMMARY]")
    for cycle, pull_ms, push_ms, total_ms in timing_data:
        print(f"Cycle {cycle}: pull={pull_ms:.2f} ms | push={push_ms:.2f} ms | total={total_ms:.2f} ms")
    if timing_data:
        avg_pull = sum(x[1] for x in timing_data) / len(timing_data)
        avg_push = sum(x[2] for x in timing_data) / len(timing_data)
        avg_total = sum(x[3] for x in timing_data) / len(timing_data)
        print(f"[AVG] pull={avg_pull:.2f} ms | push={avg_push:.2f} ms | total={avg_total:.2f} ms")
    print(f"[RUN TOTAL] {run_total_ms:.2f} ms")
    print(f"[FINAL STATUS] {controller.status()}")
    print("[RECALIBRATION NOTE] If moved volume is too small, increase usteps_per_ml. If moved volume is too large, decrease usteps_per_ml.")


#pump_raw_test strips things down even further and just steps the motor a fixed number of steps
def run_pump_raw_test(controller: SingleHatController):
    print("\n[PUMP RAW TEST]")
    print(f"RAW_TEST_STEPS = {RAW_TEST_STEPS}")
    print(f"RAW_TEST_DIRECTION = {RAW_TEST_DIRECTION}")
    print(f"RAW_TEST_STYLE = {RAW_TEST_STYLE}")
    print(f"RAW_TEST_STEP_DELAY_S = {RAW_TEST_STEP_DELAY_S}")
    print("Press ENTER to start raw step test.")
    input()
    t_ms = controller.move_raw_steps(RAW_TEST_STEPS, RAW_TEST_DIRECTION, RAW_TEST_STEP_DELAY_S, RAW_TEST_STYLE)
    print(f"[TIMING] raw test total = {t_ms:.2f} ms")


#this is the real full cycle
#the arm goes to intake, the syringe pulls, the arm returns, the arm goes to outtake, the syringe pushes, then the arm returns again
def run_full_sequence(controller: SingleHatController, arm_id: str, cfg: dict, full_cycles: int, full_volume_ml: float, final_volume_ml: float, push_mode: str):
    rest = cfg["rest"]
    intake = cfg["intake"]
    outtake = cfg["outtake"]

    print(f"\n[FULL RUN PROFILE] {arm_id}")
    print(f"REST    = {rest}")
    print(f"INTAKE  = {intake}")
    print(f"OUTTAKE = {outtake}")
    print(f"usteps_per_ml = {cfg['usteps_per_ml']}")
    print(f"Expected pull avg (full cycles) = {cfg['sim_pull_dwell_s'] * 1000.0:.2f} ms")
    print(f"Expected push avg (full cycles) = {cfg['sim_push_dwell_s'] * 1000.0:.2f} ms")
    print(f"Base lead delay = {cfg['base_lead_delay_s']:.2f} s")
    print(f"Shoulder ramp step = {cfg['shoulder_ramp_step_us']} us")
    print(f"Shoulder ramp delay = {cfg['shoulder_ramp_step_delay_s']:.2f} s")
    print(f"Shoulder->elbow delay = {cfg['shoulder_to_elbow_delay_s']:.2f} s")
    print(f"Rest shoulder lead = {cfg['rest_shoulder_lead_delay_s']:.2f} s")
    print(f"Rest elbow delay = {cfg['rest_elbow_delay_s']:.2f} s")
    print(f"Move settle = {cfg['move_settle_s']:.2f} s")
    print(f"Intake cup settle = {cfg['intake_cup_settle_s']:.2f} s")
    print(f"Outtake cup settle = {cfg['outtake_cup_settle_s']:.2f} s")
    print(f"Pull step delay = {cfg['pull_step_delay_s']:.4f} s | Push step delay = {cfg['push_step_delay_s']:.4f} s")

    if cfg["startup_delay_s"] > 0:
        print(f"[WAIT] startup delay {cfg['startup_delay_s']:.2f} s")
        time.sleep(cfg["startup_delay_s"])
    """
    print("\n[FULL RUN] Press ENTER once to start.")
    print("Press ENTER again during the run to request stop.")
    input()
    """
    print("[START] Full run starting automatically...")
    print("Press ENTER during the run to request stop.")

    stop_thread = threading.Thread(target=wait_for_stop_enter, args=(pump,), daemon=True)
    stop_thread.start()

    controller.move_to_pose("REST", rest, cfg)
    controller.clear_stop()
    controller.sequence_started = True
    controller.current_volume_ml = 0.0

    timing_data = []
    run_start = time.perf_counter()
    total_cycles = full_cycles + 1
    for cycle in range(1, total_cycles + 1):
        if controller.stop_requested:
            break

        volume_ml = full_volume_ml if cycle <= full_cycles else final_volume_ml
        cycle_start = time.perf_counter()

        print(f"\n--- Cycle {cycle}/{total_cycles}: move to intake ---")
        move_intake_ms = controller.move_to_pose("INTAKE", intake, cfg)
        if not wait_or_stop(cfg["intake_cup_settle_s"], controller):
            break

        pull_ms = controller.bolus("PULL", volume_ml, cfg)
        if controller.stop_requested:
            break

        print(f"--- Cycle {cycle}/{total_cycles}: move to rest ---")
        move_rest1_ms = controller.move_to_pose("REST", rest, cfg)
        if controller.stop_requested:
            break

        print(f"--- Cycle {cycle}/{total_cycles}: move to outtake ---")
        move_outtake_ms = controller.move_to_pose("OUTTAKE", outtake, cfg)
        if not wait_or_stop(cfg["outtake_cup_settle_s"], controller):
            break

        push_ms = do_push(controller, push_mode, volume_ml, cfg)
        if controller.stop_requested:
            break

        print(f"--- Cycle {cycle}/{total_cycles}: move to rest ---")
        move_rest2_ms = controller.move_to_pose("REST", rest, cfg)

        cycle_total_ms = (time.perf_counter() - cycle_start) * 1000.0
        timing_data.append((cycle, move_intake_ms, pull_ms, move_rest1_ms, move_outtake_ms, push_ms, move_rest2_ms, cycle_total_ms))
        print(
            f"[TIMING] Cycle {cycle}: "
            f"move_intake={move_intake_ms:.2f} ms | "
            f"pull={pull_ms:.2f} ms | "
            f"move_rest1={move_rest1_ms:.2f} ms | "
            f"move_outtake={move_outtake_ms:.2f} ms | "
            f"push={push_ms:.2f} ms | "
            f"move_rest2={move_rest2_ms:.2f} ms | "
            f"total={cycle_total_ms:.2f} ms"
        )

    run_total_ms = (time.perf_counter() - run_start) * 1000.0
    if controller.stop_requested:
        print("\n[STOPPED] Sequence stopped early.")
    else:
        print("\n[FINISHED] Automatic sequence complete.")

    print("\n[TIMING SUMMARY]")
    for row in timing_data:
        cycle, move_intake_ms, pull_ms, move_rest1_ms, move_outtake_ms, push_ms, move_rest2_ms, total_ms = row
        print(
            f"Cycle {cycle}: move_intake={move_intake_ms:.2f} ms | pull={pull_ms:.2f} ms | "
            f"move_rest1={move_rest1_ms:.2f} ms | move_outtake={move_outtake_ms:.2f} ms | "
            f"push={push_ms:.2f} ms | move_rest2={move_rest2_ms:.2f} ms | total={total_ms:.2f} ms"
        )
    print(f"[RUN TOTAL] {run_total_ms:.2f} ms")
    print(f"[FINAL STATUS] {controller.status()}")
    controller.sequence_started = False


#main decides which mode to run so you can use the same file for arm-only testing, pump-only testing, or the full combined run
def main():
    if ARM_ID not in ARM_CONFIGS:
        raise ValueError(f"Unknown ARM_ID '{ARM_ID}'")

    cfg = ARM_CONFIGS[ARM_ID]
    controller = SingleHatController(addr=HAT_ADDRESS, i2c_bus=I2C_BUS)

    if RUN_MODE == "arm_test":
        run_arm_test(controller, ARM_ID, cfg, ARM_TEST_CYCLES)
    elif RUN_MODE == "pump_test":
        run_pump_test(controller, ARM_ID, cfg, PUMP_TEST_CYCLES, PUMP_TEST_VOLUME_ML, PUMP_TEST_PUSH_MODE)
    elif RUN_MODE == "pump_raw_test":
        run_pump_raw_test(controller)
    else:
        run_full_sequence(controller, ARM_ID, cfg, FULL_CYCLES, FULL_VOLUME_ML, FINAL_VOLUME_ML, PUSH_MODE)


if __name__ == "__main__":
    main()