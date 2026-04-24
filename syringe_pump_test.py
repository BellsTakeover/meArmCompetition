"""
syringe_pump.py

Automatic syringe pump sequence:
- waits for start input (ENTER)
- runs 8 cycles of 10 mL intake + 10 mL outtake
- runs 1 final cycle of 8 mL intake + 8 mL outtake
- then stops

MAYBE DO RAMP + FAST BURST
--start slow the increase speed to max 
ADD DELAYS ONCE WE FIGURE IT OUT --> ADD TO MAIN CODE

Notes:
- "PULL" = intake
- "PUSH" = outtake
- Uses MotorKit stepper control
- No need for user control anymore
- Developed from OpenSyringePump by manimino
"""

import atexit
import time
import threading

import board
from adafruit_motor import stepper as STEPPER
from adafruit_motorkit import MotorKit


class SyringePumpMotorKit:
    """
    Syringe pump controller adapted for Adafruit MotorKit stepper control.
    """

    #Constants from Arduino sketch
    #1 mL = 125 steps --> 100 steps was about 0.4mL 
    SYRINGE_VOLUME_ML = 10.0
    #80mm for full barrel & 61 mm for 0mL-10mL ==> use 61 mm
    SYRINGE_BARREL_LENGTH_MM = 61.0  # check conversion from in --> mm

    THREADED_ROD_PITCH_MM_PER_REV = 1.25
    STEPS_PER_REVOLUTION = 200.0
    MICROSTEPS_PER_STEP = 1.0  # 16 for microsteps

    MAX_SAFE_VOLUME_ML = 10   # safety margin so syringe does not overrun --> change to 9.5 if

    def __init__(
        self,
        stepper_num=1,
        default_style=STEPPER.DOUBLE,
        normal_step_delay_s=0.0010, #slower=0.0015, fast = 0.001
        burst_step_delay_s=0.0002, #fastest=0.0002, slower=0.001
    ):
        self.kit = MotorKit(i2c=board.I2C())

        if stepper_num == 1:
            self.stepper = self.kit.stepper1
        elif stepper_num == 2:
            self.stepper = self.kit.stepper2
        else:
            raise ValueError("stepper_num must be 1 or 2")

        self.default_style = default_style
        self.normal_step_delay_s = normal_step_delay_s
        self.burst_step_delay_s = burst_step_delay_s
        #about 416 steps
        self.usteps_per_mm = (
            self.MICROSTEPS_PER_STEP
            * self.STEPS_PER_REVOLUTION
            / self.THREADED_ROD_PITCH_MM_PER_REV
        )

        self.usteps_per_ml = 125

        self.ml_bolus = 0.500
        self.ml_big_bolus = 1.000
        self.ml_used = 0.0          # total dispensed tracker
        self.current_volume_ml = 0.0  # current amount inside syringe
        self.stepper_pos = 0

        self._lock = threading.Lock()

        atexit.register(self.release)

        #Makes sure that it can stop when still running      
        self.stop_requested = False
        self.sequence_started = False

    def release(self):
        self.stepper.release()

    def ml_to_steps(self, ml):
        return max(0, int(round(ml * self.usteps_per_ml)))

    def remaining_capacity_ml(self):
        return max(0.0, self.MAX_SAFE_VOLUME_ML - self.current_volume_ml)

    def status(self):
        return (
            f"Current={self.current_volume_ml:.3f} mL | "
            f"Remaining={self.remaining_capacity_ml():.3f} mL | "
            f"Dispensed={self.ml_used:.3f} mL | "
            f"Pos={self.stepper_pos} steps"
        )

    def request_stop(self):
        self.stop_requested = True
        print("\n[STOP] Stop requested. Finishing up")

    def clear_stop(self):
        self.stop_requested = False

    def _step_direction(self, direction):
        """
        Swap FORWARD/BACKWARD here if your hardware moves the wrong way.
        """
        if direction == "PUSH":
            return STEPPER.FORWARD
        elif direction == "PULL":
            return STEPPER.BACKWARD
        else:
            raise ValueError("direction must be 'PUSH' or 'PULL'")

    def _move_steps(self, steps, direction, step_delay_s, style=None):
        if style is None:
            style = self.default_style

        move_dir = self._step_direction(direction)

        for _ in range(steps):
            #includes the stop requested function
            if self.stop_requested:
                break

            self.stepper.onestep(direction=move_dir, style=style)
            time.sleep(step_delay_s)

            if direction == "PUSH":
                self.stepper_pos += 1
            else:
                self.stepper_pos -= 1

    def bolus(self, direction, ml, step_delay_s=None, style=None):
        """
        Normal push/pull bolus.
        """
        with self._lock:
            if step_delay_s is None:
                step_delay_s = self.normal_step_delay_s

            if direction == "PULL":
                allowed_ml = self.remaining_capacity_ml()
                actual_ml = min(ml, allowed_ml)
            elif direction == "PUSH":
                actual_ml = min(ml, self.current_volume_ml)
            else:
                raise ValueError("direction must be 'PUSH' or 'PULL'")

            if actual_ml <= 0:
                print(f"[WARN] {direction} skipped. {self.status()}")
                return 0.0

            requested_steps = self.ml_to_steps(actual_ml)
            start_pos = self.stepper_pos
            start_time = time.perf_counter()

            print(f"[ACTION] {direction} {actual_ml:.3f} mL ({requested_steps} steps)")
            self._move_steps(requested_steps, direction, step_delay_s, style=style)

            moved_steps = abs(self.stepper_pos - start_pos)
            moved_ml = moved_steps / self.usteps_per_ml

            if direction == "PULL":
                self.current_volume_ml += moved_ml
            elif direction == "PUSH":
                self.current_volume_ml -= moved_ml
                self.ml_used += moved_ml

            self.current_volume_ml = max(0.0, min(self.current_volume_ml, self.MAX_SAFE_VOLUME_ML))

            print(f"[DONE] {direction} complete | moved={moved_ml:.3f} mL | {self.status()}")
            return (time.perf_counter() - start_time) * 1000

    # EDIT if it needs to be slower!
    def fast_push_burst(
        self,
        ml,
        settle_s=0.10,
        preload_ml=0.02,
        preload_step_delay_s=0.0018,
        burst_step_delay_s=None,
        anti_drip_retract_ml=0.005,
        anti_drip_delay_s=0.0018,
        style=None,
    ):
        """
        Controlled fast dispensing:
        1. brief settle time
        2. small preload push
        3. rapid burst push
        4. tiny retract to reduce dripping
        """
        with self._lock:
            if burst_step_delay_s is None:
                burst_step_delay_s = self.burst_step_delay_s
            if style is None:
                style = self.default_style

            if ml <= 0:
                return 0.0

            total_push_needed = preload_ml + ml
            if total_push_needed > self.current_volume_ml:
                ml = max(0.0, self.current_volume_ml - preload_ml)

            if ml <= 0:
                print(f"[WARN] FAST PUSH BURST skipped. {self.status()}")
                return 0.0

            start_time = time.perf_counter()
            print(f"[ACTION] FAST PUSH BURST {ml:.3f} mL")
            time.sleep(settle_s)

            if self.stop_requested:
                return 0.0

            if preload_ml > 0:
                preload_steps = self.ml_to_steps(preload_ml)
                start_pos = self.stepper_pos
                self._move_steps(
                    preload_steps,
                    "PUSH",
                    preload_step_delay_s,
                    style=style
                )
                moved_steps = abs(self.stepper_pos - start_pos)
                moved_ml = moved_steps / self.usteps_per_ml
                self.current_volume_ml -= moved_ml
                self.ml_used += moved_ml
            #added stop request for better control and testing
            if self.stop_requested:
                self.current_volume_ml = max(0.0, min(self.current_volume_ml, self.MAX_SAFE_VOLUME_ML))
                print(f"[DONE] FAST PUSH BURST stopped early | {self.status()}")
                return 0.0

            burst_steps = self.ml_to_steps(ml)
            start_pos = self.stepper_pos
            self._move_steps(
                burst_steps,
                "PUSH",
                burst_step_delay_s,
                style=style
            )
            moved_steps = abs(self.stepper_pos - start_pos)
            moved_ml = moved_steps / self.usteps_per_ml
            self.current_volume_ml -= moved_ml
            self.ml_used += moved_ml

            if (not self.stop_requested) and anti_drip_retract_ml > 0:
                retract_ml = min(anti_drip_retract_ml, self.remaining_capacity_ml())
                if retract_ml > 0:
                    retract_steps = self.ml_to_steps(retract_ml)
                    start_pos = self.stepper_pos
                    self._move_steps(
                        retract_steps,
                        "PULL",
                        anti_drip_delay_s,
                        style=style
                    )
                    moved_steps = abs(self.stepper_pos - start_pos)
                    moved_ml = moved_steps / self.usteps_per_ml
                    self.current_volume_ml += moved_ml

            self.current_volume_ml = max(0.0, min(self.current_volume_ml, self.MAX_SAFE_VOLUME_ML))
            print(f"[DONE] FAST PUSH BURST complete | {self.status()}")
            return (time.perf_counter() - start_time) * 1000

    def run_automatic_sequence(
        self,
        full_cycles=8,
        full_volume_ml=9.0,
        final_volume_ml=8.0,
        push_mode="fast",
        pause_after_pull_s=0.5,
        pause_after_push_s=0.75,
    ):
        self.clear_stop()
        self.sequence_started = True

        # start automatic cycles from empty state
        self.current_volume_ml = 0.0

        print("[START] Automatic pump sequence starting...")
        print(f"[INFO] {self.status()}")

        timing_data = []
        run_start = time.perf_counter()

        for cycle in range(1, full_cycles + 1):
            if self.stop_requested:
                break

            cycle_start = time.perf_counter()

            print(f"\n--- Cycle {cycle}/{full_cycles + 1}: intake {full_volume_ml} mL ---")
            pull_ms = self.bolus("PULL", ml=full_volume_ml)
            if self.stop_requested:
                break
            time.sleep(pause_after_pull_s)

            print(f"--- Cycle {cycle}/{full_cycles + 1}: outtake {full_volume_ml} mL ---")
            if push_mode == "fast":
                push_ms = self.fast_push_burst(ml=full_volume_ml)
            elif push_mode == "ramp":
                push_ms = self.ramp_push_test(ml=full_volume_ml)
            else:
                push_ms = self.bolus("PUSH", ml=full_volume_ml)

            if self.stop_requested:
                break
            time.sleep(pause_after_push_s)

            cycle_total_ms = (time.perf_counter() - cycle_start) * 1000
            timing_data.append((cycle, pull_ms, push_ms, cycle_total_ms))

            print(f"[TIMING] Cycle {cycle}: pull={pull_ms:.2f} ms | push={push_ms:.2f} ms | total={cycle_total_ms:.2f} ms")

        if not self.stop_requested:
            final_cycle_num = full_cycles + 1
            cycle_start = time.perf_counter()

            print(f"\n--- Cycle {final_cycle_num}/{full_cycles + 1}: intake {final_volume_ml} mL ---")
            pull_ms = self.bolus("PULL", ml=final_volume_ml)
            if not self.stop_requested:
                time.sleep(pause_after_pull_s)

                print(f"--- Cycle {final_cycle_num}/{full_cycles + 1}: outtake {final_volume_ml} mL ---")
                if push_mode == "fast":
                    push_ms = self.fast_push_burst(ml=final_volume_ml)
                elif push_mode == "ramp":
                    push_ms = self.ramp_push_test(ml=final_volume_ml)
                else:
                    push_ms = self.bolus("PUSH", ml=final_volume_ml)

                if not self.stop_requested:
                    time.sleep(pause_after_push_s)

                cycle_total_ms = (time.perf_counter() - cycle_start) * 1000
                timing_data.append((final_cycle_num, pull_ms, push_ms, cycle_total_ms))
                print(f"[TIMING] Cycle {final_cycle_num}: pull={pull_ms:.2f} ms | push={push_ms:.2f} ms | total={cycle_total_ms:.2f} ms")

        run_total_ms = (time.perf_counter() - run_start) * 1000

        if self.stop_requested:
            print("\n[STOPPED] Automatic sequence stopped early.")
        else:
            print("\n[FINISHED] Automatic sequence complete.")

        print("\n[TIMING SUMMARY]")
        for cycle, pull_ms, push_ms, total_ms in timing_data:
            print(f"Cycle {cycle}: pull={pull_ms:.2f} ms | push={push_ms:.2f} ms | total={total_ms:.2f} ms")

        print(f"[RUN TOTAL] {run_total_ms:.2f} ms")
        print(f"[FINAL STATUS] {self.status()}")
        self.sequence_started = False

    # Reset
    def reset_position_forward(self, max_volume_ml=10):
        print("[RESET] Forward reset...")

        if max_volume_ml is None:
            max_volume_ml = self.current_volume_ml

        steps = self.ml_to_steps(max_volume_ml)

        self._move_steps(
            steps,
            "PUSH",
            step_delay_s=self.normal_step_delay_s,
            style=self.default_style,
        )

        self.stepper_pos = 0
        self.current_volume_ml = 0.0

        print("[RESET] Done (syringe empty).")

    def reset_position_backward(self, max_volume_ml=10):
        print("[RESET] Safe backward...")

        if max_volume_ml is None:
            max_volume_ml = self.MAX_SAFE_VOLUME_ML

        steps = self.ml_to_steps(max_volume_ml)

        self._move_steps(
            steps,
            "PULL",
            step_delay_s=self.normal_step_delay_s,
            style=self.default_style,
        )

        self.stepper_pos = 0
        self.current_volume_ml = self.MAX_SAFE_VOLUME_ML

        print("[RESET] Done (max safe pulled-back position).")
    
    #For necessary forced STOP
    def wait_for_stop_enter(self):
        input()
        if self.sequence_started:
            self.request_stop()
    """
    #run test case
    def test_run(pump):
        pump.bolus("PULL", ml=1.0)
    """

    def move_raw_steps(self, steps, direction="PULL", step_delay_s=None, style=None):
        if step_delay_s is None:
            step_delay_s = self.normal_step_delay_s
        print(f"[RAW TEST] {direction} {steps} steps")
        self._move_steps(steps, direction, step_delay_s, style=style)
        print(f"[RAW TEST DONE] {self.status()}")
    
    def ramp_push_test(
        self,
        ml,
        settle_s=0.10,
        slow_fraction=0.2,
        slow_step_delay_s=0.0012,
        fast_step_delay_s=0.0002,
        anti_drip_retract_ml=0.005,
        anti_drip_delay_s=0.0018,
        style=None,
    ):
        """
        Ramp test dispensing:
        1. brief settle time
        2. slow ramp phase to build pressure
        3. faster main push phase
        4. tiny retract to reduce dripping
        """
        with self._lock:
            if style is None:
                style = self.default_style

            if ml <= 0:
                return 0.0

            actual_ml = min(ml, self.current_volume_ml)
            if actual_ml <= 0:
                print(f"[WARN] RAMP PUSH TEST skipped. {self.status()}")
                return 0.0

            start_time = time.perf_counter()
            print(f"[ACTION] RAMP PUSH TEST {actual_ml:.3f} mL")
            time.sleep(settle_s)

            if self.stop_requested:
                return 0.0

            total_steps = self.ml_to_steps(actual_ml)
            slow_steps = int(total_steps * slow_fraction)
            fast_steps = total_steps - slow_steps

            start_pos_total = self.stepper_pos

            # slow ramp phase
            if slow_steps > 0:
                self._move_steps(
                    slow_steps,
                    "PUSH",
                    slow_step_delay_s,
                    style=style
                )

            if self.stop_requested:
                moved_steps = abs(self.stepper_pos - start_pos_total)
                moved_ml = moved_steps / self.usteps_per_ml
                self.current_volume_ml -= moved_ml
                self.ml_used += moved_ml
                self.current_volume_ml = max(0.0, min(self.current_volume_ml, self.MAX_SAFE_VOLUME_ML))
                print(f"[DONE] RAMP PUSH TEST stopped early | {self.status()}")
                return 0.0

            # fast phase
            if fast_steps > 0:
                self._move_steps(
                    fast_steps,
                    "PUSH",
                    fast_step_delay_s,
                    style=style
                )

            moved_steps = abs(self.stepper_pos - start_pos_total)
            moved_ml = moved_steps / self.usteps_per_ml
            self.current_volume_ml -= moved_ml
            self.ml_used += moved_ml

            # tiny retract to help stop drip
            if (not self.stop_requested) and anti_drip_retract_ml > 0:
                retract_ml = min(anti_drip_retract_ml, self.remaining_capacity_ml())
                if retract_ml > 0:
                    retract_steps = self.ml_to_steps(retract_ml)
                    start_pos = self.stepper_pos
                    self._move_steps(
                        retract_steps,
                        "PULL",
                        anti_drip_delay_s,
                        style=style
                    )
                    moved_steps = abs(self.stepper_pos - start_pos)
                    moved_ml = moved_steps / self.usteps_per_ml
                    self.current_volume_ml += moved_ml

            self.current_volume_ml = max(0.0, min(self.current_volume_ml, self.MAX_SAFE_VOLUME_ML))
            print(f"[DONE] RAMP PUSH TEST complete | {self.status()}")
            return (time.perf_counter() - start_time) * 1000


def main():
    pump = SyringePumpMotorKit(
        stepper_num=1,
        default_style=STEPPER.DOUBLE,
        normal_step_delay_s=0.0015,
        burst_step_delay_s=0.0004,
    )

    print("MotorKit syringe pump ready.")

    while True:
        print("MotorKit syringe pump ready.")
        print("Press Enter once to start fast")
        print("     This will do:")
        print("         - 8 cycles of 10 mL intake/outtake")
        print("         - 1 cycle of 8 mL intake/outtake")
        print("q to quit")
        print("r to reset")
        print("t to test for 1 mL")
        print("p to test ramp acceleration")
        print("n to test normal automatic cycle")
        print("Press ENTER again to stop ")

        cmd = input("> ").strip().lower()

        if cmd == "q":
            pump.release()
            print("Exiting :(")
            break

        elif cmd == "r":
            print("Press 1 for PUSH reset and press 2 for PULL reset.")
            r_cmd = input("> ").strip().lower()

            if r_cmd == "1":
                pump.reset_position_forward()
            elif r_cmd == "2":
                pump.reset_position_backward()

            print(pump.status())

        elif cmd == "t":
            pump.bolus("PULL", ml=1.0)
            print(pump.status())

        elif cmd == "n":
            try:
                print("Enter again to exit")
                stop_thread = threading.Thread(target=pump.wait_for_stop_enter, daemon=True)
                stop_thread.start()

                pump.run_automatic_sequence(
                    full_cycles=8,
                    full_volume_ml=9.0,
                    final_volume_ml=8.0,
                    push_mode="normal",
                    pause_after_pull_s=0.5,
                    pause_after_push_s=0.75,
                )
            except Exception as e:
                print(f"[ERROR] {e}")

        elif cmd == "p":
            try:
                print("Enter again to exit")
                stop_thread = threading.Thread(target=pump.wait_for_stop_enter, daemon=True)
                stop_thread.start()

                pump.run_automatic_sequence(
                    full_cycles=8,
                    full_volume_ml=10.0,
                    final_volume_ml=8.0,
                    push_mode="ramp",
                    pause_after_pull_s=0.5,
                    pause_after_push_s=0.75,
                )
            except Exception as e:
                print(f"[ERROR] {e}")

        else:
            try:
                print("Enter again to exit")
                stop_thread = threading.Thread(target=pump.wait_for_stop_enter, daemon=True)
                stop_thread.start()

                pump.run_automatic_sequence(
                    full_cycles=8,
                    full_volume_ml=10.0,
                    final_volume_ml=8.0,
                    push_mode="fast",
                    pause_after_pull_s=0.5,
                    pause_after_push_s=0.75,
                )
            except Exception as e:
                print(f"[ERROR] {e}")

        #clear
        pump.clear_stop()


if __name__ == "__main__":
    main()

"""
Reference notes:
| Mode       | What it does                | Steps per rev |
| ---------- | --------------------------- | ------------- |
| SINGLE     | one coil at a time          | 200           |
| DOUBLE     | two coils (stronger torque) | 200           |
| INTERLEAVE | half steps                  | 400           |
| MICROSTEP  | tiny fractional steps       | ~3200         |
Microstep=smoother control, more accurate water intake --> hopefully no jerking and excessive vibrations
"""
"""
Reference notes:
| Mode       | What it does                | Steps per rev |
| ---------- | --------------------------- | ------------- |
| SINGLE     | one coil at a time          | 200           |
| DOUBLE     | two coils (stronger torque) | 200           |
| INTERLEAVE | half steps                  | 400           |
| MICROSTEP  | tiny fractional steps       | ~3200         |
Microstep=smoother control, more accurate water intake --> hopefully no jerking and excessive vibrations
"""