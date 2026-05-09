"""
- base     -> channel 0
- shoulder -> channel 1
- elbow    -> channel 14
"""

import time
from Adafruit_MotorHAT import Adafruit_MotorHAT

PWM_ADDRESS = 0x60
I2C_BUS = 1
PWM_FREQUENCY_HZ = 50

CHANNELS = {
    "base": 0,
    "shoulder": 1,
    "elbow": 14,
}

MOVE_DELAY_S = 1.0
SETTLE_DELAY_S = 0.5

JOINT_PRESETS = {
    "base":     {"center": 1500, "low": 1200, "high": 1800},
    "shoulder": {"center": 1500, "low": 1250, "high": 1750},
    "elbow":    {"center": 1500, "low": 1250, "high": 1750},
}

mh = Adafruit_MotorHAT(addr=PWM_ADDRESS, i2c_bus=I2C_BUS)

def set_pwm_freq_50hz():
    mh._pwm.setPWMFreq(PWM_FREQUENCY_HZ)

def set_servo_pulse(channel: int, pulse_us: int) -> None:
    pulse_length = (1000000.0 / PWM_FREQUENCY_HZ) / 4096.0
    pulse_width = int(float(pulse_us) / pulse_length)
    mh._pwm.setPWM(channel, 0, pulse_width)

def move_joint(name: str, pulse_us: int, wait_s: float = MOVE_DELAY_S) -> None:
    ch = CHANNELS[name]
    print(f"[MOVE] {name} (ch {ch}) -> {pulse_us} us")
    set_servo_pulse(ch, pulse_us)
    time.sleep(wait_s)

def center_joint(name: str) -> None:
    move_joint(name, JOINT_PRESETS[name]["center"])

def sweep_joint(name: str) -> None:
    cfg = JOINT_PRESETS[name]
    print(f"\n--- Testing {name.upper()} ---")
    move_joint(name, cfg["center"])
    time.sleep(SETTLE_DELAY_S)
    move_joint(name, cfg["low"])
    time.sleep(SETTLE_DELAY_S)
    move_joint(name, cfg["high"])
    time.sleep(SETTLE_DELAY_S)
    move_joint(name, cfg["center"])
    print(f"--- Done testing {name.upper()} ---\n")

def all_center() -> None:
    for name in ["base", "shoulder", "elbow"]:
        center_joint(name)
    print("[INFO] Centered base, shoulder, elbow")

def all_sweep() -> None:
    for name in ["base", "shoulder", "elbow"]:
        sweep_joint(name)

def print_menu() -> None:
    print("\nmeArm joint tester")
    print("------------------")
    print("1 = test base only")
    print("2 = test shoulder only")
    print("3 = test elbow only")
    print("4 = center all three")
    print("5 = sweep all three")
    print("q = quit")

def main() -> None:
    print(f"[INFO] Opening PWM at address 0x{PWM_ADDRESS:02X}")
    set_pwm_freq_50hz()
    print("[INFO] PWM frequency set to 50 Hz")
    print("[INFO] Testing channels:", CHANNELS)
    print("[INFO] Make sure servo 5V power is connected and ground is shared.")

    while True:
        print_menu()
        cmd = input("> ").strip().lower()

        if cmd == "1":
            sweep_joint("base")
        elif cmd == "2":
            sweep_joint("shoulder")
        elif cmd == "3":
            sweep_joint("elbow")
        elif cmd == "4":
            all_center()
        elif cmd == "5":
            all_sweep()
        elif cmd == "q":
            print("Exiting.")
            break
        else:
            print("Unknown choice.")

if __name__ == "__main__":
    main()
