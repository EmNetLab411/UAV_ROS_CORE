#!/usr/bin/env python3
import argparse
from smbus2 import SMBus

ALL_LED_OFF_H = 0xFD
ALL_LED_OFF_L = 0xFC
ALL_LED_ON_H  = 0xFB
ALL_LED_ON_L  = 0xFA

def main():
    p = argparse.ArgumentParser(description="Emergency OFF all PCA9685 channels")
    p.add_argument("--bus", type=int, default=1, help="I2C bus number (default 1)")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=0x40, help="I2C address (e.g., 0x40)")
    args = p.parse_args()
    with SMBus(args.bus) as bus:
        bus.write_byte_data(args.addr, ALL_LED_ON_L, 0)
        bus.write_byte_data(args.addr, ALL_LED_ON_H, 0)
        bus.write_byte_data(args.addr, ALL_LED_OFF_L, 0)
        bus.write_byte_data(args.addr, ALL_LED_OFF_H, 0x10)  # full off
    print("All channels OFF")

if __name__ == "__main__":
    main()