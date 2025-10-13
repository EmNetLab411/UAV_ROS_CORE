#!/usr/bin/env python3
import math
import time
from typing import Dict, List

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from pca9685_servo_control.srv import SetAngle, SetAngleResponse

# Use smbus2 to directly control PCA9685 (no Blinka dependency, selectable bus)
from smbus2 import SMBus

# PCA9685 registers/constants
MODE1      = 0x00
MODE2      = 0x01
PRESCALE   = 0xFE
LED0_ON_L  = 0x06
LED0_ON_H  = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L  = 0xFA
ALL_LED_ON_H  = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

RESTART = 0x80
SLEEP   = 0x10
ALLCALL = 0x01
OUTDRV  = 0x04

class PCA9685:
    def __init__(self, bus: int, address: int, oscillator_hz: int = 25000000):
        self._busnum = bus
        self._addr = address
        self._oscillator_hz = oscillator_hz
        self._bus = SMBus(self._busnum)
        self._init_device()

    def _write8(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val & 0xFF)

    def _read8(self, reg):
        return self._bus.read_byte_data(self._addr, reg)

    def _init_device(self):
        # Initialize device safely
        # MODE2: totem pole (OUTDRV), MODE1: ALLCALL, wake from sleep
        self._write8(MODE2, OUTDRV)
        self._write8(MODE1, ALLCALL)
        time.sleep(0.005)
        mode1 = self._read8(MODE1)
        mode1 = mode1 & ~SLEEP  # wake up
        self._write8(MODE1, mode1)
        time.sleep(0.005)

    def set_pwm_freq(self, freq_hz: float):
        prescaleval = float(self._oscillator_hz) / (4096.0 * float(freq_hz)) - 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self._read8(MODE1)
        newmode = (oldmode & 0x7F) | SLEEP
        self._write8(MODE1, newmode)
        self._write8(PRESCALE, prescale)
        self._write8(MODE1, oldmode)
        time.sleep(0.005)
        self._write8(MODE1, oldmode | RESTART)

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * channel
        self._write8(base + 0, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0x0F)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0x0F)

    def set_off(self, channel: int):
        # Full OFF bit on OFF_H (bit 4)
        base = LED0_ON_L + 4 * channel
        self._write8(base + 0, 0x00)
        self._write8(base + 1, 0x00)
        self._write8(base + 2, 0x00)
        self._write8(base + 3, 0x10)

    def set_all_off(self):
        self._write8(ALL_LED_ON_L, 0)
        self._write8(ALL_LED_ON_H, 0)
        self._write8(ALL_LED_OFF_L, 0)
        self._write8(ALL_LED_OFF_H, 0x10)

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass


class ServoControllerNode:
    def __init__(self):
        # Load params
        self.busnum = rospy.get_param("~i2c_bus", 1)
        self.address = int(rospy.get_param("~i2c_address", 0x40))
        self.pwm_freq = float(rospy.get_param("~pwm_frequency_hz", 50.0))
        self.oscillator_hz = int(rospy.get_param("~oscillator_hz", 25000000))
        self.enable_on_start = bool(rospy.get_param("~enable_on_start", False))

        self.pulse_us_min = float(rospy.get_param("~pulse_us_min", 600.0))
        self.pulse_us_max = float(rospy.get_param("~pulse_us_max", 2400.0))
        self.neutral_deg = float(rospy.get_param("~neutral_deg", 90.0))

        self.joint_names: List[str] = rospy.get_param("~joint_names")
        self.channels: List[int] = rospy.get_param("~channels")
        self.limits_deg: Dict[str, Dict[str, float]] = rospy.get_param("~limits_deg", {})

        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 1.0))
        self.timeout_behavior = rospy.get_param("~timeout_behavior", "hold")
        self.shutdown_behavior = rospy.get_param("~shutdown_behavior", "neutral")

        if not self.joint_names or not self.channels or len(self.joint_names) != len(self.channels):
            rospy.logfatal("joint_names and channels must be provided with equal length.")
            raise RuntimeError("Invalid joint mapping")

        # State
        self.num_joints = len(self.joint_names)
        self.name_to_idx = {n: i for i, n in enumerate(self.joint_names)}
        self.channel_by_idx = {i: ch for i, ch in enumerate(self.channels)}
        self.current_deg = [self.neutral_deg] * self.num_joints
        self.last_cmd_time = [rospy.get_time()] * self.num_joints
        self.enabled = self.enable_on_start

        # Device
        try:
            self.pca = PCA9685(self.busnum, self.address, self.oscillator_hz)
            self.pca.set_pwm_freq(self.pwm_freq)
        except Exception as e:
            rospy.logfatal("Failed to initialize PCA9685 on /dev/i2c-%d addr 0x%02X: %s",
                           self.busnum, self.address, str(e))
            raise

        # Apply neutral or off on start
        if self.enabled:
            self._apply_all(self.current_deg)
        else:
            self._apply_behavior_all("off")  # outputs off until enabled

        # ROS interfaces
        self.pub_joint = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.srv_set_angle = rospy.Service("set_angle", SetAngle, self.handle_set_angle)

        # Safety/utility services
        self.srv_enable = rospy.Service("enable", Trigger, self.handle_enable)
        self.srv_disable = rospy.Service("disable", Trigger, self.handle_disable)
        self.srv_home = rospy.Service("home", Trigger, self.handle_home)

        # Optional: accept JointState command to set multiple joints at once
        self.sub_cmd = rospy.Subscriber("command", JointState, self.handle_joint_command, queue_size=10)

        # Timers
        self.pub_timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)), self._publish_status)
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.05), self._watchdog_tick)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("PCA9685 Servo Controller started on /dev/i2c-%d addr 0x%02X, freq=%.1fHz, enabled=%s",
                      self.busnum, self.address, self.pwm_freq, str(self.enabled))

    def angle_to_count(self, angle_deg: float) -> int:
        # Map angle [0..180] -> [pulse_us_min..pulse_us_max]
        span_us = self.pulse_us_max - self.pulse_us_min
        frac = max(0.0, min(1.0, angle_deg / 180.0))
        pulse_us = self.pulse_us_min + frac * span_us
        counts = int(round((pulse_us / 1e6) * self.pwm_freq * 4096.0))
        return max(0, min(4095, counts))

    def apply_joint(self, idx: int, angle_deg: float):
        ch = self.channel_by_idx[idx]
        counts = self.angle_to_count(angle_deg)
        self.pca.set_pwm(ch, 0, counts)

    def handle_set_angle(self, req):
        try:
            idx = self.channels.index(req.channel)
        except ValueError:
            return SetAngleResponse(False, f"Channel {req.channel} not managed by this node")

        # Apply limits per joint
        name = self.joint_names[idx]
        lim = self.limits_deg.get(name, {"min": 0.0, "max": 180.0})
        target = max(lim.get("min", 0.0), min(lim.get("max", 180.0), float(req.angle_deg)))

        if not self.enabled:
            rospy.logwarn("Enabling outputs due to incoming set_angle command")
            self.enabled = True

        try:
            self.apply_joint(idx, target)
            self.current_deg[idx] = target
            self.last_cmd_time[idx] = rospy.get_time()
            return SetAngleResponse(True, f"Set {name} (ch {req.channel}) to {target:.1f} deg")
        except Exception as e:
            rospy.logerr("I2C error on set_angle: %s", str(e))
            return SetAngleResponse(False, f"I2C error: {str(e)}")

    def handle_joint_command(self, msg):
        now = rospy.get_time()
        changed = False
        if msg.name and len(msg.name) == len(msg.position):
            for nm, pos in zip(msg.name, msg.position):
                if nm in self.name_to_idx and pos is not None:
                    idx = self.name_to_idx[nm]
                    deg = math.degrees(pos) if abs(pos) < 6.3 else float(pos)  # radians or degrees
                    self._set_idx_with_limits(idx, deg, now)
                    changed = True
        elif not msg.name and msg.position:
            for idx, pos in enumerate(msg.position[:self.num_joints]):
                deg = math.degrees(pos) if abs(pos) < 6.3 else float(pos)
                self._set_idx_with_limits(idx, deg, now)
                changed = True
        else:
            rospy.logwarn("JointState command ignored: names/positions mismatch")

        if changed and not self.enabled:
            rospy.logwarn("Enabling outputs due to JointState command")
            self.enabled = True

    def handle_enable(self, req):
        try:
            self._apply_all(self.current_deg)
            self.enabled = True
            return TriggerResponse(success=True, message="Outputs enabled")
        except Exception as e:
            rospy.logerr("Enable failed: %s", str(e))
            return TriggerResponse(success=False, message=f"Enable failed: {str(e)}")

    def handle_disable(self, req):
        try:
            self._apply_behavior_all("off")
            self.enabled = False
            return TriggerResponse(success=True, message="Outputs disabled (OFF)")
        except Exception as e:
            rospy.logerr("Disable failed: %s", str(e))
            return TriggerResponse(success=False, message=f"Disable failed: {str(e)}")

    def handle_home(self, req):
        try:
            for idx in range(self.num_joints):
                self._move_to_neutral(idx)
            self.enabled = True
            return TriggerResponse(success=True, message="Moved all joints to neutral")
        except Exception as e:
            rospy.logerr("Home failed: %s", str(e))
            return TriggerResponse(success=False, message=f"Home failed: {str(e)}")

    def _set_idx_with_limits(self, idx: int, deg: float, now: float):
        nm = self.joint_names[idx]
        lim = self.limits_deg.get(nm, {"min": 0.0, "max": 180.0})
        target = max(lim.get("min", 0.0), min(lim.get("max", 180.0), deg))
        self.apply_joint(idx, target)
        self.current_deg[idx] = target
        self.last_cmd_time[idx] = now

    def _publish_status(self, _evt):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(self.joint_names)
        js.position = [math.radians(d) for d in self.current_deg]  # ROS convention: radians
        self.pub_joint.publish(js)

    def _watchdog_tick(self, _evt):
        if self.command_timeout_sec <= 0:
            return
        now = rospy.get_time()
        for idx in range(self.num_joints):
            if (now - self.last_cmd_time[idx]) > self.command_timeout_sec:
                if self.timeout_behavior == "hold":
                    pass
                elif self.timeout_behavior == "neutral":
                    self._move_to_neutral(idx)
                elif self.timeout_behavior == "off":
                    self._turn_off(idx)
                self.last_cmd_time[idx] = now

    def _move_to_neutral(self, idx: int):
        ch = self.channel_by_idx[idx]
        counts = self.angle_to_count(self.neutral_deg)
        self.pca.set_pwm(ch, 0, counts)
        self.current_deg[idx] = self.neutral_deg

    def _turn_off(self, idx: int):
        ch = self.channel_by_idx[idx]
        self.pca.set_off(ch)

    def _apply_behavior_all(self, behavior: str):
        for idx in range(self.num_joints):
            if behavior == "hold":
                self.apply_joint(idx, self.current_deg[idx])
            elif behavior == "neutral":
                self._move_to_neutral(idx)
            elif behavior == "off":
                self._turn_off(idx)

    def _apply_all(self, deg_list: List[float]):
        for idx, deg in enumerate(deg_list):
            self.apply_joint(idx, deg)

    def _on_shutdown(self):
        rospy.logwarn("Shutting down: applying %s behavior to all channels", self.shutdown_behavior)
        try:
            self._apply_behavior_all(self.shutdown_behavior)
            time.sleep(0.01)
            self.pca.close()
        except Exception as e:
            rospy.logerr("Error during shutdown: %s", str(e))


def main():
    rospy.init_node("pca9685_servo_controller", anonymous=False)
    try:
        ServoControllerNode()
        rospy.spin()
    except Exception as e:
        rospy.logfatal("Failed to start ServoControllerNode: %s", str(e))


if __name__ == "__main__":
    main()