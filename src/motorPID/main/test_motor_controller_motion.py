#!/usr/bin/env python3
# coding: utf-8

import os
import sys
import time
import unittest

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from az_el_controller import MotorController, StepperConfig


class DummyStore:
    def __init__(self):
        self.axes = {}
        self.fault = None

    def save_axis_position(self, axis_name: str, position_deg: float, reason: str):
        self.axes[axis_name] = {"position_deg": float(position_deg), "reason": reason}

    def load_axis_position(self, axis_name: str):
        data = self.axes.get(axis_name)
        return None if data is None else float(data["position_deg"])

    def save_fault(self, axis_name: str, code: str, message: str):
        self.fault = {"axis": axis_name, "code": code, "message": message}

    def clear_fault(self):
        self.fault = None


class DummyNotifier:
    def __init__(self, store: DummyStore):
        self.store = store

    def notify(self, axis_name: str, code: str, message: str):
        self.store.save_fault(axis_name, code, message)

    def clear(self):
        self.store.clear_fault()


class TestMotorControllerMotion(unittest.TestCase):
    def setUp(self):
        self.store = DummyStore()
        self.notifier = DummyNotifier(self.store)
        self.cfg = StepperConfig(
            name="TEST",
            step_pin=17,
            dir_pin=27,
            en_pin=22,
            enable_limits=False,
            soft_limit_min_deg=-1000.0,
            soft_limit_max_deg=1000.0,
            max_speed_sps=2200.0,
            accel_sps2=5000.0,
            microstep=8,
        )
        self.motor = MotorController(self.cfg, is_sim=True, store=self.store, notifier=self.notifier)
        time.sleep(0.03)

    def tearDown(self):
        self.motor.close()

    def _wait_speed_below(self, threshold: float = 1.0, timeout_s: float = 1.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if abs(self.motor.get_status()["current_speed"]) <= threshold:
                return True
            time.sleep(0.01)
        return False

    def test_single_step_movement(self):
        start = self.motor.get_internal_deg()
        ok = self.motor.single_step(direction=1, speed_sps=300.0)
        self.assertTrue(ok)
        end = self.motor.get_internal_deg()
        expected = 360.0 / (self.cfg.steps_per_rev * self.cfg.microstep)
        self.assertAlmostEqual(end - start, expected, delta=0.1)

    def test_continuous_rotation_and_hold(self):
        start = self.motor.get_internal_deg()
        self.motor.rotate(direction=1, speed_sps=600.0)
        time.sleep(0.12)
        mid = self.motor.get_internal_deg()
        self.assertGreater(mid - start, 0.1)

        self.motor.hold_position()
        self.assertTrue(self._wait_speed_below(threshold=0.5, timeout_s=1.0))
        hold_start = self.motor.get_internal_deg()
        time.sleep(0.12)
        hold_end = self.motor.get_internal_deg()
        self.assertAlmostEqual(hold_end, hold_start, delta=0.1)

    def test_move_deg_accuracy_plus_minus_0p1(self):
        start = self.motor.get_internal_deg()
        ok = self.motor.move_deg(5.0, speed_sps=450.0, timeout_s=3.0)
        self.assertTrue(ok)
        end = self.motor.get_internal_deg()
        self.assertAlmostEqual(end - start, 5.0, delta=0.1)

    def test_emergency_stop(self):
        self.motor.rotate(direction=1, speed_sps=700.0)
        time.sleep(0.05)
        self.motor.emergency_stop("test-stop")
        st = self.motor.get_status()
        self.assertTrue(st["fault_latched"])
        self.assertIn("test-stop", st["fault_msg"])
        self.assertAlmostEqual(st["target_speed"], 0.0, delta=1e-6)
        self.assertAlmostEqual(st["current_speed"], 0.0, delta=1e-6)

    def test_invalid_command(self):
        with self.assertRaises(ValueError):
            self.motor.rotate(direction=0, speed_sps=100.0)
        with self.assertRaises(ValueError):
            self.motor.set_target_speed(float("nan"))
        with self.assertRaises(ValueError):
            self.motor.move_steps(0.0)

    def test_high_speed_progress_no_stall(self):
        # Proxy missed-step test in simulation: command max speed and ensure progress + no stall fault.
        self.motor.rotate(direction=1, speed_sps=self.cfg.max_speed_sps)
        time.sleep(0.15)
        self.motor.hold_position()
        self.assertTrue(self._wait_speed_below(threshold=2.0, timeout_s=1.2))
        st = self.motor.get_status()
        self.assertFalse(st["fault_latched"])
        self.assertGreater(self.motor.get_internal_deg(), 0.2)


if __name__ == "__main__":
    unittest.main()
