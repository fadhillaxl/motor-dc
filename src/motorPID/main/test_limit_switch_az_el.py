#!/usr/bin/env python3
# coding: utf-8

import importlib.util
import os
import sys
import types
import unittest


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)


def _install_dummy_gpio():
    gpio = types.SimpleNamespace()
    gpio.BCM = 11
    gpio.OUT = 1
    gpio.IN = 0
    gpio.LOW = 0
    gpio.HIGH = 1
    gpio.PUD_UP = 1
    gpio.PUD_DOWN = 0
    gpio.setmode = lambda *args, **kwargs: None
    gpio.setwarnings = lambda *args, **kwargs: None
    gpio.setup = lambda *args, **kwargs: None
    gpio.input = lambda *args, **kwargs: gpio.LOW
    gpio.output = lambda *args, **kwargs: None
    gpio.cleanup = lambda *args, **kwargs: None

    rpi_mod = types.ModuleType("RPi")
    rpi_mod.GPIO = gpio
    sys.modules["RPi"] = rpi_mod
    sys.modules["RPi.GPIO"] = gpio


_install_dummy_gpio()


def _install_dummy_wt901_sdk():
    lib_mod = types.ModuleType("lib")
    device_model_mod = types.ModuleType("lib.device_model")

    class DummyDeviceModel:
        def __init__(self, *args, **kwargs):
            pass

    device_model_mod.DeviceModel = DummyDeviceModel

    data_proc_mod = types.ModuleType("lib.data_processor.roles.jy901s_dataProcessor")

    class DummyJY901SDataProcessor:
        pass

    data_proc_mod.JY901SDataProcessor = DummyJY901SDataProcessor

    proto_mod = types.ModuleType("lib.protocol_resolver.roles.protocol_485_resolver")

    class DummyProtocol485Resolver:
        pass

    proto_mod.Protocol485Resolver = DummyProtocol485Resolver

    sys.modules["lib"] = lib_mod
    sys.modules["lib.device_model"] = device_model_mod
    sys.modules["lib.data_processor.roles.jy901s_dataProcessor"] = data_proc_mod
    sys.modules["lib.protocol_resolver.roles.protocol_485_resolver"] = proto_mod


_install_dummy_wt901_sdk()

import az_el_controller as azel_mod


def _load_keyboard_module():
    path = os.path.join(THIS_DIR, "keyboard-motor-stepper.py")
    spec = importlib.util.spec_from_file_location("keyboard_motor_stepper_mod", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


keyboard_mod = _load_keyboard_module()
MODULES_UNDER_TEST = [azel_mod, keyboard_mod]


class TestAZLimitSwitch(unittest.TestCase):
    def test_shortest_path_basic(self):
        for mod in MODULES_UNDER_TEST:
            azls = mod.AZLimitSwitch(280.0)
            decision = azls.calculateShortestPath(10.0, 20.0)
            self.assertTrue(decision["allowed"])
            self.assertEqual(decision["direction"], 1)
            self.assertAlmostEqual(decision["distance_deg"], 10.0, delta=1e-6)

    def test_limit_280_prevents_jump_270_to_350(self):
        for mod in MODULES_UNDER_TEST:
            azls = mod.AZLimitSwitch(280.0)
            decision = azls.validateMovement(270.0, 350.0, current_direction=1)
            self.assertTrue(decision["allowed"])
            self.assertEqual(decision["direction"], -1)
            self.assertTrue(decision["cw_cross_limit"])
            self.assertFalse(decision["ccw_cross_limit"])

    def test_wrap_around_0_360(self):
        for mod in MODULES_UNDER_TEST:
            azls = mod.AZLimitSwitch(280.0)
            decision = azls.calculateShortestPath(359.0, 1.0)
            self.assertTrue(decision["allowed"])
            self.assertEqual(decision["direction"], 1)
            self.assertAlmostEqual(decision["distance_deg"], 2.0, delta=1e-6)

    def test_reverse_direction(self):
        for mod in MODULES_UNDER_TEST:
            self.assertEqual(mod.AZLimitSwitch.reverseDirection(1), -1)
            self.assertEqual(mod.AZLimitSwitch.reverseDirection(-1), 1)
            self.assertEqual(mod.AZLimitSwitch.reverseDirection(0), 0)

    def test_detect_direction(self):
        for mod in MODULES_UNDER_TEST:
            azls = mod.AZLimitSwitch(280.0)
            self.assertEqual(azls.detectMovementDirection(359.0, 1.0), 1)
            self.assertEqual(azls.detectMovementDirection(1.0, 359.0), -1)
            self.assertEqual(azls.detectMovementDirection(10.0, 10.02, eps_deg=0.1), 0)


class TestELLimitSwitch(unittest.TestCase):
    def test_validate_in_range(self):
        for mod in MODULES_UNDER_TEST:
            ells = mod.ELLimitSwitch(0.0, 90.0)
            decision = ells.validateElevation(45.0, current_el_deg=30.0)
            self.assertTrue(decision["allowed"])
            self.assertEqual(decision["reason"], "ok")

    def test_reject_out_of_range(self):
        for mod in MODULES_UNDER_TEST:
            ells = mod.ELLimitSwitch(0.0, 90.0)
            decision = ells.validateElevation(95.0, current_el_deg=30.0)
            self.assertFalse(decision["allowed"])
            self.assertEqual(decision["reason"], "target_out_of_range")
            self.assertEqual(decision["clamped_target_deg"], 90.0)

    def test_soft_stop_upper_and_lower(self):
        for mod in MODULES_UNDER_TEST:
            ells = mod.ELLimitSwitch(0.0, 90.0)
            upper = ells.validateElevation(90.0, current_el_deg=89.95)
            lower = ells.validateElevation(0.0, current_el_deg=0.02)
            self.assertFalse(upper["allowed"])
            self.assertEqual(upper["reason"], "soft_stop_upper")
            self.assertFalse(lower["allowed"])
            self.assertEqual(lower["reason"], "soft_stop_lower")


if __name__ == "__main__":
    unittest.main()
