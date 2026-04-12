#!/usr/bin/env python3
# coding: utf-8

import os
import sys
import unittest

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from AdaptivePID import LimitSwitchConfig, az_range_contains, validate_limit_range


class TestLimitRangeValidation(unittest.TestCase):
    def test_validate_az_ok(self):
        validate_limit_range(0, 360, "az")
        validate_limit_range(40, 20, "az")  # wrap range

    def test_validate_az_bad(self):
        with self.assertRaises(ValueError):
            validate_limit_range(-1, 10, "az")
        with self.assertRaises(ValueError):
            validate_limit_range(0, 361, "az")

    def test_validate_el_ok(self):
        validate_limit_range(0, 90, "el")

    def test_validate_el_bad(self):
        with self.assertRaises(ValueError):
            validate_limit_range(30, 20, "el")


class TestAzRangeContains(unittest.TestCase):
    def test_non_wrap(self):
        self.assertTrue(az_range_contains(50, 40, 90))
        self.assertFalse(az_range_contains(10, 40, 90))

    def test_wrap(self):
        self.assertTrue(az_range_contains(350, 300, 40))
        self.assertTrue(az_range_contains(20, 300, 40))
        self.assertFalse(az_range_contains(100, 300, 40))


class TestLimitSwitchConfig(unittest.TestCase):
    def test_config_validate_ok(self):
        cfg = LimitSwitchConfig(
            enabled=True,
            az_min_deg=300,
            az_max_deg=40,
            el_min_deg=0,
            el_max_deg=90,
            az_dir_sign=1,
            el_dir_sign=-1,
            safety_margin_deg=1.0,
        )
        cfg.validate()

    def test_config_validate_bad_dir(self):
        cfg = LimitSwitchConfig(az_dir_sign=0)
        with self.assertRaises(ValueError):
            cfg.validate()


if __name__ == "__main__":
    unittest.main()
