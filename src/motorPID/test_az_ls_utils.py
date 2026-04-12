#!/usr/bin/env python3
# coding: utf-8

import unittest
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from az_ls_utils import az_ls_allows_motion, validate_az_ls


class TestAzLsValidation(unittest.TestCase):
    def test_validate_accepts_valid_range(self):
        self.assertEqual(validate_az_ls(0), 0.0)
        self.assertEqual(validate_az_ls(40), 40.0)
        self.assertEqual(validate_az_ls(180), 180.0)
        self.assertEqual(validate_az_ls(359), 359.0)
        self.assertEqual(validate_az_ls(360), 360.0)

    def test_validate_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            validate_az_ls(-1)
        with self.assertRaises(ValueError):
            validate_az_ls(361)


class TestAzLsMotionRules(unittest.TestCase):
    def test_az_ls_zero_allows_full_rotation(self):
        self.assertTrue(az_ls_allows_motion(10, 20, 0))
        self.assertTrue(az_ls_allows_motion(359, 1, 0))
        self.assertTrue(az_ls_allows_motion(1, 359, 0))

    def test_az_ls_40_blocks_crossing_boundary(self):
        self.assertTrue(az_ls_allows_motion(5, 15, 40))
        self.assertTrue(az_ls_allows_motion(320, 330, 40))
        self.assertFalse(az_ls_allows_motion(39.5, 40.5, 40))
        self.assertFalse(az_ls_allows_motion(40.5, 39.5, 40))

    def test_az_ls_180_blocks_crossing_boundary(self):
        self.assertTrue(az_ls_allows_motion(120, 130, 180))
        self.assertFalse(az_ls_allows_motion(179.9, 180.1, 180))
        self.assertFalse(az_ls_allows_motion(180.1, 179.9, 180))

    def test_az_ls_359_blocks_crossing_boundary(self):
        self.assertTrue(az_ls_allows_motion(100, 120, 359))
        self.assertFalse(az_ls_allows_motion(358.9, 359.1, 359))
        self.assertFalse(az_ls_allows_motion(359.1, 358.9, 359))


if __name__ == "__main__":
    unittest.main()
