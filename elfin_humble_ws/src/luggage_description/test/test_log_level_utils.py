#!/usr/bin/env python3
"""Unit tests for runtime log-level selection.

The level has to be resolved before ``rospy.init_node``, so it comes from the
environment rather than a parameter. These tests pin that contract, including
the fallback behaviour -- a typo in the launch file must not silently mute a
node.
"""

import os
import unittest

from luggage_description.log_level_utils import (
    ENV_VAR,
    normalize_level_name,
    resolve_level_name,
    resolve_log_level,
)

ROSPY_DEBUG, ROSPY_INFO, ROSPY_WARN, ROSPY_ERROR = 1, 2, 4, 8


class TestLogLevelUtils(unittest.TestCase):
    def test_defaults_to_info_when_unset(self):
        self.assertEqual(resolve_log_level(env={}), ROSPY_INFO)
        self.assertEqual(resolve_level_name(env={}), "info")

    def test_selects_warn(self):
        env = {ENV_VAR: "warn"}
        self.assertEqual(resolve_log_level(env=env), ROSPY_WARN)
        self.assertEqual(resolve_level_name(env=env), "warn")

    def test_warning_is_an_alias_for_warn(self):
        self.assertEqual(resolve_log_level(env={ENV_VAR: "warning"}), ROSPY_WARN)
        self.assertEqual(normalize_level_name("WARNING"), "warn")

    def test_case_and_whitespace_insensitive(self):
        self.assertEqual(resolve_log_level(env={ENV_VAR: "  WARN "}), ROSPY_WARN)

    def test_unknown_value_falls_back_instead_of_muting(self):
        """A launch-file typo must not silently raise the level."""
        self.assertEqual(resolve_log_level(env={ENV_VAR: "quiet"}), ROSPY_INFO)
        self.assertEqual(resolve_level_name(env={ENV_VAR: "quiet"}), "info")

    def test_debug_and_error_are_available(self):
        self.assertEqual(resolve_log_level(env={ENV_VAR: "debug"}), ROSPY_DEBUG)
        self.assertEqual(resolve_log_level(env={ENV_VAR: "error"}), ROSPY_ERROR)

    def test_explicit_default_is_honoured(self):
        self.assertEqual(resolve_log_level(default="warn", env={}), ROSPY_WARN)


if __name__ == "__main__":
    unittest.main()
