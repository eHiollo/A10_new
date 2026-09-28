"""Offline checks for the controller's dynamic password arithmetic."""

from __future__ import annotations

import contextlib
import io
import sys
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "client"))

from kaanh_passport import dynamic_password, login_command, main, password_sum  # noqa: E402


class PassportTest(unittest.TestCase):
    def test_manufacturer_changes_with_date_not_hour(self):
        morning = datetime(2026, 5, 26, 8)
        evening = morning.replace(hour=23)
        self.assertEqual(password_sum("Manufacturer", morning), 2057)
        self.assertEqual(dynamic_password("Manufacturer", morning), dynamic_password("Manufacturer", evening))
        self.assertNotEqual(
            dynamic_password("Manufacturer", morning),
            dynamic_password("Manufacturer", morning.replace(day=27)),
        )

    def test_developer_includes_local_hour(self):
        instant = datetime(2026, 5, 26, 0, tzinfo=timezone.utc)
        controller_time = instant.astimezone(timezone(timedelta(hours=8)))
        self.assertEqual(password_sum("Developer", instant), 2057)
        self.assertEqual(password_sum("Developer", controller_time), 2065)
        self.assertNotEqual(dynamic_password("Developer", instant), dynamic_password("Developer", controller_time))

    def test_rejects_stored_password_roles(self):
        with self.assertRaises(ValueError):
            dynamic_password("Administrator", datetime(2026, 5, 26))

    def test_login_command_shape(self):
        command = login_command("Manufacturer", datetime(2026, 5, 26))
        self.assertRegex(command, r"^login --user=Manufacturer --pwd=[0-9a-f]{32}$")

    def test_historical_developer_date_requires_hour(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(["--user", "Developer", "--date", "2026-05-26"])
        self.assertEqual(error.exception.code, 2)

    def test_invalid_hour_is_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(["--hour", "24"])
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
