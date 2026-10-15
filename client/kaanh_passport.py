"""Derive the kaanhbot controller's dynamic privileged-role password.

The controller uses its own local clock. Run this on a host with the same
date, hour and timezone as the controller, or supply the controller time.
"""

from __future__ import annotations

import argparse
import hashlib
from datetime import datetime


ROLES = ("Manufacturer", "Developer")


def password_sum(user: str, when: datetime) -> int:
    """Return the decimal integer hashed by the vendor's dynamic password path."""
    if user not in ROLES:
        raise ValueError(f"Dynamic password is only defined for {ROLES!r}")
    value = when.year + when.month + when.day
    if user == "Developer":
        value += when.hour
    return value


def dynamic_password(user: str = "Manufacturer", when: datetime | None = None) -> str:
    """Calculate a dynamic role password for the given local controller time."""
    if when is None:
        when = datetime.now().astimezone()
    return hashlib.md5(str(password_sum(user, when)).encode("ascii")).hexdigest()


def login_command(user: str = "Manufacturer", when: datetime | None = None) -> str:
    """Build a command for the controller command interface (not the action port)."""
    return f"login --user={user} --pwd={dynamic_password(user, when)}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Calculate a kaanhbot dynamic role password locally")
    parser.add_argument("--user", choices=ROLES, default="Manufacturer")
    parser.add_argument("--date", help="Controller-local date (YYYY-MM-DD); defaults to current local date")
    parser.add_argument("--hour", type=int, help="Controller-local hour (0-23); defaults to current local hour")
    parser.add_argument("--command", action="store_true", help="Print a controller login command")
    args = parser.parse_args(argv)

    if args.hour is not None and not 0 <= args.hour <= 23:
        parser.error("--hour must be between 0 and 23")
    if args.user == "Developer" and args.date and args.hour is None:
        parser.error("--hour is required with --date for Developer")

    when = datetime.now().astimezone()
    if args.date:
        try:
            selected_date = datetime.strptime(args.date, "%Y-%m-%d").date()
        except ValueError:
            parser.error("--date must be a valid date in YYYY-MM-DD format")
        when = when.replace(year=selected_date.year, month=selected_date.month, day=selected_date.day)
    if args.hour is not None:
        when = when.replace(hour=args.hour)

    print(login_command(args.user, when) if args.command else dynamic_password(args.user, when))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
