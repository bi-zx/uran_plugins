"""Command line calibration and discovery tools for the SO101 arm.

The commands deliberately use only the local Feetech driver, so they can run
on the CyberDog image without ROS or the LeRobot Python package.
"""
from __future__ import print_function

import argparse
import os

from .calibration import JOINT_NAMES, empty_calibration, load_calibration, save_calibration
from .feetech_bus import (FeetechBus, FeetechError, PRESENT_POSITION_ADDRESS,
                          discover_port, probe)


def _resolve_port(value, baudrate):
    value = str(value or "auto").strip()
    if value.lower() in ("", "auto", "discover"):
        value = discover_port(baudrate, range(1, 7))
        if not value:
            raise FeetechError("no Feetech servo bus found during automatic port scan")
    return value


def _common(parser):
    parser.add_argument("--port", default="auto", help="serial device or auto")
    parser.add_argument("--baudrate", type=int, default=1000000)


def command_probe(args):
    port = _resolve_port(args.port, args.baudrate)
    found = probe(port, args.baudrate, range(1, 7))
    for motor_id in sorted(found):
        item = found[motor_id]
        print("id={} model={} position_raw={}".format(motor_id, item["model"], item["position_raw"]))
    return 0 if found else 2


def _read_positions(args):
    bus = FeetechBus(_resolve_port(args.port, args.baudrate), args.baudrate)
    bus.open()
    try:
        return bus.read_positions(range(1, 7))
    finally:
        bus.close()


def command_record_center(args):
    positions = _read_positions(args)
    data = load_calibration(args.output)
    if not data.get("joints"):
        data = empty_calibration()
    for name, motor_id in zip(JOINT_NAMES, range(1, 7)):
        data["joints"][name]["raw_center"] = int(positions[motor_id])
    save_calibration(args.output, data)
    print("saved center positions to {}".format(args.output))
    return 0


def command_record_limits(args):
    port = _resolve_port(args.port, args.baudrate)
    data = load_calibration(args.output)
    if not data.get("joints"):
        raise RuntimeError("calibration file has no joints; run record-center first")
    bus = FeetechBus(port, args.baudrate)
    bus.open()
    try:
        print("Disable torque and leave a visible margin from every hard stop.")
        for name, motor_id in zip(JOINT_NAMES, range(1, 7)):
            print("\nJoint {} (ID {})".format(name, motor_id))
            input("Move to the safe LOWER endpoint, then press ENTER: ")
            raw_a = int(bus.read(motor_id, PRESENT_POSITION_ADDRESS, 2, signed=True))
            input("Move to the safe UPPER endpoint, then press ENTER: ")
            raw_b = int(bus.read(motor_id, PRESENT_POSITION_ADDRESS, 2, signed=True))
            raw_min, raw_max = sorted((raw_a, raw_b))
            if raw_min == raw_max:
                raise RuntimeError("{} lower and upper readings are equal".format(name))
            answer = input("Does the desired positive joint direction increase raw? [Y/n]: ")
            direction = -1 if answer.strip().lower() in ("n", "no", "0", "-") else 1
            item = data["joints"][name]
            item.update(raw_min=raw_min, raw_max=raw_max, direction=direction)
            save_calibration(args.output, data)
            print("{}: raw_min={} raw_max={} direction={}".format(name, raw_min, raw_max, direction))
    finally:
        bus.close()
    data["enabled"] = True
    save_calibration(args.output, data)
    print("calibration complete: {}".format(args.output))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="SO101 Feetech probe and calibration")
    sub = parser.add_subparsers(dest="command")
    for name, help_text, function in (("probe", "probe motor IDs", command_probe),
                                      ("record-center", "record current centers", command_record_center),
                                      ("record-limits", "interactively record limits", command_record_limits)):
        child = sub.add_parser(name, help=help_text)
        _common(child)
        child.add_argument("--output", default="/SDCARD/URAN/config/uran_arm_calibration.yaml")
        child.set_defaults(function=function)
    args = parser.parse_args(argv)
    if not getattr(args, "command", None):
        parser.print_help()
        return 2
    try:
        return args.function(args)
    except (KeyboardInterrupt, EOFError):
        print("calibration cancelled")
        return 130
    except Exception as exc:
        print("{} failed: {}".format(args.command, exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
