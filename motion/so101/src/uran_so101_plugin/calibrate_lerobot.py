#!/usr/bin/env python3
"""Standalone SO-101 Feetech calibration tool.

This script intentionally does not import LeRobot.  It talks to the six
STS3215 servos through the public ``feetech-servo-sdk`` package and writes a
calibration JSON file with the same fields used by LeRobot:

    id, drive_mode, homing_offset, range_min, range_max

The SO-101 motor IDs are the factory IDs used by the follower arm:
1 shoulder_pan, 2 shoulder_lift, 3 elbow_flex, 4 wrist_flex,
5 wrist_roll, and 6 gripper.
"""

import argparse
import json
import platform
import select
import sys
import time
import os
from pathlib import Path
from types import MethodType
from typing import Dict, Iterable, List, Optional, Tuple


MOTOR_NAMES: Dict[int, str] = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
WRIST_ROLL_ID = 5
ENCODER_MAX = 4095
HALF_TURN = ENCODER_MAX // 2

# STS3215 control-table addresses (protocol 0).
ADDR_MIN_POSITION = 9
ADDR_MAX_POSITION = 11
ADDR_HOMING_OFFSET = 31
ADDR_OPERATING_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56


def _require_sdk():
    """Import the SDK lazily so ``--help`` still works without dependencies."""

    try:
        import scservo_sdk as scs  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on local machine
        raise SystemExit(
            "缺少 Feetech SDK。请在当前 Python 环境执行:\n"
            "  python -m pip install 'uran-so101-plugin[calibration]'"
        ) from exc
    return scs


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    """Decode Feetech's sign-magnitude register representation."""

    mask = 1 << sign_bit
    return -(value & ~mask) if value & mask else value


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    """Encode a signed value for a Feetech sign-magnitude register."""

    mask = 1 << sign_bit
    return ((-value) | mask) if value < 0 else value


def patch_packet_timeout(port_handler) -> None:
    """Use a conservative timeout for the PyPI SDK's protocol-0 reader."""

    def set_packet_timeout(self, packet_length: int) -> None:
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (
            self.tx_time_per_byte * 3.0
        ) + 50

    port_handler.setPacketTimeout = MethodType(set_packet_timeout, port_handler)


class FeetechBus:
    """Small protocol-0 wrapper for the six SO-101 STS3215 servos."""

    def __init__(self, port_name: str, baudrate: int, scs) -> None:
        self.scs = scs
        self.port = scs.PortHandler(port_name)
        patch_packet_timeout(self.port)
        self.packet = scs.PacketHandler(0)
        if not self.port.setBaudRate(baudrate):
            raise RuntimeError(f"无法设置串口波特率 {baudrate}")
        if not self.port.is_open:
            raise RuntimeError(f"无法打开串口 {port_name}")

    def close(self) -> None:
        if self.port.is_open:
            self.port.closePort()

    def _check(self, result: int, error: int, motor_id: int, operation: str) -> None:
        if result != self.scs.COMM_SUCCESS:
            reason = self.packet.getTxRxResult(result)
            raise RuntimeError(f"舵机 {motor_id} {operation} 通信失败: {reason}")
        if error:
            reason = self.packet.getRxPacketError(error)
            raise RuntimeError(f"舵机 {motor_id} {operation} 返回错误: {reason}")

    def ping(self, motor_id: int) -> int:
        model, result, error = self.packet.ping(self.port, motor_id)
        self._check(result, error, motor_id, "ping")
        return model

    def write1(self, motor_id: int, address: int, value: int, operation: str) -> None:
        result, error = self.packet.write1ByteTxRx(self.port, motor_id, address, value)
        self._check(result, error, motor_id, operation)

    def write2(self, motor_id: int, address: int, value: int, operation: str) -> None:
        result, error = self.packet.write2ByteTxRx(self.port, motor_id, address, value)
        self._check(result, error, motor_id, operation)

    def read2(self, motor_id: int, address: int, operation: str) -> int:
        value, result, error = self.packet.read2ByteTxRx(self.port, motor_id, address)
        self._check(result, error, motor_id, operation)
        return int(value)

    def read_position(self, motor_id: int) -> int:
        # Present_Position is a 12-bit unsigned position for the normal SO-101
        # range.  Decode bit 15 as the SDK does, for completeness.
        raw = self.read2(motor_id, ADDR_PRESENT_POSITION, "读取位置")
        return decode_sign_magnitude(raw, 15)

    def disable_torque(self, motor_id: int) -> None:
        self.write1(motor_id, ADDR_TORQUE_ENABLE, 0, "关闭力矩")
        self.write1(motor_id, ADDR_LOCK, 0, "解除写保护")

    def enable_torque(self, motor_id: int) -> None:
        self.write1(motor_id, ADDR_TORQUE_ENABLE, 1, "开启力矩")
        self.write1(motor_id, ADDR_LOCK, 1, "开启写保护")

    def sync_write_positions(self, positions: Dict[int, int]) -> None:
        """Send all goal positions in one protocol-0 sync-write packet."""

        writer = self.scs.GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, 2
        )
        for motor_id, position in positions.items():
            position = int(max(0, min(ENCODER_MAX, position)))
            if not writer.addParam(
                motor_id, [position & 0xFF, (position >> 8) & 0xFF]
            ):
                raise RuntimeError(f"无法加入舵机 {motor_id} 的同步写入参数")
        result = writer.txPacket()
        if result != self.scs.COMM_SUCCESS:
            reason = self.packet.getTxRxResult(result)
            raise RuntimeError(f"同步写入目标位置失败: {reason}")
        writer.clearParam()

    def reset_limits(self, motor_id: int) -> None:
        self.write2(motor_id, ADDR_HOMING_OFFSET, 0, "清零中值偏移")
        self.write2(motor_id, ADDR_MIN_POSITION, 0, "清零最小限位")
        self.write2(motor_id, ADDR_MAX_POSITION, ENCODER_MAX, "设置最大限位")

    def set_homing_offset(self, motor_id: int, offset: int) -> None:
        # Homing_Offset is a signed sign-magnitude register with bit 11 as
        # sign.  This is the same encoding used by LeRobot's Feetech driver.
        encoded = encode_sign_magnitude(offset, 11)
        self.write2(motor_id, ADDR_HOMING_OFFSET, encoded, "写入中值偏移")

    def set_limits(self, motor_id: int, minimum: int, maximum: int) -> None:
        self.write2(motor_id, ADDR_MIN_POSITION, minimum, "写入最小限位")
        self.write2(motor_id, ADDR_MAX_POSITION, maximum, "写入最大限位")


def enter_pressed() -> bool:
    """Non-blocking Enter detection for the range-recording loop."""

    if platform.system() == "Windows":  # pragma: no cover - Windows only
        import msvcrt

        if msvcrt.kbhit():
            return msvcrt.getch() in (b"\r", b"\n")
        return False

    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return False
    return sys.stdin.readline().strip() == ""


def read_positions(bus: FeetechBus, motor_ids: Iterable[int]) -> Dict[int, int]:
    positions: Dict[int, int] = {}
    for motor_id in motor_ids:
        # A transient serial packet error should not silently create a false
        # range.  Retry a few times, then abort with the motor ID visible.
        last_error: Optional[Exception] = None
        for _ in range(3):
            try:
                positions[motor_id] = bus.read_position(motor_id)
                break
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.01)
        else:
            assert last_error is not None
            raise last_error
    return positions


def render_ranges(
    positions: Dict[int, int], mins: Dict[int, int], maxes: Dict[int, int], first: bool
) -> None:
    if not first:
        print(f"\033[{len(positions) + 2}A", end="")
    print("-------------------------------------------")
    print(f"{'NAME':<15} | {'MIN':>6} | {'POS':>6} | {'MAX':>6}")
    for motor_id, position in positions.items():
        print(
            f"{MOTOR_NAMES[motor_id]:<15} | {mins[motor_id]:6d} | "
            f"{position:6d} | {maxes[motor_id]:6d}"
        )
    sys.stdout.flush()


def record_ranges(bus: FeetechBus, motor_ids: List[int]) -> Tuple[Dict[int, int], Dict[int, int]]:
    print("\n开始记录运动范围。请依次缓慢移动每个舵机接近两端机械限位。")
    print("完成后在当前终端按 Enter；不要撞击限位，也不要移动 wrist_roll。\n")
    positions = read_positions(bus, motor_ids)
    mins = positions.copy()
    maxes = positions.copy()
    first = True

    while True:
        positions = read_positions(bus, motor_ids)
        for motor_id, position in positions.items():
            mins[motor_id] = min(mins[motor_id], position)
            maxes[motor_id] = max(maxes[motor_id], position)
        render_ranges(positions, mins, maxes, first)
        first = False
        if enter_pressed():
            break
        time.sleep(0.05)

    invalid = [MOTOR_NAMES[motor_id] for motor_id in motor_ids if mins[motor_id] == maxes[motor_id]]
    if invalid:
        raise RuntimeError(f"这些舵机没有观察到运动范围: {', '.join(invalid)}")
    return mins, maxes


def write_calibration(
    output: Path,
    offsets: Dict[int, int],
    mins: Dict[int, int],
    maxes: Dict[int, int],
) -> None:
    data = {}
    for motor_id, name in MOTOR_NAMES.items():
        data[name] = {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": int(offsets[motor_id]),
            "range_min": int(mins[motor_id]),
            "range_max": int(maxes[motor_id]),
        }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=4, ensure_ascii=False)
        file.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="独立的 SO-101 STS3215 舵机校准工具")
    parser.add_argument("--port", required=True, help="舵机串口")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="总线波特率 (默认: 1000000)")
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="输出 JSON 路径",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="输出文件已存在时直接重新校准，不再询问",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output: Path = args.output.expanduser().resolve()

    if output.exists() and not args.force:
        answer = input(
            f"校准文件已存在: {output}\n"
            "直接按 Enter 保留旧文件；输入 c 后按 Enter 重新校准: "
        )
        if answer.strip().lower() != "c":
            print("保留旧校准文件，未连接机械臂。")
            return 0

    scs = _require_sdk()
    motor_ids = list(MOTOR_NAMES)
    bus: Optional[FeetechBus] = None
    try:
        bus = FeetechBus(args.port, args.baudrate, scs)
        print(f"已打开 {args.port}，波特率 {args.baudrate}")
        for motor_id in motor_ids:
            model = bus.ping(motor_id)
            print(f"  ID {motor_id}: {MOTOR_NAMES[motor_id]} (model {model})")

        print("\n正在关闭舵机力矩并清除旧限位。")
        for motor_id in motor_ids:
            bus.disable_torque(motor_id)
            bus.write1(motor_id, ADDR_OPERATING_MODE, 0, "设置位置模式")
            bus.reset_limits(motor_id)

        input(
            "\n请将所有舵机手动放到各自运动范围的中间位置，"
            "然后在当前终端按 Enter..."
        )
        actual_positions = read_positions(bus, motor_ids)
        offsets = {motor_id: actual_positions[motor_id] - HALF_TURN for motor_id in motor_ids}
        for motor_id in motor_ids:
            bus.set_homing_offset(motor_id, offsets[motor_id])

        print("\n中值已记录:")
        for motor_id in motor_ids:
            print(
                f"  {MOTOR_NAMES[motor_id]:<15} "
                f"raw={actual_positions[motor_id]:4d} "
                f"homing_offset={offsets[motor_id]:5d}"
            )

        range_ids = [motor_id for motor_id in motor_ids if motor_id != WRIST_ROLL_ID]
        mins, maxes = record_ranges(bus, range_ids)
        mins[WRIST_ROLL_ID] = 0
        maxes[WRIST_ROLL_ID] = ENCODER_MAX

        for motor_id in motor_ids:
            bus.set_limits(motor_id, mins[motor_id], maxes[motor_id])
        write_calibration(output, offsets, mins, maxes)

        print(f"\n校准完成，已写入舵机限位和中值偏移。文件: {output}")
        print("注意: wrist_roll 按 LeRobot 规则使用完整范围 [0, 4095]。")
        return 0
    except KeyboardInterrupt:
        print("\n已中断。舵机力矩保持关闭，请先确认机械臂安全。", file=sys.stderr)
        return 130
    finally:
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
