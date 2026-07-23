#!/usr/bin/env python3

import re
import sys
from pathlib import Path


HEADER = Path(__file__).resolve().parents[1] / "HeroLauncher.hpp"
SOURCE = HEADER.read_text(encoding="utf-8")


def braced_body(source: str, opening_brace: int) -> str:
    depth = 1
    position = opening_brace + 1
    while depth and position < len(source):
        if source[position] == "{":
            depth += 1
        elif source[position] == "}":
            depth -= 1
        position += 1

    if depth:
        raise AssertionError("unterminated braced block")
    return source[opening_brace + 1 : position - 1]


def function_body(name: str) -> str:
    match = re.search(rf"\bvoid\s+{name}\s*\([^)]*\)\s*\{{", SOURCE)
    if match is None:
        raise AssertionError(f"missing {name}()")
    return braced_body(SOURCE, match.end() - 1)


def require(pattern: str, text: str, message: str) -> None:
    if re.search(pattern, text, re.DOTALL) is None:
        raise AssertionError(message)


def main() -> None:
    update = function_body("Update")
    control = function_body("Control")

    bare_updates = re.findall(
        r"^\s*(?:fric_motor_\s*\[[^]]+\]|motor_trig_)\s*->\s*Update\(\)\s*;",
        update,
        re.MULTILINE,
    )
    if bare_updates:
        raise AssertionError(
            "discarded motor Update() status: " + ", ".join(bare_updates)
        )

    require(
        r"bool\s+motors_online\s*=\s*true\s*;",
        update,
        "Update() must initialize the all-motor online fold",
    )
    require(
        r"const\s+auto\s+FRIC_STATUS\s*=\s*fric_motor_\s*\[\s*i\s*\]\s*"
        r"->\s*Update\(\)\s*;",
        update,
        "Update() must capture every friction motor status",
    )
    require(
        r"motors_online\s*=\s*FRIC_STATUS\s*==\s*LibXR::ErrorCode::OK\s*"
        r"&&\s*motors_online\s*;",
        update,
        "Update() must fold every friction motor status",
    )
    require(
        r"const\s+auto\s+TRIG_STATUS\s*=\s*motor_trig_\s*->\s*Update\(\)\s*;",
        update,
        "Update() must capture the trigger motor status",
    )
    require(
        r"motors_online_\s*=\s*motors_online\s*&&\s*"
        r"TRIG_STATUS\s*==\s*LibXR::ErrorCode::OK\s*;",
        update,
        "motors_online_ must combine friction and trigger status",
    )

    guard = re.match(r"\s*if\s*\(\s*!motors_online_\s*\)\s*\{", control)
    if guard is None:
        raise AssertionError("Control() must begin with the offline-motor guard")
    guard_body = braced_body(control, guard.end() - 1)
    require(
        r"motor_trig_\s*->\s*Relax\(\)\s*;",
        guard_body,
        "offline guard must relax the trigger motor",
    )
    require(
        r"for\s*\(\s*Motor\s*\*\s*const\s+MOTOR\s*:\s*fric_motor_\s*\)\s*\{\s*"
        r"MOTOR\s*->\s*Relax\(\)\s*;\s*\}",
        guard_body,
        "offline guard must relax every friction motor",
    )
    require(r"Reset\(\)\s*;", guard_body, "offline guard must reset launcher state")
    require(r"return\s*;", guard_body, "offline guard must return before PID output")

    print("HeroLauncher all-motor online regression: PASS")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as error:
        print(f"HeroLauncher all-motor online regression: FAIL: {error}", file=sys.stderr)
        sys.exit(1)
