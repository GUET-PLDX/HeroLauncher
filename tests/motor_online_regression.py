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


def function_body(source: str, name: str) -> str:
    match = re.search(rf"\bvoid\s+{name}\s*\([^)]*\)\s*\{{", source)
    if match is None:
        raise AssertionError(f"missing {name}()")
    return braced_body(source, match.end() - 1)


def require(pattern: str, text: str, message: str) -> None:
    if re.search(pattern, text, re.DOTALL) is None:
        raise AssertionError(message)


def validate(source: str) -> None:
    thread_func = function_body(source, "ThreadFunc")
    update = function_body(source, "Update")
    control = function_body(source, "Control")
    set_mode = function_body(source, "SetMode")
    reset = function_body(source, "Reset")

    require(
        r"bool\s+motor_fault_latched_\s*=\s*false\s*;",
        source,
        "motor fault latch must start clear",
    )

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
        r"fric_motor_status_\s*\[\s*i\s*\]\s*=\s*FRIC_STATUS\s*;",
        update,
        "Update() must retain each friction motor status for diagnostics",
    )
    require(
        r"previous_fric_motor_status_\s*\[\s*i\s*\]\s*=\s*"
        r"fric_motor_status_\s*\[\s*i\s*\]\s*;",
        update,
        "Update() must preserve each previous friction motor status",
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
        r"trig_motor_status_\s*=\s*TRIG_STATUS\s*;",
        update,
        "Update() must retain trigger status for diagnostics",
    )
    require(
        r"previous_trig_motor_status_\s*=\s*trig_motor_status_\s*;",
        update,
        "Update() must preserve the previous trigger status",
    )
    require(
        r"motors_online_\s*=\s*motors_online\s*&&\s*"
        r"TRIG_STATUS\s*==\s*LibXR::ErrorCode::OK\s*;",
        update,
        "motors_online_ must combine friction and trigger status",
    )
    require(
        r"LogMotorFaultTransitions\(\)\s*;\s*"
        r"if\s*\(\s*!motors_online_\s*&&\s*!motor_fault_latched_\s*\)\s*\{\s*"
        r"motor_fault_latched_\s*=\s*true\s*;\s*Reset\(\)\s*;\s*\}",
        update,
        "Update() must diagnose per-motor transitions and latch the aggregate fault",
    )
    if re.search(r"motor_fault_latched_\s*=\s*false", update):
        raise AssertionError("online Update() must not automatically clear the fault latch")

    require(
        r"mutex_\.Lock\(\)\s*;\s*self->Update\(\)\s*;\s*self->Solve\(\)\s*;\s*"
        r"self->Control\(\)\s*;\s*self->mutex_\.Unlock\(\)\s*;",
        thread_func,
        "ThreadFunc() must keep Update, Solve, and Control under one mutex ownership",
    )

    guard = re.match(
        r"\s*if\s*\(\s*motor_fault_latched_\s*\|\|\s*!motors_online_\s*\)\s*\{",
        control,
    )
    if guard is None:
        raise AssertionError("Control() must begin with the latched motor-fault guard")
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

    require(
        r"const\s+auto\s+REQUESTED_MODE\s*=\s*"
        r"static_cast<LauncherEvent>\(mode\)\s*;\s*"
        r"if\s*\(\s*!motors_online_\s*\)\s*\{\s*"
        r"motor_fault_latched_\s*=\s*true\s*;\s*"
        r"if\s*\(\s*REQUESTED_MODE\s*!=\s*LauncherEvent::SET_FRICMODE_RELAX\s*&&\s*"
        r"REQUESTED_MODE\s*!=\s*LauncherEvent::SET_FRICMODE_SAFE\s*\)\s*\{\s*"
        r"return\s*;\s*\}\s*launcher_state_\s*=\s*REQUESTED_MODE\s*;\s*return\s*;\s*\}\s*"
        r"motor_fault_latched_\s*=\s*false\s*;\s*"
        r"launcher_state_\s*=\s*REQUESTED_MODE\s*;",
        set_mode,
        "SetMode() must latch startup/offline state, reject unsafe prearm, and only rearm online",
    )

    require(
        r"launcher_state_\s*=\s*LauncherEvent::SET_FRICMODE_RELAX\s*;",
        reset,
        "Reset() must clear the launcher mode to RELAX",
    )
    require(
        r"trig_mode_\s*=\s*TrigMode::RELAX\s*;",
        reset,
        "Reset() must clear the trigger mode to RELAX",
    )
    if re.search(r"motor_fault_latched_\s*=\s*false", reset):
        raise AssertionError("Reset() must preserve the motor fault latch")
    for flag in ("fire_flag_", "enable_fire_", "mark_launch_"):
        require(
            rf"{flag}\s*=\s*false\s*;",
            reset,
            f"Reset() must clear {flag}",
        )

    log_motor_fault = function_body(source, "LogMotorFaultTransitions")
    require(
        r"fric_motor_status_\s*\[\s*i\s*\]\s*!=\s*"
        r"previous_fric_motor_status_\s*\[\s*i\s*\]\s*&&\s*"
        r"fric_motor_status_\s*\[\s*i\s*\]\s*!=\s*LibXR::ErrorCode::OK",
        log_motor_fault,
        "diagnostics must log every friction motor transition to non-OK",
    )
    require(
        r"XR_LOG_ERROR\([^;]*i[^;]*fric_motor_status_\s*\[\s*i\s*\]",
        log_motor_fault,
        "friction fault log must contain motor index and status",
    )
    require(
        r"trig_motor_status_\s*!=\s*previous_trig_motor_status_\s*&&\s*"
        r"trig_motor_status_\s*!=\s*LibXR::ErrorCode::OK",
        log_motor_fault,
        "diagnostics must log every trigger transition to non-OK",
    )
    require(
        r"XR_LOG_ERROR\([^;]*trig_motor_status_",
        log_motor_fault,
        "trigger fault log must contain its status",
    )


def require_mutation_killed(description: str, mutated_source: str) -> None:
    if mutated_source == SOURCE:
        raise AssertionError(f"mutation target was not found: {description}")
    try:
        validate(mutated_source)
    except AssertionError:
        return
    raise AssertionError(f"mutation survived: {description}")


def main() -> None:
    validate(SOURCE)

    require_mutation_killed(
        "online recovery bypassed the fault latch",
        SOURCE.replace(
            "if (motor_fault_latched_ || !motors_online_)",
            "if (!motors_online_)",
            1,
        ),
    )
    require_mutation_killed(
        "startup READY request bypassed the offline gate",
        SOURCE.replace(
            "if (!motors_online_) {\n      motor_fault_latched_ = true;",
            "if (motor_fault_latched_ && !motors_online_) {\n"
            "      motor_fault_latched_ = true;",
            1,
        ),
    )
    require_mutation_killed(
        "second motor fault transition was not diagnosed",
        SOURCE.replace(
            "fric_motor_status_[i] != previous_fric_motor_status_[i] &&",
            "motor_fault_latched_ == false &&",
            1,
        ),
    )
    require_mutation_killed(
        "Control ran after releasing the state mutex",
        SOURCE.replace(
            "self->Control();\n      self->mutex_.Unlock();",
            "self->mutex_.Unlock();\n      self->Control();",
            1,
        ),
    )

    print("HeroLauncher all-motor online regression: PASS (4 mutations killed)")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as error:
        print(f"HeroLauncher all-motor online regression: FAIL: {error}", file=sys.stderr)
        sys.exit(1)
