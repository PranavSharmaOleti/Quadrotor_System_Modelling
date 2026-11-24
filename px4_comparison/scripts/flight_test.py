#!/usr/bin/env python3
"""
Stage 7-8 validation: connect MAVSDK-Python to PX4 SITL X500, arm, takeoff,
hover, land. Exit 0 = all PASS.

Run: .venv/bin/python flight_test.py
"""
import asyncio
import sys
import time

from mavsdk import System
from mavsdk.action import ActionError

CONNECTION = "udpin://0.0.0.0:14540"  # PX4 SITL offboard/MAVSDK port (verified: PX4 binds local 14580, sends to remote 14540)
CONNECT_TIMEOUT_S = 10.0
HEALTH_TIMEOUT_S = 60.0   # EKF/GPS warm-up in SITL
HOVER_S = 10.0
MIN_AIRBORNE_ALT_M = 0.5
LAND_TIMEOUT_S = 40.0


def result(stage: int, passed: bool, msg: str) -> None:
    print(f"[{'PASS' if passed else 'FAIL'}] stage {stage}: {msg}", flush=True)


async def wait_connected(drone: System) -> float:
    t0 = time.monotonic()
    async for state in drone.core.connection_state():
        if state.is_connected:
            return time.monotonic() - t0
        if time.monotonic() - t0 > CONNECT_TIMEOUT_S:
            raise TimeoutError("no system within %.0fs" % CONNECT_TIMEOUT_S)
    raise TimeoutError("connection_state stream ended")


async def wait_position_ok(drone: System) -> float:
    t0 = time.monotonic()
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            return time.monotonic() - t0
        if time.monotonic() - t0 > HEALTH_TIMEOUT_S:
            raise TimeoutError("position health not OK within %.0fs" % HEALTH_TIMEOUT_S)
    raise TimeoutError("health stream ended")


async def relative_alt(drone: System) -> float:
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m


async def main() -> int:
    fails = 0
    drone = System()

    # ---- stage 7: connect ----
    print(f"[..] stage 7: connecting on {CONNECTION}", flush=True)
    try:
        await drone.connect(system_address=CONNECTION)
        dt = await asyncio.wait_for(wait_connected(drone), CONNECT_TIMEOUT_S + 5)
        result(7, dt <= CONNECT_TIMEOUT_S, f"system connected in {dt:.1f}s")
    except Exception as e:
        result(7, False, f"connect failed: {e}")
        return 1

    # ---- stage 8a: health (GPS/EKF) ----
    try:
        dt = await asyncio.wait_for(wait_position_ok(drone), HEALTH_TIMEOUT_S + 5)
        result(8, True, f"global position healthy after {dt:.1f}s")
    except Exception as e:
        result(8, False, f"health wait failed: {e}")
        return 1

    # ---- stage 8b: arm ----
    try:
        await drone.action.arm()
        async for armed in drone.telemetry.armed():
            result(8, armed, "vehicle ARMED" if armed else "arm reported disarmed")
            break
    except ActionError as e:
        result(8, False, f"arm failed: {e}")
        return 1

    # ---- stage 8c: takeoff ----
    try:
        await drone.action.set_takeoff_altitude(2.0)
        await drone.action.takeoff()
        print("[..] taking off...", flush=True)
        await asyncio.sleep(8)
        alt = await relative_alt(drone)
        airborne = alt >= MIN_AIRBORNE_ALT_M
        result(8, airborne, f"after takeoff: relative altitude = {alt:.2f} m (need >= {MIN_AIRBORNE_ALT_M})")
        if not airborne:
            fails += 1
    except ActionError as e:
        result(8, False, f"takeoff failed: {e}")
        return 1

    # ---- stage 8d: hover ----
    alts = []
    t_end = time.monotonic() + HOVER_S
    while time.monotonic() < t_end:
        alts.append(await relative_alt(drone))
        await asyncio.sleep(1.0)
    lo, hi = min(alts), max(alts)
    hover_ok = hi >= MIN_AIRBORNE_ALT_M and (hi - lo) < 1.0
    result(8, hover_ok,
           f"hover {HOVER_S:.0f}s: alt min={lo:.2f} max={hi:.2f} m over {len(alts)} samples")
    if not hover_ok:
        fails += 1

    # ---- stage 8e: land ----
    try:
        await drone.action.land()
        print("[..] landing...", flush=True)
        t0 = time.monotonic()
        landed = False
        while time.monotonic() - t0 < LAND_TIMEOUT_S:
            async for in_air in drone.telemetry.in_air():
                if not in_air:
                    landed = True
                break
            if landed:
                break
            await asyncio.sleep(1.0)
        result(8, landed, f"landed after {time.monotonic() - t0:.0f}s"
               if landed else f"still in air after {LAND_TIMEOUT_S:.0f}s")
        if not landed:
            fails += 1
    except ActionError as e:
        result(8, False, f"land failed: {e}")
        return 1

    # best-effort disarm
    try:
        await drone.action.disarm()
    except ActionError:
        pass

    print(f"\nRESULT: {'ALL STAGES 7-8 PASS' if fails == 0 else f'{fails} FAILURE(S)'}", flush=True)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    import os
    try:
        code = asyncio.run(main())
    except Exception as e:  # noqa: BLE001
        print(f"[FAIL] unhandled error: {e}", flush=True)
        code = 1
    sys.stdout.flush()
    # os._exit: gRPC client keeps non-daemon threads that block normal interpreter exit
    os._exit(code)
