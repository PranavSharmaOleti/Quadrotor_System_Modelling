#!/usr/bin/env python3
"""attitude_step.py - PX4 SITL hover + roll attitude-step experiment (MAVSDK-Python).

Scenario (standard X500, no firmware changes, no ROS 2):

  connect -> readiness -> takeoff -> stable hover
  -> hover-thrust strategy (read MC_HOVER_THRUST, trim altitude in level hold)
  -> offboard control, 5 Hz attitude setpoint stream
  -> LEVEL baseline hold
  -> ROLL STEP (commanded attitude change, held for --hold-s)
  -> RETURN to level + settle
  -> offboard stop (PX4 -> Hold mode) -> land -> disarm
  -> write manifest (params + phase timestamps) and report the ULog

Usage (container must be up: sudo docker compose up -d):

  .venv/bin/python attitude_step.py --roll-deg 10 --hold-s 8 --altitude 2.5

Then verify the logged setpoint sequence:

  .venv/bin/python verify_step_ulog.py
"""

import argparse
import asyncio
import glob
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import Attitude
from mavsdk.param import ParamError

BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "px4-data" / "px4" / "rootfs" / "fs" / "log"
DEFAULT_MANIFEST = BASE_DIR / "attitude_step_run.json"

STREAM_DT = 0.2          # 5 Hz setpoint stream (PX4 needs >= 2 Hz for OFFBOARD)
ALT_TOL_M = 0.35         # hover considered stable within this band
STABLE_S = 1.5           # ...for this long
TRIM_BAND_M = 0.30       # start trimming thrust outside this altitude band
TRIM_STEP = 0.02         # thrust trim increment
TRIM_CLAMP = 0.15        # clamp trim around hover thrust
SAFETY_FLOOR_M = 1.0     # abort-safe altitude floor during step phases
SAFETY_STEP = 0.05       # one-shot thrust bump if floor is hit


def utcnow() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def newest_ulg() -> str | None:
    files = glob.glob(str(LOG_DIR / "**" / "*.ulg"), recursive=True)
    return max(files, key=os.path.getmtime) if files else None


class Streamer:
    """Continuously streams the current target attitude at 5 Hz.

    Started BEFORE offboard.start() so PX4 sees a live setpoint stream when it
    switches to OFFBOARD (required by PX4). Set-attitude calls that fail before
    offboard is active are tolerated and retried.
    """

    def __init__(self, drone: System):
        self._drone = drone
        self._target = Attitude(0.0, 0.0, 0.0, 0.5)
        self._task: asyncio.Task | None = None
        self.errors = 0

    def set(self, roll: float, pitch: float, yaw: float, thrust: float) -> None:
        self._target = Attitude(roll, pitch, yaw, thrust)

    async def _run(self) -> None:
        while True:
            try:
                await self._drone.offboard.set_attitude(self._target)
            except asyncio.CancelledError:
                raise
            except Exception:
                # Expected before offboard.start(); counts only transient after.
                self.errors += 1
            await asyncio.sleep(STREAM_DT)

    def start(self) -> None:
        self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None


async def wait_connection(drone: System, address: str, timeout: float = 30.0) -> None:
    print(f"[connect] {address} ...")
    t0 = time.monotonic()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[connect] connected ({time.monotonic() - t0:.1f}s)")
            return
        if time.monotonic() - t0 > timeout:
            raise TimeoutError(f"no connection to {address} within {timeout:.0f}s")


async def wait_ready(drone: System, timeout: float = 60.0) -> None:
    print("[ready] waiting for armable + position health ...")
    t0 = time.monotonic()
    async for h in drone.telemetry.health():
        ok = h.is_armable and h.is_local_position_ok and h.is_global_position_ok and h.is_home_position_ok
        if ok:
            print("[ready] health OK (armable, local/global/home position)")
            return
        if time.monotonic() - t0 > timeout:
            raise TimeoutError("vehicle not ready (health) within timeout")


async def wait_hover(drone: System, alt: float, timeout: float = 60.0) -> None:
    print(f"[takeoff] waiting for stable hover at {alt:.1f} m ...")
    t0 = time.monotonic()
    stable_since: float | None = None
    async for p in drone.telemetry.position():
        rel = p.relative_altitude_m
        if abs(rel - alt) <= ALT_TOL_M:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= STABLE_S:
                print(f"[hover] stable at {rel:.2f} m")
                return
        else:
            stable_since = None
        if time.monotonic() - t0 > timeout:
            raise TimeoutError(f"hover not stable at {alt:.1f} m within {timeout:.0f}s")


async def capture_yaw(drone: System) -> float:
    async for e in drone.telemetry.attitude_euler():
        print(f"[yaw] holding yaw at {e.yaw_deg:.1f} deg")
        return e.yaw_deg
    return 0.0


async def wait_landed(drone: System, timeout: float = 45.0) -> None:
    t0 = time.monotonic()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[land] vehicle on ground")
            return
        if time.monotonic() - t0 > timeout:
            raise TimeoutError("landing not confirmed within timeout")
    raise TimeoutError("landed-state stream ended unexpectedly")


async def hold_phase(
    streamer: Streamer,
    roll: float,
    yaw: float,
    thrust: float,
    duration: float,
    name: str,
    phases: dict,
    latest: dict,
    alt: float,
    trim: bool = False,
    hover: float = 0.5,
) -> float:
    """Stream `roll` for `duration` seconds. Returns the thrust used (may be trimmed)."""
    print(f"[{name}] roll={roll:+.1f} deg, thrust={thrust:.3f}, {duration:.1f}s")
    phases[name] = {"start_epoch": time.time(), "start_utc": utcnow()}
    t_end = time.monotonic() + duration
    safety_used = False
    while time.monotonic() < t_end:
        rel = latest.get("rel_alt")
        if rel is not None:
            if trim:
                err = rel - alt
                if abs(err) > TRIM_BAND_M:
                    thrust = clamp(
                        thrust + (-TRIM_STEP if err > 0 else TRIM_STEP),
                        hover - TRIM_CLAMP,
                        hover + TRIM_CLAMP,
                    )
            elif rel < SAFETY_FLOOR_M and not safety_used:
                thrust += SAFETY_STEP
                safety_used = True
                print(f"[{name}] SAFETY: alt {rel:.2f} m < {SAFETY_FLOOR_M} m -> thrust {thrust:.3f}")
        streamer.set(roll, 0.0, yaw, thrust)
        await asyncio.sleep(0.5)
    phases[name].update(end_epoch=time.time(), end_utc=utcnow())
    print(f"[{name}] done, thrust={thrust:.3f}")
    return thrust


async def run(args: argparse.Namespace) -> int:
    manifest: dict = {
        "experiment": "attitude_step",
        "created_utc": utcnow(),
        "connect": args.connect,
        "vehicle": "gz_x500 (standard)",
        "params": {
            "roll_deg": args.roll_deg,
            "level_s": args.level_s,
            "hold_s": args.hold_s,
            "settle_s": args.settle_s,
            "altitude_m": args.altitude,
            "stream_hz": round(1.0 / STREAM_DT, 1),
            "hover_thrust_arg": args.hover_thrust,
        },
        "phases": {},
        "status": "running",
        "ulog": None,
    }
    phases = manifest["phases"]
    latest: dict = {"rel_alt": None}
    drone = System()
    streamer = Streamer(drone)
    pos_task: asyncio.Task | None = None
    offboard_started = False
    in_air = False
    status = "aborted"
    exit_code = 1

    try:
        await drone.connect(args.connect)  # initialise plugins
        await wait_connection(drone, args.connect)
        try:
            await drone.telemetry.set_rate_position(10.0)
        except Exception:
            pass  # default rate is fine if this fails

        async def track_position():
            async for p in drone.telemetry.position():
                latest["rel_alt"] = p.relative_altitude_m

        pos_task = asyncio.create_task(track_position())

        await wait_ready(drone)

        # --- takeoff ---
        phases["takeoff"] = {"start_epoch": time.time(), "start_utc": utcnow()}
        await drone.action.set_takeoff_altitude(args.altitude)
        await drone.action.arm()
        print("[arm] armed")
        await drone.action.takeoff()
        print("[takeoff] TAKEOFF mode")
        await wait_hover(drone, args.altitude)
        phases["takeoff"].update(end_epoch=time.time(), end_utc=utcnow())
        in_air = True

        # --- hover-thrust strategy ---
        if args.hover_thrust is not None:
            hover = args.hover_thrust
            print(f"[thrust] using --hover-thrust {hover:.3f}")
        else:
            try:
                hover = await drone.param.get_param_float("MC_HOVER_THRUST")
                print(f"[thrust] MC_HOVER_THRUST = {hover:.3f}")
            except ParamError:
                hover = 0.5
                print("[thrust] MC_HOVER_THRUST unreadable, defaulting to 0.500")
        hover = clamp(hover, 0.30, 0.70)
        manifest["params"]["hover_thrust_used"] = hover

        yaw = await capture_yaw(drone)
        manifest["yaw_deg"] = yaw

        # --- offboard handover ---
        thrust = hover
        streamer.set(0.0, 0.0, yaw, thrust)
        streamer.start()
        await asyncio.sleep(0.6)  # let a few setpoints flow before the mode switch
        await drone.offboard.start()
        offboard_started = True
        print(f"[offboard] OFFBOARD mode active, streaming at "
              f"{manifest['params']['stream_hz']} Hz "
              f"(pre-start setpoint rejects: {streamer.errors})")
        streamer.errors = 0  # count only errors from here on

        # --- scenario ---
        thrust = await hold_phase(streamer, 0.0, yaw, thrust, args.level_s,
                                  "level", phases, latest, args.altitude,
                                  trim=True, hover=hover)
        thrust_frozen = thrust
        manifest["params"]["thrust_frozen"] = round(thrust_frozen, 4)

        await hold_phase(streamer, args.roll_deg, yaw, thrust_frozen, args.hold_s,
                         "step", phases, latest, args.altitude)

        await hold_phase(streamer, 0.0, yaw, thrust_frozen, args.settle_s,
                         "return_settle", phases, latest, args.altitude)

        # --- normal shutdown: offboard -> Hold -> land -> disarm ---
        await streamer.stop()
        phases["offboard_stop"] = {"start_epoch": time.time(), "start_utc": utcnow()}
        await drone.offboard.stop()
        offboard_started = False
        print("[offboard] stopped -> Hold mode")
        await asyncio.sleep(1.0)

        phases["land"] = {"start_epoch": time.time(), "start_utc": utcnow()}
        await drone.action.land()
        print("[land] AUTO.LAND")
        await wait_landed(drone)
        phases["land"].update(end_epoch=time.time(), end_utc=utcnow())
        in_air = False
        try:
            await drone.action.disarm()
            print("[disarm] disarmed")
        except ActionError as e:
            print(f"[disarm] already disarmed ({e})")

        status = "completed"
        exit_code = 0

    except BaseException as e:
        status = f"aborted: {type(e).__name__}: {e}"
        print(f"[abort] {status}")
        raise
    finally:
        # Always try to leave the vehicle safe and write the manifest.
        await streamer.stop()
        if offboard_started:
            try:
                await drone.offboard.stop()
            except Exception:
                pass
        if in_air:
            print("[cleanup] attempting landing ...")
            try:
                await drone.action.land()
                await asyncio.wait_for(wait_landed(drone), timeout=40)
                in_air = False
            except Exception as e:
                print(f"[cleanup] landing failed: {e}")
            try:
                await drone.action.disarm()
            except Exception:
                pass
        if pos_task is not None:
            pos_task.cancel()

        manifest["status"] = status
        manifest["finished_utc"] = utcnow()
        manifest["stream_errors"] = streamer.errors
        manifest["ulog"] = newest_ulg()
        with open(args.manifest, "w") as f:
            json.dump(manifest, f, indent=2)

        print(f"[manifest] {args.manifest}")
        if manifest["ulog"]:
            print(f"[ulog] {manifest['ulog']}")
        print("[verify] .venv/bin/python verify_step_ulog.py --manifest "
              f"{Path(args.manifest).name}")

    return exit_code


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PX4 SITL hover + roll attitude step")
    p.add_argument("--connect", default="udpin://0.0.0.0:14540",
                   help="MAVSDK system address (default: %(default)s)")
    p.add_argument("--altitude", type=float, default=2.5,
                   help="hover altitude in m (default: %(default)s)")
    p.add_argument("--roll-deg", type=float, default=10.0,
                   help="roll step amplitude in deg, + = roll right (default: %(default)s)")
    p.add_argument("--level-s", type=float, default=5.0,
                   help="level baseline hold before the step (default: %(default)s)")
    p.add_argument("--hold-s", type=float, default=8.0,
                   help="how long to hold the stepped attitude (default: %(default)s)")
    p.add_argument("--settle-s", type=float, default=5.0,
                   help="level settle time after the step (default: %(default)s)")
    p.add_argument("--hover-thrust", type=float, default=None,
                   help="override hover thrust 0..1 (default: read MC_HOVER_THRUST)")
    p.add_argument("--manifest", default=str(DEFAULT_MANIFEST),
                   help="manifest output path (default: %(default)s)")
    return p.parse_args()


if __name__ == "__main__":
    code = 1
    try:
        code = asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\n[interrupt] Ctrl+C - cleanup attempted, see manifest")
        code = 130
    except Exception as e:
        print(f"[error] {type(e).__name__}: {e}")
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # gRPC worker threads are non-daemon; exit hard
